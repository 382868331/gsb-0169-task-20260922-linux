"""独立参考实现与确定性波形生成器（仅供测试使用）。

本模块**不导入 odbus**：
- 波形用裸元组 ``(tick, driver, scl, sda)`` 表示，电平取 0/1；
- 参考解析器 :func:`ref_simulate` 用密集时刻扫描 + 位字符串累加实现，
  与被测核心的“事件分组 + 逐组合并”是两套独立写法；
- 被测核心由测试文件把裸元组转换为 ``odbus.Update`` 后再调用，
  期望值全部来自这里的独立计算，而不是调用核心得到。
"""

from __future__ import annotations

from dataclasses import dataclass, field

# 电平常量
H = 1  # 释放（上拉为高）
L = 0  # 拉低


class RefError(ValueError):
    """独立参考检出的错误（与核心错误类型刻意保持独立）。"""


class RefConflict(RefError):
    pass


class RefStretch(RefError):
    pass


@dataclass
class RefBlock:
    value: int
    ack: int


@dataclass
class RefSeg:
    addr: int | None
    rw: int | None
    blocks: list[RefBlock] = field(default_factory=list)
    arb_bits: list[int] = field(default_factory=list)
    closed: bool = False
    repeated: bool = False

    @property
    def data(self) -> tuple[int, ...]:
        return tuple(b.value for b in self.blocks[1:])

    @property
    def acks(self) -> tuple[int, ...]:
        return tuple(b.ack for b in self.blocks)


# --------------------------- 波形生成器 ---------------------------

class Generator:
    """按固定 2-tick 位脉冲节奏生成裸更新。

    每拍：tick t 为 SCL 下降沿（数据就绪），t+1 为上升沿（采样）。
    每个 tick 同时为 M/V 两个驱动器各给一条绝对状态更新，
    天然覆盖“同刻不同驱动器双线变化”。
    """

    def __init__(self) -> None:
        self.t = 0
        self.raw: list[tuple[int, str, int, int]] = []

    def both(self, m: tuple[int, int], v: tuple[int, int]) -> None:
        self.raw.append((self.t, "M", m[0], m[1]))
        self.raw.append((self.t, "V", v[0], v[1]))

    def advance(self, n: int = 1) -> None:
        self.t += n

    # --- 基本构件 ---

    def start(self) -> None:
        # SCL 高，SDA 高->低 => START
        self.both((H, L), (H, H))
        self.advance()

    def pulse(self, m_sda: int, v_sda: int, stretch: int = 0) -> None:
        # 下降沿 + 数据；可选拉伸：上升前在低电平上保持 stretch 个 tick
        self.both((L, m_sda), (H, v_sda))
        self.advance()
        for _ in range(stretch):
            self.both((L, m_sda), (H, v_sda))
            self.advance()
        self.both((H, m_sda), (H, v_sda))
        self.advance()

    def byte_master_sends(self, value: int, ack: int, stretch: int = 0) -> None:
        for k in range(8):
            self.pulse((value >> (7 - k)) & 1, H, stretch if k == 0 else 0)
        # ACK：主机释放 SDA，从机给 ACK/NACK
        self.pulse(H, ack)

    def byte_slave_sends(self, value: int, master_ack: int) -> None:
        for k in range(8):
            self.pulse(H, (value >> (7 - k)) & 1)
        # 第 9 拍：从机释放，主机给 ACK/NACK
        self.pulse(master_ack, H)

    def stop(self, last_ack_level: int) -> None:
        """生成 STOP。"""
        if last_ack_level == 0:
            # 总线 SDA 为低：先同刻完成 V 释放 / M 接管（合并后仍为低，无边沿），
            # 再由 M 释放，SCL 高时 SDA 0->1。
            self.both((H, L), (H, H))
            self.advance()
            self.both((H, H), (H, H))
            self.advance()
        else:
            # NACK 后总线已高：补一个 SCL 脉冲把 SDA 拉低，再做 STOP。
            # 该脉冲在 STOP 前只产生一个残余采样位，不影响完整字节计数。
            self.both((L, L), (H, H))
            self.advance()
            self.both((H, L), (H, H))
            self.advance()
            self.both((H, H), (H, H))
            self.advance()

    def repeated_start(self) -> None:
        """在 ACK=0（总线 SDA 低）之后生成重复 START。"""
        self.both((H, L), (H, H))   # 同刻 V 释放 / M 接管，AND 保持低
        self.advance()
        self.both((L, L), (H, H))   # SCL 降
        self.advance()
        self.both((L, H), (H, H))   # SCL 低时 SDA 升
        self.advance()
        self.both((H, H), (H, H))   # SCL 升（残余脉冲，随后被 START 清零）
        self.advance()
        self.both((H, L), (H, H))   # SCL 高时 SDA 降 => 重复 START
        self.advance()


@dataclass
class Scenario:
    raw: list[tuple[int, str, int, int]]
    expected: list[list[RefSeg]]  # transactions -> segments
    stretch_limit: int | None
    expect_timeout: bool
    max_low_run: int


def gen_scenario(seed: int) -> Scenario:
    """确定性 LCG 场景：若干事务，含写/读、ACK/NACK、重复 START、拉伸、缺 STOP。"""
    state = seed & 0xFFFFFFFF

    def rnd() -> int:
        nonlocal state
        state = (state * 1103515245 + 12345) & 0x7FFFFFFF
        return state

    g = Generator()
    expected: list[list[RefSeg]] = []
    max_low_run = 0
    force_timeout = (seed % 11 == 5)

    n_tx = 1 + (rnd() % 3)
    for _ in range(n_tx):
        cur_tx: list[RefSeg] = []
        n_seg = 1 + (rnd() % 2)  # 1~2 段（第二段即重复 START）
        for si in range(n_seg):
            if si == 0:
                g.start()
                repeated = False
            else:
                g.repeated_start()
                repeated = True
            addr7 = rnd() & 0x7F
            rw = rnd() & 1
            addr_byte = (addr7 << 1) | rw
            seg = RefSeg(addr=addr7, rw=rw, repeated=repeated)
            more_segments = si < n_seg - 1

            stretch = 0
            if rnd() % 3 == 0:
                stretch = 1 + (rnd() % 3)  # 首拍前保持低 1~3 tick
                max_low_run = max(max_low_run, stretch + 1)
            if force_timeout and si == n_seg - 1:
                stretch = 6
                max_low_run = max(max_low_run, 7)
            # 后面还有重复 START 时，本段所有 ACK 强制为 0（总线保持低）。
            addr_ack = 0 if (more_segments or rnd() % 5 == 0) else 1
            g.byte_master_sends(addr_byte, addr_ack, stretch)
            seg.blocks.append(RefBlock(addr_byte, addr_ack))

            n_data = rnd() % 3
            last_ack = addr_ack
            for _d in range(n_data):
                forced = 0 if more_segments else None
                if rw == 0:
                    val = rnd() & 0xFF
                    ack = forced if forced is not None else (0 if rnd() % 4 else 1)
                    g.byte_master_sends(val, ack)
                else:
                    val = rnd() & 0xFF
                    ack = forced if forced is not None else (0 if rnd() % 4 else 1)
                    g.byte_slave_sends(val, ack)
                seg.blocks.append(RefBlock(val, ack))
                last_ack = ack

            cur_tx.append(seg)
            if more_segments:
                continue
            if rnd() % 4 == 0 and not force_timeout:
                # 缺 STOP：仅最后一个事务开放，且之后不再追加事务。
                seg.closed = False
                expected.append(cur_tx)
                cur_tx = []  # noqa: 标记已落盘
                break
            g.stop(last_ack)
            seg.closed = True
        if cur_tx:
            expected.append(cur_tx)
        if expected and not expected[-1][-1].closed:
            break  # 开放事务后流即结束

    limit = 4 if force_timeout else 10
    return Scenario(
        raw=g.raw,
        expected=expected,
        stretch_limit=limit,
        expect_timeout=force_timeout,
        max_low_run=max_low_run,
    )


# --------------------------- 独立参考解析器 ---------------------------

def ref_simulate(
    raw: list[tuple[int, str, int, int]],
    stretch_limit: int | None = None,
) -> list[list[RefSeg]]:
    """密集时刻扫描版参考解析器（与核心完全独立的实现方式）。

    - 先把更新排进 ``tick -> driver -> [scl,sda]``，拒绝同驱动器同刻重复；
    - 在 [t0, t1] 每个整数 tick 上回放（缺更新的 tick 状态保持）；
    - 用位字符串累积 9 个采样拼成字节；
    - 仲裁直接以驱动器 'M' 为主机，规则独立重述：地址 8 位恒查，
      数据位仅写方向（rw==0）查，读段从机发数不查。
    """
    if not raw:
        return []

    table: dict[int, dict[str, list[int]]] = {}
    order: list[str] = []
    seen: set[tuple[int, str]] = set()
    for t, d, scl, sda in raw:
        if (t, d) in seen:
            raise RefConflict(f"duplicate driver {d!r} at tick {t}")
        seen.add((t, d))
        table.setdefault(t, {})[d] = [scl, sda]
        if d not in order:
            order.append(d)

    t0, t1 = min(table), max(table)
    cur: dict[str, list[int]] = {d: [H, H] for d in order}
    pscl = psda = H
    completed: list[list[RefSeg]] = []
    cur_tx: list[RefSeg] | None = None
    seg: RefSeg | None = None
    bits = ""
    low_since: int | None = None

    def line_level(line: int) -> int:
        v = H
        for d in order:
            v &= cur[d][line]
        return v

    for t in range(t0, t1 + 1):
        for d, pair in table.get(t, {}).items():
            cur[d] = [pair[0], pair[1]]
        scl, sda = line_level(0), line_level(1)

        # 拉伸（与核心同一精确口径：duration > limit 才报）
        if stretch_limit is not None:
            if scl == L and low_since is not None and t - low_since > stretch_limit:
                raise RefStretch(f"stretch timeout at tick {t}")
        if scl == H:
            low_since = None
        elif pscl == H:
            low_since = t

        if pscl == H and scl == H and psda == H and sda == L:
            # START / 重复 START
            if cur_tx is None:
                cur_tx = []
            seg = RefSeg(addr=None, rw=None, repeated=bool(cur_tx))
            cur_tx.append(seg)
            bits = ""
            pscl, psda = scl, sda
            continue
        if pscl == H and scl == H and psda == L and sda == H:
            # STOP
            if seg is not None:
                seg.closed = True
            completed.append(cur_tx if cur_tx is not None else [])
            cur_tx = None
            seg = None
            bits = ""
            pscl, psda = scl, sda
            continue

        if pscl == L and scl == H and seg is not None:
            bits += str(sda)
            pos = len(bits) - 1
            master_sends = len(seg.blocks) == 0 or seg.rw == 0
            if pos < 8 and master_sends and cur["M"][1] == H and sda == L:
                seg.arb_bits.append(pos)
            if len(bits) == 9:
                val = int(bits[:8], 2)
                seg.blocks.append(RefBlock(val, int(bits[8])))
                if len(seg.blocks) == 1:
                    seg.rw = val & 1
                    seg.addr = val >> 1
                bits = ""

        pscl, psda = scl, sda

    if cur_tx is not None:
        completed.append(cur_tx)
    return completed
