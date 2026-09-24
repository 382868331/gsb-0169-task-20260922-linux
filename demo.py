#!/usr/bin/env python3
"""odbus 固定输入演示（确定性波形，真实计算，无网络/硬件/sleep/预录结果）。

运行：python demo.py

两部分：
A. 旧驱动器级接口（parse_updates）：
   1. 正常读事务：主机 M 读从机 E（地址 0x53）的数据 0x9E，含每一位的原始来源；
   2. 仲裁失败：主机 M 发送 1 却被竞争者 C 拉低，解析出 ArbitrationLoss；
   3. 异常边界（实际触发，非模拟）：冲突更新 / SCL 拉伸超时。

B. 新线采样接口（parse_samples，整数时间戳 SCL/SDA）：
   4. 干净线采样解析出写事务；
   5. 最小稳定宽度滤波：SCL 毛刺（否则位错位）与 SCL 高时的 SDA 毛刺
      （否则伪造 STOP/START）都被剔除；
   6. 阈值等号边界：宽度 == min_width 的脉冲保留；
   7. 实际错误：同一时刻重复采样被拒绝。
"""

from __future__ import annotations

from odbus import SAMPLE_DRIVER, Drive, Sample, Update, parse_samples, parse_updates
from odbus.errors import ConflictingUpdateError, DuplicateSampleError, StretchTimeoutError

R = Drive.RELEASE
D = Drive.LOW

# 线采样演示统一的最小稳定宽度（tick）。
MIN_W = 3


class Wave:
    """固定 2-tick 脉冲节奏的波形构建器（绝对驱动状态模型）。

    每个 tick 为所有驱动器各提交一条绝对状态更新（同刻先合并再判边沿）。
    """

    def __init__(self, *names: str) -> None:
        self.t = 0
        self.names = names
        self.st = {n: [R, R] for n in names}
        self.updates: list[Update] = []

    def commit_all(self) -> None:
        for n in self.names:
            scl, sda = self.st[n]
            self.updates.append(Update(tick=self.t, driver=n, scl=scl, sda=sda))

    def advance(self, n: int = 1) -> None:
        self.t += n

    def bit_pulse(self, owner: str, sda: Drive) -> None:
        """一个 SCL 脉冲：下降沿 owner 放好 SDA（其余驱动器释放 SDA），上升沿采样。

        SCL 始终由 M 控制；非 owner 的驱动器在该拍释放 SDA，避免拉住总线。
        """
        for n in self.names:
            self.st[n][1] = sda if n == owner else R
        self.st["M"][0] = D
        self.commit_all()
        self.advance()
        self.st["M"][0] = R
        self.commit_all()
        self.advance()

    def start(self) -> None:
        self.st["M"] = [R, D]
        for n in self.names:
            if n != "M":
                self.st[n] = [R, R]
        self.commit_all()
        self.advance()

    def stop_after_nack_high(self) -> None:
        """NACK（总线 SDA 高）后：补一个脉冲拉低 SDA，再 SCL 高时释放 => STOP。"""
        self.st["M"] = [D, D]
        self.commit_all()
        self.advance()
        self.st["M"] = [R, D]
        self.commit_all()
        self.advance()
        self.st["M"] = [R, R]
        self.commit_all()
        self.advance()


def build_read_transaction() -> list[Update]:
    """M 读 E(0x53)：START + 地址读字节(从机ACK) + 数据0x9E(主机NACK) + STOP。"""
    w = Wave("M", "E")
    w.start()
    addr = (0x53 << 1) | 1  # 0xA7 读
    for k in range(8):
        w.bit_pulse("M", R if ((addr >> (7 - k)) & 1) else D)
    w.bit_pulse("E", D)     # 从机 ACK=0
    data = 0x9E
    for k in range(8):
        w.bit_pulse("E", R if ((data >> (7 - k)) & 1) else D)
    w.bit_pulse("M", R)     # 主机 NACK=1
    w.stop_after_nack_high()
    return w.updates


def build_arbitration_case() -> list[Update]:
    """M 与 C 同起始；地址首位 M 发 1、C 拉低 => M 仲裁丢失，随后补齐 9 拍到 STOP。"""
    ups: list[Update] = []
    t = 0
    ups += [Update(t, "M", R, D), Update(t, "C", R, D)]  # 同时 START
    t += 1
    # bit7：M 释放 SDA（发1），C 拉低（发0）
    ups += [Update(t, "M", D, R), Update(t, "C", R, D)]
    t += 1
    ups += [Update(t, "M", R, R), Update(t, "C", R, D)]  # 上升采样=0 => 丢失
    t += 1
    # 其余 7 位双方都释放（读到全 1），第 9 拍 C 给 ACK=0
    for _ in range(7):
        ups += [Update(t, "M", D, R), Update(t, "C", R, R)]
        t += 1
        ups += [Update(t, "M", R, R), Update(t, "C", R, R)]
        t += 1
    ups += [Update(t, "M", D, R), Update(t, "C", R, D)]
    t += 1
    ups += [Update(t, "M", R, R), Update(t, "C", R, D)]
    t += 1
    # STOP：C 释放 / M 接管（同刻 AND 保持低），下一拍 M 释放 => STOP
    ups += [Update(t, "M", R, D), Update(t, "C", R, R)]
    t += 1
    ups += [Update(t, "M", R, R)]
    return ups


def print_header(title: str) -> None:
    bar = "=" * 68
    print(f"\n{bar}\n{title}\n{bar}")


class SampleWave:
    """线采样波形构建器：每个 SCL 相宽度 = MIN_W（真实信号压住等号边界）。"""

    def __init__(self) -> None:
        self.t = 0
        self.scl = self.sda = 1
        self.samples: list[Sample] = []

    def set(self, scl: int, sda: int) -> None:
        if (scl, sda) != (self.scl, self.sda):
            self.samples.append(Sample(self.t, scl, sda))
            self.scl, self.sda = scl, sda

    def advance(self, n: int = MIN_W) -> None:
        self.t += n

    def start(self) -> None:
        self.set(1, 0)          # SCL 高时 SDA 1->0
        self.advance()

    def pulse(self, sda: int) -> None:
        self.set(0, sda)       # SCL 降，放好数据
        self.advance()
        self.set(1, sda)       # SCL 升，采样
        self.advance()

    def byte(self, value: int, ack: int) -> None:
        for k in range(8):
            self.pulse((value >> (7 - k)) & 1)
        self.pulse(ack)


def build_sample_write(addr7: int, data: tuple[int, ...]) -> list[Sample]:
    """线采样写事务：START + 地址(写,ACK=0) + 数据(ACK=0) + STOP。"""
    w = SampleWave()
    w.start()
    w.byte((addr7 << 1) | 0, 0)
    for v in data:
        w.byte(v, 0)
    w.set(1, 1)               # SCL 高时 SDA 0->1 => STOP
    return w.samples


def main() -> None:
    # ---------- 1. 正常读事务 ----------
    print_header("1) 正常结果：一次读事务（主机 M 读从机 E，地址 0x53）")
    result = parse_updates(build_read_transaction())
    tx = result.transactions[0]
    seg = tx.segments[0]
    print(f"事务数            : {len(result.transactions)}（已闭合 STOP={tx.closed}）")
    print(f"驱动器（注册序）  : {result.drivers}")
    print(f"地址              : 0x{seg.address:02X}  读写位={seg.rw}（1=读）")
    print(f"数据字节          : {' '.join(f'0x{x:02X}' for x in seg.data)}")
    print(f"ACK（每字节第9拍）: {seg.acks}  （0=ACK, 1=NACK）")
    print("原始位来源（数据字节 0x9E = 10011110，由从机 E 驱动 SDA）：")
    data_block = seg.blocks[1]
    for b in data_block.bits:
        who = ",".join(b.source.sda_low_drivers) or "无（上拉为1）"
        print(f"  位{b.index} 值={b.value} 采样@t{b.tick:<2} "
              f"SCL {b.source.scl_low_tick}->{b.source.scl_high_tick} "
              f"SDA低驱动=[{who}]")
    print(f"第 9 拍 NACK      : 值={data_block.ack.value} "
          f"SDA低驱动={data_block.ack.source.sda_low_drivers or '（无，读为1=NACK）'}")

    # ---------- 2. 仲裁失败 ----------
    print_header("2) 仲裁失败：主机 M 发 1，竞争者 C 同刻拉低 SDA")
    res2 = parse_updates(build_arbitration_case())
    seg2 = res2.transactions[0].segments[0]
    print(f"首字节总线实际值  : 0x{seg2.blocks[0].value:02X}（M 期望的首位 1 被裁成 0）")
    for loss in seg2.arbitration_losses:
        print(f"  ArbitrationLoss @t{loss.tick} 字节内位{loss.bit_index} "
              f"阶段={loss.phase.value} 主机={loss.master} "
              f"读到={loss.observed} 拉低者={loss.driven_low_by}")

    # ---------- 3a. 拒绝：同一驱动器同一时刻冲突更新 ----------
    print_header("3a) 异常边界（实际触发）：同一驱动器同刻冲突更新被拒绝")
    conflict = [
        Update(0, "M", R, R),
        Update(1, "M", D, R),
        Update(1, "M", R, D),   # 与上一条同 tick 同驱动器
    ]
    try:
        parse_updates(conflict)
    except ConflictingUpdateError as e:
        print(f"捕获 {type(e).__name__}: {e}")

    # ---------- 3b. 异常：时钟拉伸超时 ----------
    print_header("3b) 异常边界（实际触发）：SCL 低电平超过 stretch_limit")
    stretch_ups = [
        Update(0, "M", R, D),          # START
        Update(1, "M", D, D),          # SCL 降，低电平开始
        Update(2, "M", D, D),
        Update(3, "M", D, D),
        Update(4, "M", D, D),
        Update(5, "M", D, D),          # 到 t=5，duration=4 > limit=3
    ]
    try:
        parse_updates(stretch_ups, stretch_limit=3)
    except StretchTimeoutError as e:
        print(f"捕获 {type(e).__name__}: {e}")

    # ============================================================
    # B. 线采样输入（整数时间戳 SCL/SDA）+ 最小稳定宽度滤波
    # ============================================================

    # ---------- 4. 正常：干净线采样 -> 写事务 ----------
    print_header("4) 线采样正常结果：SCL/SDA 整数时间戳采样，解析写事务 0x50 <- 0xA5")
    clean = build_sample_write(0x50, (0xA5,))
    res4 = parse_samples(clean, MIN_W)
    seg = res4.transactions[0].segments[0]
    print(f"采样条数          : {len(clean)}（每个 SCL 相宽 {MIN_W} tick == min_width，保留）")
    print(f"被滤毛刺          : {len(res4.filtered)} 个")
    print(f"地址/读写/数据    : 0x{seg.address:02X} rw={seg.rw} "
          f"data={tuple(hex(x) for x in seg.data)} ACK={seg.acks} 闭合={seg.closed}")

    # ---------- 5a. SCL 毛刺：不滤波位错位，滤波后干净 ----------
    print_header("5a) 滤波：START 后 SCL 上 1-tick 低毛刺（未滤波会错位，滤波后剔除）")
    glitchy_scl = [clean[0], Sample(1, 0, 0), Sample(2, 1, 0)] + clean[1:]
    bad = parse_updates([
        Update(s.tick, SAMPLE_DRIVER,
               R if s.scl else D, R if s.sda else D)
        for s in glitchy_scl
    ])
    print(f"不滤波首字节      : 0x{bad.transactions[0].segments[0].blocks[0].value:02X}"
          f"（错误，毛刺被当成一个 SCL 脉冲）")
    res5a = parse_samples(glitchy_scl, MIN_W)
    g = res5a.filtered[0]
    print(f"滤波剔除          : {g.line.value} 短脉冲 t{g.begin_tick}->t{g.end_tick} "
          f"宽 {g.duration} < {MIN_W}")
    s5 = res5a.transactions[0].segments[0]
    print(f"滤波后首字节/数据 : 0x{s5.blocks[0].value:02X} "
          f"data={tuple(hex(x) for x in s5.data)}（正确 0xA0 / 0xA5）")

    # ---------- 5b. SDA 毛刺：不滤波伪造 STOP/START，滤波后单事务 ----------
    print_header("5b) 滤波：SCL 高时 SDA 1-tick 跳高毛刺（未滤波伪造 STOP+START）")
    only_addr = build_sample_write(0x50, ())
    before = only_addr[:-1]
    lt = before[-1].tick
    glitchy_sda = before + [
        Sample(lt + 1, 1, 1),   # 宽 1 跳高（未滤波视为 STOP）
        Sample(lt + 2, 1, 0),   # 回低（未滤波视为再次 START）
        Sample(lt + 3, 1, 1),   # 真正 STOP
    ]
    n_bad_tx = len(parse_updates([
        Update(s.tick, SAMPLE_DRIVER, R if s.scl else D, R if s.sda else D)
        for s in glitchy_sda
    ]).transactions)
    res5b = parse_samples(glitchy_sda, MIN_W)
    print(f"不滤波事务数      : {n_bad_tx}（毛刺被误判成 STOP+START，事务被劈开）")
    print(f"滤波后事务/段数   : {len(res5b.transactions)} / "
          f"{len(res5b.transactions[0].segments)}（仍为单事务单段）")
    print(f"被滤毛刺          : {res5b.filtered[0].line.value} "
          f"宽 {res5b.filtered[0].duration} 的高脉冲，剔除后两侧低电平合并")

    # ---------- 6. 等号边界：宽度 == min_width 保留，再窄 1 tick 剔除 ----------
    print_header("6) 阈值等号边界：width == min_width 保留；width == min_width-1 剔除")
    kept = parse_samples([Sample(0, 1, 0), Sample(MIN_W, 1, 1)], MIN_W)
    dropped = parse_samples([Sample(0, 1, 0), Sample(MIN_W - 1, 1, 1)], MIN_W)
    print(f"低宽 {MIN_W} (==min_width): 事务数={len(kept.transactions)}（START/STOP 保留）")
    print(f"低宽 {MIN_W - 1} (<min_width) : 事务数={len(dropped.transactions)}，"
          f"剔除毛刺 {len(dropped.filtered)} 个（不产生 START/STOP）")

    # ---------- 7. 实际错误：同一时刻重复采样 ----------
    print_header("7) 线采样异常（实际触发）：同一整数时刻重复采样被拒绝")
    dup = [Sample(0, 1, 1), Sample(1, 0, 1), Sample(1, 1, 0)]
    try:
        parse_samples(dup, MIN_W)
    except DuplicateSampleError as e:
        print(f"捕获 {type(e).__name__}: {e}")

    print("\n演示完成：以上结果均由 odbus 对固定输入真实计算得到（无 sleep、无预录结果）。")


if __name__ == "__main__":
    main()
