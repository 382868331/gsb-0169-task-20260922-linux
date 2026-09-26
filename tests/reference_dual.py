"""双主机逐拍仿真的独立参考实现（仅供测试使用，不导入 odbus）。

与被测核心 odbus.masters 刻意保持两套写法：
- 输入是每台主机每拍的 (scl, sda) 0/1 主张；
- 逐拍按**真值表**合成导线：bus = a AND b（两者任一 0 则线为 0）；
- 用 while 循环 + 位串累加解码，不使用 odbus 的任何数据结构；
- 不启动真实 I2C 设备，纯离线整数计算。
"""

from __future__ import annotations

from dataclasses import dataclass, field

H = 1  # 释放
L = 0  # 拉低


@dataclass
class RefConflict:
    tick: int
    byte_index: int
    bit_index: int
    loser: str


@dataclass
class RefStretch:
    start: int
    end: int
    waiters: tuple[str, ...]


@dataclass
class RefResult:
    winner: str | None
    loser: str | None
    first_conflict: RefConflict | None
    conflicts: list[RefConflict] = field(default_factory=list)
    stretches: list[RefStretch] = field(default_factory=list)
    # 每个事务 = 若干段；每段 = 9 拍一组的字节值列表（含 ACK 位单独记录）。
    bytes_per_tx: list[list[int]] = field(default_factory=list)
    acks_per_tx: list[list[int]] = field(default_factory=list)
    closed: list[bool] = field(default_factory=list)


def ref_run(scripts: dict[str, list[tuple[int, int]]]) -> RefResult:
    """逐拍真值表合成 + 仲裁判定 + 位串解码的独立参考。"""
    names = list(scripts)
    assert len(names) == 2
    n = len(scripts[names[0]])
    assert all(len(scripts[k]) == n for k in names)

    lost = {k: False for k in names}
    res = RefResult(winner=None, loser=None, first_conflict=None)

    in_tx = False
    bit_in_byte = 0
    byte_count = 0
    seg_rw = None
    cur_bits = ""       # 当前字节已采数据位（'0'/'1' 累加）
    cur_bytes: list[int] = []
    cur_acks: list[int] = []
    pscl = psda = H
    low_since = None
    waiters: set[str] = set()

    tick = 0
    while tick < n:
        lv = {}
        for k in names:
            if lost[k]:
                lv[k] = (H, H)
            else:
                lv[k] = scripts[k][tick]
        # 真值表：任一为 0（拉低）则线为 0
        scl = 0 if (lv[names[0]][0] == 0 or lv[names[1]][0] == 0) else 1
        sda = 0 if (lv[names[0]][1] == 0 or lv[names[1]][1] == 0) else 1

        if pscl == H and scl == H and psda == H and sda == L:
            in_tx = True
            bit_in_byte = 0
            byte_count = 0
            seg_rw = None
            cur_bits = ""
            cur_bytes = []
            cur_acks = []
        elif pscl == H and scl == H and psda == L and sda == H:
            if in_tx:
                res.bytes_per_tx.append(cur_bytes)
                res.acks_per_tx.append(cur_acks)
                res.closed.append(True)
            in_tx = False
            bit_in_byte = 0
            byte_count = 0
            seg_rw = None
            cur_bits = ""
        elif pscl == L and scl == H and in_tx:
            if bit_in_byte < 8:
                master_sends = byte_count == 0 or seg_rw == 0
                if master_sends and sda == L:
                    for k in names:
                        if lost[k] or lv[k][1] == L:
                            continue
                        c = RefConflict(tick=tick, byte_index=byte_count,
                                        bit_index=bit_in_byte, loser=k)
                        res.conflicts.append(c)
                        if res.first_conflict is None:
                            res.first_conflict = c
                        lost[k] = True
                cur_bits += str(sda)
            else:
                cur_acks.append(sda)
            bit_in_byte += 1
            if bit_in_byte == 9:
                cur_bytes.append(int(cur_bits, 2))
                if byte_count == 0:
                    seg_rw = int(cur_bits, 2) & 1
                byte_count += 1
                bit_in_byte = 0
                cur_bits = ""

        if scl == L:
            if low_since is None:
                low_since = tick
                waiters = set()
            for k in names:
                if not lost[k] and lv[k][0] == H:
                    waiters.add(k)
        elif pscl == L and low_since is not None:
            if waiters:
                res.stretches.append(
                    RefStretch(start=low_since, end=tick,
                               waiters=tuple(k for k in names if k in waiters))
                )
            low_since = None
            waiters = set()

        pscl, psda = scl, sda
        tick += 1

    if in_tx:
        res.bytes_per_tx.append(cur_bytes)
        res.acks_per_tx.append(cur_acks)
        res.closed.append(False)

    if res.conflicts:
        res.loser = res.conflicts[0].loser
        res.winner = next(k for k in names if k != res.loser)
    return res

