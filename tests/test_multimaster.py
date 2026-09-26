"""两个开漏主机逐拍驱动的仲裁迭代测试。

覆盖：地址阶段分胜负、数据阶段才分胜负、完全相同发送序列、ACK 阶段不误判、
时钟拉伸与仲裁分开记录、异常路径；并以 tests/reference.py 中**不导入 odbus**
的独立参考（逐拍真值表合成导线）做差分对照。不启动真实 I2C 设备。
"""

from __future__ import annotations

import unittest

from odbus import (
    BitPhase,
    ConflictingUpdateError,
    Drive,
    InvalidUpdateError,
    StretchTimeoutError,
    Update,
    iterate_masters,
)

from reference import H, L, RefConflict, RefError, ref_iterate

R = Drive.RELEASE
D = Drive.LOW


class Script:
    """单主机的逐拍驱动主张脚本（2-tick 位脉冲节奏，绝对状态模型）。"""

    def __init__(self, name: str) -> None:
        self.name = name
        self.t = 0
        self.raw: list[tuple[int, str, int, int]] = []

    def emit(self, scl: int, sda: int, n: int = 1) -> None:
        for _ in range(n):
            self.raw.append((self.t, self.name, scl, sda))
            self.t += 1

    def start(self) -> None:
        self.emit(H, L)

    def bit(self, b: int, high: int = 1) -> None:
        self.emit(L, b)
        self.emit(H, b, high)

    def byte(self, value: int, ack: int = 1) -> None:
        for k in range(8):
            self.bit((value >> (7 - k)) & 1)
        self.bit(ack)  # 第 9 拍：ack=1 释放（无从机 => NACK），ack=0 拉低

    def stop(self) -> None:
        self.emit(L, L)
        self.emit(H, L)
        self.emit(H, H)


def merge(*scripts: Script) -> list[tuple[int, str, int, int]]:
    raw: list[tuple[int, str, int, int]] = []
    for s in scripts:
        raw.extend(s.raw)
    return sorted(raw, key=lambda e: e[0])


def to_updates(raw):
    return [
        Update(tick=t, driver=d, scl=R if scl == H else D, sda=R if sda == H else D)
        for (t, d, scl, sda) in raw
    ]


def scenario_address_phase():
    """地址第 5 位分胜负；失败方后续的 STOP 序列被释放机制吞掉。"""
    w, l = Script("W"), Script("L")
    w.start(); l.start()
    w.byte(0xA0); l.byte(0xA2)          # 0xA0=10100000, 0xA2=10100010 -> bit6 分歧
    w.bit(0); l.bit(0)                  # 数据 bit0（t19..t20）
    w.bit(1, high=2)                    # t21..t23：W 的 SCL 高电平拉满 2 tick
    l.emit(0, 1); l.emit(1, 0); l.emit(1, 1)  # L 若不释放将在 t23 制造假 STOP
    for k in range(2, 8):
        b = (0x5A >> (7 - k)) & 1
        w.bit(b); l.bit(b)
    w.bit(1); l.bit(1)                  # ACK：均释放（NACK）
    w.stop(); l.stop()                  # t38..t40
    return merge(w, l)


def scenario_data_phase():
    """地址相同，数据字节首位才分胜负。"""
    w, l = Script("W"), Script("L")
    w.start(); l.start()
    w.byte(0xA0); l.byte(0xA0)
    w.bit(0); l.bit(1)                  # 数据 bit0：L 发 1，W 发 0 -> L 负
    for _ in range(7):
        w.bit(0); l.bit(1)
    w.bit(1); l.bit(1)
    w.stop(); l.stop()
    return merge(w, l)


def scenario_identical():
    """完全相同的发送序列：无仲裁丢失，两家并列。"""
    w, l = Script("W"), Script("L")
    for s in (w, l):
        s.start()
        s.byte(0xA0)
        s.byte(0x5A)
        s.stop()
    return merge(w, l)


def scenario_ack_not_judged():
    """ACK 阶段总线被另一方拉低：不能按数据位规则误判。"""
    w, l = Script("W"), Script("L")
    w.start(); l.start()
    for k in range(8):
        b = (0xA0 >> (7 - k)) & 1
        w.bit(b); l.bit(b)
    w.bit(1); l.bit(0)                  # ACK 拍：W 释放、L 拉低 -> 总线 ACK=0
    w.byte(0x3C); l.byte(0x3C)
    w.stop(); l.stop()
    return merge(w, l)


def scenario_stretch():
    """W 在地址首位多拉低 SCL 两拍：拉伸与仲裁分开记录。"""
    w, l = Script("W"), Script("L")
    w.start(); l.start()
    w.emit(L, 1, 3); w.emit(H, 1)       # t1..t4：W 拉低 3 拍
    l.emit(L, 1); l.emit(H, 1, 3)       # L 第 1 拍后即释放，被拖住
    for k in range(1, 8):
        b = (0xA0 >> (7 - k)) & 1
        w.bit(b); l.bit(b)
    w.bit(1); l.bit(1)
    w.byte(0x11); l.byte(0x11)
    w.stop(); l.stop()
    return merge(w, l)


class AddressPhaseTests(unittest.TestCase):
    def test_address_phase_decides(self):
        it = iterate_masters(to_updates(scenario_address_phase()))
        self.assertEqual(it.winners, ("W",))
        self.assertEqual(it.winner, "W")
        self.assertEqual(len(it.losses), 1)
        loss = it.losses[0]
        self.assertEqual(loss.master, "L")
        self.assertEqual(loss.tick, 14)          # bit6 上升沿
        self.assertEqual(loss.bit_index, 6)
        self.assertEqual(loss.phase, BitPhase.ADDRESS)
        self.assertEqual(loss.observed, 0)
        self.assertEqual(loss.driven_low_by, ("W",))
        # 赢家的解码事务完整到达 STOP，未被失败方的假 STOP 打断
        txs = it.result.transactions
        self.assertEqual(len(txs), 1)
        seg = txs[0].segments[0]
        self.assertTrue(txs[0].closed)
        self.assertEqual(seg.stop.tick, 40)
        self.assertEqual(seg.address, 0x50)
        self.assertEqual(seg.rw, 0)
        self.assertEqual(seg.data, (0x5A,))
        self.assertEqual(seg.acks, (1, 1))
        self.assertEqual(it.stretch_events, ())

    def test_loser_releases_from_next_phase(self):
        it = iterate_masters(to_updates(scenario_address_phase()))
        # 失败方 L 自判决（t14）后的下一驱动阶段起，实际总线更新全部释放：
        # 其后续拉低主张（含 STOP 序列）不再上线。
        for u in it.effective_updates:
            if u.driver == "L" and u.tick >= 15:
                self.assertEqual(u.scl, R)
                self.assertEqual(u.sda, R)
        # 判决当拍（t14）仍是 L 自己的主张（本就为释放）
        at14 = [u for u in it.effective_updates if u.driver == "L" and u.tick == 14]
        self.assertEqual(len(at14), 1)


class DataPhaseTests(unittest.TestCase):
    def test_data_phase_decides(self):
        it = iterate_masters(to_updates(scenario_data_phase()))
        self.assertEqual(it.winners, ("W",))
        self.assertEqual(len(it.losses), 1)
        loss = it.losses[0]
        self.assertEqual(loss.master, "L")
        self.assertEqual(loss.tick, 20)
        self.assertEqual(loss.bit_index, 0)
        self.assertEqual(loss.phase, BitPhase.DATA)
        seg = it.result.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x50)
        self.assertEqual(seg.data, (0x00,))      # 赢家数据原样解码
        self.assertTrue(it.result.transactions[0].closed)


class IdenticalSequenceTests(unittest.TestCase):
    def test_identical_sequences_no_loser(self):
        it = iterate_masters(to_updates(scenario_identical()))
        self.assertEqual(it.losses, ())
        self.assertEqual(it.winners, ("W", "L"))
        self.assertIsNone(it.winner)             # 并列，无唯一赢家
        seg = it.result.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x50)
        self.assertEqual(seg.data, (0x5A,))
        self.assertTrue(it.result.transactions[0].closed)


class AckPhaseTests(unittest.TestCase):
    def test_ack_phase_not_judged_as_data(self):
        it = iterate_masters(to_updates(scenario_ack_not_judged()))
        self.assertEqual(it.losses, ())          # ACK 拍不按数据位规则误判
        self.assertEqual(it.winners, ("W", "L"))
        seg = it.result.transactions[0].segments[0]
        self.assertEqual(seg.acks, (0, 1))       # 地址 ACK=0（L 拉低），数据 NACK=1
        self.assertEqual(seg.data, (0x3C,))


class StretchTests(unittest.TestCase):
    def test_stretch_recorded_separately(self):
        it = iterate_masters(to_updates(scenario_stretch()))
        self.assertEqual(it.losses, ())          # 纯拉伸，无仲裁
        self.assertEqual(it.winners, ("W", "L"))
        self.assertEqual(len(it.stretch_events), 1)
        ev = it.stretch_events[0]
        self.assertEqual(ev.start_tick, 2)
        self.assertEqual(ev.end_tick, 4)
        self.assertEqual(ev.holders, ("W",))
        self.assertEqual(ev.released, ("L",))
        seg = it.result.transactions[0].segments[0]
        self.assertEqual(seg.data, (0x11,))

    def test_stretch_limit_boundary(self):
        raw = scenario_stretch()
        # 低电平持续 3 tick（t1..t3）：limit=3 恰好合法，limit=2 必须报错。
        it = iterate_masters(to_updates(raw), stretch_limit=3)
        self.assertEqual(len(it.result.transactions), 1)
        with self.assertRaises(StretchTimeoutError):
            iterate_masters(to_updates(raw), stretch_limit=2)


class DifferentialTests(unittest.TestCase):
    """与 tests/reference.py 的独立逐拍真值表参考对照。"""

    def check(self, raw):
        ref = ref_iterate(raw)
        it = iterate_masters(to_updates(raw))
        got_eff = [
            (u.tick, u.driver, u.scl.line_level, u.sda.line_level)
            for u in it.effective_updates
        ]
        self.assertEqual(got_eff, ref.effective)
        got_losses = {l.master: (l.tick, l.bit_index) for l in it.losses}
        self.assertEqual(got_losses, ref.losses)
        self.assertEqual(it.winners, ref.winners)
        got_st = [(e.start_tick, e.end_tick, e.holders, e.released)
                  for e in it.stretch_events]
        self.assertEqual(got_st, ref.stretches)

    def test_address_phase(self):
        self.check(scenario_address_phase())

    def test_data_phase(self):
        self.check(scenario_data_phase())

    def test_identical(self):
        self.check(scenario_identical())

    def test_ack_not_judged(self):
        self.check(scenario_ack_not_judged())

    def test_stretch(self):
        self.check(scenario_stretch())


class ValidationTests(unittest.TestCase):
    def test_single_master_rejected(self):
        raw = [(0, "W", H, L), (1, "W", L, L), (2, "W", H, L)]
        with self.assertRaises(InvalidUpdateError):
            iterate_masters(to_updates(raw))

    def test_empty_rejected(self):
        with self.assertRaises(InvalidUpdateError):
            iterate_masters([])

    def test_conflicting_intent_rejected(self):
        ups = [
            Update(0, "W", R, D), Update(0, "L", R, D),
            Update(1, "W", D, D), Update(1, "W", R, R),  # 同刻同主机第二条
        ]
        with self.assertRaises(ConflictingUpdateError):
            iterate_masters(ups)

    def test_bool_tick_rejected(self):
        ups = [Update(True, "W", R, D), Update(0, "L", R, D)]
        with self.assertRaises(InvalidUpdateError):
            iterate_masters(ups)

    def test_float_and_nan_tick_rejected(self):
        for bad in (1.5, float("nan"), float("inf")):
            ups = [Update(0, "W", R, D), Update(bad, "L", R, D)]
            with self.assertRaises(InvalidUpdateError):
                iterate_masters(ups)

    def test_bad_drive_rejected(self):
        ups = [Update(0, "W", 1, D), Update(0, "L", R, D)]
        with self.assertRaises(InvalidUpdateError):
            iterate_masters(ups)

    def test_reference_conflict_and_count(self):
        # 独立参考自身的异常路径：同刻同主机重复、主机数不足。
        with self.assertRaises(RefConflict):
            ref_iterate([(0, "W", H, L), (0, "W", H, H), (0, "L", H, L)])
        with self.assertRaises(RefError):
            ref_iterate([(0, "W", H, L)])


if __name__ == "__main__":
    unittest.main()
