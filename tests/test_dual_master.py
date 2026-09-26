"""双主机逐拍驱动仲裁（odbus.masters）的测试。

覆盖：地址阶段分胜负、数据阶段分胜负、完全相同发送序列、ACK 阶段不误判、
失败方释放总线不打断赢家、时钟拉伸与仲裁分开记录、独立参考差分、异常路径。
"""

from __future__ import annotations

import unittest

from odbus import Drive, StretchTimeoutError
from odbus.errors import InvalidUpdateError
from odbus.masters import (
    DriveTick,
    pad_script,
    run_dual_master,
    write_script,
)

from reference_dual import ref_run

R, L = Drive.RELEASE, Drive.LOW


def to_raw(script):
    """DriveTick 脚本 -> 独立参考的 0/1 元组脚本。"""
    return [(t.scl.line_level, t.sda.line_level) for t in script]


def aligned(a, b):
    n = max(len(a), len(b))
    return pad_script(a, n), pad_script(b, n)


class DualMasterScenariosTest(unittest.TestCase):
    def test_address_phase_decides(self):
        # A 地址 0x50（...1010000），B 地址 0x51：bit6 B 发 1、A 发 0 => B 丢。
        sa, sb = aligned(write_script(0x50, [0x11, 0x22]),
                         write_script(0x51, [0x11, 0x22]))
        res = run_dual_master({"A": sa, "B": sb})
        self.assertEqual(res.winner, "A")
        self.assertEqual(res.loser, "B")
        fc = res.first_conflict
        self.assertIsNotNone(fc)
        self.assertEqual((fc.byte_index, fc.bit_index), (0, 6))
        self.assertEqual(fc.phase.value, "address")
        self.assertEqual(fc.tick, 14)  # START 后第 7 个脉冲的上升沿
        self.assertEqual((fc.loser, fc.winner), ("B", "A"))
        seg = res.parse.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x50)
        self.assertEqual(seg.rw, 0)
        self.assertEqual(seg.data, (0x11, 0x22))
        self.assertTrue(seg.closed)

    def test_data_phase_decides(self):
        # 地址相同（0x34），数据字节不同：A 发 0x00，B 发 0x80 => B 在数据首位丢。
        sa, sb = aligned(write_script(0x34, [0x00]), write_script(0x34, [0x80]))
        res = run_dual_master({"A": sa, "B": sb})
        self.assertEqual(res.winner, "A")
        self.assertEqual(res.loser, "B")
        fc = res.first_conflict
        self.assertEqual((fc.byte_index, fc.bit_index), (1, 0))
        self.assertEqual(fc.phase.value, "data")
        self.assertEqual(fc.tick, 20)  # 地址 9 拍 + 数据首位的上升沿
        seg = res.parse.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x34)
        self.assertEqual(seg.data, (0x00,))
        self.assertTrue(seg.closed)

    def test_identical_sequences_no_conflict(self):
        script = write_script(0x50, [0x11, 0x22])
        res = run_dual_master({"A": script, "B": list(script)})
        self.assertIsNone(res.winner)
        self.assertIsNone(res.loser)
        self.assertIsNone(res.first_conflict)
        self.assertEqual(res.conflicts, ())
        seg = res.parse.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x50)
        self.assertEqual(seg.data, (0x11, 0x22))
        self.assertTrue(seg.closed)

    def test_ack_phase_not_misjudged(self):
        # ACK 拍 B 拉低 SDA（应答）、A 释放：按数据位规则会误判 A 丢，这里必须不判。
        sa = write_script(0x50, [0x11])
        sb = list(write_script(0x50, [0x11]))
        # 地址字节 ACK 拍：tick 17（SCL 低）/ tick 18（SCL 高采样）
        sb[17] = DriveTick(L, L)
        sb[18] = DriveTick(R, L)
        res = run_dual_master({"A": sa, "B": sb})
        self.assertEqual(res.conflicts, ())
        self.assertIsNone(res.loser)
        seg = res.parse.transactions[0].segments[0]
        self.assertEqual(seg.acks[0], 0)  # 总线确实采到 ACK=0
        self.assertTrue(seg.closed)

    def test_loser_releases_and_cannot_break_winner(self):
        # B 在地址阶段丢失后，其脚本后续的 STOP 动作不得打断 A 的事务。
        sa, sb = aligned(write_script(0x50, [0x11, 0x22]),
                         write_script(0x51, [0x11, 0x22]))
        res = run_dual_master({"A": sa, "B": sb})
        conflict_tick = res.first_conflict.tick
        # 失败方从下一驱动阶段起全部释放
        for u in res.updates:
            if u.driver == "B" and u.tick > conflict_tick:
                self.assertEqual((u.scl, u.sda), (R, R))
        # 赢家的单个事务完整闭合（B 的 STOP 没有提前切走总线）
        self.assertEqual(len(res.parse.transactions), 1)
        tx = res.parse.transactions[0]
        self.assertTrue(tx.closed)
        self.assertEqual(tx.segments[0].data, (0x11, 0x22))

    def test_stretch_recorded_separately_from_arbitration(self):
        # 完全相同的发送，但 B 在首个数据位多拉低 SCL 3 拍（时钟拉伸）。
        sa = list(write_script(0x33, [0x5A]))
        sb = list(write_script(0x33, [0x5A]))
        bit_sda = sb[1].sda
        sb[2:2] = [DriveTick(L, bit_sda)] * 3  # B 持低 => 总线低电平延长
        sa[2:2] = [DriveTick(R, bit_sda)] * 3  # A 已释放，被迫等待
        res = run_dual_master({"A": sa, "B": sb})
        self.assertEqual(res.conflicts, ())
        self.assertEqual(res.arbitration_losses, ())
        self.assertEqual(len(res.stretches), 1)
        st = res.stretches[0]
        self.assertEqual(st.duration, 4)
        self.assertIn("A", st.waiters)
        self.assertIn("B", st.holders)
        seg = res.parse.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x33)
        self.assertEqual(seg.data, (0x5A,))
        self.assertTrue(seg.closed)

    def test_stretch_limit_passthrough(self):
        sa = list(write_script(0x33, [0x5A]))
        sb = list(write_script(0x33, [0x5A]))
        bit_sda = sb[1].sda
        sb[2:2] = [DriveTick(L, bit_sda)] * 6
        sa[2:2] = [DriveTick(R, bit_sda)] * 6
        with self.assertRaises(StretchTimeoutError):
            run_dual_master({"A": sa, "B": sb}, stretch_limit=3)


class DualMasterDifferentialTest(unittest.TestCase):
    """与 tests/reference_dual.py 的独立真值表参考逐字段对照。"""

    def _check(self, sa, sb, tag):
        sa, sb = aligned(sa, sb)
        res = run_dual_master({"A": sa, "B": sb})
        ref = ref_run({"A": to_raw(sa), "B": to_raw(sb)})
        self.assertEqual(res.winner, ref.winner, tag)
        self.assertEqual(res.loser, ref.loser, tag)
        if ref.first_conflict is None:
            self.assertIsNone(res.first_conflict, tag)
        else:
            fc = res.first_conflict
            self.assertEqual(
                (fc.tick, fc.byte_index, fc.bit_index, fc.loser),
                (ref.first_conflict.tick, ref.first_conflict.byte_index,
                 ref.first_conflict.bit_index, ref.first_conflict.loser),
                tag,
            )
        self.assertEqual(len(res.stretches), len(ref.stretches), tag)
        got_bytes = [
            [blk.value for blk in seg.blocks]
            for tx in res.parse.transactions
            for seg in tx.segments
        ]
        self.assertEqual(got_bytes, ref.bytes_per_tx, tag)
        got_closed = [tx.closed for tx in res.parse.transactions]
        self.assertEqual(got_closed, ref.closed, tag)

    def test_address_phase(self):
        self._check(write_script(0x50, [0x11, 0x22]),
                    write_script(0x51, [0x11, 0x22]), "addr")

    def test_data_phase(self):
        self._check(write_script(0x34, [0x00]),
                    write_script(0x34, [0x80]), "data")

    def test_identical(self):
        self._check(write_script(0x50, [0x11, 0x22]),
                    write_script(0x50, [0x11, 0x22]), "identical")

    def test_ack_low_by_peer(self):
        sa = write_script(0x50, [0x11])
        sb = list(write_script(0x50, [0x11]))
        sb[17] = DriveTick(L, L)
        sb[18] = DriveTick(R, L)
        self._check(sa, sb, "ack")

    def test_stretch(self):
        sa = list(write_script(0x33, [0x5A]))
        sb = list(write_script(0x33, [0x5A]))
        bit_sda = sb[1].sda
        sb[2:2] = [DriveTick(L, bit_sda)] * 3
        sa[2:2] = [DriveTick(R, bit_sda)] * 3
        self._check(sa, sb, "stretch")

    def test_multiple_data_bytes_diverge_late(self):
        self._check(write_script(0x10, [0xAA, 0x55, 0x00]),
                    write_script(0x10, [0xAA, 0x55, 0x01]), "late")


class DualMasterValidationTest(unittest.TestCase):
    def test_requires_exactly_two_masters(self):
        s = write_script(0x50)
        with self.assertRaises(InvalidUpdateError):
            run_dual_master({"A": s})
        with self.assertRaises(InvalidUpdateError):
            run_dual_master({"A": s, "B": s, "C": s})

    def test_equal_length_required(self):
        with self.assertRaises(InvalidUpdateError):
            run_dual_master({"A": write_script(0x50, [0x11]),
                             "B": write_script(0x50)})

    def test_empty_rejected(self):
        with self.assertRaises(InvalidUpdateError):
            run_dual_master({"A": [], "B": []})

    def test_invalid_drive_values(self):
        s = write_script(0x50)
        bad = list(s)
        bad[0] = DriveTick(R, True)  # bool 不是合法 Drive
        with self.assertRaises(InvalidUpdateError):
            run_dual_master({"A": bad, "B": s})
        bad2 = list(s)
        bad2[0] = (R, 1)  # 裸 int 不是 Drive
        with self.assertRaises(InvalidUpdateError):
            run_dual_master({"A": bad2, "B": s})
        bad3 = list(s)
        bad3[0] = "LOW"
        with self.assertRaises(InvalidUpdateError):
            run_dual_master({"A": bad3, "B": s})

    def test_bad_master_name(self):
        s = write_script(0x50)
        with self.assertRaises(InvalidUpdateError):
            run_dual_master({"": s, "B": s})
        with self.assertRaises(InvalidUpdateError):
            run_dual_master({123: s, "B": s})

    def test_non_mapping_rejected(self):
        s = write_script(0x50)
        with self.assertRaises(InvalidUpdateError):
            run_dual_master([s, s])

    def test_tuple_form_accepted(self):
        s = write_script(0x50)
        as_tuples = [(t.scl, t.sda) for t in s]
        res = run_dual_master({"A": as_tuples, "B": as_tuples})
        self.assertIsNone(res.loser)
        self.assertTrue(res.parse.transactions[0].closed)

    def test_pad_script(self):
        s = write_script(0x50)
        padded = pad_script(s, len(s) + 5)
        self.assertEqual(len(padded), len(s) + 5)
        with self.assertRaises(InvalidUpdateError):
            pad_script(s, len(s) - 1)


if __name__ == "__main__":
    unittest.main()

