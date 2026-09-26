"""十位寻址（题内形式）测试：逐拍生成波形，对照独立字节状态机。

题内形式：首字节 ``11110`` + 地址高两位 + 方向；写方向后再接地址低八位；
读取须先在同一事务内完成写地址阶段，再重复 START 发同高位读头。

期望值来自 tests/reference.py 的 :func:`ref_ten_bit`（不导入 odbus 的
独立字节状态机）；被测核心结果逐字段对照，并检查各地址字节的 ACK 来源位置。
"""

from __future__ import annotations

import unittest

from odbus import Drive, Update, parse_updates

from reference import Generator, H, ref_ten_bit


def to_updates(raw):
    return [
        Update(
            tick=t,
            driver=d,
            scl=Drive.RELEASE if scl == H else Drive.LOW,
            sda=Drive.RELEASE if sda == H else Drive.LOW,
        )
        for (t, d, scl, sda) in raw
    ]


def header(high: int, rw: int) -> int:
    """十位寻址首字节：11110 + 地址高两位 + 方向位。"""
    return 0xF0 | ((high & 0x03) << 1) | (rw & 1)


class TenBitWave:
    """逐拍生成十位寻址场景，同时记录字节流供独立状态机对照。"""

    def __init__(self) -> None:
        self.g = Generator()
        self.txs: list[list[list[tuple[int, int]]]] = []

    # --- 结构构件 ---

    def begin_tx(self) -> None:
        self.txs.append([])
        self.g.start()
        self.txs[-1].append([])

    def repeated_start(self) -> None:
        self.g.repeated_start()
        self.txs[-1].append([])

    def stop(self) -> None:
        last_ack = self.txs[-1][-1][-1][1]
        self.g.stop(last_ack)

    # --- 字节构件（记录 (值, ACK)）---

    def master_byte(self, value: int, ack: int) -> None:
        self.g.byte_master_sends(value, ack)
        self.txs[-1][-1].append((value, ack))

    def slave_byte(self, value: int, master_ack: int) -> None:
        self.g.byte_slave_sends(value, master_ack)
        self.txs[-1][-1].append((value, master_ack))

    # --- 解析并对照独立状态机 ---

    def check(self, test: unittest.TestCase) -> None:
        result = parse_updates(to_updates(self.g.raw))
        expected = ref_ten_bit(self.txs)
        test.assertEqual(len(result.transactions), len(expected), "事务数不一致")
        for ti, (tx, exp_tx) in enumerate(zip(result.transactions, expected)):
            test.assertEqual(len(tx.segments), len(exp_tx), f"tx={ti} 段数不一致")
            for si, (seg, exp) in enumerate(zip(tx.segments, exp_tx)):
                where = f"tx={ti} seg={si}"
                if exp.kind == "seven":
                    test.assertEqual(seg.addressing, "seven", where)
                    test.assertIsNone(seg.ten_bit, where)
                elif exp.kind == "ten_write":
                    test.assertEqual(seg.addressing, "ten", where)
                    test.assertIsNotNone(seg.ten_bit, where)
                    test.assertFalse(seg.ten_bit.read_header, where)
                    test.assertEqual(seg.ten_bit.complete, exp.complete, where)
                else:
                    test.assertEqual(seg.addressing, "ten", where)
                    test.assertIsNotNone(seg.ten_bit, where)
                    test.assertTrue(seg.ten_bit.read_header, where)
                    test.assertEqual(seg.ten_bit.matched, exp.matched, where)
                    test.assertEqual(seg.ten_bit.complete, exp.complete, where)
                test.assertEqual(seg.address, exp.address, f"{where} 地址")
                test.assertEqual(seg.data, exp.data, f"{where} 数据")
                test.assertEqual(seg.acks, exp.acks, f"{where} ACK")
                # 各地址字节的 ACK 值与 ACK 来源位置
                addr_blocks = seg.address_blocks
                test.assertEqual(
                    tuple(b.ack.value for b in addr_blocks), exp.addr_acks,
                    f"{where} 地址字节 ACK",
                )
                for bi, block in enumerate(addr_blocks):
                    src = block.ack.source
                    test.assertEqual(src.scl_high_tick, block.ack.tick,
                                     f"{where} 地址块{bi} ACK 采样时刻")
                    # 本生成器中 ACK=0 由从机 V 拉低，NACK=1 时无人拉低
                    if block.ack.value == 0:
                        test.assertEqual(src.sda_low_drivers, ("V",),
                                         f"{where} 地址块{bi} ACK 来源")
                    else:
                        test.assertEqual(src.sda_low_drivers, (),
                                         f"{where} 地址块{bi} NACK 来源")


class TenBitTests(unittest.TestCase):
    def test_legal_write(self):
        """合法十位写：写头 + 低八位均被 ACK，随后字节是数据。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte(header(0b10, 0), 0)   # 写头，高两位=10
        w.master_byte(0xAB, 0)              # 低八位
        w.master_byte(0x11, 0)              # 数据
        w.master_byte(0x22, 1)              # 数据（末字节 NACK）
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        seg = result.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x2AB)
        self.assertEqual(seg.data, (0x11, 0x22))
        self.assertEqual(len(seg.address_blocks), 2)
        self.assertEqual(seg.ten_bit.high, 0b10)
        self.assertEqual(seg.ten_bit.low, 0xAB)
        self.assertTrue(seg.ten_bit.complete)

    def test_write_then_read(self):
        """写地址阶段完成后，重复 START 发同高位读头 => 匹配成功。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte(header(0b01, 0), 0)
        w.master_byte(0x34, 0)
        w.repeated_start()
        w.master_byte(header(0b01, 1), 0)   # 读头，同一高位
        w.slave_byte(0x9E, 1)               # 从机数据，主机 NACK
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        tx = result.transactions[0]
        self.assertEqual(len(tx.segments), 2)
        read_seg = tx.segments[1]
        self.assertTrue(read_seg.start.repeated)
        self.assertEqual(read_seg.address, 0x134)
        self.assertEqual(read_seg.data, (0x9E,))
        self.assertTrue(read_seg.ten_bit.matched)
        self.assertEqual(len(read_seg.address_blocks), 1)

    def test_low_address_nack(self):
        """低八位被 NACK：地址未收全，后续字节不得当作数据。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte(header(0b11, 0), 0)
        w.master_byte(0x56, 1)              # 低八位被 NACK
        w.master_byte(0x77, 1)              # 总线上仍有字节，但不是数据
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        seg = result.transactions[0].segments[0]
        self.assertIsNone(seg.address)
        self.assertEqual(seg.data, ())
        self.assertFalse(seg.ten_bit.complete)
        self.assertEqual(len(seg.blocks), 3)  # 字节仍如实记录

    def test_header_nack(self):
        """写头被 NACK：低八位即使被 ACK 也不算收全。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte(header(0b00, 0), 1)   # 写头 NACK
        w.master_byte(0x01, 0)
        w.master_byte(0x99, 0)
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        seg = result.transactions[0].segments[0]
        self.assertIsNone(seg.address)
        self.assertEqual(seg.data, ())

    def test_wrong_read_header_without_write_phase(self):
        """错误读头：事务内没有已完成的写地址阶段 => 不匹配。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte(header(0b01, 1), 0)   # 直接发读头
        w.slave_byte(0x42, 1)
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        seg = result.transactions[0].segments[0]
        self.assertIsNone(seg.address)
        self.assertFalse(seg.ten_bit.matched)
        self.assertEqual(seg.data, ())

    def test_wrong_read_header_high_mismatch(self):
        """错误读头：写地址阶段的高位与读头高位不同 => 不匹配。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte(header(0b01, 0), 0)
        w.master_byte(0x34, 0)
        w.repeated_start()
        w.master_byte(header(0b10, 1), 0)   # 高位不同
        w.slave_byte(0x42, 1)
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        read_seg = result.transactions[0].segments[1]
        self.assertIsNone(read_seg.address)
        self.assertFalse(read_seg.ten_bit.matched)
        self.assertEqual(read_seg.data, ())

    def test_incomplete_state_not_carried_over_stop(self):
        """STOP 后不得借用上次事务残留：新事务写头后缺低八位 => 未收全。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte(header(0b01, 0), 0)
        w.master_byte(0x34, 0)
        w.stop()
        # 第二个事务：写头被 ACK 后直接 STOP，低八位缺失
        w.begin_tx()
        w.master_byte(header(0b01, 0), 0)
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        seg2 = result.transactions[1].segments[0]
        self.assertIsNone(seg2.address)
        self.assertFalse(seg2.ten_bit.complete)
        self.assertEqual(seg2.data, ())

    def test_read_header_not_matched_across_transactions(self):
        """上一事务完成的写地址阶段不能被下一事务的读头借用。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte(header(0b01, 0), 0)
        w.master_byte(0x34, 0)
        w.stop()
        w.begin_tx()
        w.master_byte(header(0b01, 1), 0)   # 新事务直接发读头
        w.slave_byte(0x42, 1)
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        read_seg = result.transactions[1].segments[0]
        self.assertIsNone(read_seg.address)
        self.assertFalse(read_seg.ten_bit.matched)

    def test_incomplete_write_phase_then_read_header(self):
        """写地址阶段未完成（缺低八位）就重复 START 发读头 => 不匹配。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte(header(0b01, 0), 0)
        w.repeated_start()                  # 低八位未发就 Sr
        w.master_byte(header(0b01, 1), 0)
        w.slave_byte(0x42, 1)
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        tx = result.transactions[0]
        self.assertIsNone(tx.segments[0].address)
        self.assertFalse(tx.segments[1].ten_bit.matched)
        self.assertEqual(tx.segments[1].data, ())

    def test_seven_bit_unaffected(self):
        """七位地址仍按旧逻辑解释。"""
        w = TenBitWave()
        w.begin_tx()
        w.master_byte((0x53 << 1) | 0, 0)
        w.master_byte(0x5A, 0)
        w.stop()
        w.check(self)
        result = parse_updates(to_updates(w.g.raw))
        seg = result.transactions[0].segments[0]
        self.assertEqual(seg.addressing, "seven")
        self.assertEqual(seg.address, 0x53)
        self.assertEqual(seg.data, (0x5A,))
        self.assertEqual(len(seg.address_blocks), 1)


if __name__ == "__main__":
    unittest.main()
