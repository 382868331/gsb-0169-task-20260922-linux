from __future__ import annotations

import unittest

from odbus import Drive, Update, parse_updates
from tenbit_reference import (  # type: ignore[reportMissingImports]
    RefTenBlock,
    TenWave,
    ref_ten_simulate,
    ten_high_byte,
)

H = Drive.RELEASE
L = Drive.LOW


def to_updates(raw):
    return [
        Update(t, d, H if scl else L, H if sda else L)
        for t, d, scl, sda in raw
    ]


def build_legal_write():
    g = TenWave()
    g.start()
    g.master_byte(ten_high_byte(2, 0), 0)
    g.master_byte(0xA1, 0)
    g.master_byte(0x3C, 1)
    g.stop(1)
    return g.raw


def build_write_then_read():
    g = TenWave()
    g.start()
    g.master_byte(ten_high_byte(2, 0), 0)
    g.master_byte(0xA1, 0)
    g.repeated_start()
    g.master_byte(ten_high_byte(2, 1), 0)
    g.slave_byte(0x6D, 1)
    g.stop(1)
    return g.raw


def build_low_nack_then_wrong_read():
    g = TenWave()
    g.start()
    g.master_byte(ten_high_byte(2, 0), 0)
    g.master_byte(0xA1, 1)
    g.repeated_start()
    g.master_byte(ten_high_byte(2, 1), 1)
    g.slave_byte(0x6D, 1)
    g.stop(1)
    return g.raw


def build_incomplete_then_wrong_read():
    g = TenWave()
    g.start()
    g.master_byte(ten_high_byte(2, 0), 0)
    for _ in range(3):
        g.pulse("M", 1)
    g.repeated_start()
    g.master_byte(ten_high_byte(2, 1), 1)
    g.slave_byte(0x77, 1)
    g.stop(1)
    return g.raw


def build_wrong_high_after_valid_write():
    g = TenWave()
    g.start()
    g.master_byte(ten_high_byte(2, 0), 0)
    g.master_byte(0xA1, 0)
    g.repeated_start()
    g.master_byte(ten_high_byte(3, 1), 0)
    g.slave_byte(0x6D, 1)
    g.stop(1)
    return g.raw


def build_stop_then_read_must_not_reuse_low_byte():
    g = TenWave()
    g.start()
    g.master_byte(ten_high_byte(2, 0), 0)
    g.master_byte(0xA1, 0)
    g.stop(0)
    g.start()
    g.master_byte(ten_high_byte(2, 1), 0)
    g.slave_byte(0x6D, 1)
    g.stop(1)
    return g.raw


class TenBitAddressingTests(unittest.TestCase):
    def assert_matches_reference(self, raw):
        expected = ref_ten_simulate(raw)
        result = parse_updates(to_updates(raw))
        self.assertEqual(len(result.transactions), len(expected))
        for actual_tx, ref_tx in zip(result.transactions, expected):
            self.assertEqual(len(actual_tx.segments), len(ref_tx))
            for seg, rseg in zip(actual_tx.segments, ref_tx):
                self.assertEqual(seg.address_mode, rseg.mode)
                self.assertEqual(seg.ten_bit_high, rseg.high)
                self.assertEqual(seg.ten_bit_low, rseg.low)
                self.assertEqual(seg.ten_bit_address_complete, rseg.complete)
                self.assertEqual(seg.ten_bit_address_acknowledged, rseg.acknowledged)
                self.assertEqual(seg.address, rseg.address)
                self.assertEqual(seg.data, rseg.data)
                self.assertEqual(
                    [(b.value, b.ack.value) for b in seg.blocks],
                    [(b.value, b.ack) for b in rseg.blocks],
                )
                self.assertEqual(
                    [(b.value, b.ack.value) for b in seg.address_blocks],
                    [(b.value, b.ack) for b in rseg.address_blocks],
                )
                for actual_block, ref_block in zip(seg.blocks, rseg.blocks):
                    self.assertEqual(actual_block.ack.tick, ref_block.ack_tick)
                    expected_phase = {
                        "address_header": "address",
                        "address_low": "address_low",
                        "data": "data",
                    }[ref_block.role]
                    self.assertEqual(actual_block.phase.value, expected_phase)
                    if ref_block.ack == 0:
                        self.assertEqual(actual_block.ack.source.sda_low_drivers, ("V",))
                    else:
                        self.assertEqual(actual_block.ack.source.sda_low_drivers, ())
                for actual_block, ref_block in zip(seg.address_blocks, rseg.address_blocks):
                    self.assertIsInstance(ref_block, RefTenBlock)
                    self.assertEqual(actual_block.ack.tick, ref_block.ack_tick)

    def test_legal_ten_bit_write(self):
        raw = build_legal_write()
        self.assert_matches_reference(raw)
        seg = parse_updates(to_updates(raw)).transactions[0].segments[0]
        self.assertEqual(seg.address, 0x2A1)
        self.assertEqual(seg.data, (0x3C,))

    def test_write_phase_then_repeated_start_same_high_read(self):
        raw = build_write_then_read()
        self.assert_matches_reference(raw)
        write_seg, read_seg = parse_updates(to_updates(raw)).transactions[0].segments
        self.assertEqual(write_seg.address, 0x2A1)
        self.assertEqual(read_seg.address, 0x2A1)
        self.assertEqual(read_seg.data, (0x6D,))
        self.assertEqual([b.value for b in read_seg.address_blocks], [0xF4, 0xA1, 0xF5])
        self.assertEqual(
            [b.ack.source.sda_low_drivers for b in read_seg.address_blocks],
            [("V",), ("V",), ("V",)],
        )

    def test_low_address_nack_blocks_later_bytes_and_sr_read(self):
        raw = build_low_nack_then_wrong_read()
        self.assert_matches_reference(raw)
        write_seg, read_seg = parse_updates(to_updates(raw)).transactions[0].segments
        self.assertIsNone(write_seg.address)
        self.assertEqual(write_seg.data, ())
        self.assertFalse(read_seg.ten_bit_address_complete)
        self.assertIsNone(read_seg.address)
        self.assertEqual(read_seg.data, ())
        self.assertEqual([b.value for b in read_seg.blocks], [0xF5, 0x6D])

    def test_incomplete_write_address_then_wrong_read_header(self):
        raw = build_incomplete_then_wrong_read()
        self.assert_matches_reference(raw)
        write_seg, read_seg = parse_updates(to_updates(raw)).transactions[0].segments
        self.assertEqual(len(write_seg.partial_bits), 3)
        self.assertIsNone(write_seg.address)
        self.assertFalse(read_seg.ten_bit_address_complete)
        self.assertIsNone(read_seg.address)
        self.assertEqual(read_seg.data, ())

    def test_read_header_with_different_high_address_is_invalid(self):
        raw = build_wrong_high_after_valid_write()
        self.assert_matches_reference(raw)
        _write_seg, read_seg = parse_updates(to_updates(raw)).transactions[0].segments
        self.assertFalse(read_seg.ten_bit_address_complete)
        self.assertIsNone(read_seg.address)
        self.assertEqual(read_seg.data, ())

    def test_stop_clears_low_address_for_later_transaction(self):
        raw = build_stop_then_read_must_not_reuse_low_byte()
        self.assert_matches_reference(raw)
        result = parse_updates(to_updates(raw))
        self.assertEqual(len(result.transactions), 2)
        read_seg = result.transactions[1].segments[0]
        self.assertFalse(read_seg.ten_bit_address_complete)
        self.assertIsNone(read_seg.address)
        self.assertEqual(read_seg.data, ())


if __name__ == "__main__":
    unittest.main()
