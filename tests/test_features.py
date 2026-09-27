"""题目点名的特性：重复 START、时钟拉伸、同刻双线变化、缺 STOP。"""

from __future__ import annotations

import unittest

from odbus import Drive, StreamingBusDecoder, parse_updates
from odbus.errors import StretchTimeoutError

from reference import H, L

R = Drive.RELEASE
D = Drive.LOW


def _lvl(x):
    return x if isinstance(x, Drive) else (R if x == H else D)


def U(tick, driver, scl, sda):
    from odbus import Update
    return Update(tick=tick, driver=driver, scl=_lvl(scl), sda=_lvl(sda))


def read_byte_wave(raw, t, value, master_ack, last=False):
    """主机读：8 位由 V 驱动 SDA，第 9 拍主机给 ACK/NACK。返回结束 tick。"""
    for k in range(8):
        bit = (value >> (7 - k)) & 1
        raw.append(U(t, "M", L, R))
        raw.append(U(t, "V", H, R if bit else D))
        raw.append(U(t + 1, "M", H, R))
        raw.append(U(t + 1, "V", H, R if bit else D))
        t += 2
    # ACK：V 释放，M 给值
    raw.append(U(t, "M", L, D if master_ack == 0 else R))
    raw.append(U(t, "V", H, R))
    raw.append(U(t + 1, "M", H, D if master_ack == 0 else R))
    raw.append(U(t + 1, "V", H, R))
    return t + 2


def addr_byte_wave(raw, t, value, ack_by_slave=0, stretch=0):
    """主机发地址字节；V 在 ACK 位拉低；stretch 为第一拍 SCL 低保持 tick 数。"""
    for k in range(8):
        bit = (value >> (7 - k)) & 1
        raw.append(U(t, "M", L, R if bit else D))
        for _ in range(stretch if k == 0 else 0):
            t += 1
            raw.append(U(t, "M", L, R if bit else D))
        raw.append(U(t + 1, "M", H, R if bit else D))
        t += 2
    raw.append(U(t, "M", L, R))
    raw.append(U(t, "V", H, D if ack_by_slave == 0 else R))
    raw.append(U(t + 1, "M", H, R))
    raw.append(U(t + 1, "V", H, D if ack_by_slave == 0 else R))
    return t + 2


class RepeatedStartTests(unittest.TestCase):
    def test_repeated_start_write_then_read_one_transaction(self):
        raw = []
        t = 0
        raw.append(U(t, "M", H, D))  # START
        t = 1
        t = addr_byte_wave(raw, t, 0x50 << 1 | 0, 0)   # 写 0x50
        # Sr 预备：ACK=0 总线低，V 释放/M 接管，SCL 低-高，SDA 低-高(SCL低时)
        raw.append(U(t, "M", H, D)); raw.append(U(t, "V", H, R))
        t += 1
        raw.append(U(t, "M", L, D))
        t += 1
        raw.append(U(t, "M", L, R))   # SCL 低时 SDA 升
        t += 1
        raw.append(U(t, "M", H, R))   # SCL 升（残余脉冲）
        t += 1
        raw.append(U(t, "M", H, D))   # SCL 高时 SDA 降 => Sr
        t += 1
        t = addr_byte_wave(raw, t, 0x50 << 1 | 1, 0)   # 读 0x50
        t = read_byte_wave(raw, t, 0x3C, master_ack=1) # NACK 后 STOP
        # NACK 后总线高：补脉冲拉低再 STOP
        raw.append(U(t, "M", L, D)); t += 1
        raw.append(U(t, "M", H, D)); t += 1
        raw.append(U(t, "M", H, R))

        result = parse_updates(raw)
        self.assertEqual(len(result.transactions), 1)
        tx = result.transactions[0]
        self.assertTrue(tx.closed)
        self.assertEqual(len(tx.segments), 2)
        s0, s1 = tx.segments
        self.assertFalse(s0.start.repeated)
        self.assertTrue(s0.ended_by_repeated_start)
        self.assertIsNone(s0.stop)
        self.assertEqual((s0.address, s0.rw), (0x50, 0))
        self.assertTrue(s1.start.repeated)
        self.assertEqual((s1.address, s1.rw), (0x50, 1))
        self.assertEqual(s1.data, (0x3C,))
        self.assertEqual(s1.acks, (0, 1))
        self.assertIsNotNone(s1.stop)
        # 第一段 ACK 后到 Sr 前的残余脉冲不产生半截字节
        self.assertEqual(len(s0.blocks), 1)
        self.assertEqual(s0.partial_bits, [])


class StretchTests(unittest.TestCase):
    def _addr_with_stretch(self, stretch):
        raw = [U(0, "M", H, D)]
        addr_byte_wave(raw, 1, 0x48 << 1, 0, stretch=stretch)
        return raw

    def test_stretch_at_threshold_is_ok(self):
        # 第一拍 SCL 低持续：下降 t1，保持到 t=1+stretch，上升 t=1+stretch+1
        # duration 在上升沿 = stretch+1；limit 设为 stretch+1 时不应报（严格大于才报）
        raw = self._addr_with_stretch(3)
        result = parse_updates(raw, stretch_limit=4)
        seg = result.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x48)

    def test_stretch_one_tick_over_raises(self):
        raw = self._addr_with_stretch(3)
        with self.assertRaises(StretchTimeoutError) as cm:
            parse_updates(raw, stretch_limit=3)
        self.assertEqual(cm.exception.limit, 3)
        self.assertEqual(cm.exception.duration, 4)
        self.assertEqual(cm.exception.low_since, 1)

    def test_no_limit_means_no_timeout_even_very_long(self):
        raw = self._addr_with_stretch(50)
        result = parse_updates(raw)  # 不抛
        self.assertEqual(result.transactions[0].segments[0].address, 0x48)


class SimultaneousChangeTests(unittest.TestCase):
    def test_two_drivers_change_both_lines_same_tick(self):
        # 同一时刻：M 把 SCL 拉低，V 把 SDA 拉低（各自一条更新，合并后判断）
        raw = [
            U(0, "M", H, R), U(0, "V", H, R),
            U(1, "M", L, R), U(1, "V", H, D),
            U(2, "M", H, R), U(2, "V", H, D),  # SCL 升，SDA 仍低 => 采样 0（START 前，忽略）
        ]
        result = parse_updates(raw)
        self.assertEqual(result.final_scl, 1)
        self.assertEqual(result.final_sda, 0)
        self.assertEqual(result.transactions, ())

    def test_start_and_scl_glitch_same_tick_is_not_start(self):
        # 同刻 SDA 降但 SCL 也降：SCL 不为高，不算 START
        raw = [U(0, "M", L, D)]
        result = parse_updates(raw)
        self.assertEqual(result.transactions, ())

    def test_same_tick_sda_low_wins_and_source_records_both(self):
        # START 后第一位：M 与 V 同刻都拉低 SDA（仲裁场景雏形）
        raw = [U(0, "M", H, D)]
        raw.append(U(1, "M", L, D))
        raw.append(U(1, "V", H, D))
        raw.append(U(2, "M", H, D))
        raw.append(U(2, "V", H, D))
        result = parse_updates(raw)
        seg = result.transactions[0].segments[0]
        bit = seg.partial_bits[0]
        self.assertEqual(bit.value, 0)
        self.assertEqual(tuple(bit.source.sda_low_drivers), ("M", "V"))


class MissingStopTests(unittest.TestCase):
    def test_open_transaction_at_end_of_stream(self):
        raw = [U(0, "M", H, D)]
        addr_byte_wave(raw, 1, 0x61 << 1 | 1, 0)
        # 无 STOP：流直接结束（ACK 后 SDA 为低、SCL 高）
        result = parse_updates(raw)
        self.assertEqual(len(result.transactions), 1)
        tx = result.transactions[0]
        self.assertFalse(tx.closed)
        seg = tx.segments[0]
        self.assertIsNone(seg.stop)
        self.assertEqual(seg.address, 0x61)
        self.assertEqual(seg.rw, 1)

    def test_truncated_mid_byte_keeps_partial_bits(self):
        raw = [U(0, "M", H, D)]
        # 只发 3 个数据位脉冲就结束
        t = 1
        for bit in (1, 0, 1):
            raw.append(U(t, "M", L, R if bit else D)); t += 1
            raw.append(U(t, "M", H, R if bit else D)); t += 1
        result = parse_updates(raw)
        seg = result.transactions[0].segments[0]
        self.assertEqual(seg.blocks, [])
        self.assertEqual([b.value for b in seg.partial_bits], [1, 0, 1])
        self.assertFalse(result.transactions[0].closed)


class BatchStreamingTests(unittest.TestCase):
    def _simple_write(self):
        raw = [U(0, "M", H, D)]
        t = addr_byte_wave(raw, 1, 0x52 << 1 | 0, 0)
        raw.append(U(t, "M", L, D))
        raw.append(U(t, "V", H, R)); t += 1
        raw.append(U(t, "M", H, D)); t += 1
        raw.append(U(t, "M", H, R))
        return raw

    def test_split_at_every_batch_boundary_matches_one_shot(self):
        raw = self._simple_write()
        expected = parse_updates(raw)

        for cut in range(1, len(raw) - 1):
            with self.subTest(cut=cut):
                decoder = StreamingBusDecoder()
                decoder.feed(raw[:cut])
                result = decoder.feed(raw[cut:]).finish()

                self.assertEqual(len(result.transactions), 1)
                self.assertTrue(result.transactions[0].closed)
                seg = result.transactions[0].segments[0]
                self.assertEqual(seg.address, 0x52)
                self.assertEqual(seg.rw, 0)
                self.assertEqual(seg.acks, expected.transactions[0].segments[0].acks)

    def test_same_tick_updates_split_across_batches_are_merged(self):
        first = [
            U(0, "M", H, D),       # START
            U(1, "M", L, R),       # 第一位脉冲开始；本批到此结束
        ]
        second = [
            U(1, "V", H, D),       # 同一时刻另一驱动器也拉低 SDA
            U(2, "M", H, R),
            U(2, "V", H, D),       # 采样到 0
        ]

        decoder = StreamingBusDecoder()
        decoder.feed(first)
        interim = decoder.result()
        self.assertEqual(interim.transactions[0].segments[0].partial_bits, [])
        decoder.feed(second)
        seg = decoder.finish().transactions[0].segments[0]

        self.assertEqual([b.value for b in seg.partial_bits], [0])
        self.assertEqual(bit_source_drivers(seg.partial_bits[0]), ("V",))

    def test_finish_open_transaction_does_not_synthesize_stop(self):
        raw = [U(0, "M", H, D)]
        addr_byte_wave(raw, 1, 0x61 << 1 | 1, 0)

        decoder = StreamingBusDecoder()
        for update in raw:
            decoder.feed([update])
        result = decoder.finish()

        tx = result.transactions[0]
        seg = tx.segments[0]
        self.assertFalse(tx.closed)
        self.assertIsNone(seg.stop)
        self.assertEqual((seg.address, seg.rw), (0x61, 1))

    def test_stop_update_held_until_end_is_recognized_after_finish(self):
        raw = self._simple_write()
        decoder = StreamingBusDecoder()
        decoder.feed(raw[:-1])
        self.assertFalse(decoder.result().transactions[0].closed)
        decoder.feed([raw[-1]])
        self.assertTrue(decoder.finish().transactions[0].closed)

    def test_failed_batch_rolls_back_to_state_before_batch(self):
        decoder = StreamingBusDecoder(stretch_limit=3)
        decoder.feed([
            U(0, "M", H, D),
            U(1, "M", L, R),
        ])

        with self.assertRaises(StretchTimeoutError):
            decoder.feed([
                U(5, "M", L, R),
                U(6, "M", L, R),
            ])

        decoder.feed([U(2, "M", H, R)])
        seg = decoder.finish().transactions[0].segments[0]
        self.assertEqual([bit.value for bit in seg.partial_bits], [1])


def bit_source_drivers(bit):
    return tuple(
        driver
        for driver in bit.source.drivers
        if driver in bit.source.sda_low_drivers
    )


if __name__ == "__main__":
    unittest.main()
