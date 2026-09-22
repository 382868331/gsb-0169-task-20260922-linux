"""手工位序列参考测试：期望值直接硬编码，不来自被测核心也不来自生成器。"""

from __future__ import annotations

import unittest

from odbus import Drive, Line, Update, parse_updates
from odbus.model import BitPhase

from reference import H, L

R = Drive.RELEASE
D = Drive.LOW


def _lvl(x):
    """int(1/0) 与 Drive 枚举都接受，统一为 Drive。"""
    return x if isinstance(x, Drive) else (R if x == H else D)


def build(raw: list[tuple[int, str, int, int]]):
    return [
        Update(tick=t, driver=name, scl=_lvl(scl), sda=_lvl(sda))
        for (t, name, scl, sda) in raw
    ]


def write_byte(raw, t0, value, ack, prefix="", release_after=True):
    """在 t0 起生成一个“主机发 8 位 + 从机 ACK”的 9 脉冲序列，返回结束时刻。

    每个脉冲占 2 tick：下降沿(数据) -> 上升沿(采样)。
    ACK 位由从机 V 拉低/释放；release_after 时在结束 tick 让 V 释放 SDA，
    避免从机在 ACK 后继续拉住主机的数据位。
    """
    t = t0
    for k in range(8):
        bit = (value >> (7 - k)) & 1
        raw.append((t, "M", L, R if bit else D))
        raw.append((t + 1, "M", H, R if bit else D))
        t += 2
    raw.append((t, "M", L, R))           # ACK 周期主机释放 SDA
    raw.append((t, "V", H, D if ack == 0 else R))
    raw.append((t + 1, "M", H, R))
    raw.append((t + 1, "V", H, D if ack == 0 else R))
    end = t + 2
    if release_after:
        raw.append((end, "V", H, R))
    return end


class HandBuiltReferenceTests(unittest.TestCase):
    def _make_write_transaction(self, addr7, data_bytes, acks=()):
        raw = []
        # START：SCL 高，SDA 高->低（tick 0 第一条更新）
        raw.append((0, "M", H, L))
        t = 1
        addr_byte = (addr7 << 1) | 0
        total = 1 + len(data_bytes)
        t = write_byte(raw, t, addr_byte, acks[0] if acks else 0,
                       release_after=(total != 1))
        for i, val in enumerate(data_bytes):
            ack = acks[i + 1] if len(acks) > i + 1 else 0
            last = i == len(data_bytes) - 1
            # 非末字节 ACK 后 V 在结束 tick 释放 SDA（与下一拍 SCL 下降同刻合并）
            t = write_byte(raw, t, val, ack, release_after=not last)
        # STOP 收尾
        last_ack = acks[-1] if acks else 0
        if last_ack == 0:
            # SDA 低：M 与 V 同刻完成接管/释放（AND 保持低），下一拍 M 释放 => STOP
            raw.append((t, "M", H, L))
            raw.append((t, "V", H, R))
            raw.append((t + 1, "M", H, R))
        else:
            # SDA 已高：补一个脉冲拉低 SDA，再释放 => STOP
            raw.append((t, "M", L, L))
            raw.append((t + 1, "M", H, L))
            raw.append((t + 2, "M", H, R))
        return build(raw)

    def test_single_write_address_and_data(self):
        # 地址 0x50（写），数据 0xA5，两个 ACK=0
        updates = self._make_write_transaction(0x50, [0xA5], (0, 0))
        result = parse_updates(updates)
        self.assertEqual(len(result.transactions), 1)
        tx = result.transactions[0]
        self.assertTrue(tx.closed)
        self.assertEqual(len(tx.segments), 1)
        seg = tx.segments[0]
        self.assertEqual(seg.address, 0x50)
        self.assertEqual(seg.rw, 0)
        self.assertEqual(seg.data, (0xA5,))
        self.assertEqual(seg.acks, (0, 0))
        self.assertEqual(seg.master, "M")
        self.assertFalse(seg.start.repeated)
        self.assertIsNotNone(seg.stop)

    def test_bit_values_and_phases_hand_checked(self):
        updates = self._make_write_transaction(0x50, [], (0,))
        seg = parse_updates(updates).transactions[0].segments[0]
        block = seg.blocks[0]
        # 0x50<<1 = 0xA0 = 10100000
        self.assertEqual([b.value for b in block.bits],
                         [1, 0, 1, 0, 0, 0, 0, 0])
        self.assertEqual(block.ack.value, 0)
        phases = [b.phase for b in block.bits]
        self.assertEqual(phases[:7], [BitPhase.ADDRESS] * 7)
        self.assertEqual(phases[7], BitPhase.RW)
        self.assertEqual(block.ack.phase, BitPhase.ACK_ADDR)
        # 原始位来源：地址字节所有 0 位由 M 拉低 SDA；ACK 位由 V 拉低
        zero_bits = [b for b in block.bits if b.value == 0]
        self.assertTrue(zero_bits)
        for b in zero_bits:
            self.assertEqual(b.source.sda_low_drivers, ("M",))
        self.assertEqual(block.ack.source.sda_low_drivers, ("V",))
        self.assertEqual(block.bits[0].source.scl_low_drivers, ("M",))

    def test_nack_recorded_as_one(self):
        updates = self._make_write_transaction(0x4B, [0x11], (0, 1))
        seg = parse_updates(updates).transactions[0].segments[0]
        self.assertEqual(seg.address, 0x4B)
        self.assertEqual(seg.data, (0x11,))
        self.assertEqual(seg.acks, (0, 1))

    def test_all_zero_and_all_one_address(self):
        for addr in (0x00, 0x7F):
            with self.subTest(addr=addr):
                updates = self._make_write_transaction(addr, [], (0,))
                seg = parse_updates(updates).transactions[0].segments[0]
                self.assertEqual(seg.address, addr)

    def test_empty_input(self):
        result = parse_updates([])
        self.assertEqual(result.transactions, ())
        self.assertEqual(result.drivers, ())
        self.assertEqual(result.final_scl, 1)
        self.assertEqual(result.final_sda, 1)

    def test_line_enum_values(self):
        # Line 枚举仅作公开 API 暴露，这里固定其字符串值
        self.assertEqual(Line.SCL.value, "SCL")
        self.assertEqual(Line.SDA.value, "SDA")
        self.assertEqual(Drive.LOW.line_level, 0)
        self.assertEqual(Drive.RELEASE.line_level, 1)


if __name__ == "__main__":
    unittest.main()
