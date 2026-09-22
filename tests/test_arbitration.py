"""仲裁丢失：主机发送 1（释放 SDA）却采样到 0；读段从机发数不误报。"""

from __future__ import annotations

import unittest

from odbus import Drive, Update, parse_updates

from reference import H, L

R = Drive.RELEASE
D = Drive.LOW


def _lvl(x):
    return x if isinstance(x, Drive) else (R if x == H else D)


def U(tick, driver, scl, sda):
    return Update(tick=tick, driver=driver, scl=_lvl(scl), sda=_lvl(sda))


def pulse_both(raw, t, m_sda, v_sda):
    """两位驱动器同刻经历一个 SCL 脉冲；返回下一 tick。"""
    raw.append(U(t, "M", L, m_sda))
    raw.append(U(t, "V", H, v_sda))
    raw.append(U(t + 1, "M", H, m_sda))
    raw.append(U(t + 1, "V", H, v_sda))
    return t + 2


class ArbitrationTests(unittest.TestCase):
    def test_arbitration_loss_on_address_one_bit(self):
        # 主机发地址 0x51（1010001? 用 0x50<<1=0xA0 第二位为 0...）
        # 构造地址字节 0x80：首位主机发 1，但竞争者 V 同时拉低 => 丢失。
        raw = [U(0, "M", H, D), U(0, "V", H, H)]  # START（M 拉 SDA）
        t = 1
        t = pulse_both(raw, t, R, D)   # bit7: M=1, V=0 => 采样 0，仲裁丢失
        # 剩余 7 位 + ACK 随意补齐（主机已败，仍让 M 驱动，只验证事件记录）
        for k in range(7):
            t = pulse_both(raw, t, R, H)
        t = pulse_both(raw, t, R, D)   # ACK=0
        raw.append(U(t, "M", H, D)); raw.append(U(t, "V", H, R))
        raw.append(U(t + 1, "M", H, R))

        result = parse_updates(raw)
        seg = result.transactions[0].segments[0]
        losses = seg.arbitration_losses
        self.assertEqual(len(losses), 1)
        loss = losses[0]
        self.assertEqual(loss.bit_index, 0)
        self.assertEqual(loss.master, "M")
        self.assertEqual(loss.observed, 0)
        self.assertEqual(loss.driven_low_by, ("V",))
        # 采样位本身仍按总线 AND 结果记录：0x80 的首位被裁成 0
        self.assertEqual(seg.blocks[0].bits[0].value, 0)
        # 原始位来源同时记录 M 已释放、V 拉低
        self.assertEqual(seg.blocks[0].bits[0].source.sda_low_drivers, ("V",))

    def test_arbitration_loss_in_write_data_phase(self):
        raw = [U(0, "M", H, D)]
        t = 1
        # 地址字节 0x00（写）全部 0，不触发
        for _ in range(8):
            t = pulse_both(raw, t, D, H)
        t = pulse_both(raw, t, R, D)  # ACK
        # 数据字节首位：M 发 1，V 拉低 => 丢失
        t = pulse_both(raw, t, R, D)
        for _ in range(7):
            t = pulse_both(raw, t, R, H)
        t = pulse_both(raw, t, R, D)
        raw.append(U(t, "M", H, D)); raw.append(U(t, "V", H, R))
        raw.append(U(t + 1, "M", H, R))

        seg = parse_updates(raw).transactions[0].segments[0]
        self.assertEqual(len(seg.arbitration_losses), 1)
        self.assertEqual(seg.arbitration_losses[0].bit_index, 0)

    def test_read_data_bit_zero_is_not_arbitration_loss(self):
        # 读事务：地址 0x50 读；数据 0x00 由从机 V 驱动全部 0，M 全程释放 SDA。
        raw = [U(0, "M", H, D)]
        t = 1
        addr = 0x50 << 1 | 1
        for k in range(8):
            bit = (addr >> (7 - k)) & 1
            t = pulse_both(raw, t, R if bit else D, H)
        t = pulse_both(raw, t, R, D)  # ACK 由 V
        # 数据 0x00：V 发 0，M 释放
        for _ in range(8):
            t = pulse_both(raw, t, R, D)
        t = pulse_both(raw, t, D, H)  # 主机 ACK
        raw.append(U(t, "M", H, D)); raw.append(U(t, "V", H, R))
        raw.append(U(t + 1, "M", H, R))

        seg = parse_updates(raw).transactions[0].segments[0]
        self.assertEqual(seg.rw, 1)
        self.assertEqual(seg.data, (0x00,))
        self.assertEqual(seg.arbitration_losses, [])

    def test_master_driving_zero_never_reported_as_loss(self):
        # 主机自己拉低时读到 0 是正常的，不算丢失。
        raw = [U(0, "M", H, D)]
        t = 1
        for _ in range(8):
            t = pulse_both(raw, t, D, H)
        t = pulse_both(raw, t, R, D)
        raw.append(U(t, "M", H, D)); raw.append(U(t, "V", H, R))
        raw.append(U(t + 1, "M", H, R))
        seg = parse_updates(raw).transactions[0].segments[0]
        self.assertEqual(seg.arbitration_losses, [])


if __name__ == "__main__":
    unittest.main()
