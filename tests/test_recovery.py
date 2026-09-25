"""总线恢复（驱动器模型）：有效脉冲计数、STOP 生成、bus_stuck、中止、位隔离。

时序约定（stuck_prefix 之后，恢复请求时刻为 T）：
- T   ：主机释放 SDA（恢复动作 1）；
- T+1 ：脉冲 1 拉低 SCL；T+2：释放 SCL，真正观察到升高 => 脉冲 1 计数；
- 脉冲 j：拉低于 T+2j-1，上升计数于 T+2j；
- 从机在第 k 个脉冲的拉低期（T+2k-1，与主机动作同刻）释放 SDA；
- 采到 SDA=1 后：T+2k+1 SCL低+SDA低，T+2k+2 放 SCL，T+2k+3 放 SDA => STOP。
"""

from __future__ import annotations

import unittest

from odbus import (
    BitPhase,
    Drive,
    RecoveryOutcome,
    RecoveryRequest,
    Update,
    parse_updates,
)
from odbus.errors import (
    BusStuckError,
    InvalidUpdateError,
    LimitError,
    StretchTimeoutError,
)

R = Drive.RELEASE
D = Drive.LOW


def stuck_prefix(n_bits: int) -> tuple[list[Update], int]:
    """M 发 START + n_bits 个地址位（全 1），随后 S 把 SDA 卡低、M 停在 SCL 低。

    返回 (updates, 恢复请求时刻 T)。卡死后 SCL 低、SDA 低，主机不再出手。
    """
    ups = [Update(0, "M", R, D)]  # t0 START
    t = 1
    for _ in range(n_bits):
        ups.append(Update(t, "M", D, R))
        t += 1  # SCL 降
        ups.append(Update(t, "M", R, R))
        t += 1  # SCL 升，采样 1
    ups.append(Update(t, "M", D, R))
    ups.append(Update(t, "S", R, D))  # S 卡死 SDA；M SCL 停在低
    t += 1
    return ups, t


def release_case(n_bits: int, k: int) -> tuple[list[Update], int]:
    """S 在第 k 个恢复脉冲的拉低期（与主机动作同刻）释放 SDA。"""
    ups, t = stuck_prefix(n_bits)
    ups.append(Update(t + 2 * k - 1, "S", R, R))
    return ups, t


class ReleaseAfterPulseTests(unittest.TestCase):
    """第 1、8、9 个有效脉冲后 SDA 释放 => 生成合法 STOP 结束。"""

    def test_release_after_pulse_1_8_9(self):
        for k in (1, 8, 9):
            with self.subTest(pulses=k):
                ups, t = release_case(3, k)
                result = parse_updates(
                    ups, stretch_limit=10, recovery=RecoveryRequest("M", t)
                )
                self.assertEqual(len(result.recoveries), 1)
                rec = result.recoveries[0]
                self.assertIs(rec.outcome, RecoveryOutcome.STOPPED)
                self.assertEqual(rec.stop_tick, t + 2 * k + 3)
                # 恰好 k 个有效脉冲；第 k 个采到 SDA=1，之前都是 0
                self.assertEqual(len(rec.pulses), k)
                for j, p in enumerate(rec.pulses):
                    self.assertEqual(p.index, j + 1)
                    self.assertEqual(p.fall_tick, t + 2 * (j + 1) - 1)
                    self.assertEqual(p.rise_tick, t + 2 * (j + 1))
                    self.assertEqual(p.sda, 1 if j == k - 1 else 0)
                    self.assertFalse(p.stretched)
                # 恢复位单独标记：k 个脉冲采样 + STOP 生成的 1 个上升沿
                self.assertEqual(len(rec.bits), k + 1)
                self.assertTrue(
                    all(b.phase is BitPhase.RECOVERY for b in rec.bits)
                )
                self.assertEqual([b.value for b in rec.bits], [0] * (k - 1) + [1, 0])
                # 恢复生成的 STOP 正常闭合事务；残余位不拼字节、不残留
                self.assertEqual(len(result.transactions), 1)
                tx = result.transactions[0]
                self.assertTrue(tx.closed)
                seg = tx.segments[0]
                self.assertIsNotNone(seg.stop)
                self.assertEqual(seg.stop.tick, t + 2 * k + 3)
                self.assertEqual(seg.blocks, [])
                self.assertEqual(seg.partial_bits, [])
                self.assertEqual((result.final_scl, result.final_sda), (1, 1))

    def test_host_updates_allowed_again_after_recovery_done(self):
        ups, t = release_case(3, 1)
        ups.append(Update(t + 6, "M", R, R))  # STOP(t+5) 之后主机恢复输入
        result = parse_updates(ups, recovery=RecoveryRequest("M", t))
        self.assertIs(result.recoveries[0].outcome, RecoveryOutcome.STOPPED)


class StuckSclTests(unittest.TestCase):
    """SCL 被他人无限拉低：沿用 stretch_timeout；脉冲绝不凭计划时间计数。"""

    def _case(self):
        ups, t = stuck_prefix(3)
        # T+2 是脉冲 1 计划观察升高的时刻：T 同刻把 SCL 拉住不放
        ups.append(Update(t + 2, "T", D, R))
        for dt in range(3, 12):  # S 周期性重申卡死状态，让事件流继续前进
            ups.append(Update(t + dt, "S", R, D))
        return ups, t

    def test_infinite_scl_low_raises_stretch_timeout(self):
        ups, t = self._case()
        with self.assertRaises(StretchTimeoutError) as cm:
            parse_updates(ups, stretch_limit=5, recovery=RecoveryRequest("M", t))
        e = cm.exception
        self.assertEqual(e.low_since, t - 1)  # SCL 自卡死拍起一直为低
        self.assertEqual(e.duration, 6)
        self.assertEqual(e.limit, 5)

    def test_no_limit_stream_ends_incomplete_and_no_pulse_counted(self):
        ups, t = self._case()
        result = parse_updates(ups, recovery=RecoveryRequest("M", t))
        rec = result.recoveries[0]
        # 计划的上升沿从未真正出现：0 个有效脉冲、0 个恢复位
        self.assertIs(rec.outcome, RecoveryOutcome.INCOMPLETE)
        self.assertEqual(rec.pulses, [])
        self.assertEqual(rec.bits, [])
        # 之前没收完的 3 个位原样保留，未被恢复污染
        seg = result.transactions[0].segments[0]
        self.assertEqual([b.value for b in seg.partial_bits], [1, 1, 1])


class NeverReleaseSdaTests(unittest.TestCase):
    """始终不放 SDA：打满 9 个有效脉冲后报 bus_stuck。"""

    def test_nine_pulses_then_bus_stuck(self):
        ups, t = stuck_prefix(3)
        with self.assertRaises(BusStuckError) as cm:
            parse_updates(ups, recovery=RecoveryRequest("M", t))
        e = cm.exception
        self.assertEqual(e.host, "M")
        self.assertEqual(e.pulses, 9)
        self.assertEqual(e.max_pulses, 9)
        self.assertEqual(e.tick, t + 18)  # 第 9 个脉冲的上升沿
        self.assertEqual(e.sda_low_drivers, ("S",))
        # 报错记录里 9 个脉冲全部有效、采到的 9 位全是 0
        self.assertEqual(len(e.record.pulses), 9)
        self.assertEqual([b.value for b in e.record.bits], [0] * 9)

    def test_max_pulses_boundary_three(self):
        # max_pulses=3：第 3 个脉冲后仍低 => 第 3 个上升沿即报错
        ups, t = stuck_prefix(3)
        with self.assertRaises(BusStuckError) as cm:
            parse_updates(ups, recovery=RecoveryRequest("M", t, max_pulses=3))
        self.assertEqual(cm.exception.pulses, 3)
        self.assertEqual(cm.exception.tick, t + 6)


class AbortTests(unittest.TestCase):
    """其他主机发 START：恢复中止，正常识别继续，不抢占其事务。"""

    def test_other_master_start_aborts_recovery(self):
        ups = [Update(2, "B", R, D)]  # t=2：B 发起 START（与恢复请求同刻）
        t = 3
        addr = 0x50 << 1 | 0  # 0xA0 写
        for k in range(8):
            bit = (addr >> (7 - k)) & 1
            ups.append(Update(t, "B", D, R if bit else D))
            t += 1
            ups.append(Update(t, "B", R, R if bit else D))
            t += 1
        ups.append(Update(t, "B", D, R))  # ACK 位：无从机 => NACK=1
        t += 1
        ups.append(Update(t, "B", R, R))
        t += 1
        ups.append(Update(t, "B", D, D))  # STOP 预备：SCL 低时拉低 SDA
        t += 1
        ups.append(Update(t, "B", R, D))
        t += 1
        ups.append(Update(t, "B", R, R))  # STOP

        result = parse_updates(ups, recovery=RecoveryRequest("M", 2))
        rec = result.recoveries[0]
        self.assertIs(rec.outcome, RecoveryOutcome.ABORTED)
        self.assertEqual(rec.abort_tick, 2)
        self.assertEqual(rec.aborted_by, ("B",))
        self.assertEqual(rec.pulses, [])
        self.assertEqual(rec.bits, [])
        # B 的事务被正常完整识别（恢复没有抢占）
        self.assertEqual(len(result.transactions), 1)
        tx = result.transactions[0]
        self.assertTrue(tx.closed)
        seg = tx.segments[0]
        self.assertEqual((seg.address, seg.rw), (0x50, 0))
        self.assertEqual(seg.acks, (1,))


class RecoveryBitSeparationTests(unittest.TestCase):
    """恢复期间采到的位单独标记，不拼进之前没收完的数据字节。"""

    def test_partial_byte_not_completed_by_recovery_bits(self):
        # 7 个残余位 + 2 个恢复脉冲：若被拼接将凑满 9 拍成字节
        ups, t = stuck_prefix(7)
        ups.append(Update(t + 6, "T", D, R))  # 脉冲 3 计划上升处：T 拉住 SCL
        for dt in range(7, 10):
            ups.append(Update(t + dt, "S", R, D))
        result = parse_updates(ups, recovery=RecoveryRequest("M", t))
        rec = result.recoveries[0]
        self.assertIs(rec.outcome, RecoveryOutcome.INCOMPLETE)
        self.assertEqual(len(rec.pulses), 2)
        self.assertEqual(len(rec.bits), 2)
        self.assertTrue(all(b.phase is BitPhase.RECOVERY for b in rec.bits))
        seg = result.transactions[0].segments[0]
        self.assertEqual(seg.blocks, [])  # 7+2 没有拼成任何字节
        self.assertEqual(len(seg.partial_bits), 7)
        self.assertTrue(all(b.phase is BitPhase.ADDRESS for b in seg.partial_bits))


class MergeTraceTests(unittest.TestCase):
    """逐事件列出实际线值，检查所有翻转来自驱动器合并（同刻多驱动更新）。"""

    def test_every_line_value_is_the_and_merge_of_drivers(self):
        ups, t = release_case(3, 1)
        result = parse_updates(ups, recovery=RecoveryRequest("M", t), trace=True)
        trace = result.trace
        self.assertTrue(trace)
        # 逐事件：线值 == 所有驱动器的 AND（有拉低者即低）
        listing = []
        for s in trace:
            self.assertEqual(s.scl, 0 if s.scl_low_drivers else 1)
            self.assertEqual(s.sda, 0 if s.sda_low_drivers else 1)
            listing.append(
                f"t={s.tick:<3} SCL={s.scl} SDA={s.sda} "
                f"SCL低={s.scl_low_drivers or '()'} "
                f"SDA低={s.sda_low_drivers or '()'}"
            )
        # 所有翻转都对应驱动集合的变化（没有凭空跳变）
        for a, b in zip(trace, trace[1:]):
            if (a.scl, a.sda) != (b.scl, b.sda):
                self.assertNotEqual(
                    (a.scl_low_drivers, a.sda_low_drivers),
                    (b.scl_low_drivers, b.sda_low_drivers),
                    msg=f"t={b.tick} 线值翻转但驱动集合未变",
                )
        self.assertEqual(len(listing), len(trace))

    def test_same_tick_host_action_and_slave_release_merge(self):
        # t+1：主机脉冲拉低 SCL 与从机释放 SDA 同刻 => 合并后 SCL=0、SDA=1
        ups, t = release_case(3, 1)
        result = parse_updates(ups, recovery=RecoveryRequest("M", t), trace=True)
        by_tick = {s.tick: s for s in result.trace}
        merged = by_tick[t + 1]
        self.assertEqual((merged.scl, merged.sda), (0, 1))
        self.assertEqual(merged.scl_low_drivers, ("M",))
        self.assertEqual(merged.sda_low_drivers, ())
        # 下一拍 SCL 真正升高 => 脉冲 1 计数且采到 SDA=1
        rise = by_tick[t + 2]
        self.assertEqual((rise.scl, rise.sda), (1, 1))
        rec = result.recoveries[0]
        self.assertEqual(len(rec.pulses), 1)
        self.assertEqual(rec.pulses[0].sda, 1)

    def test_stretched_pulse_records_wait(self):
        # SCL 被 T 拉住 2 tick 后放行：脉冲只在真正升高时计数，且标记 stretched
        ups, t = stuck_prefix(3)
        ups.append(Update(t + 2, "T", D, R))   # 计划上升处拉住 SCL
        ups.append(Update(t + 3, "T", D, R))
        ups.append(Update(t + 4, "T", R, R))   # 放行 => 此刻才真正升高
        ups.append(Update(t + 5, "S", R, R))   # S 释放 SDA（脉冲 2 拉低期同刻）
        result = parse_updates(
            ups, stretch_limit=10, recovery=RecoveryRequest("M", t)
        )
        rec = result.recoveries[0]
        self.assertIs(rec.outcome, RecoveryOutcome.STOPPED)
        self.assertEqual(len(rec.pulses), 2)
        p1 = rec.pulses[0]
        self.assertEqual(p1.rise_tick, t + 4)  # 不是计划的 t+2
        self.assertTrue(p1.stretched)
        self.assertFalse(rec.pulses[1].stretched)


class RecoveryValidationTests(unittest.TestCase):
    def test_reject_non_request_recovery(self):
        with self.assertRaises(InvalidUpdateError) as cm:
            parse_updates([], recovery="M")
        self.assertEqual(cm.exception.field, "recovery")

    def test_reject_bool_and_bad_max_pulses(self):
        with self.assertRaises(InvalidUpdateError) as cm:
            parse_updates([], recovery=RecoveryRequest("M", 0, max_pulses=True))
        self.assertEqual(cm.exception.field, "max_pulses")
        for bad in (0, -1, 65):
            with self.subTest(bad=bad):
                with self.assertRaises(LimitError):
                    parse_updates([], recovery=RecoveryRequest("M", 0, max_pulses=bad))

    def test_reject_bad_host_and_tick(self):
        with self.assertRaises(InvalidUpdateError):
            parse_updates([], recovery=RecoveryRequest("", 0))
        with self.assertRaises(InvalidUpdateError):
            parse_updates([], recovery=RecoveryRequest("M", True))

    def test_host_update_during_recovery_rejected(self):
        ups, t = stuck_prefix(3)
        ups.append(Update(t + 1, "M", R, R))  # 恢复期间主机输入 => 拒绝
        with self.assertRaises(InvalidUpdateError) as cm:
            parse_updates(ups, recovery=RecoveryRequest("M", t))
        self.assertEqual(cm.exception.field, "driver")
        self.assertEqual(cm.exception.driver, "M")

    def test_host_update_before_request_is_fine(self):
        ups, t = release_case(3, 1)  # stuck_prefix 里主机在请求前的更新必须可用
        result = parse_updates(ups, recovery=RecoveryRequest("M", t))
        self.assertIn("M", result.drivers)


if __name__ == "__main__":
    unittest.main()
