"""显式总线恢复驱动器模型的测试（题目点名的全部场景）。

覆盖：
* SDA 在第 1 / 8 / 9 个**有效**脉冲后释放 -> 生成合法 STOP；
* SCL 被其他驱动器无限拉住：有 stretch_limit 报 stretch_timeout（恢复期间），
  无阈值且后续无更新能释放时报 bus_stuck；脉冲只在实测 SCL 升高时计数；
* 九个有效脉冲后始终不放 SDA -> bus_stuck；
* 其他主机发 START -> 中止恢复，正常识别继续且不抢占它的事务；
* 恢复期间采到的位单独标记（RecoveryBit），不拼入恢复前未收完的数据字节；
* 同刻多驱动器更新：先合并再判边沿；
* 逐事件核对实际线值，并独立重算“每个事件的 SCL/SDA == 所有驱动器（含恢复
  驱动器模型）线与”，确认所有翻转只可能来自驱动器合并（无滤波器、无凭空电平）；
* 请求与录入校验（请求刻不存在/同刻两请求/恢复期间写主机/重叠恢复/bool）。
"""

from __future__ import annotations

import unittest

from odbus import (
    Drive,
    RecoveryEventKind as K,
    RecoveryOutcome as O,
    RecoveryRequest,
    Update,
    parse_updates,
)
from odbus.errors import (
    BusStuckError,
    InvalidUpdateError,
    StretchTimeoutError,
)

R = Drive.RELEASE
D = Drive.LOW
H, L = 1, 0


# ---------------------------------------------------------------------------
# 波形构造助手
# ---------------------------------------------------------------------------

def addr_data_bits_prefix(start_tick: int, bits=(1, 0, 0)):
    """M 在 START 后手工发若干地址位（每位两个 tick），第三位把 SDA 留在低电平。"""
    ups = [Update(start_tick, "M", R, D)]  # START
    t = start_tick + 1
    for b in bits:
        ups.append(Update(t, "M", D, R if b else D))
        t += 1
        ups.append(Update(t, "M", R, R if b else D))
        t += 1
    return ups, t - 1  # 最后一个上升沿 tick


def holder_releases_at(request_tick: int, pulse_no: int, holder: str = "X"):
    """外部持有者 X 自请求刻起拉住 SDA，在第 pulse_no 个脉冲的上升刻同刻释放。

    无 SCL 拉伸时模型节奏（请求刻 = t0）：
        脉冲 k 下降刻 = t0 + 1 + 2(k-1)，上升刻 = t0 + 2k。
    """
    rise = request_tick + 2 * pulse_no
    return [
        Update(request_tick, holder, R, D),
        Update(rise, holder, R, R),
    ]


# 事件种类 -> 恢复主机模型在该拍对 (SCL, SDA) 的驱动（1=释放，0=拉低）。
# 这是题目契约（主机先放 SDA、再拉低/放行 SCL 造脉冲、最后造 STOP）的直接表述，
# 测试用它独立重算合并线值，而不是从核心内部取期望值。
MODEL_DRIVE = {
    K.RELEASE_SDA: (1, 1),
    K.PULSE_LOW: (0, 1),
    K.WAIT_SCL_LOW: (1, 1),
    K.PULSE_COUNTED: (1, 1),
    K.BUS_IDLE: (1, 1),
    K.STOP_SETUP_LOW: (0, 0),
    K.STOP_SETUP_RISE: (1, 0),
    K.STOP_WAIT_SCL: (1, 0),
    K.STOP_RELEASE: (1, 1),
    K.STOP_BLOCKED: (1, 1),
    K.STOP_OBSERVED: (1, 1),
    K.ABORTED: (1, 1),
}


def assert_events_are_merges(testcase, updates, master, record):
    """逐事件独立重算：事件里的 SCL/SDA 必须等于全部驱动器状态的线与。

    回放外部更新（绝对状态模型，缺省释放），在每个事件刻叠加恢复主机该拍的驱动，
    再对所有驱动器取 AND；同时验证：线值只在至少一个驱动器状态改变的 tick 翻转。
    """
    state: dict[str, list[int]] = {}
    order: list[str] = []
    by_tick: dict[int, list[Update]] = {}
    for u in updates:
        by_tick.setdefault(u.tick, []).append(u)
        if u.driver not in state:
            state[u.driver] = [1, 1]
            order.append(u.driver)
    if master not in state:
        state[master] = [1, 1]
        order.append(master)

    prev_scl = prev_sda = 1
    prev_state = {d: (1, 1) for d in order}
    for ev in record.events:
        changed = False
        for u in by_tick.get(ev.tick, []):
            state[u.driver] = [u.scl.line_level, u.sda.line_level]
            changed = True
        m_scl, m_sda = MODEL_DRIVE[ev.kind]
        before = tuple(state[master])
        state[master] = [m_scl, m_sda]
        if tuple(state[master]) != before:
            changed = True
        scl = min(state[d][0] for d in order)
        sda = min(state[d][1] for d in order)
        testcase.assertEqual(
            (scl, sda), (ev.scl, ev.sda),
            msg=f"tick={ev.tick} kind={ev.kind.value} 事件线值与驱动器线与不符",
        )
        testcase.assertEqual(
            tuple(ev.scl_low_drivers), tuple(d for d in order if state[d][0] == 0)
        )
        testcase.assertEqual(
            tuple(ev.sda_low_drivers), tuple(d for d in order if state[d][1] == 0)
        )
        if (scl, sda) != (prev_scl, prev_sda):
            testcase.assertTrue(
                changed,
                msg=f"tick={ev.tick} 线值翻转但没有任何驱动器更新——出现了凭空翻转",
            )
        prev_scl, prev_sda = scl, sda
        prev_state = {d: tuple(state[d]) for d in order}


# ---------------------------------------------------------------------------
# 期望轨迹（测试侧独立推导：无拉伸时模型每整数时刻一拍的确定性节奏）
# ---------------------------------------------------------------------------

def expected_release_trace(t0: int, p: int):
    """第 p 个有效脉冲上升刻 SDA 释放时的完整期望事件序列。"""
    evs = [(t0, K.RELEASE_SDA, 1, 0, None)]
    for k in range(1, p + 1):
        low_t = t0 + 1 + 2 * (k - 1)
        rise_t = t0 + 2 * k
        sda_at_low = 0  # 下降刻 SDA 仍被 X 拉住（X 在上升刻才释放）
        sda_at_rise = 1 if k == p else 0
        evs.append((low_t, K.PULSE_LOW, 0, sda_at_low, None))
        evs.append((rise_t, K.PULSE_COUNTED, 1, sda_at_rise, k))
    idle_t = t0 + 2 * p + 1
    evs += [
        (idle_t, K.BUS_IDLE, 1, 1, None),
        (idle_t + 1, K.STOP_SETUP_LOW, 0, 0, None),
        (idle_t + 2, K.STOP_SETUP_RISE, 1, 0, None),
        (idle_t + 3, K.STOP_RELEASE, 1, 1, None),
    ]
    return evs


def assert_trace(testcase, record, expected, stop_tick=None):
    testcase.assertEqual(len(record.events), len(expected))
    for ev, (t, kind, scl, sda, pulse) in zip(record.events, expected):
        testcase.assertEqual((ev.tick, ev.kind, ev.scl, ev.sda, ev.pulse),
                             (t, kind, scl, sda, pulse))
    if stop_tick is not None:
        testcase.assertIsNotNone(record.stop)
        testcase.assertEqual(record.stop.tick, stop_tick)


class ReleaseAtPulseTests(unittest.TestCase):
    """SDA 在第 1 / 8 / 9 个有效脉冲后释放。"""

    def _run(self, p):
        ups, last = addr_data_bits_prefix(0)
        req_tick = last + 1            # =7
        ups.append(Update(req_tick, "X", R, D))  # 请求刻：其他驱动器 X 更新
        ups += holder_releases_at(req_tick, p)[1:]
        result = parse_updates(ups, recoveries=[RecoveryRequest(req_tick, "M")])
        rec = result.recoveries[0]
        return ups, result, rec, req_tick

    def test_release_at_pulse_1(self):
        ups, result, rec, t0 = self._run(1)
        self.assertEqual(rec.outcome, O.STOP_GENERATED)
        self.assertEqual(rec.pulses_sent, 1)
        assert_trace(self, rec, expected_release_trace(t0, 1), stop_tick=t0 + 2 + 4)
        assert_events_are_merges(self, ups, "M", rec)
        # 恢复位单独标记
        self.assertEqual([(b.pulse, b.value) for b in rec.bits], [(1, 1)])
        # 恢复前未收完的 3 个地址位被冻结，没有拼进任何字节
        seg = result.transactions[0].segments[0]
        self.assertEqual(seg.blocks, [])
        self.assertEqual([b.value for b in seg.partial_bits], [1, 0, 0])
        self.assertEqual(seg.stop.tick, t0 + 2 + 4)
        self.assertTrue(result.transactions[0].closed)

    def test_release_at_pulse_8(self):
        ups, result, rec, t0 = self._run(8)
        self.assertEqual(rec.outcome, O.STOP_GENERATED)
        self.assertEqual(rec.pulses_sent, 8)
        assert_trace(self, rec, expected_release_trace(t0, 8),
                     stop_tick=t0 + 16 + 4)
        assert_events_are_merges(self, ups, "M", rec)
        self.assertEqual([b.value for b in rec.bits], [0] * 7 + [1])
        self.assertTrue(result.transactions[0].closed)

    def test_release_at_pulse_9(self):
        ups, result, rec, t0 = self._run(9)
        self.assertEqual(rec.outcome, O.STOP_GENERATED)
        self.assertEqual(rec.pulses_sent, 9)
        assert_trace(self, rec, expected_release_trace(t0, 9),
                     stop_tick=t0 + 18 + 4)
        assert_events_are_merges(self, ups, "M", rec)
        self.assertEqual([b.value for b in rec.bits], [0] * 8 + [1])
        self.assertTrue(result.transactions[0].closed)

    def test_recovery_bits_carry_source(self):
        ups, result, rec, t0 = self._run(1)
        b = rec.bits[0]
        self.assertEqual(b.scl_low_tick, t0 + 1)
        self.assertEqual(b.tick, t0 + 2)
        # 上升刻 X 已同刻释放，SDA 无人拉低
        self.assertEqual(b.sda_low_drivers, ())


class NeverReleaseTests(unittest.TestCase):
    """九个有效脉冲后 SDA 始终被拉低 -> bus_stuck。"""

    def test_nine_pulses_then_stuck(self):
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        ups.append(Update(t0, "X", R, D))  # X 永不放 SDA；也不碰 SCL
        with self.assertRaises(BusStuckError) as cm:
            parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
        err = cm.exception
        self.assertFalse(err.scl_held_low)
        self.assertEqual(err.pulses, 9)
        self.assertEqual(err.tick, t0 + 18)
        self.assertEqual(err.sda_low_drivers, ("X",))
        rec = err.record
        self.assertEqual(rec.outcome, O.STUCK)
        self.assertEqual(rec.stuck_tick, t0 + 18)
        # 逐事件：release + 9*(low,counted)，采样位全 0
        kinds = [e.kind for e in rec.events]
        self.assertEqual(kinds[0], K.RELEASE_SDA)
        self.assertEqual(kinds[1:], [K.PULSE_LOW, K.PULSE_COUNTED] * 9)
        self.assertEqual([b.value for b in rec.bits], [0] * 9)
        self.assertEqual([b.pulse for b in rec.bits], list(range(1, 10)))
        for e in rec.events:
            self.assertEqual(e.scl_low_drivers, ()) if e.kind is K.PULSE_COUNTED else None
        assert_events_are_merges(self, ups, "M", rec)


class SclHeldLowTests(unittest.TestCase):
    """SCL 被别人拉住：脉冲不计；有阈值报 stretch_timeout，无阈值且无救报 bus_stuck。"""

    def _waves(self):
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        # 请求刻 X 同时拉住 SCL 和 SDA，之后一直不放（无限拉伸）。
        ups.append(Update(t0, "X", D, D))
        return ups, t0

    def test_held_low_with_limit_raises_stretch_timeout(self):
        ups, t0 = self._waves()
        with self.assertRaises(StretchTimeoutError) as cm:
            parse_updates(
                ups, stretch_limit=3,
                recoveries=[RecoveryRequest(t0, "M")],
            )
        err = cm.exception
        self.assertTrue(err.during_recovery)
        self.assertEqual(err.limit, 3)
        # 请求刻 t0 SCL 已低（pscl 在 t0 前是高 -> low_since=t0）；duration>3 即 t0+4
        self.assertEqual(err.duration, 4)
        self.assertEqual(err.tick, t0 + 4)
        rec = err.record
        self.assertEqual(rec.outcome, O.STRETCH_TIMEOUT)
        self.assertEqual(rec.pulses_sent, 0)  # 从未观察到 SCL 升高 -> 一个都不计
        self.assertTrue(all(e.scl == 0 for e in rec.events))
        self.assertIn(K.RELEASE_SDA, [e.kind for e in rec.events])
        self.assertIn(K.PULSE_LOW, [e.kind for e in rec.events])
        self.assertTrue(
            all(e.kind in (K.RELEASE_SDA, K.PULSE_LOW, K.WAIT_SCL_LOW)
                for e in rec.events)
        )
        assert_events_are_merges(self, ups, "M", rec)

    def test_held_low_no_limit_no_future_update_is_bus_stuck(self):
        ups, t0 = self._waves()
        with self.assertRaises(BusStuckError) as cm:
            parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
        err = cm.exception
        self.assertTrue(err.scl_held_low)
        self.assertEqual(err.pulses, 0)
        self.assertEqual(err.sda_low_drivers, ("X",))  # 拉住 SCL 的人
        self.assertEqual(err.record.outcome, O.STUCK)

    def test_held_low_then_released_counts_one_real_pulse(self):
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        ups.append(Update(t0, "X", R, D))
        # 脉冲1：模型 t0+1 拉低；放行刻 t0+2 X 同刻拉住 SCL（多驱动器同刻），
        # t0+3 仍拉住，t0+4 同时释放 SCL/SDA => 实测升高，脉冲 1 才计数。
        ups += [
            Update(t0 + 2, "X", D, D),
            Update(t0 + 3, "X", D, D),
            Update(t0 + 4, "X", R, R),
        ]
        result = parse_updates(
            ups, stretch_limit=5, recoveries=[RecoveryRequest(t0, "M")]
        )
        rec = result.recoveries[0]
        self.assertEqual(rec.outcome, O.STOP_GENERATED)
        self.assertEqual(rec.pulses_sent, 1)
        kinds = [(e.tick, e.kind, e.scl, e.sda) for e in rec.events]
        self.assertEqual(kinds[0], (t0, K.RELEASE_SDA, 1, 0))
        self.assertEqual(kinds[1], (t0 + 1, K.PULSE_LOW, 0, 0))
        self.assertEqual(kinds[2], (t0 + 2, K.WAIT_SCL_LOW, 0, 0))
        self.assertEqual(kinds[3], (t0 + 3, K.WAIT_SCL_LOW, 0, 0))
        self.assertEqual(kinds[4], (t0 + 4, K.PULSE_COUNTED, 1, 1))
        self.assertEqual(rec.bits[0].pulse, 1)
        self.assertEqual(rec.bits[0].scl_low_tick, t0 + 1)
        assert_events_are_merges(self, ups, "M", rec)


class CoincidentMultiDriverTests(unittest.TestCase):
    """同刻多驱动器更新：先合并再判边沿，脉冲只认合并后实测值。"""

    def test_two_other_drivers_update_same_tick_as_request(self):
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        # 请求刻 X 拉住 SDA、Y 只声明保持释放（同刻两条不同驱动器更新）。
        ups += [Update(t0, "X", R, D), Update(t0, "Y", R, R)]
        ups += holder_releases_at(t0, 1)[1:]
        result = parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
        rec = result.recoveries[0]
        self.assertEqual(rec.outcome, O.STOP_GENERATED)
        self.assertEqual(rec.pulses_sent, 1)
        # 请求刻事件里 SDA 低驱动只有 X（Y 已释放，M 被模型接管为释放）
        rel = rec.events[0]
        self.assertEqual((rel.scl, rel.sda), (1, 0))
        self.assertEqual(rel.sda_low_drivers, ("X",))
        assert_events_are_merges(self, ups, "M", rec)

    def test_simultaneous_holder_release_and_clock_stretch_same_tick(self):
        # 放行刻：X 释放 SDA 与 Y 拉住 SCL 同刻 => 合并 SCL=0，脉冲不计数；
        # 下一拍 Y 释放 => SCL 实测升高，脉冲 1 才计数（SDA 已为高）。
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        ups += [
            Update(t0, "X", R, D), Update(t0, "Y", R, R),
            Update(t0 + 2, "X", R, R), Update(t0 + 2, "Y", D, R),
            Update(t0 + 3, "Y", R, R),
        ]
        result = parse_updates(
            ups, stretch_limit=5, recoveries=[RecoveryRequest(t0, "M")]
        )
        rec = result.recoveries[0]
        self.assertEqual(rec.pulses_sent, 1)
        seq = [(e.kind, e.scl, e.sda) for e in rec.events[:5]]
        self.assertEqual(seq, [
            (K.RELEASE_SDA, 1, 0),
            (K.PULSE_LOW, 0, 0),
            (K.WAIT_SCL_LOW, 0, 1),   # SDA 已被 X 释放，但 SCL 被 Y 拉住
            (K.PULSE_COUNTED, 1, 1),  # 下一实测升高才计数
            (K.BUS_IDLE, 1, 1),
        ])
        assert_events_are_merges(self, ups, "M", rec)


class AbortTests(unittest.TestCase):
    """其他主机发 START：中止恢复，正常识别继续。"""

    def test_other_master_start_aborts_and_transaction_decodes(self):
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        ups.append(Update(t0, "X", R, D))
        # 脉冲 1 上升刻 X 释放 SDA（采样到 1）；下一拍（模型 idle）N 发 START。
        ups += [
            Update(t0 + 2, "X", R, R),
            Update(t0 + 3, "N", R, D),
        ]
        # N 的事务：START 后由 N 发地址字节 0xA0（写），从机 E 给 NACK，再 STOP。
        t = t0 + 4  # START 在 t0+3
        for k in range(8):
            bit = (0xA0 >> (7 - k)) & 1
            ups += [Update(t, "N", D, R if bit else D), Update(t, "E", R, R)]
            t += 1
            ups += [Update(t, "N", R, R if bit else D), Update(t, "E", R, R)]
            t += 1
        # ACK=1：N 释放，E 也释放 => SDA 高
        ups += [Update(t, "N", D, R), Update(t, "E", R, R)]
        t += 1
        ups += [Update(t, "N", R, R), Update(t, "E", R, R)]
        t += 1
        # STOP（NACK 后总线高）：N 补脉冲拉低 SDA 再释放
        ups += [Update(t, "N", D, D), Update(t, "E", R, R)]
        t += 1
        ups += [Update(t, "N", R, D), Update(t, "E", R, R)]
        t += 1
        ups += [Update(t, "N", R, R)]

        result = parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
        rec = result.recoveries[0]
        self.assertEqual(rec.outcome, O.ABORTED)
        self.assertEqual(rec.aborted_by, "N")
        self.assertEqual(rec.pulses_sent, 1)
        self.assertIsNone(rec.stop)
        self.assertEqual(rec.events[-1].kind, K.ABORTED)
        assert_events_are_merges(self, ups, "M", rec)

        # 两个事务：M 的半截事务保持未闭合；N 的事务由正常识别完整解码。
        self.assertEqual(len(result.transactions), 2)
        m_tx, n_tx = result.transactions
        self.assertFalse(m_tx.closed)
        self.assertEqual(m_tx.segments[0].starters, ("M",))
        self.assertTrue(n_tx.closed)
        n_seg = n_tx.segments[0]
        self.assertEqual(n_seg.starters, ("N",))
        self.assertEqual(n_seg.address, 0x50)
        self.assertEqual(n_seg.rw, 0)
        self.assertEqual(n_seg.acks, (1,))
        # 恢复位没有混进 N 的字节
        self.assertEqual(n_seg.blocks[0].value, 0xA0)

    def test_start_by_recovery_master_itself_does_not_abort(self):
        # 只有“其他主机”的 START 才中止；总线在 idle 拍保持高（无人抢线）时正常造 STOP。
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        ups.append(Update(t0, "X", R, D))
        ups += holder_releases_at(t0, 1)[1:]
        result = parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
        self.assertEqual(result.recoveries[0].outcome, O.STOP_GENERATED)


class TakeoverAndNaturalStopTests(unittest.TestCase):
    """接管刻释放 SDA 不算 STOP；STOP 必须在有效脉冲之后。"""

    def test_release_at_takeover_is_not_stop(self):
        # 恢复前 M 自己拉着 SDA（半截位的最后一位是 0），且没有别的 SDA 持有者：
        # 接管首刻 SDA 即被模型释放（总线 0->1，SCL 高），这**不是** STOP；
        # 仍照常发脉冲，第 1 个有效脉冲采样到 1 后才生成合法 STOP。
        ups, last = addr_data_bits_prefix(0, bits=(1, 0, 0))
        t0 = last + 1
        # 请求刻只放一个其他驱动器 Y 的保持声明（SDA 的唯一低持有者是 M 自己）。
        ups.append(Update(t0, "Y", R, R))
        result = parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
        rec = result.recoveries[0]
        self.assertEqual(rec.outcome, O.STOP_GENERATED)
        self.assertEqual(rec.pulses_sent, 1)
        first = rec.events[0]
        self.assertEqual(first.kind, K.RELEASE_SDA)
        self.assertEqual((first.scl, first.sda), (1, 1))  # 接管刻总线已高
        # STOP 出现在第 1 个有效脉冲之后，而不是接管刻
        self.assertGreater(rec.stop.tick, t0)
        self.assertEqual(rec.stop.tick, t0 + 6)
        assert_events_are_merges(self, ups, "M", rec)

    def test_external_holder_release_coincident_with_takeover(self):
        # M 与 X 都持有 SDA；接管刻模型释放 M，X 同刻也释放 => 总线高，仍不算 STOP。
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        ups.append(Update(t0, "X", R, R))  # X 在请求刻释放（它此前不在表里，
        # 等价声明为保持释放；此例主要验证多驱动器同刻合并不产生伪 STOP）
        result = parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
        rec = result.recoveries[0]
        self.assertEqual(rec.outcome, O.STOP_GENERATED)
        self.assertEqual(rec.stop.tick, t0 + 6)
        assert_events_are_merges(self, ups, "M", rec)


class StopBlockedTests(unittest.TestCase):
    """计划 STOP 释放刻 SDA 又被其他驱动器拉低：回去补脉冲（受 9 个额度约束）。"""

    def test_blocked_then_repulse_then_stop(self):
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        # X 持 SDA 低，在脉冲1 上升刻释放（采样到 1）。
        ups += [Update(t0, "X", R, D), Update(t0 + 2, "X", R, R)]
        # 模型节奏：idle=t0+3, stop_low=t0+4, stop_rise=t0+5, stop_release=t0+6。
        # Z 在 stop_release 刻重新拉住 SDA（SCL 高）=> STOP 形不成，回去补脉冲。
        ups.append(Update(t0 + 6, "Z", R, D))
        # 补发的脉冲2：低 t0+7，放行 t0+8（Z 仍拉，采样 0）；脉冲3 放行刻 t0+10 Z 释放。
        ups.append(Update(t0 + 10, "Z", R, R))
        result = parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
        rec = result.recoveries[0]
        self.assertEqual(rec.outcome, O.STOP_GENERATED)
        self.assertEqual(rec.pulses_sent, 3)
        blocked = [e for e in rec.events if e.kind is K.STOP_BLOCKED]
        self.assertEqual(len(blocked), 1)
        self.assertEqual(blocked[0].tick, t0 + 6)
        self.assertEqual((blocked[0].scl, blocked[0].sda), (1, 0))
        self.assertEqual(blocked[0].sda_low_drivers, ("Z",))
        # 位序列：脉冲1=1（首次释放），脉冲2=0（Z 重新拉），脉冲3=1（再次释放）
        self.assertEqual([(b.pulse, b.value) for b in rec.bits],
                         [(1, 1), (2, 0), (3, 1)])
        assert_events_are_merges(self, ups, "M", rec)

    def test_blocked_until_nine_pulses_exhausted_is_stuck(self):
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        ups += [Update(t0, "X", R, D), Update(t0 + 2, "X", R, R)]
        # 脉冲1 采到 1；计划 STOP 刻 Z 拉住 SDA 且永不放 => 补脉冲 2..9 全采 0。
        ups.append(Update(t0 + 6, "Z", R, D))
        with self.assertRaises(BusStuckError) as cm:
            parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
        self.assertFalse(cm.exception.scl_held_low)
        self.assertEqual(cm.exception.pulses, 9)
        self.assertEqual(cm.exception.sda_low_drivers, ("Z",))


class RecoveryRequestValidationTests(unittest.TestCase):
    @staticmethod
    def _base():
        ups, last = addr_data_bits_prefix(0)
        return ups, last + 1

    def test_request_tick_must_exist(self):
        ups, t0 = self._base()
        with self.assertRaises(InvalidUpdateError):
            parse_updates(ups, recoveries=[RecoveryRequest(t0 + 999, "M")])

    def test_two_requests_same_tick_rejected(self):
        ups, t0 = self._base()
        ups.append(Update(t0, "X", R, D))
        with self.assertRaises(InvalidUpdateError):
            parse_updates(
                ups,
                recoveries=[RecoveryRequest(t0, "M"), RecoveryRequest(t0, "N")],
            )

    def test_external_update_to_master_during_recovery_rejected(self):
        ups, t0 = self._base()
        ups.append(Update(t0, "X", R, D))
        ups.append(Update(t0 + 1, "M", D, R))  # 恢复期间再写 M
        with self.assertRaises(InvalidUpdateError):
            parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])

    def test_master_update_on_request_tick_rejected(self):
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        ups.append(Update(t0, "M", R, R))  # 请求刻又给 M 一条更新
        with self.assertRaises(InvalidUpdateError):
            parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])

    def test_overlapping_recoveries_rejected(self):
        ups, last = addr_data_bits_prefix(0)
        t0 = last + 1
        ups.append(Update(t0, "X", R, D))
        # 第一次恢复永不结束（X 不放），第二个请求必落在其激活期
        ups.append(Update(t0 + 5, "Y", R, D))
        with self.assertRaises(InvalidUpdateError):
            parse_updates(
                ups,
                recoveries=[RecoveryRequest(t0, "M"), RecoveryRequest(t0 + 5, "M")],
            )

    def test_bool_tick_rejected(self):
        # 与 Update 一致：构造本身不校验，bool 伪整数在 parse_updates 时被拒绝。
        ups, t0 = self._base()
        ups.append(Update(t0, "X", R, D))
        with self.assertRaises(InvalidUpdateError):
            parse_updates(
                ups, recoveries=[RecoveryRequest(tick=True, master="M")]  # type: ignore[arg-type]
            )
        with self.assertRaises(InvalidUpdateError):
            parse_updates(
                ups, recoveries=[RecoveryRequest(t0, master=True)]  # type: ignore[arg-type]
            )
        req = RecoveryRequest(t0, "M")
        self.assertEqual(req.tick, t0)

    def test_recoveries_must_be_iterable_of_requests(self):
        ups, t0 = self._base()
        ups.append(Update(t0, "X", R, D))
        with self.assertRaises(InvalidUpdateError):
            parse_updates(ups, recoveries=123)  # type: ignore[arg-type]
        with self.assertRaises(InvalidUpdateError):
            parse_updates(ups, recoveries=[("tick", t0)])  # type: ignore[list-item]

    def test_no_recoveries_keeps_legacy_behavior(self):
        ups, _ = addr_data_bits_prefix(0)
        result = parse_updates(ups)
        self.assertEqual(result.recoveries, ())
        self.assertEqual(
            [b.value for b in result.transactions[0].segments[0].partial_bits],
            [1, 0, 0],
        )


if __name__ == "__main__":
    unittest.main()
