"""busdecode 的单元测试。

所有期望值均由手工位序列推导(注释中给出推导),不调用被测核心生成。
"""

import unittest

from busdecode import (
    BusDecodeError,
    BusDecoder,
    ConflictingUpdateError,
    InvalidTimeError,
    InvalidUpdateError,
    TimeOrderError,
    Update,
    Level,
    Line,
)


class Script:
    """测试侧的波形脚本构造器(独立于被测核心)。

    每个 step 是一拍:同一拍内的多条更新构成一个批次。
    """

    def __init__(self):
        self.t = 0
        self.steps = []  # list of (t, [(driver, line, level), ...])

    def step(self, *updates, dt=1):
        self.t += dt
        self.steps.append((self.t, list(updates)))
        return self


def run(decoder, script, finalize=False):
    for t, updates in script.steps:
        decoder.apply(t, updates)
    if finalize:
        decoder.finalize()
    return decoder


def master_write_script(address, data_bytes, ack=True):
    """主机 m 写事务:START, 地址字节(写), 若干数据字节(从机 s 应答), STOP。"""
    sc = Script()
    sc.step(("m", "sda", "low"))  # START
    sc.step(("m", "scl", "low"))

    def bit(b):
        sc.step(("m", "sda", "low" if b == 0 else "release"))
        sc.step(("m", "scl", "release"))  # 上升沿采样
        sc.step(("m", "scl", "low"))

    def slave_ack():
        sc.step(("m", "sda", "release"), ("s", "sda", "low"))
        sc.step(("m", "scl", "release"))
        sc.step(("m", "scl", "low"))
        sc.step(("s", "sda", "release"))

    byte = (address << 1) | 0
    for i in range(7, -1, -1):
        bit((byte >> i) & 1)
    slave_ack()
    for data in data_bytes:
        for i in range(7, -1, -1):
            bit((data >> i) & 1)
        slave_ack()
    # STOP:SCL 低时确保 SDA 低,释放 SCL,再释放 SDA
    sc.step(("m", "sda", "low"))
    sc.step(("m", "scl", "release"))
    sc.step(("m", "sda", "release"))
    return sc


class TestWriteTransaction(unittest.TestCase):
    def test_address_data_ack_and_bit_provenance(self):
        # 手工推导:地址 0x50 写 -> 地址字节 0xA0 = 1010 0000;
        # 数据 0x3C = 0011 1100;从机均 ACK(线上 0)。
        sc = master_write_script(0x50, [0x3C])
        dec = run(BusDecoder(master="m"), sc)

        self.assertEqual(len(dec.transactions), 1)
        txn = dec.transactions[0]
        self.assertTrue(txn.complete)
        self.assertEqual(len(txn.segments), 1)
        seg = txn.segments[0]
        self.assertEqual(seg.address, 0x50)
        self.assertFalse(seg.read)
        self.assertTrue(seg.address_ack)
        self.assertEqual(len(seg.data, ), 1)
        self.assertEqual(seg.data[0].value, 0x3C)
        self.assertTrue(seg.data[0].ack)

        # 地址位来源:8 位,值序列 1,0,1,0,0,0,0,0
        self.assertEqual([b.value for b in seg.address_bits],
                         [1, 0, 1, 0, 0, 0, 0, 0])
        self.assertTrue(all(b.role == "address" for b in seg.address_bits))
        # 数据位来源:0x3C = 0,0,1,1,1,1,0,0
        self.assertEqual([b.value for b in seg.data[0].bits],
                         [0, 0, 1, 1, 1, 1, 0, 0])
        self.assertTrue(all(b.role == "data" for b in seg.data[0].bits))
        # 采样时刻严格递增,且与事务时间范围一致
        times = [b.time for b in seg.address_bits]
        self.assertEqual(times, sorted(times))
        self.assertEqual(len(set(times)), 8)
        self.assertGreater(times[0], txn.start_time)
        self.assertLess(times[-1], txn.stop_time)
        # 主机发的 0 位:拉低 SDA 的是 "m";ACK 位:拉低的是从机 "s"
        self.assertEqual(seg.address_bits[0].low_drivers, ())  # 发 1,无人拉低
        self.assertEqual(seg.address_bits[1].low_drivers, ("m",))
        self.assertEqual(seg.address_ack_bit.low_drivers, ("s",))
        self.assertEqual(seg.address_ack_bit.role, "ack")
        self.assertEqual(seg.address_ack_bit.value, 0)

        kinds = [e.kind for e in dec.events]
        self.assertEqual(kinds.count("start"), 1)
        self.assertEqual(kinds.count("stop"), 1)
        self.assertNotIn("arbitration_lost", kinds)
        self.assertNotIn("stretch_timeout", kinds)

    def test_start_stop_event_times(self):
        sc = master_write_script(0x10, [])
        dec = run(BusDecoder(), sc)
        starts = [e for e in dec.events if e.kind == "start"]
        stops = [e for e in dec.events if e.kind == "stop"]
        self.assertEqual(len(starts), 1)
        self.assertEqual(len(stops), 1)
        self.assertFalse(starts[0].repeated)
        txn = dec.transactions[0]
        self.assertEqual(txn.start_time, starts[0].time)
        self.assertEqual(txn.stop_time, stops[0].time)
        # START 发生在第 1 拍(t=1)
        self.assertEqual(txn.start_time, 1)


class TestRepeatedStart(unittest.TestCase):
    def test_repeated_start_two_segments(self):
        # 手工推导:写地址 0x50(无数据)-> 重复 START -> 读地址 0x50
        # (地址字节 0xA1)-> 从机发 0x7E -> 主机 NACK -> STOP。
        sc = Script()
        sc.step(("m", "sda", "low"))   # START
        sc.step(("m", "scl", "low"))

        def mbit(b):
            sc.step(("m", "sda", "low" if b == 0 else "release"))
            sc.step(("m", "scl", "release"))
            sc.step(("m", "scl", "low"))

        def slave_ack():
            sc.step(("m", "sda", "release"), ("s", "sda", "low"))
            sc.step(("m", "scl", "release"))
            sc.step(("m", "scl", "low"))
            sc.step(("s", "sda", "release"))

        for i in range(7, -1, -1):     # 0xA0
            mbit((0xA0 >> i) & 1)
        slave_ack()
        # 重复 START:SCL 低时主机先拉高 SDA,释放 SCL,再拉低 SDA
        sc.step(("m", "sda", "release"))
        sc.step(("m", "scl", "release"))
        sc.step(("m", "sda", "low"))   # repeated START
        sc.step(("m", "scl", "low"))
        for i in range(7, -1, -1):     # 0xA1(读)
            mbit((0xA1 >> i) & 1)
        slave_ack()
        # 从机发 0x7E = 0111 1110
        for i in range(7, -1, -1):
            b = (0x7E >> i) & 1
            sc.step(("s", "sda", "low" if b == 0 else "release"))
            sc.step(("m", "scl", "release"))
            sc.step(("m", "scl", "low"))
        sc.step(("s", "sda", "release"))
        # 主机 NACK(保持释放),然后 STOP
        sc.step(("m", "scl", "release"))
        sc.step(("m", "scl", "low"))
        sc.step(("m", "sda", "low"))
        sc.step(("m", "scl", "release"))
        sc.step(("m", "sda", "release"))

        dec = run(BusDecoder(master="m"), sc)
        self.assertEqual(len(dec.transactions), 1)
        txn = dec.transactions[0]
        self.assertTrue(txn.complete)
        self.assertEqual(len(txn.segments), 2)

        seg0, seg1 = txn.segments
        self.assertFalse(seg0.repeated)
        self.assertTrue(seg1.repeated)
        self.assertEqual((seg0.address, seg0.read), (0x50, False))
        self.assertEqual((seg1.address, seg1.read), (0x50, True))
        self.assertEqual(seg0.data, [])
        self.assertEqual(len(seg1.data), 1)
        self.assertEqual(seg1.data[0].value, 0x7E)
        self.assertFalse(seg1.data[0].ack)  # NACK

        starts = [e for e in dec.events if e.kind == "start"]
        self.assertEqual(len(starts), 2)
        self.assertTrue(starts[1].repeated)
        # 读方向数据位由从机驱动 0,不得误判仲裁丢失
        self.assertNotIn("arbitration_lost", [e.kind for e in dec.events])


class TestClockStretch(unittest.TestCase):
    def _script_with_low_gap(self, low_gap):
        """START 后 SCL 拉低 low_gap 拍再释放,然后直接 STOP。"""
        sc = Script()
        sc.step(("m", "sda", "low"))          # START, t=1
        sc.step(("m", "scl", "low"))          # SCL 下降, t=2
        sc.step(("m", "scl", "release"), dt=low_gap)  # 低电平持续 low_gap
        sc.step(("m", "sda", "release"))      # STOP
        return sc

    def test_stretch_timeout_reported_once(self):
        # 阈值 10,低电平 11 > 10 -> 报 stretch_timeout,low_duration=11
        dec = run(BusDecoder(stretch_threshold=10),
                  self._script_with_low_gap(11))
        ev = [e for e in dec.events if e.kind == "stretch_timeout"]
        self.assertEqual(len(ev), 1)
        self.assertEqual(ev[0].low_duration, 11)
        # SCL 在 t=2 下降,持续 11,上升沿在 t=13
        self.assertEqual(ev[0].time, 13)

    def test_exactly_at_threshold_is_not_timeout(self):
        # 严格大于才报:低电平 == 阈值 -> 不报
        dec = run(BusDecoder(stretch_threshold=10),
                  self._script_with_low_gap(10))
        self.assertNotIn("stretch_timeout", [e.kind for e in dec.events])

    def test_threshold_disabled(self):
        dec = run(BusDecoder(), self._script_with_low_gap(10_000))
        self.assertNotIn("stretch_timeout", [e.kind for e in dec.events])

    def test_still_low_at_finalize(self):
        # SCL 一直低,finalize 时超过阈值 -> 在 finalize 时刻报告
        sc = Script()
        sc.step(("m", "sda", "low"))
        sc.step(("m", "scl", "low"))   # t=2 起低
        dec = run(BusDecoder(stretch_threshold=5), sc)
        dec.finalize(at=20)            # 低电平 18 > 5
        ev = [e for e in dec.events if e.kind == "stretch_timeout"]
        self.assertEqual(len(ev), 1)
        self.assertEqual(ev[0].time, 20)
        self.assertEqual(ev[0].low_duration, 18)


class TestSameTickBothLines(unittest.TestCase):
    def test_sda_fall_with_scl_fall_same_tick_is_not_start(self):
        # 同一拍 SCL 下降且 SDA 下降:SCL 并非本拍前后都高 -> 不是 START
        sc = Script()
        sc.step(("m", "scl", "low"), ("m", "sda", "low"))
        dec = run(BusDecoder(), sc)
        self.assertEqual(dec.events, [])
        self.assertFalse(dec.in_transaction)

    def test_sda_fall_while_scl_high_is_start(self):
        sc = Script()
        sc.step(("m", "sda", "low"))
        dec = run(BusDecoder(), sc)
        self.assertEqual([e.kind for e in dec.events], ["start"])
        self.assertTrue(dec.in_transaction)

    def test_scl_rise_and_sda_change_same_tick_samples_new_sda(self):
        # START 后,同一拍 SCL 上升且 SDA 变 0:采样到新值 0,不产生 START。
        # 之后补 7 个 1 位 + ACK + STOP,地址字节 = 0111 1111 = 0x7F
        # -> 地址 0x3F,读。
        sc = Script()
        sc.step(("m", "sda", "low"))                 # START
        sc.step(("m", "scl", "low"))
        sc.step(("m", "scl", "release"))             # 同拍:SCL 升,SDA 仍低 -> 采样 0
        sc.step(("m", "scl", "low"))
        for _ in range(7):                           # 7 个 1
            sc.step(("m", "sda", "release"))
            sc.step(("m", "scl", "release"))
            sc.step(("m", "scl", "low"))
        # ACK(从机)
        sc.step(("s", "sda", "low"))
        sc.step(("m", "scl", "release"))
        sc.step(("m", "scl", "low"))
        sc.step(("s", "sda", "release"))
        # STOP
        sc.step(("m", "sda", "low"))
        sc.step(("m", "scl", "release"))
        sc.step(("m", "sda", "release"))

        dec = run(BusDecoder(), sc)
        starts = [e for e in dec.events if e.kind == "start"]
        self.assertEqual(len(starts), 1)  # 同拍双线变化没有产生额外 START
        seg = dec.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x3F)
        self.assertTrue(seg.read)
        self.assertEqual(seg.address_bits[0].value, 0)
        self.assertEqual([b.value for b in seg.address_bits],
                         [0, 1, 1, 1, 1, 1, 1, 1])


class TestMissingStop(unittest.TestCase):
    def test_finalize_flags_missing_stop(self):
        sc = master_write_script(0x50, [0x3C])
        # 去掉最后三拍 STOP 序列,只留事务中段
        sc.steps = sc.steps[:-3]
        dec = run(BusDecoder(), sc, finalize=True)
        kinds = [e.kind for e in dec.events]
        self.assertIn("missing_stop", kinds)
        self.assertNotIn("stop", kinds)
        self.assertEqual(len(dec.transactions), 1)
        txn = dec.transactions[0]
        self.assertFalse(txn.complete)
        self.assertIsNone(txn.stop_time)
        # 已接收的字节仍然保留
        self.assertEqual(txn.segments[0].data[0].value, 0x3C)


class TestArbitration(unittest.TestCase):
    def _two_master_script(self):
        """m1 发地址 0x55(字节 0xAA),m2 发地址 0x2A(字节 0x54)。

        第 7 位(首位)m1 发 1(释放)、m2 发 0(拉低)-> 线上为 0,
        m1 仲裁丢失并退出;其余位由 m2 独占。从机 s 应答。
        """
        sc = Script()
        sc.step(("m1", "sda", "low"), ("m2", "sda", "low"))   # 双主机同时 START
        sc.step(("m1", "scl", "low"), ("m2", "scl", "low"))
        # 首位:m1=1(释放), m2=0(拉低)
        sc.step(("m1", "sda", "release"), ("m2", "sda", "low"))
        sc.step(("m1", "scl", "release"), ("m2", "scl", "release"))
        sc.step(("m1", "scl", "low"), ("m2", "scl", "low"))
        # m1 退出:释放 SCL 与 SDA(SDA 本就已释放),不再驱动
        sc.step(("m1", "scl", "release"))
        # m2 发完 0x54 剩余 7 位 = 101 0100
        for i in range(6, -1, -1):
            b = (0x54 >> i) & 1
            sc.step(("m2", "sda", "low" if b == 0 else "release"))
            sc.step(("m2", "scl", "release"))
            sc.step(("m2", "scl", "low"))
        # 从机 ACK
        sc.step(("m2", "sda", "release"), ("s", "sda", "low"))
        sc.step(("m2", "scl", "release"))
        sc.step(("m2", "scl", "low"))
        sc.step(("s", "sda", "release"))
        # STOP(m2)
        sc.step(("m2", "sda", "low"))
        sc.step(("m2", "scl", "release"))
        sc.step(("m2", "sda", "release"))
        return sc

    def test_arbitration_lost_when_master_sends_1_but_reads_0(self):
        dec = run(BusDecoder(master="m1"), self._two_master_script())
        lost = [e for e in dec.events if e.kind == "arbitration_lost"]
        self.assertEqual(len(lost), 1)
        self.assertEqual(lost[0].role, "address")
        # 线上字节为 0x54:地址 0x2A,写,被 ACK
        seg = dec.transactions[0].segments[0]
        self.assertEqual((seg.address, seg.read, seg.address_ack),
                         (0x2A, False, True))
        # 首位采样:拉低 SDA 的是 m2
        self.assertEqual(seg.address_bits[0].value, 0)
        self.assertEqual(seg.address_bits[0].low_drivers, ("m2",))

    def test_losing_master_perspective_no_false_positive_for_winner(self):
        # 从 m2(获胜方)视角:它发的位与线上一致,不应报仲裁丢失
        dec = run(BusDecoder(master="m2"), self._two_master_script())
        self.assertNotIn("arbitration_lost", [e.kind for e in dec.events])

    def test_arbitration_reported_once_per_transaction(self):
        # m1 全程释放(发全 1),m2 拉低全部 8 位:只报第一次丢失
        sc = Script()
        sc.step(("m1", "sda", "low"), ("m2", "sda", "low"))
        sc.step(("m1", "scl", "low"), ("m2", "scl", "low"))
        sc.step(("m1", "sda", "release"))
        for _ in range(8):
            sc.step(("m1", "scl", "release"), ("m2", "scl", "release"))
            sc.step(("m1", "scl", "low"), ("m2", "scl", "low"))
        dec = run(BusDecoder(master="m1"), sc)
        lost = [e for e in dec.events if e.kind == "arbitration_lost"]
        self.assertEqual(len(lost), 1)


class TestConflictAndAtomicity(unittest.TestCase):
    def test_conflicting_update_rejected(self):
        dec = BusDecoder()
        with self.assertRaises(ConflictingUpdateError):
            dec.apply(1, [("m", "sda", "low"), ("m", "sda", "release")])

    def test_failed_batch_leaves_no_partial_change(self):
        sc = master_write_script(0x50, [])
        dec = run(BusDecoder(), sc)
        n_events = len(dec.events)
        n_txn = len(dec.transactions)
        sda_before, scl_before = dec.sda, dec.scl
        with self.assertRaises(ConflictingUpdateError):
            dec.apply(sc.t, [("m", "sda", "low"), ("m", "sda", "release")])
        # 状态与失败前完全一致
        self.assertEqual((dec.sda, dec.scl), (sda_before, scl_before))
        self.assertEqual(len(dec.events), n_events)
        self.assertEqual(len(dec.transactions), n_txn)

    def test_same_driver_different_lines_same_tick_ok(self):
        sc = Script()
        sc.step(("m", "sda", "low"), ("m", "scl", "low"))
        dec = run(BusDecoder(), sc)  # 不抛错;同拍 SDA 降 + SCL 降不构成 START
        self.assertEqual((dec.scl, dec.sda), (Level.LOW, Level.LOW))

    def test_identical_duplicate_update_is_idempotent(self):
        sc = Script()
        sc.step(("m", "sda", "low"), ("m", "sda", "low"))
        dec = run(BusDecoder(), sc)
        self.assertEqual([e.kind for e in dec.events], ["start"])

    def test_update_dataclass_accepted(self):
        dec = BusDecoder()
        dec.apply(1, [Update("m", Line.SDA, Level.LOW)])
        self.assertEqual(dec.sda, Level.LOW)


class TestInputValidation(unittest.TestCase):
    def test_rejects_non_int_time(self):
        dec = BusDecoder()
        for bad in (True, 1.5, float("nan"), float("inf"), -float("inf"), "1", None):
            with self.assertRaises(InvalidTimeError, msg=f"t={bad!r}"):
                dec.apply(bad, [])

    def test_rejects_negative_and_huge_time(self):
        dec = BusDecoder()
        with self.assertRaises(InvalidTimeError):
            dec.apply(-1, [])
        with self.assertRaises(InvalidTimeError):
            dec.apply(2**63, [])

    def test_rejects_time_going_backwards(self):
        dec = BusDecoder()
        dec.apply(5, [])
        with self.assertRaises(TimeOrderError):
            dec.apply(4, [])
        dec.apply(5, [])  # 同刻允许

    def test_rejects_bad_update_fields(self):
        dec = BusDecoder()
        with self.assertRaises(InvalidUpdateError):
            dec.apply(1, [("m", "sda", "wiggle")])
        with self.assertRaises(InvalidUpdateError):
            dec.apply(1, [("m", "sck", "low")])
        with self.assertRaises(InvalidUpdateError):
            dec.apply(1, [("", "sda", "low")])
        with self.assertRaises(InvalidUpdateError):
            dec.apply(1, [(5, "sda", "low")])
        with self.assertRaises(InvalidUpdateError):
            dec.apply(1, [("m", "sda")])
        with self.assertRaises(InvalidUpdateError):
            dec.apply(1, "not-updates")

    def test_rejects_bad_constructor_args(self):
        with self.assertRaises(InvalidTimeError):
            BusDecoder(stretch_threshold=True)
        with self.assertRaises(InvalidTimeError):
            BusDecoder(stretch_threshold=0)
        with self.assertRaises(InvalidTimeError):
            BusDecoder(stretch_threshold=float("nan"))
        with self.assertRaises(InvalidUpdateError):
            BusDecoder(master=7)
        # 所有错误都是 ValueError 的子类,便于定位
        self.assertTrue(issubclass(InvalidTimeError, ValueError))
        self.assertTrue(issubclass(ConflictingUpdateError, BusDecodeError))

    def test_finalize_time_validated(self):
        dec = BusDecoder()
        dec.apply(5, [])
        with self.assertRaises(TimeOrderError):
            dec.finalize(at=4)
        with self.assertRaises(InvalidTimeError):
            dec.finalize(at=True)


class TestOrphansAndTruncation(unittest.TestCase):
    def test_orphan_bit_outside_transaction(self):
        sc = Script()
        sc.step(("m", "scl", "low"))
        sc.step(("m", "scl", "release"))  # 无 START 的时钟脉冲
        dec = run(BusDecoder(), sc)
        kinds = [e.kind for e in dec.events]
        self.assertEqual(kinds, ["orphan_bit"])
        self.assertEqual(dec.transactions, [])

    def test_orphan_stop_without_transaction(self):
        sc = Script()
        sc.step(("m", "sda", "low"), ("m", "scl", "low"))  # 非 START(同拍双线)
        sc.step(("m", "scl", "release"))
        sc.step(("m", "sda", "release"))  # SCL 高时 SDA 上升,但无事务
        dec = run(BusDecoder(), sc)
        # SCL 上升沿先采到一个事务外孤立位,随后 SDA 上升构成孤立 STOP
        self.assertEqual([e.kind for e in dec.events],
                         ["orphan_bit", "orphan_stop"])

    def test_truncated_byte_reported_at_finalize(self):
        # START 后只发 3 位就停止输入:finalize 报告 truncated_byte。
        # (STOP/重复 START 前的余数位属协议惯例,不报。)
        sc = Script()
        sc.step(("m", "sda", "low"))   # START
        sc.step(("m", "scl", "low"))
        for _ in range(3):             # 只发 3 位
            sc.step(("m", "sda", "release"))
            sc.step(("m", "scl", "release"))
            sc.step(("m", "scl", "low"))
        dec = run(BusDecoder(), sc, finalize=True)
        trunc = [e for e in dec.events if e.kind == "truncated_byte"]
        self.assertEqual(len(trunc), 1)
        self.assertIn("3 dangling bit", trunc[0].detail)
        # 同时因缺 STOP 报告 missing_stop;事务 stop_time 为 None
        self.assertIn("missing_stop", [e.kind for e in dec.events])
        txn = dec.transactions[0]
        self.assertFalse(txn.complete)
        self.assertIsNone(txn.segments[0].address)

    def test_dangling_bits_before_stop_are_not_truncation(self):
        # STOP 建立时钟产生的余数位不视为截断(协议惯例)
        sc = master_write_script(0x50, [0x3C])
        dec = run(BusDecoder(), sc)
        self.assertNotIn("truncated_byte", [e.kind for e in dec.events])


if __name__ == "__main__":
    unittest.main()
