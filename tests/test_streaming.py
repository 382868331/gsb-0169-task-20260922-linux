"""分批（增量）识别：IncrementalDecoder 与一次性 parse_updates 结果必须一致。

覆盖：
- 在任意更新下标处切成两批（含 START/ACK/STOP 附近与同刻分组中间）；
- 多批随机切分；
- 同刻更新跨批到达时先收齐再采样；
- finish 时未完成事务按未完成返回，不补 STOP；
- 跨批次的非递减/冲突校验、失败批次无部分变更、finish 后拒绝 feed。
"""

from __future__ import annotations

import unittest

from odbus import Drive, IncrementalDecoder, Update, parse_updates
from odbus.errors import (
    ConflictingUpdateError,
    InvalidUpdateError,
    StretchTimeoutError,
)

from reference import H, gen_scenario


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


def feed_in_batches(updates, cuts, stretch_limit=None):
    dec = IncrementalDecoder(stretch_limit=stretch_limit)
    prev = 0
    for c in cuts:
        dec.feed(updates[prev:c])
        prev = c
    dec.feed(updates[prev:])
    return dec.finish()


class SplitEquivalenceTests(unittest.TestCase):
    """任意切分点都不改变识别结果（批次边界可落在 START/ACK/STOP 附近）。"""

    def test_every_two_batch_split_matches_one_shot(self):
        checked_mid_tick = 0
        for seed in range(48):
            sc = gen_scenario(seed)
            if sc.expect_timeout:
                continue
            updates = to_updates(sc.raw)
            want = parse_updates(updates, stretch_limit=sc.stretch_limit)
            ticks = [u.tick for u in updates]
            for k in range(len(updates) + 1):
                got = feed_in_batches(updates, [k], stretch_limit=sc.stretch_limit)
                self.assertEqual(
                    want, got, f"seed={seed} 在 updates[{k}] 处切分后结果不一致"
                )
                if 0 < k < len(updates) and ticks[k - 1] == ticks[k]:
                    checked_mid_tick += 1
        # 确认确实覆盖到“同一时刻的更新被拆到两批”的情形。
        self.assertGreater(checked_mid_tick, 0)

    def test_random_multi_batch_splits(self):
        state = 0xC0FFEE

        def rnd():
            nonlocal state
            state = (state * 1103515245 + 12345) & 0x7FFFFFFF
            return state

        for seed in range(48):
            sc = gen_scenario(seed)
            if sc.expect_timeout:
                continue
            updates = to_updates(sc.raw)
            want = parse_updates(updates, stretch_limit=sc.stretch_limit)
            for _ in range(5):
                n_batches = 1 + rnd() % 7
                cuts = sorted(rnd() % (len(updates) + 1) for _ in range(n_batches - 1))
                got = feed_in_batches(updates, cuts, stretch_limit=sc.stretch_limit)
                self.assertEqual(
                    want, got, f"seed={seed} cuts={cuts} 多批切分结果不一致"
                )

    def test_empty_batches_are_noop(self):
        sc = gen_scenario(0)
        updates = to_updates(sc.raw)
        want = parse_updates(updates, stretch_limit=sc.stretch_limit)
        dec = IncrementalDecoder(stretch_limit=sc.stretch_limit)
        dec.feed([])
        dec.feed(updates[:5])
        dec.feed([])
        dec.feed(updates[5:])
        self.assertEqual(want, dec.finish())


class IncompleteTransactionTests(unittest.TestCase):
    """finish 显式结束输入：未完成事务按未完成返回，不补 STOP。"""

    def _missing_stop_scenario(self):
        for seed in range(48):
            sc = gen_scenario(seed)
            if not sc.expect_timeout and sc.expected and not sc.expected[-1][-1].closed:
                return seed, sc
        self.fail("生成器中未找到缺 STOP 场景")

    def test_finish_keeps_incomplete_transaction_open(self):
        seed, sc = self._missing_stop_scenario()
        updates = to_updates(sc.raw)
        result = feed_in_batches(updates, [len(updates) // 2],
                                 stretch_limit=sc.stretch_limit)
        last_tx = result.transactions[-1]
        self.assertFalse(last_tx.closed)
        self.assertIsNone(last_tx.segments[-1].stop)
        # 与一次性解析完全一致（含 partial_bits，不伪造 STOP/半截字节）。
        self.assertEqual(
            parse_updates(updates, stretch_limit=sc.stretch_limit), result
        )

    def test_finish_mid_byte_preserves_partial_bits(self):
        # 手工位序列：START + 地址字节 + 数据位的 3 拍，随后直接 finish。
        R, L_ = Drive.RELEASE, Drive.LOW
        raw = []

        def u(t, d, scl, sda):
            raw.append(Update(tick=t, driver=d, scl=scl, sda=sda))

        t = 0
        u(t, "M", R, L_)
        t += 1
        for k in range(8):  # 地址字节 0b1010001_0（写）
            bit = (0xA2 >> (7 - k)) & 1
            u(t, "M", L_, R if bit else L_)
            u(t + 1, "M", R, R if bit else L_)
            t += 2
        # ACK 位（从机拉低）
        u(t, "M", L_, R)
        u(t, "V", R, L_)
        u(t + 1, "M", R, R)
        u(t + 1, "V", R, L_)
        t += 2
        u(t, "V", R, R)
        # 数据字节只给 3 拍（0,1,1）
        for bit in (0, 1, 1):
            u(t, "M", L_, R if bit else L_)
            u(t + 1, "M", R, R if bit else L_)
            t += 2

        one_shot = parse_updates(raw)
        for k in (len(raw) - 6, len(raw) - 1, len(raw)):
            got = feed_in_batches(raw, [k])
            self.assertEqual(one_shot, got)
        seg = one_shot.transactions[-1].segments[-1]
        self.assertFalse(seg.closed)
        self.assertEqual([b.value for b in seg.partial_bits], [0, 1, 1])


class CrossBatchValidationTests(unittest.TestCase):
    def test_conflict_across_batches_same_tick(self):
        R = Drive.RELEASE
        dec = IncrementalDecoder()
        dec.feed([Update(tick=3, driver="M", scl=R, sda=R)])
        with self.assertRaises(ConflictingUpdateError) as cm:
            dec.feed([Update(tick=3, driver="M", scl=R, sda=R)])
        self.assertEqual(cm.exception.tick, 3)
        self.assertEqual(cm.exception.driver, "M")

    def test_non_monotonic_across_batches(self):
        R = Drive.RELEASE
        dec = IncrementalDecoder()
        dec.feed([Update(tick=5, driver="M", scl=R, sda=R)])
        with self.assertRaises(InvalidUpdateError) as cm:
            dec.feed([Update(tick=4, driver="V", scl=R, sda=R)])
        self.assertEqual(cm.exception.field, "tick")

    def test_failed_batch_leaves_no_partial_change(self):
        sc = gen_scenario(1)
        updates = to_updates(sc.raw)
        half = len(updates) // 2
        dec = IncrementalDecoder(stretch_limit=sc.stretch_limit)
        dec.feed(updates[:half])
        last = updates[-1]
        bad = list(updates[half:]) + [
            Update(tick=last.tick, driver=last.driver, scl=last.scl, sda=last.sda)
        ]
        with self.assertRaises(ConflictingUpdateError):
            dec.feed(bad)
        # 失败批次未留下部分变更：重新喂正确批次后结果与一次性解析一致。
        dec.feed(updates[half:])
        self.assertEqual(
            parse_updates(updates, stretch_limit=sc.stretch_limit), dec.finish()
        )

    def test_feed_after_finish_rejected_and_finish_idempotent(self):
        sc = gen_scenario(2)
        updates = to_updates(sc.raw)
        dec = IncrementalDecoder(stretch_limit=sc.stretch_limit)
        dec.feed(updates)
        first = dec.finish()
        self.assertIs(first, dec.finish())
        with self.assertRaises(InvalidUpdateError):
            dec.feed(updates[:1])

    def test_stretch_timeout_raises_during_feed(self):
        for seed in range(48):
            sc = gen_scenario(seed)
            if not sc.expect_timeout:
                continue
            updates = to_updates(sc.raw)
            dec = IncrementalDecoder(stretch_limit=sc.stretch_limit)
            with self.assertRaises(StretchTimeoutError) as cm:
                dec.feed(updates)
                dec.finish()
            self.assertGreater(cm.exception.duration, sc.stretch_limit)
            return
        self.fail("生成器中未找到超时场景")

    def test_stretch_boundary_exact_limit_ok_incremental(self):
        # SCL 低持续恰好 == limit 不得报错（边界相等合法），跨批到达也一样。
        R, L_ = Drive.RELEASE, Drive.LOW
        raw = [
            Update(0, "M", R, L_),               # START
            Update(1, "M", L_, L_),              # SCL 降
            Update(5, "M", R, L_),               # 低持续 4 tick == limit
            Update(6, "M", R, R),                # STOP
        ]
        want = parse_updates(raw, stretch_limit=4)
        got = feed_in_batches(raw, [2], stretch_limit=4)
        self.assertEqual(want, got)


if __name__ == "__main__":
    unittest.main()
