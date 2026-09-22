"""差分测试：确定性生成器场景 × 独立参考解析器 × 被测核心 三方对照。

期望值来自 tests/reference.py（不导入 odbus）；核心结果逐字段比对。
覆盖：写/读、ACK/NACK、重复 START、时钟拉伸、缺 STOP、同刻双线合并。
"""

from __future__ import annotations

import unittest

from odbus import Drive, Update, parse_updates
from odbus.errors import StretchTimeoutError

from reference import (
    H,
    RefStretch,
    gen_scenario,
    ref_simulate,
)


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


class DifferentialTests(unittest.TestCase):
    def compare_scenario(self, seed):
        sc = gen_scenario(seed)
        updates = to_updates(sc.raw)

        if sc.expect_timeout:
            # 独立参考与核心都必须在同一边界（duration > limit）报错。
            with self.assertRaises(RefStretch):
                ref_simulate(sc.raw, sc.stretch_limit)
            with self.assertRaises(StretchTimeoutError) as cm:
                parse_updates(updates, stretch_limit=sc.stretch_limit)
            self.assertGreater(cm.exception.duration, sc.stretch_limit)
            self.assertEqual(
                cm.exception.duration - 1, sc.stretch_limit
            )  # 恰好超出 1 tick
            return

        ref_txs = ref_simulate(sc.raw, sc.stretch_limit)
        result = parse_updates(updates, stretch_limit=sc.stretch_limit)

        self.assertEqual(len(result.transactions), len(ref_txs),
                         f"seed={seed} 事务数不一致")
        for ti, (tx, ref_segs) in enumerate(zip(result.transactions, ref_txs)):
            self.assertEqual(len(tx.segments), len(ref_segs),
                             f"seed={seed} tx={ti} 段数不一致")
            self.assertEqual(tx.closed, ref_segs[-1].closed,
                             f"seed={seed} tx={ti} 关闭状态不一致")
            for si, (seg, rseg) in enumerate(zip(tx.segments, ref_segs)):
                self.assertEqual(seg.start.repeated, rseg.repeated,
                                 f"seed={seed} tx={ti} seg={si} repeated")
                self.assertEqual(seg.address, rseg.addr)
                self.assertEqual(seg.rw, rseg.rw)
                self.assertEqual(
                    [b.value for b in seg.blocks],
                    [b.value for b in rseg.blocks],
                    f"seed={seed} tx={ti} seg={si} 字节值",
                )
                self.assertEqual(
                    [b.ack.value for b in seg.blocks],
                    [b.ack for b in rseg.blocks],
                    f"seed={seed} tx={ti} seg={si} ACK",
                )
                self.assertEqual(seg.data, rseg.data)
                self.assertEqual(seg.acks, rseg.acks)
                self.assertIsNotNone(seg.stop) if rseg.closed else self.assertIsNone(seg.stop)
                self.assertEqual(
                    sorted(a.bit_index for a in seg.arbitration_losses),
                    sorted(rseg.arb_bits),
                    f"seed={seed} tx={ti} seg={si} 仲裁位",
                )

    def test_seeds_0_to_47(self):
        for seed in range(48):
            with self.subTest(seed=seed):
                self.compare_scenario(seed)

    def test_read_segments_exist_in_some_seed(self):
        """确保场景集中确实出现过读段、重复 START 与缺 STOP，避免空覆盖。"""
        kinds = {"read": False, "sr": False, "open": False, "stretch": False}
        for seed in range(48):
            sc = gen_scenario(seed)
            for tx in sc.expected:
                for seg in tx:
                    if seg.rw == 1:
                        kinds["read"] = True
                    if seg.repeated:
                        kinds["sr"] = True
                    if not seg.closed:
                        kinds["open"] = True
            if not sc.expect_timeout:
                # stretch=1..3 的场景
                if sc.max_low_run >= 2:
                    kinds["stretch"] = True
        self.assertTrue(all(kinds.values()), f"场景覆盖不全: {kinds}")


if __name__ == "__main__":
    unittest.main()
