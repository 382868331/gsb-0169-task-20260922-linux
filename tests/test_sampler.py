"""采样输入 + 最小稳定宽度滤波测试。

期望值来源（均不调用被测滤波实现本身）：

* 手工用例：波形与地址/ACK/毛刺字段全部硬编码，宽度边界手算；
* :class:`RefFilterCase` 中的 ``ref_filter_grid``：与 ``odbus.sampler`` 完全
  独立的滤波写法——把零阶保持波形铺成整数 tick 稠密网格，反复扫描运行段、
  从左到右剔除亚宽度区间直到不动点；
* 事务期望值由 ``tests/reference.py`` 的 ``ref_simulate``（不导入 odbus）
  对滤波后的电平解码得到。
"""

from __future__ import annotations

import math
import unittest

from odbus import Drive, Line, Sample, parse_samples, parse_updates
from odbus.errors import (
    DuplicateSampleError,
    InvalidSampleError,
    StretchTimeoutError,
)

from reference import ref_simulate

W = 10          # 干净波形每个电平的持续 tick（>= 2*min_width，便于挖毛刺）
MW = 4          # 最小稳定宽度：毛刺宽 1~2 被剔除，干净段宽 10 保留


# ------------------------- 独立参考：稠密网格滤波 -------------------------

def ref_filter_grid(grid, t0, min_width):
    """对稠密电平序列做最小稳定宽度滤波（独立实现）。

    grid: list[int]，下标相对 t0 的每个整数 tick 的电平（零阶保持已铺满）。
    每轮压缩运行段，从左到右找第一个“两侧电平相同且宽度 < min_width”的
    内层段，用两侧电平填平，再从头扫描，直到没有可剔除段。

    返回 (filtered_grid, removed)；removed 元素为
    (abs_start, abs_end_exclusive, level, width)。
    """
    seq = list(grid)

    def runs():
        out = []
        i = 0
        while i < len(seq):
            j = i + 1
            while j < len(seq) and seq[j] == seq[i]:
                j += 1
            out.append((seq[i], i, j))
            i = j
        return out

    removed = []
    while True:
        rs = runs()
        hit = None
        for k in range(1, len(rs) - 1):
            lv, s, e = rs[k]
            if rs[k - 1][0] == rs[k + 1][0] and (e - s) < min_width:
                hit = (k, lv, s, e)
                break
        if hit is None:
            break
        _, lv, s, e = hit
        removed.append((t0 + s, t0 + e, lv, e - s))
        seq[s:e] = [rs[hit[0] - 1][0]] * (e - s)
    return seq, removed


# ------------------------- 确定性干净采样波形生成 -------------------------

def clean_sample_grid(addr_byte, data_bytes=(), acks=()):
    """生成单驱动器写事务的稠密 (scl, sda) 电平网格。

    每个电平持续 W tick；固定 ACK=0（简化 STOP 生成）。
    返回 (grid, t0)，grid 为 [(scl, sda), ...]，t0=0。
    """
    grid = []

    def hold(scl, sda, n=W):
        grid.extend([(scl, sda)] * n)

    hold(1, 1)            # 空闲
    hold(1, 0)            # START（SCL 高，SDA 降）
    bits = []
    for byte in [addr_byte, *data_bytes]:
        bits += [(byte >> (7 - k)) & 1 for k in range(8)]
        bits.append(0)    # ACK=0
    for b in bits:
        hold(0, b)        # SCL 降，数据就绪
        hold(1, b)        # SCL 升，采样
    hold(1, 1)            # STOP（SCL 高，SDA 0->1；末位 ACK=0）
    hold(1, 1)            # 空闲尾巴
    return grid, 0


def inject_glitch(grid, line_idx, start, length):
    """把 grid 中某根线（0=SCL,1=SDA）的 [start, start+length) 翻成反电平。

    调用方保证该区间两侧电平相同（挖出一个亚宽度脉冲）。
    """
    for t in range(start, start + length):
        scl, sda = grid[t]
        flip = 1 - (scl if line_idx == 0 else sda)
        if line_idx == 0:
            grid[t] = (flip, sda)
        else:
            grid[t] = (scl, flip)


def grid_to_samples(grid, t0=0):
    return [Sample(tick=t0 + i, scl=scl, sda=sda) for i, (scl, sda) in enumerate(grid)]


def grid_to_ref_raw(grid, t0=0):
    """滤波后稠密网格 -> ref_simulate 的裸更新（仅电平变化时刻）。"""
    raw = []
    prev = None
    for i, (scl, sda) in enumerate(grid):
        if (scl, sda) != prev:
            raw.append((t0 + i, "M", scl, sda))
            prev = (scl, sda)
    return raw


# ------------------------- 手工用例 -------------------------

class HandBuiltSampleTests(unittest.TestCase):
    def test_clean_write_transaction_hand_checked(self):
        # 地址 0xA0 = 0x50<<1（写），ACK=0，无数据字节，STOP 闭合。
        grid, _ = clean_sample_grid(0xA0)
        out = parse_samples(grid_to_samples(grid), min_width=MW)
        self.assertEqual(out.filtered_glitches, ())
        txs = out.transactions
        self.assertEqual(len(txs), 1)
        seg = txs[0].segments[0]
        self.assertEqual(seg.address, 0x50)
        self.assertEqual(seg.rw, 0)
        self.assertEqual(seg.data, ())
        self.assertEqual(seg.acks, (0,))
        self.assertIsNotNone(seg.stop)
        # BUS 是唯一驱动器
        self.assertEqual(out.result.drivers, ("BUS",))

    def test_write_with_data_bytes(self):
        grid, _ = clean_sample_grid(0xA0, data_bytes=[0xA5, 0x3C])
        out = parse_samples(grid_to_samples(grid), min_width=MW)
        seg = out.transactions[0].segments[0]
        self.assertEqual(seg.address, 0x50)
        self.assertEqual(seg.data, (0xA5, 0x3C))
        self.assertEqual(seg.acks, (0, 0, 0))

    def test_empty_and_single_sample(self):
        out = parse_samples([], min_width=2)
        self.assertEqual(out.transactions, ())
        self.assertEqual(out.filtered_glitches, ())
        self.assertEqual(out.min_width, 2)
        out1 = parse_samples([Sample(5, 1, 1)], min_width=2)
        self.assertEqual(out1.transactions, ())
        self.assertEqual(out1.result.final_scl, 1)

    def test_scl_high_blip_during_low_is_filtered(self):
        # SCL 低电平段中部插 1-tick 高毛刺（距两侧边界各 >= MW）：
        # 不得产生额外上升沿/额外采样位。
        grid, _ = clean_sample_grid(0xA0)
        # 第一个 SCL 低段 [20,30)：取 t=24（左余 4、右余 5，均 >= MW=4）
        inject_glitch(grid, 0, 24, 1)
        self.assertEqual(grid[24][0], 1)
        out = parse_samples(grid_to_samples(grid), min_width=MW)
        seg = out.transactions[0].segments[0]
        self.assertEqual(len(seg.blocks), 1)          # 仍恰好 9 个脉冲
        self.assertEqual(seg.address, 0x50)
        gl = [g for g in out.filtered_glitches if g.line.value == "SCL"]
        self.assertEqual(len(gl), 1)
        self.assertEqual((gl[0].start_tick, gl[0].end_tick, gl[0].width), (24, 25, 1))
        self.assertEqual(gl[0].level, 1)

    def test_sda_blip_during_scl_low_does_not_change_sampled_bit(self):
        # 地址首位为 1（0xA0 bit7=1），该位 SCL 低段期间 SDA 出现 1-tick 低毛刺：
        # 采样发生在 SCL 上升沿，毛刺在低电平期间，滤除后首位仍为 1。
        grid, _ = clean_sample_grid(0xA0)
        # 首位 SDA=1 的运行段自 t=20 起；在 t=24 挖低毛刺（远离段边界）
        inject_glitch(grid, 1, 24, 1)
        out = parse_samples(grid_to_samples(grid), min_width=MW)
        seg = out.transactions[0].segments[0]
        self.assertEqual(seg.blocks[0].bits[0].value, 1)
        self.assertTrue(any(g.line.value == "SDA" and g.start_tick == 24
                            for g in out.filtered_glitches))

    def test_narrow_sda_fall_when_idle_is_not_a_start(self):
        # 空闲（SCL 高）时 SDA 低脉冲宽 2 < MW=4 => 滤除，不产生事务。
        samples = [
            Sample(0, 1, 1), Sample(10, 1, 0), Sample(11, 1, 0), Sample(12, 1, 1),
            Sample(30, 1, 1),
        ]
        out = parse_samples(samples, min_width=4)
        self.assertEqual(len(out.transactions), 0)
        self.assertEqual(len(out.filtered_glitches), 1)
        g = out.filtered_glitches[0]
        self.assertEqual((g.start_tick, g.end_tick, g.width, g.level), (10, 12, 2, 0))

    def test_width_equal_threshold_is_kept(self):
        # width == min_width（=2）必须保留：空闲期一个宽 2 的 SDA 低脉冲，
        # 它满足 START（SCL 高、SDA 1->0），随后 0->1 是 STOP：空事务被记录。
        samples = [
            Sample(0, 1, 1), Sample(10, 1, 0), Sample(11, 1, 0), Sample(12, 1, 1),
        ]
        out = parse_samples(samples, min_width=2)
        self.assertEqual(out.filtered_glitches, ())
        self.assertEqual(len(out.transactions), 1)  # START 被识别 => 保留即差异可见

    def test_width_one_below_threshold_filtered(self):
        samples = [
            Sample(0, 1, 1), Sample(10, 1, 0), Sample(11, 1, 0), Sample(12, 1, 1),
        ]
        out = parse_samples(samples, min_width=3)  # 宽 2 < 3
        self.assertEqual(out.transactions, ())
        self.assertEqual(len(out.filtered_glitches), 1)

    def test_scl_high_blip_equal_width_boundary(self):
        # START 后 SCL 低、一个 SCL 高脉冲、再 SCL 低、最后升高：
        #   高脉冲宽恰好 == min_width => 保留 => 它的上升沿额外采样一位
        #   （共 2 个上升沿）；min_width 加 1 => 高脉冲被滤 => 只剩 1 个上升沿。
        samples = [
            Sample(0, 1, 1), Sample(1, 1, 0),                       # START
            Sample(2, 0, 0), Sample(9, 1, 0), Sample(13, 0, 0),     # 高脉冲 [9,13) 宽4
            Sample(20, 1, 0),                                       # 最终上升沿
        ]
        # 不发 STOP：旧解码器对不足字节的残余位只在开放事务的 partial_bits 中保留
        # （STOP 是干净边界，会清残余位——沿用旧语义）。
        kept = parse_samples(samples, min_width=4)
        seg_k = kept.transactions[0].segments[0]
        n_bits_k = len(seg_k.partial_bits) + sum(len(b.bits) for b in seg_k.blocks)
        self.assertEqual(n_bits_k, 2)        # t9 与 t20 两个上升沿
        self.assertEqual([b.tick for b in seg_k.partial_bits], [9, 20])
        self.assertFalse(kept.transactions[0].closed)
        self.assertEqual(
            [g for g in kept.filtered_glitches if g.line.value == "SCL"], []
        )

        filt = parse_samples(samples, min_width=5)
        seg_f = filt.transactions[0].segments[0]
        n_bits_f = len(seg_f.partial_bits) + sum(len(b.bits) for b in seg_f.blocks)
        self.assertEqual(n_bits_f, 1)        # 高脉冲被滤，仅 t20 上升沿
        self.assertEqual([b.tick for b in seg_f.partial_bits], [20])
        scl_gl = [g for g in filt.filtered_glitches if g.line.value == "SCL"]
        self.assertEqual(len(scl_gl), 1)
        self.assertEqual(
            (scl_gl[0].start_tick, scl_gl[0].end_tick, scl_gl[0].width, scl_gl[0].level),
            (9, 13, 4, 1),
        )

    def test_same_tick_sda_fall_with_scl_fall_is_not_start(self):
        # 同刻 SDA 降 + SCL 降：SCL 非“前后均高”，不算 START（固定规则）。
        samples = [
            Sample(0, 1, 1),
            Sample(1, 0, 0),   # 两根线同刻降
            Sample(2, 1, 0),
            Sample(3, 1, 1),
        ]
        out = parse_samples(samples, min_width=1)
        self.assertEqual(out.transactions, ())

    def test_both_lines_change_same_tick_merged(self):
        # 同刻 SCL 与 SDA 都跳变，滤波阶段两线独立；解码阶段是一条 BUS 更新。
        samples = [
            Sample(0, 1, 1),
            Sample(1, 0, 0),
            Sample(2, 0, 1),
            Sample(3, 1, 1),
        ]
        out = parse_samples(samples, min_width=1)
        # 无 START；BUS 最终两线均高
        self.assertEqual(out.result.final_scl, 1)
        self.assertEqual(out.result.final_sda, 1)

    def test_repeated_start_through_sample_frontend(self):
        # 两个写段（Sr）：第一段写 0x50，重复 START 后写 0x61。
        # Sr 的 SDA 升高必须发生在 SCL 低电平期间，否则会被当作 STOP。
        def seg_bits(grid, addr):
            hold = lambda a, b, n=W: grid.extend([(a, b)] * n)
            hold(1, 0)                      # START / Sr（SCL 高 SDA 降）
            for k in range(9):
                b = ((addr >> (7 - k)) & 1) if k < 8 else 0
                hold(0, b)
                hold(1, b)

        grid = [(1, 1)] * W
        seg_bits(grid, 0xA0)                # ACK=0 结束于 (1,0)
        # Sr 预备序列（每段持续 W）：SCL 降 -> SCL 低时 SDA 升 -> SCL 升（残余脉冲）
        grid.extend([(0, 0)] * W)
        grid.extend([(0, 1)] * W)
        grid.extend([(1, 1)] * W)
        seg_bits(grid, 0xC2)                # 首个 hold(1,0) 即 Sr
        grid.extend([(1, 1)] * W)           # STOP（SCL 高 SDA 0->1）
        out = parse_samples(grid_to_samples(grid), min_width=MW)
        tx = out.transactions[0]
        self.assertEqual(len(tx.segments), 2)
        self.assertFalse(tx.segments[0].start.repeated)
        self.assertTrue(tx.segments[0].ended_by_repeated_start)
        self.assertTrue(tx.segments[1].start.repeated)
        self.assertEqual(tx.segments[0].address, 0x50)
        self.assertEqual(tx.segments[1].address, 0x61)
        self.assertIsNotNone(tx.segments[1].stop)

    def test_first_and_last_runs_never_filtered(self):
        # 首段（无左侧确认）与末段（无右侧确认）短区间按观测保留。
        out = parse_samples([Sample(0, 1, 0), Sample(1, 1, 1)], min_width=8)
        self.assertEqual(out.filtered_glitches, ())
        out2 = parse_samples([Sample(0, 1, 1), Sample(5, 1, 0)], min_width=8)
        self.assertEqual(out2.filtered_glitches, ())
        self.assertEqual(out2.result.final_sda, 0)


# ------------------------- 失败用例 -------------------------

class SampleValidationTests(unittest.TestCase):
    def test_duplicate_tick_rejected_even_identical(self):
        with self.assertRaises(DuplicateSampleError) as cm:
            parse_samples([Sample(1, 1, 1), Sample(1, 1, 1)], min_width=2)
        self.assertEqual(cm.exception.tick, 1)
        self.assertEqual((cm.exception.first_index, cm.exception.index), (0, 1))

    def test_ticks_strictly_increasing(self):
        with self.assertRaises(InvalidSampleError) as cm:
            parse_samples([Sample(5, 1, 1), Sample(4, 1, 1)], min_width=2)
        self.assertEqual(cm.exception.field, "tick")
        self.assertEqual(cm.exception.index, 1)

    def test_bad_levels(self):
        for bad in (2, -1, True, False, 1.0, "1", None, math.nan):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidSampleError):
                    parse_samples([Sample(0, bad, 1)], min_width=2)
                with self.assertRaises(InvalidSampleError):
                    parse_samples([Sample(0, 1, bad)], min_width=2)

    def test_bad_tick(self):
        for bad in (True, 1.0, math.inf, math.nan, "0", None):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidSampleError):
                    parse_samples([Sample(bad, 1, 1)], min_width=2)

    def test_tick_out_of_range_is_limit_error(self):
        from odbus import LimitError, MAX_TICK
        with self.assertRaises(LimitError):
            parse_samples([Sample(MAX_TICK + 1, 1, 1)], min_width=2)
        with self.assertRaises(LimitError):
            parse_samples([Sample(-1, 1, 1)], min_width=2)

    def test_bad_min_width(self):
        for bad in (0, -1, True, False, 1.0, math.nan, math.inf, "3", None):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidSampleError):
                    parse_samples([], min_width=bad)

    def test_non_sample_element(self):
        with self.assertRaises(InvalidSampleError) as cm:
            parse_samples([(0, 1, 1)], min_width=2)
        self.assertEqual(cm.exception.field, "sample")
        self.assertEqual(cm.exception.index, 0)

    def test_stretch_limit_passthrough_raises(self):
        # SCL 持续为低超过 stretch_limit：滤波后由旧解码器抛 StretchTimeoutError。
        # 下降 t5，上升 t11：上升沿补检 duration=6 > limit=5。
        samples = [Sample(0, 1, 0), Sample(5, 0, 0), Sample(10, 0, 0), Sample(11, 1, 0)]
        with self.assertRaises(StretchTimeoutError) as cm:
            parse_samples(samples, min_width=1, stretch_limit=5)
        self.assertEqual(cm.exception.duration, 6)
        self.assertEqual(cm.exception.low_since, 5)

    def test_stretch_limit_boundary_equal_ok(self):
        # 下降 t5，上升 t10：上升沿补检 duration=5 == limit=5，严格大于才报，不抛。
        samples = [Sample(0, 1, 0), Sample(5, 0, 0), Sample(9, 0, 0), Sample(10, 1, 0)]
        parse_samples(samples, min_width=1, stretch_limit=5)

    def test_legacy_updates_api_still_works(self):
        # 旧入口未受影响：直接用 Update + parse_updates 的冒烟断言。
        from odbus import Update
        ups = [
            Update(0, "M", Drive.RELEASE, Drive.LOW),
            Update(1, "M", Drive.LOW, Drive.LOW),
            Update(2, "M", Drive.RELEASE, Drive.LOW),
        ]
        result = parse_updates(ups)
        self.assertEqual(result.final_scl, 1)
        seg = result.transactions[0].segments[0]
        self.assertEqual(seg.partial_bits[0].value, 0)


# ------------------------- 差分：独立网格滤波 × ref_simulate -------------------------

class DifferentialFilterTests(unittest.TestCase):
    def build_glitched_scenario(self, seed):
        """确定性 LCG：干净写事务网格 + 若干挖到运行段中部的 1~2 tick 毛刺。

        只枚举“剔除后两侧残段仍宽 >= MW”的注入点，保证毛刺被单独剔除、
        干净波形完整还原（不发生跨段级联）。
        """
        state = (seed * 2654435761 + 12345) & 0xFFFFFFFF

        def rnd(n):
            nonlocal state
            state = (state * 1103515245 + 12345) & 0x7FFFFFFF
            return state % n

        addr = (rnd(128) << 1) & 0xFF     # 偶地址（写）
        n_data = rnd(3)
        data = [rnd(256) for _ in range(n_data)]
        grid, t0 = clean_sample_grid(addr, data_bytes=data)

        def run_bounds(line_idx, t):
            b = t
            while b > 0 and grid[b - 1][line_idx] == grid[t][line_idx]:
                b -= 1
            e = t + 1
            while e < len(grid) and grid[e][line_idx] == grid[t][line_idx]:
                e += 1
            return b, e

        # 枚举全部可行注入点：(line, start, length)，段宽 W 内两侧余宽 >= MW。
        candidates = []
        for line_idx in (0, 1):
            t = 1
            while t < len(grid) - 1:
                b, e = run_bounds(line_idx, t)
                for length in (1, 2):
                    for start in range(b + MW, e - length - MW + 1):
                        candidates.append((line_idx, start, length))
                t = e

        injections = []
        used: list[tuple[int, int]] = []   # 已占用的 [lo, hi) 区间
        n_choose = 1 + rnd(min(4, len(candidates)))
        for _ in range(n_choose):
            for _try in range(40):
                line_idx, start, length = candidates[rnd(len(candidates))]
                lo, hi = start - W, start + length + W
                if all(hi <= u0 or lo >= u1 for u0, u1 in used):
                    break
            else:
                break
            flipped = 1 - grid[start][line_idx]
            inject_glitch(grid, line_idx, start, length)
            used.append((start - W, start + length + W))
            injections.append((line_idx, start, start + length, length, flipped))
        return grid, t0, injections, addr, data

    def test_seeds_against_independent_grid_filter(self):
        covered = {"SCL": False, "SDA": False, "w2": False, "data": False}
        for seed in range(40):
            with self.subTest(seed=seed):
                grid, t0, injections, addr, data = self.build_glitched_scenario(seed)
                scl_grid = [g[0] for g in grid]
                sda_grid = [g[1] for g in grid]

                scl_f, scl_rm = ref_filter_grid(scl_grid, t0, MW)
                sda_f, sda_rm = ref_filter_grid(sda_grid, t0, MW)

                out = parse_samples(grid_to_samples(grid, t0), min_width=MW)

                # 1) 被剔除毛刺集合与独立参考逐项一致
                # ref 元组为 (start, end_exclusive, level, width)，统一字段顺序
                expect_rm = (
                    {("SCL", r[0], r[1], r[3], r[2]) for r in scl_rm}
                    | {("SDA", r[0], r[1], r[3], r[2]) for r in sda_rm}
                )
                got_rm = {
                    (g.line.value, g.start_tick, g.end_tick, g.width, g.level)
                    for g in out.filtered_glitches
                }
                self.assertEqual(got_rm, expect_rm, f"seed={seed} 毛刺集合不一致")
                for r in scl_rm:
                    covered["SCL"] = True
                    if r[3] == 2:
                        covered["w2"] = True
                for r in sda_rm:
                    covered["SDA"] = True
                if data:
                    covered["data"] = True

                # 2) 滤波后电平与独立参考逐 tick 一致（经 BUS 更新回放）
                merged = list(zip(scl_f, sda_f))
                ref_raw = grid_to_ref_raw(merged, t0)
                ref_txs = ref_simulate(ref_raw)

                txs = out.transactions
                self.assertEqual(len(txs), len(ref_txs), f"seed={seed} 事务数")
                for ti, (tx, ref_segs) in enumerate(zip(txs, ref_txs)):
                    self.assertEqual(len(tx.segments), len(ref_segs),
                                     f"seed={seed} 段数")
                    for si, (seg, rseg) in enumerate(zip(tx.segments, ref_segs)):
                        self.assertEqual(seg.address, rseg.addr)
                        self.assertEqual(seg.rw, rseg.rw)
                        self.assertEqual([b.value for b in seg.blocks],
                                         [b.value for b in rseg.blocks])
                        self.assertEqual([b.ack.value for b in seg.blocks],
                                         [b.ack for b in rseg.blocks])
                        self.assertIsNotNone(seg.stop)
                # 干净地址/数据在滤除毛刺后必须完整还原
                seg = txs[0].segments[0]
                self.assertEqual(seg.address, addr >> 1)
                self.assertEqual(seg.data, tuple(data))
                self.assertEqual(seg.acks, (0,) * (1 + len(data)))
        self.assertTrue(all(covered.values()), f"差分覆盖不全: {covered}")

    def test_no_glitches_when_min_width_is_one(self):
        # min_width=1：任何脉冲宽度都 >=1，滤波必须是恒等映射。
        grid, t0, _, addr, data = self.build_glitched_scenario(7)
        out = parse_samples(grid_to_samples(grid, t0), min_width=1)
        self.assertEqual(out.filtered_glitches, ())

    def test_default_min_width_is_one(self):
        # 不传 min_width 时默认 1（恒等滤波），采样接口可直接解码。
        grid, _ = clean_sample_grid(0xA0)
        out = parse_samples(grid_to_samples(grid))
        self.assertEqual(out.min_width, 1)
        self.assertEqual(out.transactions[0].segments[0].address, 0x50)

    def test_glitch_report_sorted_and_lines_independent(self):
        # 同一时刻两线各自有毛刺：报告按 (时刻, SCL先于SDA) 排序，
        # 两线滤波互不影响（即使毛刺同刻）。
        samples = [Sample(t, 1, 1) for t in range(0, 30)]
        # t=10 两线同时出现 1-tick 低脉冲，t=11 恢复
        samples[10] = Sample(10, 0, 0)
        out = parse_samples(samples, min_width=3)
        self.assertEqual(len(out.filtered_glitches), 2)
        g0, g1 = out.filtered_glitches
        self.assertEqual((g0.line, g0.start_tick, g0.width), (Line.SCL, 10, 1))
        self.assertEqual((g1.line, g1.start_tick, g1.width), (Line.SDA, 10, 1))
        # 两线毛刺都被填平，无任何事务
        self.assertEqual(out.transactions, ())

    def test_glitch_filtered_removes_false_start(self):
        # START 时刻的 SDA 下降若是亚宽度毛刺（在恢复的 SCL 高电平之间），
        # 被滤后根本没有 START：验证“被过滤毛刺”不会产生幽灵事务。
        samples = [
            Sample(0, 1, 1),
            Sample(5, 1, 0), Sample(6, 1, 1),   # SDA 低脉冲宽 1
            Sample(20, 1, 1),
        ]
        out = parse_samples(samples, min_width=4)
        self.assertEqual(out.transactions, ())
        self.assertEqual(len(out.filtered_glitches), 1)


if __name__ == "__main__":
    unittest.main()
