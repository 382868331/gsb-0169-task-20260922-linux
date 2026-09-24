"""线采样输入（Sample）+ 最小稳定宽度滤波的测试。

期望值来源（不调用被测核心生成）：
- 事务结果（地址/数据/ACK/段数/STOP）全部手算硬编码；
- 滤波行为另有 :func:`ref_filter_events` —— 与核心“事件栈”完全不同的朴素
  “反复删除最左短 run 并重建”写法，用确定性随机两线序列做小规模枚举对照；
- 与旧 ``parse_updates`` 的一致性只作补充断言，不作为期望值来源。
"""

from __future__ import annotations

import math
import unittest

from odbus import (
    Drive,
    Sample,
    Update,
    filter_samples,
    parse_samples,
    parse_updates,
    samples_to_updates,
)
from odbus.errors import (
    DuplicateSampleError,
    InvalidUpdateError,
    LimitError,
    StretchTimeoutError,
)
from odbus.model import Line

IDLE = 1
W = 3  # 本组手算用例统一的最小稳定宽度


# --------------------------- 采样波形构造（仅生成输入） ---------------------------

class SampleWave:
    """生成线采样波形（绝对线电平）。

    每个 SCL 相（低/高）宽度都取 ``W``（== min_width），恰好压住等号边界：
    真实脉冲必须保留。毛刺用更窄的宽度（1 tick）单独注入。
    """

    PW = W  # 相宽

    def __init__(self) -> None:
        self.t = 0
        self.scl = 1
        self.sda = 1
        self.samples: list[Sample] = []

    def set(self, scl: int, sda: int) -> None:
        if scl != self.scl or sda != self.sda:
            self.samples.append(Sample(self.t, scl, sda))
            self.scl, self.sda = scl, sda

    def advance(self, n: int) -> None:
        self.t += n

    def start(self) -> None:
        # SCL 高，SDA 高->低；其后高相保持 PW 再进首个 SCL 降
        self.set(1, 0)
        self.advance(self.PW)

    def pulse(self, sda: int) -> None:
        # 下降沿放好 SDA（SCL 低时允许 SDA 变化），PW 后上升沿采样，再 PW 高相
        self.set(0, sda)
        self.advance(self.PW)
        self.set(1, sda)
        self.advance(self.PW)

    def byte(self, value: int, ack: int) -> None:
        for k in range(8):
            self.pulse((value >> (7 - k)) & 1)
        self.pulse(ack)

    def stop_after_ack_low(self) -> None:
        # 末拍 ACK=0（SCL=1,SDA=0）已保持 PW：SCL 高时 SDA 0->1 即 STOP
        self.set(1, 1)


def write_transaction_samples(addr7: int, data: tuple[int, ...]) -> list[Sample]:
    w = SampleWave()
    w.start()
    w.byte((addr7 << 1) | 0, 0)  # 写，从机 ACK=0
    for v in data:
        w.byte(v, 0)
    w.stop_after_ack_low()
    return w.samples


# --------------------------- 独立参考滤波器（朴素 run 删除法） ---------------------------

def ref_filter_events(events, idle, min_width):
    """独立参考（朴素 O(n^2) 段表，与核心的“堆+链表”是两套写法）。

    段表每项 ``[begin, end, level]``；首段 begin=None 表 -inf（流首空闲），
    末段 end=None 表 +inf（流末开放），宽度均视为无穷。
    每轮选宽度最小（等宽取最早 begin）的有限段：>=min_width 即停；
    否则记为被删毛刺，把它与右段并入左段（左右电平相同）后重扫。
    """
    # 段表：head(-inf..t0,idle)，中间有限段，tail(tn..+inf,last)
    segs = [[None, events[0][0], idle]]
    for k in range(1, len(events)):
        segs.append([events[k - 1][0], events[k][0], events[k - 1][1]])
    segs.append([events[-1][0], None, events[-1][1]])

    def width(s):
        return None if (s[0] is None or s[1] is None) else s[1] - s[0]

    removed: list[tuple[int, int, int]] = []
    while True:
        best = None
        for i, s in enumerate(segs):
            wd = width(s)
            if wd is None:
                continue
            if best is None or (wd, s[0]) < (best[0], segs[best[1]][0]):
                best = (wd, i)
        if best is None or best[0] >= min_width:
            break
        i = best[1]
        b, e, lv = segs[i]
        removed.append((b, e, lv))
        left, right = segs[i - 1], segs[i + 1]
        left[1] = right[1]           # 左段延伸到右段结束（电平相同，合并）
        del segs[i + 1], segs[i]

    out: list[tuple[int, int]] = []
    prev = idle
    for b, _e, lv in segs[1:]:       # 跳过流首空闲哨兵
        if lv != prev:
            out.append((b, lv))
            prev = lv
    return out, removed


def ref_line_events(samples, attr):
    out, prev = [], IDLE
    for s in samples:
        v = getattr(s, attr)
        if v != prev:
            out.append((s.tick, v))
            prev = v
    return out


def ref_filter_whole(samples, min_width):
    """参考：两线各自滤波 + tick 并集合并，返回 (joint_samples, glitches)。"""
    per_line = {}
    glitches = []
    for line, attr in ((Line.SCL, "scl"), (Line.SDA, "sda")):
        ev = ref_line_events(samples, attr)
        if not ev:
            per_line[line] = {}
            continue
        kept, removed = ref_filter_events(ev, IDLE, min_width)
        per_line[line] = dict(kept)
        for b, e, lv in removed:
            glitches.append((b, e, lv, line))
    ticks = sorted(set(per_line[Line.SCL]) | set(per_line[Line.SDA]))
    scl = sda = IDLE
    joint = []
    for t in ticks:
        scl = per_line[Line.SCL].get(t, scl)
        sda = per_line[Line.SDA].get(t, sda)
        if not joint or joint[-1].scl != scl or joint[-1].sda != sda:
            joint.append(Sample(t, scl, sda))
    glitches.sort(key=lambda g: (g[0], 0 if g[3] is Line.SCL else 1))
    return joint, glitches


# --------------------------- 纯滤波：正常 / 毛刺 / 边界 ---------------------------

class FilterBasicTests(unittest.TestCase):
    def test_no_glitch_preserves_steps(self):
        # SDA 在 SCL 高时低宽 4（>=3），保留 START/STOP
        samples = [Sample(0, 1, 0), Sample(4, 1, 1)]
        fr = filter_samples(samples, W)
        self.assertEqual(fr.filtered, ())
        self.assertEqual(fr.samples, (Sample(0, 1, 0), Sample(4, 1, 1)))
        self.assertEqual(fr.min_width, W)

    def test_redundant_identical_samples_are_collapsed(self):
        # 同电平重复给样不产生边沿
        fr = filter_samples([Sample(0, 1, 1), Sample(2, 1, 1), Sample(5, 1, 1)], W)
        self.assertEqual(fr.samples, ())

    def test_scl_one_tick_low_glitch_removed(self):
        # SCL 高期间一个宽度 1 的低毛刺：SCL 本无任何真实脉冲
        samples = [
            Sample(2, 0, 1),   # SCL 进入低
            Sample(3, 1, 1),   # 宽 1 即恢复
        ]
        fr = filter_samples(samples, W)
        self.assertEqual(fr.samples, ())  # SCL 从未真正离开高
        self.assertEqual(len(fr.filtered), 1)
        g = fr.filtered[0]
        self.assertEqual((g.line, g.level, g.begin_tick, g.end_tick, g.duration),
                         (Line.SCL, 0, 2, 3, 1))
        self.assertEqual(g.tick, 3)

    def test_sda_high_glitch_that_would_fake_stop(self):
        # START 后 SCL 高、SDA 低；SDA 宽 1 跳高再回低。
        # 不滤波会被识别成 STOP 后又 START；min_width=3 滤掉后两者都不该出现。
        samples = [
            Sample(0, 1, 0),   # START
            Sample(5, 1, 1),   # 毛刺：跳高（宽 1）
            Sample(6, 1, 0),   # 回低
        ]
        # 不滤波：旧解码器见到 STOP(t5)+START(t6)
        raw = parse_updates(samples_to_updates(samples))
        self.assertEqual(len(raw.transactions), 2)
        # 滤波后：SDA 跳高段宽 1 < 3 被剔除，只剩 START，单一开放事务
        r = parse_samples(samples, W)
        self.assertEqual(len(r.transactions), 1)
        seg = r.transactions[0].segments[0]
        self.assertFalse(seg.start.repeated)
        self.assertIsNone(seg.stop)
        self.assertEqual(len(r.filtered), 1)
        self.assertEqual((r.filtered[0].line, r.filtered[0].level), (Line.SDA, 1))

    def test_width_equal_threshold_is_kept(self):
        # 低宽恰好 3 == min_width：保留（等号边界）
        fr = filter_samples([Sample(0, 1, 0), Sample(3, 1, 1)], 3)
        self.assertEqual(fr.filtered, ())
        self.assertEqual([s.sda for s in fr.samples], [0, 1])

    def test_width_one_below_threshold_is_removed(self):
        # 低宽 3，min_width=4：剔除
        fr = filter_samples([Sample(0, 1, 0), Sample(3, 1, 1)], 4)
        self.assertEqual(fr.samples, ())
        self.assertEqual(fr.filtered[0].duration, 3)

    def test_consecutive_glitches_both_removed(self):
        # SCL：高 -> 低[2,3)宽1（毛刺）-> 高[3,4)宽1（回到空闲，与空闲高合并）
        # -> 低[4,..]开放末段。低毛刺删除后两侧同高合并，高[3,4) 不单列。
        samples = [
            Sample(2, 0, 1),
            Sample(3, 1, 1),
            Sample(4, 0, 1),
        ]
        fr = filter_samples(samples, W)
        self.assertEqual([(g.begin_tick, g.end_tick, g.level) for g in fr.filtered],
                         [(2, 3, 0)])
        # 线视为一直高到 t4，再进入开放低段（末段宽度未知，保留）
        self.assertEqual(fr.samples, (Sample(4, 0, 1),))

    def test_final_unterminated_segment_never_filtered(self):
        # 流末尾最后一段没有结束时刻，宽度未知：哪怕只持续到最后一个 tick 也保留
        samples = [Sample(10, 0, 1)]
        fr = filter_samples(samples, W)
        self.assertEqual(fr.samples, (Sample(10, 0, 1),))
        self.assertEqual(fr.filtered, ())

    def test_glitch_between_two_real_levels_merges(self):
        # 低[0,5) 真实宽段，中间夹一个宽1高毛刺 [2,3)，两侧同低合并为一段
        samples = [
            Sample(0, 1, 0),
            Sample(2, 1, 1),
            Sample(3, 1, 0),
            Sample(5, 1, 1),
        ]
        fr = filter_samples(samples, W)
        self.assertEqual([(g.begin_tick, g.end_tick) for g in fr.filtered], [(2, 3)])
        self.assertEqual([(s.tick, s.sda) for s in fr.samples], [(0, 0), (5, 1)])


class SimultaneousSampleChangeTests(unittest.TestCase):
    def test_both_lines_change_same_tick_merge_into_one_sample(self):
        samples = [
            Sample(0, 1, 1),
            Sample(2, 0, 0),  # SCL、SDA 同刻下降
            Sample(3, 1, 0),
        ]
        # min_width=1：宽度 1 的相保留，但同刻 SCL 降 => SCL 不为高，不算 START
        fr = filter_samples(samples, 1)
        self.assertIn(Sample(2, 0, 0), fr.samples)
        r = parse_samples(samples, 1)
        self.assertEqual(r.transactions, ())

    def test_filtered_events_on_both_lines_same_tick(self):
        # 同 tick 两线各有一个宽 1 毛刺，分别记录，且该刻重建后无边沿
        samples = [
            Sample(0, 1, 1),
            Sample(2, 0, 0),
            Sample(3, 1, 1),
        ]
        fr = filter_samples(samples, W)
        lines = sorted((g.line for g in fr.filtered),
                       key=lambda ln: 0 if ln is Line.SCL else 1)
        self.assertEqual(lines, [Line.SCL, Line.SDA])
        self.assertEqual(fr.samples, ())


# --------------------------- 端到端：滤波后送入旧事务解码器 ---------------------------

class ParseSamplesEndToEndTests(unittest.TestCase):
    def test_write_transaction_clean_samples(self):
        # 手算：地址 0x50 写 = 0xA0，数据 0xA5，两个 ACK=0，STOP
        samples = write_transaction_samples(0x50, (0xA5,))
        r = parse_samples(samples, W)
        self.assertEqual(r.filtered, ())
        seg = r.transactions[0].segments[0]
        self.assertTrue(r.transactions[0].closed)
        self.assertEqual((seg.address, seg.rw), (0x50, 0))
        self.assertEqual(seg.data, (0xA5,))
        self.assertEqual(seg.acks, (0, 0))

    def test_glitch_on_scl_does_not_create_extra_bit(self):
        # START(t0) 后首个真实 SCL 降在 t=W；在 t=1..2 插入宽 1 的 SCL 低毛刺
        # （两侧 SCL 高在剔除后合并为同一高段）。未滤波会多采一位，滤波后必须干净。
        samples = write_transaction_samples(0x50, (0xA5,))
        # START@t0(SCL 高) 后、首个真实 SCL 降@t=W 之间，插 t1..t2 宽 1 低毛刺
        glitchy = [samples[0], Sample(1, 0, 0), Sample(2, 1, 0)] + samples[1:]
        # 对照：不滤波时毛刺产生一个假 SCL 脉冲，位错位、首字节错误
        unfiltered = parse_updates(samples_to_updates(glitchy))
        useg = unfiltered.transactions[0].segments[0]
        self.assertNotEqual(useg.blocks[0].value, 0xA0)
        # 滤波后
        r = parse_samples(glitchy, W)
        seg = r.transactions[0].segments[0]
        self.assertEqual((seg.address, seg.rw), (0x50, 0))
        self.assertEqual(seg.data, (0xA5,))
        self.assertEqual(seg.partial_bits, [])
        self.assertEqual([g.line for g in r.filtered], [Line.SCL])

    def test_sda_glitch_during_high_scl_does_not_split_transaction(self):
        # ACK 之后、STOP 之前（SCL 高、SDA 低）插 SDA 宽1高毛刺：
        # 未滤波会插入 STOP+START 把事务劈成两段；滤波后仍是单事务单段。
        samples = write_transaction_samples(0x50, ())
        # STOP 是最后一个样本；它前面 SCL 高/SDA 低段宽度为 W。
        # 在其中放宽 1 的 0->1->0（两侧同低，剔除后合并）。
        before = samples[:-1]
        last_tick = before[-1].tick  # ACK 上升沿时刻（SCL 高、SDA 低）
        self.assertEqual(last_tick % (2 * W), 0)  # 节拍核对：见 SampleWave
        injected = before + [
            Sample(last_tick + 1, 1, 1),   # 宽 1 跳高（未滤波时会被当成 STOP）
            Sample(last_tick + 2, 1, 0),   # 回低（未滤波时会再被当成 START）
            Sample(last_tick + 3, 1, 1),   # 真正 STOP（原时刻）
        ]
        # 对照：不滤波时被劈成两段
        self.assertEqual(len(parse_updates(samples_to_updates(injected)).transactions), 2)
        # 滤波后
        r = parse_samples(injected, W)
        self.assertEqual(len(r.transactions), 1)
        tx = r.transactions[0]
        self.assertEqual(len(tx.segments), 1)
        self.assertTrue(tx.closed)
        self.assertEqual(tx.segments[0].address, 0x50)
        self.assertEqual([g.line for g in r.filtered], [Line.SDA])

    def test_repeated_start_survives_filter(self):
        # 手算：START 地址0x50写 ACK=0；Sr 地址0x50读；读数据0x3C 主机NACK；STOP。
        # 每个 SCL 相宽度 W（真实信号，等号保留）。
        w = SampleWave()
        w.start()
        w.byte(0x50 << 1 | 0, 0)
        # Sr 序列（SCL 低时 SDA 升，SCL 升后 SDA 在高电平时刻降）
        w.set(0, 0); w.advance(W)
        w.set(0, 1); w.advance(W)   # SCL 低时 SDA 升
        w.set(1, 1); w.advance(W)   # SCL 升（残余脉冲采样 sda=1，随后被 Sr 清零）
        w.set(1, 0); w.advance(W)   # SCL 高时 SDA 降 => Sr
        w.byte(0x50 << 1 | 1, 0)
        w.byte(0x3C, 1)            # 读数据，主机 NACK=1（结束时 SDA 高）
        # NACK 后补一个残余脉冲把 SDA 拉低，保持 W 再 STOP
        w.set(0, 0); w.advance(W)
        w.set(1, 0); w.advance(W)
        w.set(1, 1)                # SCL 高时 SDA 0->1 => STOP
        r = parse_samples(w.samples, W)
        tx = r.transactions[0]
        self.assertEqual(len(tx.segments), 2)
        self.assertFalse(tx.segments[0].start.repeated)
        self.assertTrue(tx.segments[0].ended_by_repeated_start)
        self.assertTrue(tx.segments[1].start.repeated)
        self.assertEqual((tx.segments[1].address, tx.segments[1].rw), (0x50, 1))
        self.assertEqual(tx.segments[1].data, (0x3C,))
        self.assertEqual(r.filtered, ())
        self.assertTrue(tx.closed)

    def test_stretch_checked_after_filter(self):
        # SCL 真实低区间 [1,6) 宽 5；其中数据无变化。limit=4 应在滤波后超时。
        samples = [
            Sample(0, 1, 0),   # START
            Sample(1, 0, 0),   # SCL 降，低开始
            Sample(6, 1, 0),   # 宽 5 才升
        ]
        with self.assertRaises(StretchTimeoutError) as cm:
            parse_samples(samples, W, stretch_limit=4)
        self.assertEqual(cm.exception.duration, 5)

    def test_stretch_not_triggered_by_filtered_low_glitch(self):
        # 一个宽 1 的 SCL 低毛刺被滤掉后，根本不存在低区间，不应报拉伸
        samples = [
            Sample(0, 1, 0),   # START（SDA 低）
            Sample(2, 0, 0),
            Sample(3, 1, 0),   # 宽 1 毛刺
        ]
        r = parse_samples(samples, W, stretch_limit=2)
        # 毛刺滤除；SCL 视为恒高，无脉冲无超时，事务开放
        self.assertEqual(len(r.transactions), 1)
        self.assertEqual([g.line for g in r.filtered], [Line.SCL])

    def test_consistent_with_parse_updates_on_clean_wave(self):
        # 补充（非期望值来源）：无毛刺时 parse_samples 与把采样直接喂旧解码器一致
        samples = write_transaction_samples(0x50, (0xA5,))
        a = parse_samples(samples, W).transactions
        b = parse_updates(samples_to_updates(samples)).transactions
        self.assertEqual(len(a), len(b))
        self.assertEqual(a[0].segments[0].data, b[0].segments[0].data)
        self.assertEqual(a[0].segments[0].acks, b[0].segments[0].acks)


# --------------------------- 输入校验与错误 ---------------------------

class SampleValidationTests(unittest.TestCase):
    def test_duplicate_tick_even_identical_rejected(self):
        with self.assertRaises(DuplicateSampleError) as cm:
            parse_samples([
                Sample(0, 1, 1),
                Sample(0, 1, 1),
            ], W)
        self.assertEqual((cm.exception.tick, cm.exception.first_index, cm.exception.index),
                         (0, 0, 1))

    def test_duplicate_tick_different_value_rejected(self):
        with self.assertRaises(DuplicateSampleError):
            filter_samples([Sample(1, 0, 0), Sample(1, 1, 0)], W)

    def test_ticks_must_be_nondecreasing(self):
        with self.assertRaises(InvalidUpdateError) as cm:
            parse_samples([Sample(5, 1, 1), Sample(4, 1, 1)], W)
        self.assertEqual(cm.exception.index, 1)
        self.assertEqual(cm.exception.field, "tick")

    def test_bad_levels(self):
        for bad in (2, -1, True, False, 1.0, "1", None):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidUpdateError):
                    filter_samples([Sample(0, bad, 1)], W)
                with self.assertRaises(InvalidUpdateError):
                    filter_samples([Sample(0, 1, bad)], W)

    def test_bad_tick(self):
        for bad in (math.nan, math.inf, True, 1.5, -1):
            with self.subTest(bad=bad):
                with self.assertRaises((InvalidUpdateError, LimitError)):
                    filter_samples([Sample(bad, 1, 1)], W)

    def test_non_sample_element(self):
        with self.assertRaises(InvalidUpdateError) as cm:
            filter_samples([(0, 1, 1)], W)
        self.assertEqual(cm.exception.field, "sample")
        self.assertEqual(cm.exception.index, 0)

    def test_bad_min_width(self):
        for bad in (0, -1, True, False, 1.0, math.nan, None, "3"):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidUpdateError):
                    filter_samples([], bad)

    def test_bad_stretch_limit_propagates(self):
        with self.assertRaises(InvalidUpdateError):
            parse_samples([], W, stretch_limit=0)

    def test_error_leaves_no_partial_state_and_input_unchanged(self):
        src = [Sample(0, 1, 1), Sample(1, 1, 1)]
        snapshot = list(src)
        with self.assertRaises(DuplicateSampleError):
            parse_samples(src + [Sample(1, 0, 0)], W)
        self.assertEqual(src, snapshot)  # 纯函数，不改输入
        # samples_to_updates 对重复采样同样拒绝
        with self.assertRaises(DuplicateSampleError):
            samples_to_updates([Sample(0, 1, 1), Sample(0, 0, 0)])

    def test_empty_input_is_empty_result(self):
        fr = filter_samples([], W)
        self.assertEqual(fr.samples, ())
        r = parse_samples([], W)
        self.assertEqual(r.transactions, ())
        self.assertEqual(r.drivers, ())

    def test_update_api_still_works_alongside_samples(self):
        # 旧公开入口继续可用
        result = parse_updates([Update(0, "M", Drive.RELEASE, Drive.LOW)])
        self.assertEqual(len(result.transactions), 1)


# --------------------------- 小规模枚举：独立参考 vs 核心 ---------------------------

def _lcng(seed):
    state = seed & 0xFFFFFFFF

    def nxt():
        nonlocal state
        state = (state * 1103515245 + 12345) & 0x7FFFFFFF
        return state
    return nxt


class FilterEnumerationTests(unittest.TestCase):
    def test_random_two_line_sequences_match_reference(self):
        rnd = _lcng(20260924)
        for width in (1, 2, 3, 5):
            for _ in range(60):
                t = 0
                scl = sda = 1
                samples = []
                for _ in range(8 + rnd() % 12):
                    t += 1 + rnd() % 4
                    if rnd() % 2:
                        scl ^= 1
                    if rnd() % 2:
                        sda ^= 1
                    samples.append(Sample(t, scl, sda))
                fr = filter_samples(samples, width)
                exp_joint, exp_gl = ref_filter_whole(samples, width)
                with self.subTest(width=width, head=samples[:2]):
                    self.assertEqual(list(fr.samples), exp_joint)
                    got_gl = sorted(
                        ((g.begin_tick, g.end_tick, g.level,
                          0 if g.line is Line.SCL else 1)
                         for g in fr.filtered)
                    )
                    want_gl = sorted(
                        (b, e, lv, 0 if line is Line.SCL else 1)
                        for (b, e, lv, line) in exp_gl
                    )
                    self.assertEqual(got_gl, want_gl)


if __name__ == "__main__":
    unittest.main()
