"""输入校验与错误语义：NaN/Infinity、bool、冲突更新、可定位错误、无部分变更。"""

from __future__ import annotations

import math
import unittest

from odbus import BusRecorder, Drive, Update, parse_updates
from odbus.errors import (
    ConflictingUpdateError,
    InvalidUpdateError,
    LimitError,
    StretchTimeoutError,
)
from odbus.validation import MAX_TICK

R = Drive.RELEASE
D = Drive.LOW


class ScalarValidationTests(unittest.TestCase):
    def test_reject_nan_and_infinity_tick(self):
        for bad in (math.nan, math.inf, -math.inf):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidUpdateError) as cm:
                    parse_updates([Update(tick=bad, driver="M", scl=R, sda=R)])
                self.assertEqual(cm.exception.field, "tick")
                self.assertEqual(cm.exception.index, 0)

    def test_reject_bool_as_tick(self):
        with self.assertRaises(InvalidUpdateError) as cm:
            parse_updates([Update(tick=True, driver="M", scl=R, sda=R)])
        self.assertEqual(cm.exception.field, "tick")

    def test_reject_float_tick(self):
        with self.assertRaises(InvalidUpdateError) as cm:
            parse_updates([Update(tick=1.0, driver="M", scl=R, sda=R)])
        self.assertEqual(cm.exception.field, "tick")

    def test_reject_str_tick_and_none(self):
        for bad in ("1", None, 1 + 0j):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidUpdateError):
                    parse_updates([Update(tick=bad, driver="M", scl=R, sda=R)])

    def test_reject_bool_as_stretch_limit(self):
        with self.assertRaises(InvalidUpdateError) as cm:
            parse_updates([], stretch_limit=True)
        self.assertEqual(cm.exception.field, "stretch_limit")

    def test_reject_nonfinite_stretch_limit(self):
        for bad in (math.nan, math.inf):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidUpdateError):
                    parse_updates([], stretch_limit=bad)

    def test_stretch_limit_must_be_positive(self):
        for bad in (0, -1):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidUpdateError):
                    parse_updates([], stretch_limit=bad)

    def test_tick_bounds(self):
        parse_updates([Update(tick=MAX_TICK, driver="M", scl=R, sda=R)])
        with self.assertRaises((LimitError, InvalidUpdateError)):
            parse_updates([Update(tick=MAX_TICK + 1, driver="M", scl=R, sda=R)])
        with self.assertRaises((LimitError, InvalidUpdateError)):
            parse_updates([Update(tick=-1, driver="M", scl=R, sda=R)])

    def test_bad_driver_id(self):
        for bad in ("", "x" * 33, 7, None):
            with self.subTest(bad=bad):
                with self.assertRaises((InvalidUpdateError, LimitError)):
                    parse_updates([Update(tick=0, driver=bad, scl=R, sda=R)])

    def test_wrong_drive_type_and_bool(self):
        for bad in (1, 0, True, False, "low", None):
            with self.subTest(bad=bad):
                with self.assertRaises(InvalidUpdateError):
                    parse_updates([Update(tick=0, driver="M", scl=bad, sda=R)])
                with self.assertRaises(InvalidUpdateError):
                    parse_updates([Update(tick=0, driver="M", scl=R, sda=bad)])

    def test_non_update_element(self):
        with self.assertRaises(InvalidUpdateError) as cm:
            parse_updates([(0, "M", R, R)])
        self.assertEqual(cm.exception.field, "update")
        self.assertEqual(cm.exception.index, 0)

    def test_ticks_must_be_nondecreasing(self):
        with self.assertRaises(InvalidUpdateError) as cm:
            parse_updates([
                Update(5, "M", R, R),
                Update(4, "M", R, R),
            ])
        self.assertEqual(cm.exception.index, 1)
        self.assertEqual(cm.exception.tick, 4)


class ConflictTests(unittest.TestCase):
    def test_same_driver_same_tick_even_identical_rejected(self):
        with self.assertRaises(ConflictingUpdateError) as cm:
            parse_updates([
                Update(0, "M", R, R),
                Update(0, "M", R, R),
            ])
        self.assertEqual(cm.exception.tick, 0)
        self.assertEqual(cm.exception.driver, "M")
        self.assertEqual((cm.exception.first_index, cm.exception.index), (0, 1))

    def test_different_drivers_same_tick_allowed(self):
        result = parse_updates([
            Update(0, "M", R, R),
            Update(0, "V", R, R),
        ])
        self.assertEqual(result.drivers, ("M", "V"))

    def test_same_driver_separate_ticks_allowed(self):
        result = parse_updates([
            Update(0, "M", R, R),
            Update(1, "M", D, R),
        ])
        self.assertEqual(result.final_scl, 0)


class RecorderTests(unittest.TestCase):
    def test_recorder_rejects_before_append(self):
        rec = BusRecorder()
        with self.assertRaises(InvalidUpdateError):
            rec.add(math.nan, "M", R, R)
        self.assertEqual(len(rec), 0)  # 非法更新未留下部分变更

    def test_recoder_build_and_rebuild(self):
        rec = BusRecorder()
        rec.add(0, "M", R, R).add(1, "M", D, D)
        r1 = rec.build()
        r2 = rec.build(stretch_limit=100)  # 可重复解析，不消费记录
        self.assertEqual(len(r1.transactions), len(r2.transactions))
        self.assertEqual(r2.final_scl, 0)
        self.assertEqual(r2.final_sda, 0)

    def test_recoder_out_of_order_rejected(self):
        rec = BusRecorder()
        rec.add(2, "M", R, R)
        with self.assertRaises(InvalidUpdateError):
            rec.add(1, "M", R, R)
        self.assertEqual(len(rec), 1)


class ErrorTypeTests(unittest.TestCase):
    def test_stretch_error_fields(self):
        raw = [
            Update(0, "M", R, R),
            Update(1, "M", D, R),
            Update(5, "M", D, R),
        ]
        with self.assertRaises(StretchTimeoutError) as cm:
            parse_updates(raw, stretch_limit=3)
        e = cm.exception
        self.assertEqual(e.low_since, 1)
        self.assertEqual(e.tick, 5)
        self.assertEqual(e.duration, 4)
        self.assertEqual(e.limit, 3)
        self.assertIn("stretch_timeout", str(e))


if __name__ == "__main__":
    unittest.main()
