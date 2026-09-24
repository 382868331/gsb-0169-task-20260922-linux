"""公开接口上限与输入校验。"""

from __future__ import annotations

import math
from typing import Any

from .errors import InvalidSampleError, InvalidUpdateError, LimitError

#: 允许的最小整数时刻。
MIN_TICK = 0
#: 允许的最大整数时刻（接口边界，短演示不需要达到）。
MAX_TICK = 1_000_000_000
#: 单次 :func:`odbus.parse_updates` 接受的最大更新条数。
MAX_UPDATE_COUNT = 1_000_000
#: 一次解析中允许出现的不同驱动器数量上限。
MAX_DRIVER_COUNT = 64
#: 驱动器标识字符串长度上限（非空）。
MAX_DRIVER_ID_LEN = 32
#: stretch_limit 参数上限。
MAX_STRETCH_LIMIT = 2**31 - 1
#: 单次 :func:`odbus.parse_samples` 接受的最大采样条数。
MAX_SAMPLE_COUNT = 1_000_000
#: min_width（最小稳定宽度）参数上限。
MAX_MIN_WIDTH = 2**31 - 1


def is_plain_int(value: Any) -> bool:
    """真正的整数：显式拒绝 ``bool``（``True``/``False`` 不是合法时刻）。"""
    return isinstance(value, int) and not isinstance(value, bool)


def require_int(
    value: Any,
    *,
    field: str,
    index: int | None = None,
    tick: int | None = None,
) -> int:
    """要求参数为非布尔整数；拒绝 NaN/Infinity 浮点并给出明确原因。"""
    if isinstance(value, bool):
        raise InvalidUpdateError(
            field=field,
            reason="bool 不是合法整数参数（True/False 会被拒绝）",
            index=index,
            tick=tick,
        )
    if isinstance(value, float):
        if not math.isfinite(value):
            reason = "数值为 NaN 或 Infinity，参数必须是有限整数"
        else:
            reason = "需要 int，收到 float（本模块时刻不使用浮点）"
        raise InvalidUpdateError(field=field, reason=reason, index=index, tick=tick)
    if not isinstance(value, int):
        raise InvalidUpdateError(
            field=field,
            reason=f"需要 int，收到 {type(value).__name__}",
            index=index,
            tick=tick,
        )
    return value


def require_tick(value: Any, *, index: int | None = None) -> int:
    tick = require_int(value, field="tick", index=index)
    if tick < MIN_TICK or tick > MAX_TICK:
        raise LimitError(name=f"tick(updates[{index}])", value=tick, limit=MAX_TICK)
    return tick


def require_driver_id(value: Any, *, index: int | None = None, tick: int | None = None) -> str:
    if not isinstance(value, str):
        raise InvalidUpdateError(
            field="driver",
            reason=f"驱动器标识必须为 str，收到 {type(value).__name__}",
            index=index,
            tick=tick,
        )
    if len(value) == 0:
        raise InvalidUpdateError(
            field="driver", reason="驱动器标识不能为空字符串", index=index, tick=tick
        )
    if len(value) > MAX_DRIVER_ID_LEN:
        raise LimitError(
            name=f"driver 标识长度({value!r})", value=len(value), limit=MAX_DRIVER_ID_LEN
        )
    return value


def require_stretch_limit(value: Any) -> int | None:
    if value is None:
        return None
    limit = require_int(value, field="stretch_limit")
    if limit < 1 or limit > MAX_STRETCH_LIMIT:
        raise InvalidUpdateError(
            field="stretch_limit",
            reason=f"必须满足 1 <= stretch_limit <= {MAX_STRETCH_LIMIT}，收到 {value}",
        )
    return limit


def require_sample_tick(value: Any, *, index: int | None = None) -> int:
    """采样时刻校验（与 Update.tick 同一范围，但错误类型为采样专用）。"""
    if isinstance(value, bool):
        raise InvalidSampleError(
            field="tick",
            reason="bool 不是合法整数时刻（True/False 会被拒绝）",
            index=index,
        )
    if isinstance(value, float):
        if not math.isfinite(value):
            reason = "数值为 NaN 或 Infinity，时刻必须是有限整数"
        else:
            reason = "需要 int，收到 float（采样时刻不使用浮点）"
        raise InvalidSampleError(field="tick", reason=reason, index=index)
    if not isinstance(value, int):
        raise InvalidSampleError(
            field="tick",
            reason=f"需要 int，收到 {type(value).__name__}",
            index=index,
        )
    if value < MIN_TICK or value > MAX_TICK:
        raise LimitError(name=f"tick(samples[{index}])", value=value, limit=MAX_TICK)
    return value


def require_level(value: Any, *, field: str, index: int, tick: int) -> int:
    """采样电平：仅接受整数 0/1；bool、浮点（含 NaN）与其它值一律拒绝。"""
    if isinstance(value, bool):
        raise InvalidSampleError(
            field=field, reason="电平只接受整数 0/1，bool 会被拒绝",
            index=index, tick=tick,
        )
    if isinstance(value, float):
        if not math.isfinite(value):
            reason = "数值为 NaN 或 Infinity，电平必须是整数 0/1"
        else:
            reason = "电平只接受整数 0/1，收到 float"
        raise InvalidSampleError(field=field, reason=reason, index=index, tick=tick)
    if value not in (0, 1) or not isinstance(value, int):
        raise InvalidSampleError(
            field=field,
            reason=f"电平必须为整数 0 或 1，收到 {value!r}",
            index=index,
            tick=tick,
        )
    return value


def require_min_width(value: Any) -> int:
    """最小稳定宽度：正整数，拒绝 bool/NaN/Infinity。"""
    if isinstance(value, bool):
        raise InvalidSampleError(
            field="min_width", reason="bool 不是合法宽度（True/False 会被拒绝）"
        )
    if isinstance(value, float):
        if not math.isfinite(value):
            reason = "数值为 NaN 或 Infinity，宽度必须是有限正整数"
        else:
            reason = "需要 int，收到 float（min_width 不使用浮点）"
        raise InvalidSampleError(field="min_width", reason=reason)
    if not isinstance(value, int):
        raise InvalidSampleError(
            field="min_width", reason=f"需要 int，收到 {type(value).__name__}"
        )
    if value < 1 or value > MAX_MIN_WIDTH:
        raise InvalidSampleError(
            field="min_width",
            reason=f"必须满足 1 <= min_width <= {MAX_MIN_WIDTH}，收到 {value}",
        )
    return value
