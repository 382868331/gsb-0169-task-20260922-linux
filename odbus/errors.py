"""异常类型。

所有由本库抛出的错误都继承 :class:`BusTraceError`，便于调用方一次捕获。
错误对象携带可定位字段（更新序号、时刻、驱动器名、参数名等），不做静默纠正。
"""

from __future__ import annotations


class BusTraceError(ValueError):
    """双线开漏总线解析相关错误的基类。"""


class InvalidUpdateError(BusTraceError):
    """单条更新或参数非法。

    Attributes:
        field: 出错字段名（如 ``"tick"``、``"scl"``、``"master"``）。
        index: 更新在输入序列中的下标（从 0 起），参数级错误时为 ``None``。
        tick: 相关整数时刻（若适用）。
        driver: 相关驱动器标识（若适用）。
        reason: 人类可读的原因说明。
    """

    def __init__(
        self,
        *,
        field: str,
        reason: str,
        index: int | None = None,
        tick: int | None = None,
        driver: str | None = None,
    ) -> None:
        self.field = field
        self.reason = reason
        self.index = index
        self.tick = tick
        self.driver = driver
        loc = []
        if index is not None:
            loc.append(f"updates[{index}]")
        if tick is not None:
            loc.append(f"tick={tick}")
        if driver is not None:
            loc.append(f"driver={driver!r}")
        prefix = f"非法字段 {field!r}"
        if loc:
            prefix += "（" + "，".join(loc) + "）"
        super().__init__(f"{prefix}: {reason}")


class ConflictingUpdateError(BusTraceError):
    """同一驱动器在同一整数时刻出现了第二条更新。

    一个驱动器在一个时刻只能有一个合并后的线驱动状态；重复（即使数值相同）
    也会被拒绝，因为批处理不得隐含选择其中一条。
    """

    def __init__(self, *, tick: int, driver: str, index: int, first_index: int) -> None:
        self.tick = tick
        self.driver = driver
        self.index = index
        self.first_index = first_index
        super().__init__(
            f"驱动器 {driver!r} 在 tick={tick} 出现冲突/重复更新："
            f"updates[{first_index}] 与 updates[{index}]"
        )


class DuplicateSampleError(BusTraceError):
    """同一整数时刻出现了第二条线采样（:class:`odbus.Sample`）。

    线采样描述的是总线唯一的线值，一个时刻只能有一条采样；重复（即使数值
    相同）也会被拒绝，批处理不得隐含二选一（与驱动器更新的冲突规则一致）。
    """

    def __init__(self, *, tick: int, index: int, first_index: int) -> None:
        self.tick = tick
        self.index = index
        self.first_index = first_index
        super().__init__(
            f"tick={tick} 出现重复采样：samples[{first_index}] 与 samples[{index}]"
        )


class LimitError(BusTraceError):
    """输入超过本库公开的接口上限。"""

    def __init__(self, *, name: str, value: object, limit: int) -> None:
        self.name = name
        self.value = value
        self.limit = limit
        super().__init__(f"{name} 超过接口上限：值={value!r}，上限={limit}")


class StretchTimeoutError(BusTraceError):
    """事务进行中 SCL 持续为低的时间超过给定阈值。

    Attributes:
        tick: 检测到超时的时刻。
        low_since: 本段 SCL 低电平开始的时刻。
        duration: 触发时的低电平持续 tick 数（``tick - low_since``）。
        limit: 调用方给定的阈值；``duration > limit`` 时触发。
    """

    def __init__(self, *, tick: int, low_since: int, duration: int, limit: int) -> None:
        self.tick = tick
        self.low_since = low_since
        self.duration = duration
        self.limit = limit
        super().__init__(
            f"stretch_timeout: SCL 自 tick={low_since} 起持续为低，"
            f"到 tick={tick} 已持续 {duration} tick（阈值 {limit}，超过即报错）"
        )
