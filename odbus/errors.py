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
        during_recovery: 是否发生在总线恢复操作期间（恢复脉冲等待 SCL 升高时）。
    """

    def __init__(
        self,
        *,
        tick: int,
        low_since: int,
        duration: int,
        limit: int,
        during_recovery: bool = False,
    ) -> None:
        self.tick = tick
        self.low_since = low_since
        self.duration = duration
        self.limit = limit
        self.during_recovery = during_recovery
        where = "（恢复操作期间）" if during_recovery else ""
        super().__init__(
            f"stretch_timeout{where}: SCL 自 tick={low_since} 起持续为低，"
            f"到 tick={tick} 已持续 {duration} tick（阈值 {limit}，超过即报错）"
        )


class BusStuckError(BusTraceError):
    """恢复操作发完最多 9 个**有效** SCL 脉冲后 SDA 仍被拉低（bus_stuck）。

    “有效”指每个脉冲都在总线上**实测观察到** SCL 升高（被别的驱动器拉住、
    SCL 没有真正升高的拍不计入 9 个）；恢复期间采到的位在 ``record.bits``，
    它们不拼入任何数据字节。

    Attributes:
        tick: 第 9 个有效脉冲采样确认 SDA 仍低的时刻（或确认 SCL 被无限拉住的时刻）。
        master: 执行恢复的驱动器。
        pulses: 实测 SCL 升高的脉冲数（恒为 9）。
        sda_low_drivers: 确认时刻仍把 SDA 拉低的驱动器。
        scl_held_low: True 时表示不是 9 脉冲后 SDA 卡住，而是恢复等 SCL 升高期间
            SCL 被别的驱动器无限拉住、调用方又未给 stretch_limit（无外部更新能释放它）。
        record: 本次恢复的完整轨迹记录（异常也保留，不静默丢弃）。
    """

    def __init__(
        self,
        *,
        tick: int,
        master: str,
        pulses: int,
        sda_low_drivers: tuple[str, ...],
        record: "object | None" = None,
        scl_held_low: bool = False,
    ) -> None:
        self.tick = tick
        self.master = master
        self.pulses = pulses
        self.sda_low_drivers = tuple(sda_low_drivers)
        self.record = record
        self.scl_held_low = scl_held_low
        if scl_held_low:
            msg = (
                f"bus_stuck: 恢复驱动器 {master!r} 等待 SCL 升高时，SCL 被"
                f"[{','.join(sda_low_drivers)}]无限拉住（tick={tick}），"
                "且未配置 stretch_limit、后续也无外部更新可能释放它"
            )
        else:
            msg = (
                f"bus_stuck: 恢复驱动器 {master!r} 已发送 {pulses} 个有效 SCL 脉冲，"
                f"到 tick={tick} SDA 仍被拉低（拉低者=[{','.join(self.sda_low_drivers)}]）"
            )
        super().__init__(msg)
