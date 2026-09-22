"""双线开漏总线(I2C 风格)事务识别核心。

语义约定(本模块的确定契约,非完整行业标准):

* 各驱动器(driver)在整数时刻把 SCL/SDA 置为 RELEASE(释放,线被上拉为高)
  或 LOW(拉低)。线值为所有驱动器的"线与":任一驱动器拉低则线为低。
* 同一时刻的一批更新先全部合并,再与合并前的线值比较判断边沿。
* START:SDA 下降沿且 SCL 在本拍前后都为高。
* STOP :SDA 上升沿且 SCL 在本拍前后都为高。
* SCL 上升沿采样 SDA(取合并后的新值),每 8 位后跟 1 个 ACK 位。
* START 可在事务内重复(repeated START),开启新的地址段但不结束事务。
* 时钟低电平持续时间严格大于阈值时报告 stretch_timeout(每个低电平期只报一次)。
* 仲裁丢失:被跟踪的主机在某数据位上释放 SDA(意图发 1)但采样到 0。
  仅在主机应为发送方的位上检查(地址字节、写方向数据字节;ACK 位与
  读方向数据字节不检查)。每次事务只报告第一次丢失,START 后重新武装。
* STOP/重复 START 的建立时钟会在条件出现前多采一个余数位;不足 8 位的
  余数在 STOP/重复 START 时按协议惯例丢弃,不视为错误。输入结束
  (finalize)时仍留有未组字节的余数位,才报告 truncated_byte。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

__all__ = [
    "MAX_TIME",
    "MAX_BATCH_SIZE",
    "MAX_DRIVER_NAME_LENGTH",
    "Level",
    "Line",
    "Update",
    "BitSample",
    "Event",
    "DataByte",
    "Segment",
    "Transaction",
    "BusDecoder",
    "BusDecodeError",
    "InvalidTimeError",
    "TimeOrderError",
    "InvalidUpdateError",
    "ConflictingUpdateError",
]

#: 时刻允许的最大值(含)。
MAX_TIME = 2**63 - 1
#: 单次 apply 接受的更新条数上限。
MAX_BATCH_SIZE = 10_000
#: 驱动器名字符串长度上限。
MAX_DRIVER_NAME_LENGTH = 64


class Level(Enum):
    """驱动器对某条线的驱动电平。"""

    RELEASE = "release"
    LOW = "low"


class Line(Enum):
    """总线的两条线。"""

    SCL = "scl"
    SDA = "sda"


class BusDecodeError(ValueError):
    """本模块所有输入/状态错误的基类(可直接按 ValueError 捕获)。"""


class InvalidTimeError(BusDecodeError):
    """时刻不是合法整数(拒绝 bool、float、NaN、Infinity、负数、超上限)。"""


class TimeOrderError(BusDecodeError):
    """时刻倒退(apply/finalize 的时刻必须单调不减)。"""


class InvalidUpdateError(BusDecodeError):
    """更新条目非法(驱动器、线名、电平或批次结构不合法)。"""


class ConflictingUpdateError(BusDecodeError):
    """同一驱动器在同一批次内对同一条线给出冲突电平。"""


@dataclass(frozen=True)
class Update:
    """一条驱动更新;apply 也接受等价的 (driver, line, level) 三元组。"""

    driver: str
    line: Line
    level: Level


@dataclass(frozen=True)
class BitSample:
    """一次 SCL 上升沿采样到的原始位及其来源。"""

    time: int
    value: int  # 采样到的线值,0 或 1
    role: str  # "address" | "data" | "ack"
    low_drivers: tuple  # 采样时刻把 SDA 拉低的驱动器名(排序后的元组)


@dataclass(frozen=True)
class Event:
    """总线事件。kind 取值:
    "start"(repeated 区分重复起始)、 "stop"、 "orphan_stop"、
    "orphan_bit"、 "stretch_timeout"(带 low_duration)、
    "arbitration_lost"(带 role)、 "truncated_byte"、 "missing_stop"。
    """

    kind: str
    time: int
    repeated: bool = False
    low_duration: int | None = None
    role: str | None = None
    detail: str = ""


@dataclass
class DataByte:
    """一个数据字节:8 个数据位 + ACK 位。"""

    value: int
    ack: bool  # True=ACK(线上为 0), False=NACK
    bits: tuple  # 8 个 BitSample
    ack_bit: BitSample


@dataclass
class Segment:
    """一个地址段(START 或重复 START 到下一个 START/STOP 之间)。"""

    start_time: int
    repeated: bool
    address: int | None = None  # 7 位地址
    read: bool | None = None  # True=读, False=写
    address_ack: bool | None = None
    address_bits: tuple = ()
    address_ack_bit: BitSample | None = None
    data: list = field(default_factory=list)  # list[DataByte]


@dataclass
class Transaction:
    """一个事务:START 到 STOP;缺 STOP 时 stop_time 为 None。"""

    start_time: int
    stop_time: int | None
    segments: list = field(default_factory=list)  # list[Segment]

    @property
    def complete(self) -> bool:
        return self.stop_time is not None


def _validate_time(t, what="time"):
    if isinstance(t, bool) or not isinstance(t, int):
        raise InvalidTimeError(
            f"{what} must be an int; bool/float/NaN/Infinity are rejected, got {t!r}"
        )
    if not 0 <= t <= MAX_TIME:
        raise InvalidTimeError(f"{what} must be in [0, {MAX_TIME}], got {t}")
    return t


def _coerce_line(value):
    if isinstance(value, Line):
        return value
    if isinstance(value, str):
        try:
            return Line(value)
        except ValueError:
            pass
    raise InvalidUpdateError(f"line must be 'scl' or 'sda', got {value!r}")


def _coerce_level(value):
    if isinstance(value, Level):
        return value
    if isinstance(value, str):
        try:
            return Level(value)
        except ValueError:
            pass
    raise InvalidUpdateError(f"level must be 'release' or 'low', got {value!r}")


def _validate_driver(driver):
    if not isinstance(driver, str) or not driver:
        raise InvalidUpdateError(f"driver must be a non-empty str, got {driver!r}")
    if len(driver) > MAX_DRIVER_NAME_LENGTH:
        raise InvalidUpdateError(
            f"driver name longer than {MAX_DRIVER_NAME_LENGTH} chars: {driver!r}"
        )
    return driver


class BusDecoder:
    """双线开漏总线事务识别器。

    参数:
        master: 被跟踪仲裁的主机驱动器名;None 表示不做仲裁跟踪。
        stretch_threshold: SCL 低电平超时阈值(整数,严格大于才报);
            None 表示不检测。
    """

    def __init__(self, *, master=None, stretch_threshold=None):
        if master is not None:
            _validate_driver(master)
        if stretch_threshold is not None:
            if isinstance(stretch_threshold, bool) or not isinstance(
                stretch_threshold, int
            ):
                raise InvalidTimeError(
                    "stretch_threshold must be an int or None; "
                    f"bool/float/NaN/Infinity are rejected, got {stretch_threshold!r}"
                )
            if not 1 <= stretch_threshold <= MAX_TIME:
                raise InvalidTimeError(
                    f"stretch_threshold must be in [1, {MAX_TIME}], "
                    f"got {stretch_threshold}"
                )
        self.master = master
        self.stretch_threshold = stretch_threshold
        self.events: list[Event] = []
        self.transactions: list[Transaction] = []
        self._drivers: dict[str, dict[Line, Level]] = {}
        self._scl = Level.RELEASE
        self._sda = Level.RELEASE
        self._last_time: int | None = None
        self._scl_low_since: int | None = None
        self._stretch_reported = False
        self._txn: Transaction | None = None
        self._segment: Segment | None = None
        self._bits: list[BitSample] = []
        self._arb_lost = False

    # ------------------------------------------------------------------ 输入

    def apply(self, t, updates):
        """在时刻 t 应用一批驱动更新。

        updates 为 (driver, line, level) 三元组或 Update 的可迭代对象。
        整个批次先校验(类型、范围、同驱动器同线冲突),全部合法才改变状态,
        因此失败调用不会留下部分变更。
        """
        t = _validate_time(t)
        if self._last_time is not None and t < self._last_time:
            raise TimeOrderError(
                f"time went backwards: {t} < {self._last_time}"
            )
        merged = self._normalize_updates(updates)
        # ---- 校验全部通过,以下才开始改变状态 ----
        for (driver, line), level in merged.items():
            self._drivers.setdefault(
                driver, {Line.SCL: Level.RELEASE, Line.SDA: Level.RELEASE}
            )[line] = level
        scl1 = self._line_level(Line.SCL)
        sda1 = self._line_level(Line.SDA)
        self._process_edges(t, self._scl, self._sda, scl1, sda1)
        self._scl, self._sda = scl1, sda1
        self._last_time = t

    def finalize(self, at=None):
        """结束输入:报告未解除的时钟拉伸与缺失 STOP 的未完成事务。

        at 缺省为最后一批输入的时刻。未完成事务仍会进入 transactions,
        其 stop_time 为 None(complete 为 False),并产生 "missing_stop" 事件。
        """
        if at is None:
            at = self._last_time if self._last_time is not None else 0
        else:
            at = _validate_time(at, "finalize time")
            if self._last_time is not None and at < self._last_time:
                raise TimeOrderError(
                    f"finalize time went backwards: {at} < {self._last_time}"
                )
        if (
            self._scl is Level.LOW
            and self._scl_low_since is not None
            and self.stretch_threshold is not None
            and not self._stretch_reported
        ):
            duration = at - self._scl_low_since
            if duration > self.stretch_threshold:
                self.events.append(
                    Event("stretch_timeout", at, low_duration=duration)
                )
                self._stretch_reported = True
        if self._bits:
            self.events.append(
                Event(
                    "truncated_byte",
                    at,
                    detail=f"{len(self._bits)} dangling bit(s) discarded at finalize",
                )
            )
            self._bits = []
        if self._txn is not None:
            self.events.append(Event("missing_stop", at))
            self._close_segment()
            self.transactions.append(self._txn)
            self._txn = None

    # -------------------------------------------------------------- 状态查询

    @property
    def scl(self) -> Level:
        return self._scl

    @property
    def sda(self) -> Level:
        return self._sda

    @property
    def in_transaction(self) -> bool:
        return self._txn is not None

    # ---------------------------------------------------------------- 内部

    def _normalize_updates(self, updates):
        if isinstance(updates, (str, bytes)) or not hasattr(updates, "__iter__"):
            raise InvalidUpdateError(
                "updates must be an iterable of (driver, line, level), "
                f"got {updates!r}"
            )
        items = list(updates)
        if len(items) > MAX_BATCH_SIZE:
            raise InvalidUpdateError(
                f"batch has {len(items)} updates, limit is {MAX_BATCH_SIZE}"
            )
        merged: dict[tuple[str, Line], Level] = {}
        for item in items:
            driver, line, level = self._normalize_update(item)
            key = (driver, line)
            if key in merged and merged[key] is not level:
                raise ConflictingUpdateError(
                    f"driver {driver!r} has conflicting {line.value} updates "
                    f"in the same batch ({merged[key].value} vs {level.value})"
                )
            merged[key] = level
        return merged

    @staticmethod
    def _normalize_update(item):
        if isinstance(item, Update):
            driver, line, level = item.driver, item.line, item.level
        elif isinstance(item, (tuple, list)) and len(item) == 3:
            driver, line, level = item
        else:
            raise InvalidUpdateError(
                f"update must be an Update or (driver, line, level), got {item!r}"
            )
        return _validate_driver(driver), _coerce_line(line), _coerce_level(level)

    def _line_level(self, line) -> Level:
        for state in self._drivers.values():
            if state[line] is Level.LOW:
                return Level.LOW
        return Level.RELEASE

    def _process_edges(self, t, scl0, sda0, scl1, sda1):
        # START/STOP:SDA 边沿且 SCL 本拍前后都为高。
        if sda0 is not sda1 and scl0 is Level.RELEASE and scl1 is Level.RELEASE:
            if sda1 is Level.LOW:
                self._on_start(t)
            else:
                self._on_stop(t)
        # SCL 边沿。
        if scl0 is Level.LOW and scl1 is Level.RELEASE:
            self._on_scl_rise(t, sda1)
        elif scl0 is Level.RELEASE and scl1 is Level.LOW:
            self._scl_low_since = t
            self._stretch_reported = False

    def _on_start(self, t):
        repeated = self._txn is not None
        self.events.append(Event("start", t, repeated=repeated))
        if repeated:
            self._bits = []  # 重复 START 的建立时钟等余数位按惯例丢弃
            self._close_segment()
        else:
            self._txn = Transaction(start_time=t, stop_time=None)
        self._segment = Segment(start_time=t, repeated=repeated)
        self._arb_lost = False

    def _on_stop(self, t):
        if self._txn is None:
            self.events.append(Event("orphan_stop", t))
            return
        self.events.append(Event("stop", t))
        self._bits = []  # STOP 的建立时钟等余数位按惯例丢弃
        self._close_segment()
        self._txn.stop_time = t
        self.transactions.append(self._txn)
        self._txn = None

    def _close_segment(self):
        if self._segment is not None:
            self._txn.segments.append(self._segment)
            self._segment = None

    def _on_scl_rise(self, t, sda):
        if self._scl_low_since is not None:
            duration = t - self._scl_low_since
            if (
                self.stretch_threshold is not None
                and duration > self.stretch_threshold
                and not self._stretch_reported
            ):
                self.events.append(
                    Event("stretch_timeout", t, low_duration=duration)
                )
                self._stretch_reported = True
            self._scl_low_since = None
        value = 0 if sda is Level.LOW else 1
        if self._txn is None:
            self.events.append(
                Event("orphan_bit", t, detail=f"bit {value} sampled outside a transaction")
            )
            return
        role = (
            "ack"
            if len(self._bits) == 8
            else ("address" if self._segment.address is None else "data")
        )
        low_drivers = tuple(
            sorted(
                d
                for d, state in self._drivers.items()
                if state[Line.SDA] is Level.LOW
            )
        )
        sample = BitSample(t, value, role, low_drivers)
        self._check_arbitration(t, role, value)
        if role == "ack":
            self._complete_byte(sample)
        else:
            self._bits.append(sample)

    def _check_arbitration(self, t, role, value):
        if self.master is None or self._arb_lost or value != 0 or role == "ack":
            return
        if role == "data" and self._segment.read:
            return  # 读方向数据位由从机发送,主机本就该释放
        state = self._drivers.get(self.master)
        master_sda = state[Line.SDA] if state is not None else Level.RELEASE
        if master_sda is Level.RELEASE:
            self._arb_lost = True
            self.events.append(
                Event(
                    "arbitration_lost",
                    t,
                    role=role,
                    detail=(
                        f"master {self.master!r} released SDA (sent 1) "
                        f"but the line read 0 on a {role} bit"
                    ),
                )
            )

    def _complete_byte(self, ack_sample):
        byte_val = 0
        for b in self._bits:
            byte_val = (byte_val << 1) | b.value
        ack = ack_sample.value == 0
        seg = self._segment
        if seg.address is None:
            seg.address = byte_val >> 1
            seg.read = bool(byte_val & 1)
            seg.address_ack = ack
            seg.address_bits = tuple(self._bits)
            seg.address_ack_bit = ack_sample
        else:
            seg.data.append(DataByte(byte_val, ack, tuple(self._bits), ack_sample))
        self._bits = []
