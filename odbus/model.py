"""公开数据结构（frozen dataclass / Enum），先固定接口再实现算法。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class Drive(Enum):
    """单个驱动器对一根线的驱动状态。"""

    RELEASE = 1  # 释放（开漏高阻），线被上拉为高
    LOW = 0      # 拉低

    @property
    def line_level(self) -> int:
        """该驱动单独作用时的线电平（1=高，0=低）。"""
        return 1 if self is Drive.RELEASE else 0


class Line(Enum):
    """更新应用到哪根线。"""

    SCL = "SCL"
    SDA = "SDA"


class BitPhase(Enum):
    """一个 SCL 高脉冲期间采样到的位属于哪一段。"""

    ADDRESS = "address"
    RW = "rw"
    ACK_ADDR = "ack_addr"
    DATA = "data"
    ACK_DATA = "ack_data"


class RecoveryOutcome(Enum):
    """一次总线恢复的结局。"""

    STOP_GENERATED = "stop_generated"      # SDA 释放后由恢复模型生成合法 STOP
    STOP_OBSERVED = "stop_observed"        # 恢复期间总线自然出现合法 STOP（SCL 高时 SDA 0->1）
    ABORTED = "aborted"                    # 其他主机发起 START，恢复中止，正常识别继续
    STUCK = "stuck"                        # 9 个有效脉冲后 SDA 仍被拉低（bus_stuck）
    STRETCH_TIMEOUT = "stretch_timeout"    # 等 SCL 升高期间触发 stretch_timeout（记录保留在异常上）


class RecoveryEventKind(Enum):
    """恢复期间逐时刻事件的种类（每个恢复激活的整数时刻恰好一条）。"""

    RELEASE_SDA = "release_sda"            # 主机接管首刻：释放 SDA（SCL 也不主动拉低）
    PULSE_LOW = "pulse_low"                # 主机拉低 SCL（脉冲起点）
    WAIT_SCL_LOW = "wait_scl_low"          # 主机已放行 SCL，但实测仍低（被别人拉住，不计脉冲）
    PULSE_COUNTED = "pulse_counted"        # 实测 SCL 升高，脉冲计数并采样 SDA
    BUS_IDLE = "bus_idle"                  # SDA 已释放：两线放行观察一拍（让位其他主机 START）
    STOP_SETUP_LOW = "stop_setup_low"      # 为生成 STOP：SCL 低时把 SDA 拉低
    STOP_SETUP_RISE = "stop_setup_rise"    # SCL 升高，SDA 保持低
    STOP_WAIT_SCL = "stop_wait_scl"        # 生成 STOP 过程中 SCL 被别人拉住，继续等
    STOP_RELEASE = "stop_release"          # SCL 高时释放 SDA => 合法 STOP 边沿
    STOP_BLOCKED = "stop_blocked"          # 计划释放时 SDA 又被其他驱动器拉低，回去补脉冲
    STOP_OBSERVED = "stop_observed"        # 总线自然形成的 STOP 被采纳
    ABORTED = "aborted"                    # 其他主机 START，本刻中止恢复


#: 恢复操作最多发送的有效（实测 SCL 确实升高的）SCL 脉冲数。
MAX_RECOVERY_PULSES = 9


@dataclass(frozen=True)
class Update:
    """一条驱动器更新：驱动器 ``driver`` 在整数时刻 ``tick`` 改变线驱动。

    scl/sda 为该驱动器在该时刻**之后**持续保持的电平状态（边沿触发模型：
    若与当前状态相同则为无变化的保持声明）。同一驱动器同一时刻只允许一条
    更新（两条线的变化合并在同一条里表达）。
    """

    tick: int
    driver: str
    scl: Drive
    sda: Drive


@dataclass(frozen=True)
class BitSource:
    """某一位的原始来源信息（题目要求“输出原始位来源”）。"""

    scl_low_tick: int          # 该位脉冲 SCL 下降沿时刻（位的起始）
    scl_high_tick: int         # 采样（SCL 上升沿）时刻
    scl_low_drivers: tuple[str, ...]  # SCL 下降沿时拉低 SCL 的驱动器（时钟发起者）
    sda_low_drivers: tuple[str, ...]  # 采样时刻仍在驱动 SDA 为低的驱动器（空 => 位为 1）
    drivers: tuple[str, ...]   # 截至该时刻所有出现过的驱动器（稳定排序）

    def describe(self) -> str:
        sda = ",".join(self.sda_low_drivers) if self.sda_low_drivers else "（无，上拉为1）"
        scl = ",".join(self.scl_low_drivers) if self.scl_low_drivers else "（无）"
        return (
            f"SCL脉冲 t{self.scl_low_tick}->t{self.scl_high_tick}, "
            f"SCL低驱动=[{scl}], SDA低驱动=[{sda}]"
        )


@dataclass(frozen=True)
class Bit:
    """一个采样到的位。"""

    index: int          # 当前字节内位序 0..7（数据位）；ACK 位固定为 8
    value: int          # 0 或 1
    tick: int           # 采样时刻（SCL 上升沿）
    phase: BitPhase
    source: BitSource


@dataclass(frozen=True)
class ByteBlock:
    """完整的 8 数据位 + 1 ACK 位（9 个 SCL 脉冲）。

    流在字节中途截断时不会产生半截 ByteBlock：已采样的残余位保存在
    ``Segment.partial_bits`` 中。
    """

    phase: BitPhase             # ADDRESS（首字节）或 DATA
    value: int                  # 8 位拼成的无符号整数（MSB first）
    bits: tuple[Bit, ...]       # 恰好 8 个数据位
    ack: Bit                    # 第 9 位（0=ACK，1=NACK）
    start_tick: int             # 本字节第一个 SCL 脉冲的下降沿
    end_tick: int               # ACK 采样时刻


@dataclass(frozen=True)
class StartEvent:
    """START 或重复 START（SCL 高时 SDA 1->0）。"""

    tick: int
    repeated: bool


@dataclass(frozen=True)
class StopEvent:
    """STOP（SCL 高时 SDA 0->1）。"""

    tick: int


@dataclass(frozen=True)
class RecoveryRequest:
    """调用方的一次总线恢复请求。

    Attributes:
        tick: 请求生效的整数时刻；恢复驱动器从该时刻起接管 ``master`` 的线驱动
            （先释放 SDA）。该时刻必须存在至少一条外部更新（恢复状态机按有更新的
            时刻推进，不凭空生成时刻）。
        master: 执行恢复的驱动器标识；通常是被卡住事务的段主机。
        start_tick: 恢复前最近一次 SCL 下降沿时刻（可选，仅用于溯源）。
    """

    tick: int
    master: str
    start_tick: Optional[int] = None


@dataclass(frozen=True)
class RecoveryBit:
    """恢复期间某个**有效脉冲**上升沿采到的一位（单独标记，绝不拼入数据字节）。

    Attributes:
        pulse: 第几个有效脉冲，1..9（只有实测 SCL 升高才计数）。
        value: 采样时刻 SDA 的总线线值（0/1）。
        tick: 采样时刻（SCL 上升沿）。
        scl_low_tick: 该脉冲 SCL 下降沿时刻。
        sda_low_drivers: 采样时刻仍把 SDA 拉低的驱动器（含恢复主机若它在生成 STOP）。
    """

    pulse: int
    value: int
    tick: int
    scl_low_tick: int
    sda_low_drivers: tuple[str, ...]


@dataclass(frozen=True)
class RecoveryEvent:
    """恢复期间逐时刻的一条轨迹事件（用于逐事件核对实际线值与翻转来源）。

    恢复激活期间，每个有更新的整数时刻恰好产生一条事件；``scl``/``sda`` 是
    **合并所有驱动器（含恢复驱动器模型）之后**的总线实测线值。
    """

    tick: int
    kind: RecoveryEventKind
    scl: int                       # 合并后的实测 SCL（0/1）
    sda: int                       # 合并后的实测 SDA（0/1）
    scl_low_drivers: tuple[str, ...]
    sda_low_drivers: tuple[str, ...]
    pulse: Optional[int] = None    # kind=PULSE_COUNTED 时的有效脉冲序号
    note: str = ""


@dataclass(frozen=True)
class RecoveryRecord:
    """一次恢复操作的完整记录。

    Attributes:
        request: 对应的恢复请求。
        outcome: 恢复结局（生成/观察到 STOP、被其他主机 START 中止、bus_stuck）。
        events: 逐时刻轨迹（恢复激活的每个整数时刻一条）。
        bits: 各有效脉冲上升沿采到的位，按脉冲序号排列；这些位不属于任何字节。
        stop: 恢复成功结束时的 STOP 事件（生成或自然观察到）。
        pulses_sent: 实测 SCL 确实升高的脉冲数（0..9）；被别人拉住 SCL 的拍不计。
        stuck_tick: outcome=STUCK 时，第 9 个有效脉冲采样确认 SDA 仍低的时刻。
        aborted_by: outcome=ABORTED 时，发起 START 的其他主机。
        segment_index: 恢复前正在进行的段在其事务 segments 中的下标（无段为 None）。
    """

    request: RecoveryRequest
    outcome: RecoveryOutcome
    events: tuple[RecoveryEvent, ...]
    bits: tuple[RecoveryBit, ...]
    stop: Optional[StopEvent] = None
    pulses_sent: int = 0
    end_tick: Optional[int] = None
    stuck_tick: Optional[int] = None
    aborted_by: Optional[str] = None
    segment_index: Optional[int] = None

    @property
    def succeeded(self) -> bool:
        """是否以合法 STOP 结束（生成或自然观察到）。"""
        return self.outcome in (RecoveryOutcome.STOP_GENERATED,
                                RecoveryOutcome.STOP_OBSERVED)


@dataclass(frozen=True)
class ArbitrationLoss:
    """主机仲裁丢失事件：主机在某数据位期间释放 SDA（发送 1）但采样到 0。

    Attributes:
        tick: 检测时刻（SCL 上升沿采样时刻）。
        bit_index: 当前字节内位序（0..7；地址/RW/数据阶段均可能发生）。
        phase: 位所处阶段。
        master: 被判为“主机”的驱动器标识（当前唯一发过 START 的驱动器）。
        observed: 总线上实际采样值（恒为 0）。
        driven_low_by: 采样时刻把 SDA 拉低的其它驱动器。
    """

    tick: int
    bit_index: int
    phase: BitPhase
    master: str
    observed: int
    driven_low_by: tuple[str, ...]


@dataclass
class Segment:
    """一个“段”：一次 START（或重复 START）到下一个 rSTART/STOP/流末尾。

    START 可在事务内重复：重复 START 结束当前段（无 STOP）并在**同一事务**
    内开启新段，因此每个段都有自己的地址/RW 字节。
    """

    start: StartEvent
    starters: tuple[str, ...] = ()  # START 时刻把 SDA 拉低的驱动器（仲裁主机集合）
    blocks: list[ByteBlock] = field(default_factory=list)
    partial_bits: list[Bit] = field(default_factory=list)  # 末尾不足 8 位的残余位
    arbitration_losses: list[ArbitrationLoss] = field(default_factory=list)
    stop: Optional[StopEvent] = None
    ended_by_repeated_start: bool = False

    @property
    def closed(self) -> bool:
        """是否见到 STOP（被重复 START 切分不算关闭）。"""
        return self.stop is not None

    @property
    def master(self) -> Optional[str]:
        """本片段的发起驱动器（START 时刻 SDA 低驱动者中的首个）。"""
        return self.starters[0] if self.starters else None

    @property
    def address(self) -> Optional[int]:
        """7 位地址（首字节高 7 位）；无完整首字节时为 None。"""
        if not self.blocks:
            return None
        return self.blocks[0].value >> 1

    @property
    def rw(self) -> Optional[int]:
        """首字节最低位：0=写，1=读；无法解析时为 None。"""
        if not self.blocks:
            return None
        return self.blocks[0].value & 1

    @property
    def data(self) -> tuple[int, ...]:
        """首字节之后各完整数据字节的值（地址字节被排除）。"""
        return tuple(b.value for b in self.blocks[1:])

    @property
    def acks(self) -> tuple[int, ...]:
        """每个字节对应的 ACK 值（0=ACK，1=NACK）。"""
        return tuple(b.ack.value for b in self.blocks)


@dataclass
class Transaction:
    """一个事务：首个 START 到 STOP；重复 START 会产生多个 :class:`Segment`。"""

    segments: list[Segment]

    @property
    def closed(self) -> bool:
        """最后一个段是否见到 STOP（缺 STOP 的流末尾事务为 False）。"""
        return bool(self.segments) and self.segments[-1].closed


@dataclass(frozen=True)
class ParseResult:
    """整段更新流的解析结果。"""

    transactions: tuple[Transaction, ...]
    drivers: tuple[str, ...]
    final_scl: int
    final_sda: int
    span_ticks: int  # 最大时刻 - 最小时刻（用于校验回放，不代表严格时序）
    recoveries: tuple[RecoveryRecord, ...] = ()

    def all_segments(self) -> tuple[Segment, ...]:
        return tuple(s for t in self.transactions for s in t.segments)
