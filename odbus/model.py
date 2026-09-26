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


@dataclass
class TenBitAddress:
    """题内十位寻址形式的地址状态（首字节前缀 ``11110``）。

    首字节 = ``11110`` + 地址高两位 + 方向位；写方向（R/W=0）之后再接
    地址低八位。读方向（R/W=1）只发读头，其完整地址必须在**同一事务**内
    此前已完成的写地址阶段（写头与低八位均收到且均被 ACK）中匹配。

    Attributes:
        high: 读头/写头中携带的地址高两位（0..3）。
        low: 地址低八位。写头段来自其后的第二个字节；读头段在匹配成功时
            取自同事务已完成的写地址阶段；地址未收全/未匹配时为 None。
        read_header: True 表示本段首字节是读头（R/W=1）。
        complete: 完整十位地址是否已收全且各地址字节均被 ACK
            （读头段还要求与先前写地址阶段高位匹配）。
        matched: 读头是否匹配同事务内已完成的写地址阶段；写头段恒为 True。
        header_block: 读头/写头字节块。
        low_block: 低八位地址字节块；尚未收到时为 None。
    """

    high: int
    low: Optional[int]
    read_header: bool
    complete: bool
    matched: bool
    header_block: ByteBlock
    low_block: Optional[ByteBlock] = None

    @property
    def value(self) -> Optional[int]:
        """完整十位地址；未收全或读头未匹配时为 None。"""
        if not self.complete or self.low is None:
            return None
        return (self.high << 8) | self.low


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
    ten_bit: Optional[TenBitAddress] = None  # 十位寻址状态；七位寻址时为 None

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
        """段地址。

        七位寻址返回首字节高 7 位；十位寻址在地址收全（读头另需匹配同事务
        已完成的写地址阶段）后返回完整十位地址，未收全时为 None；
        无完整首字节时为 None。
        """
        if not self.blocks:
            return None
        if self.addressing == "ten":
            return self.ten_bit.value if self.ten_bit is not None else None
        return self.blocks[0].value >> 1

    @property
    def addressing(self) -> Optional[str]:
        """寻址形式：``"seven"``（七位）/ ``"ten"``（十位）；无首字节为 None。"""
        if not self.blocks:
            return None
        return "ten" if (self.blocks[0].value & 0xF8) == 0xF0 else "seven"

    @property
    def rw(self) -> Optional[int]:
        """首字节最低位：0=写，1=读；无法解析时为 None。"""
        if not self.blocks:
            return None
        return self.blocks[0].value & 1

    @property
    def data(self) -> tuple[int, ...]:
        """各完整数据字节的值（全部地址字节被排除）。

        七位寻址排除首字节；十位写寻址在地址收全后排除写头与低八位两个字节；
        十位读头匹配成功后排除读头字节。地址未收全（含中途 NACK）或读头未
        匹配时，后续字节一律不当作数据，返回空元组。
        """
        if not self.blocks:
            return ()
        if self.addressing == "ten":
            tb = self.ten_bit
            if tb is None or not tb.complete:
                return ()
            return tuple(b.value for b in self.blocks[1:]) if tb.read_header \
                else tuple(b.value for b in self.blocks[2:])
        return tuple(b.value for b in self.blocks[1:])

    @property
    def address_blocks(self) -> tuple[ByteBlock, ...]:
        """地址字节块（按顺序）。

        七位寻址仅首字节；十位写寻址为写头 + 已收到的低八位；十位读头为读头。
        各地址字节的 ACK 来源位置可通过 ``block.ack.source`` 取得。
        """
        if not self.blocks:
            return ()
        if self.addressing != "ten":
            return (self.blocks[0],)
        blocks = [self.blocks[0]]
        if self.ten_bit is not None and self.ten_bit.low_block is not None:
            blocks.append(self.ten_bit.low_block)
        return tuple(blocks)

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

    def all_segments(self) -> tuple[Segment, ...]:
        return tuple(s for t in self.transactions for s in t.segments)
