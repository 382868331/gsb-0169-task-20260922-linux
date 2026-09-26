"""核心解码算法。

边沿触发模型（题目契约）：

* 每个驱动器在整数时刻给出它对 SCL/SDA 的**绝对**驱动状态（RELEASE=高阻/LOW=拉低）；
* 同一时刻的多条更新先按驱动器合并（线值 = 所有驱动器的 AND），再统一判断边沿；
* SCL 保持高电平时 SDA 高->低为 START，低->高为 STOP；
* SCL 低->高采样一位；9 个脉冲 = 8 数据位（MSB first）+ 1 ACK；
* 事务内重复 START 不产生 STOP，而是在同一事务内开启新段；
* 数据位（地址/RW/数据，不含 ACK）上：主机已释放 SDA（发 1）但线采样为 0 => 仲裁丢失；
* SCL 持续为低且 ``tick - low_since > stretch_limit`` => :class:`StretchTimeoutError`。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .errors import (
    ConflictingUpdateError,
    InvalidUpdateError,
    LimitError,
    StretchTimeoutError,
)
from .model import (
    ArbitrationLoss,
    Bit,
    BitPhase as Phase,
    BitSource,
    ByteBlock,
    Drive,
    ParseResult,
    Segment,
    StartEvent,
    StopEvent,
    Transaction,
    Update,
)
from .validation import (
    MAX_DRIVER_COUNT,
    MAX_TICK,
    MAX_UPDATE_COUNT,
    require_driver_id,
    require_stretch_limit,
    require_tick,
)

_HIGH = Drive.RELEASE.line_level  # 1
_LOW = Drive.LOW.line_level       # 0


@dataclass
class _PendingPulse:
    """一个进行中的 SCL 脉冲：下降沿信息，等待上升沿采样。"""

    fall_tick: int
    scl_low_drivers: tuple[str, ...]


class BusRecorder:
    """逐条录入 :class:`Update` 的有状态构造器。

    非法更新在 :meth:`add` 时即被拒绝且**不会**被录入（无部分变更）；
    同一驱动器同一时刻的重复更新在 :meth:`build` 时一次性检出。
    """

    def __init__(self) -> None:
        self._updates: list[Update] = []

    def add(self, tick: int, driver: str, scl: Drive, sda: Drive) -> "BusRecorder":
        idx = len(self._updates)
        tick = require_tick(tick, index=idx)
        driver = require_driver_id(driver, index=idx, tick=tick)
        self._check_drive(scl, "scl", idx, tick)
        self._check_drive(sda, "sda", idx, tick)
        if self._updates and tick < self._updates[-1].tick:
            raise InvalidUpdateError(
                field="tick",
                reason=(
                    f"时刻必须非递减：tick={tick} 早于上一条 "
                    f"tick={self._updates[-1].tick}"
                ),
                index=idx,
                tick=tick,
            )
        self._updates.append(Update(tick=tick, driver=driver, scl=scl, sda=sda))
        return self

    @staticmethod
    def _check_drive(value: object, field: str, idx: int, tick: int) -> None:
        if isinstance(value, bool) or not isinstance(value, Drive):
            raise InvalidUpdateError(
                field=field,
                reason=f"必须为 Drive.RELEASE 或 Drive.LOW，收到 {value!r}",
                index=idx,
                tick=tick,
            )

    def __len__(self) -> int:
        return len(self._updates)

    def build(self, stretch_limit: int | None = None) -> ParseResult:
        return parse_updates(self._updates, stretch_limit=stretch_limit)


def _validate_updates(updates: Iterable[Update]) -> list[Update]:
    result: list[Update] = []
    last_tick: int | None = None
    for idx, u in enumerate(updates):
        if idx >= MAX_UPDATE_COUNT:
            raise LimitError(name="更新条数", value=idx + 1, limit=MAX_UPDATE_COUNT)
        if not isinstance(u, Update):
            raise InvalidUpdateError(
                field="update",
                reason=f"必须为 odbus.Update，收到 {type(u).__name__}",
                index=idx,
            )
        tick = require_tick(u.tick, index=idx)
        driver = require_driver_id(u.driver, index=idx, tick=tick)
        scl = u.scl if isinstance(u.scl, Drive) and not isinstance(u.scl, bool) else None
        sda = u.sda if isinstance(u.sda, Drive) and not isinstance(u.sda, bool) else None
        if scl is None:
            raise InvalidUpdateError(
                field="scl", reason=f"必须为 Drive 枚举，收到 {u.scl!r}", index=idx, tick=tick
            )
        if sda is None:
            raise InvalidUpdateError(
                field="sda", reason=f"必须为 Drive 枚举，收到 {u.sda!r}", index=idx, tick=tick
            )
        if last_tick is not None and tick < last_tick:
            raise InvalidUpdateError(
                field="tick",
                reason=f"时刻必须非递减：tick={tick} 早于上一条 tick={last_tick}",
                index=idx,
                tick=tick,
            )
        last_tick = tick
        result.append(Update(tick=tick, driver=driver, scl=scl, sda=sda))
    return result


def _data_phase(segment: Segment, byte_count: int, bit_in_byte: int) -> BitPhase:
    if byte_count == 0:
        return Phase.RW if bit_in_byte == 7 else Phase.ADDRESS
    if (
        segment.address_mode == "10-bit"
        and byte_count >= 1
        and (
            (segment.rw == 0 and not segment.ten_bit_address_acknowledged)
            or (segment.rw == 1 and not segment.ten_bit_address_complete)
        )
    ):
        return Phase.ADDRESS_LOW
    return Phase.DATA


def _ack_phase(segment: Segment, byte_count: int) -> BitPhase:
    if byte_count == 0:
        return Phase.ACK_ADDR
    if (
        segment.address_mode == "10-bit"
        and byte_count >= 1
        and (
            (segment.rw == 0 and not segment.ten_bit_address_acknowledged)
            or (segment.rw == 1 and not segment.ten_bit_address_complete)
        )
    ):
        return Phase.ACK_ADDR_LOW
    return Phase.ACK_DATA


def parse_updates(updates: Iterable[Update], *, stretch_limit: int | None = None) -> ParseResult:
    """解析驱动器更新流，返回 :class:`ParseResult`；输入不可解析时抛错，不做部分输出。

    Args:
        updates: 按整数时刻非递减排列的 :class:`Update` 可迭代对象。
        stretch_limit: SCL 允许连续为低的 tick 数；严格超过（``duration > limit``）即抛
            :class:`StretchTimeoutError`。``None`` 表示不检查。
    """
    stretch_limit = require_stretch_limit(stretch_limit)
    items = _validate_updates(updates)

    # --- 冲突检测：同一 (tick, driver) 只允许一条更新 ---
    seen_key: dict[tuple[int, str], int] = {}
    for idx, u in enumerate(items):
        key = (u.tick, u.driver)
        if key in seen_key:
            raise ConflictingUpdateError(
                tick=u.tick, driver=u.driver, index=idx, first_index=seen_key[key]
            )
        seen_key[key] = idx

    # --- 驱动器状态（1=RELEASE，0=LOW），缺省为释放 ---
    state: dict[str, list[int]] = {}
    driver_order: list[str] = []

    def level_of(driver: str, line: int) -> int:
        return state[driver][line] if driver in state else _HIGH

    transactions: list[Transaction] = []
    cur_tx: Transaction | None = None
    cur_seg: Segment | None = None
    pending_bits: list[Bit] = []
    byte_count = 0
    bit_in_byte = 0          # 0..8：数据位计数；8 表示下一个脉冲是 ACK
    pulse: _PendingPulse | None = None
    seg_rw: int | None = None  # 当前段方向（首字节完成后确定）；读段数据位不查主机仲裁
    pending_ten_low: ByteBlock | None = None
    pending_ten_high: int | None = None
    pending_ten_write_header: ByteBlock | None = None

    pscl = psda = _HIGH
    scl_low_since: int | None = None

    n = len(items)
    i = 0
    while i < n:
        tick = items[i].tick
        group = [items[i]]
        j = i + 1
        while j < n and items[j].tick == tick:
            group.append(items[j])
            j += 1
        i = j

        # 合并同一时刻的更新（不同驱动器彼此独立；同驱动器重复已在上面拒绝）。
        starters: list[str] = []
        for u in group:
            if u.driver not in state:
                if len(driver_order) >= MAX_DRIVER_COUNT:
                    raise LimitError(
                        name="不同驱动器数量",
                        value=len(driver_order) + 1,
                        limit=MAX_DRIVER_COUNT,
                    )
                state[u.driver] = [_HIGH, _HIGH]
                driver_order.append(u.driver)
            old_scl, old_sda = state[u.driver]
            new_scl, new_sda = u.scl.line_level, u.sda.line_level
            state[u.driver][0] = new_scl
            state[u.driver][1] = new_sda
            # START 来源：本时刻 SDA 由释放变为拉低的驱动器（稍后按注册序排序）。
            if old_sda == _HIGH and new_sda == _LOW:
                starters.append(u.driver)

        scl = min((d[0] for d in state.values()), default=_HIGH)
        sda = min((d[1] for d in state.values()), default=_HIGH)
        starters_sorted = tuple(d for d in driver_order if d in starters)

        # 同一时刻合并后再判断边沿。START/STOP 要求 SCL 前后均为高。
        scl_high = pscl == _HIGH and scl == _HIGH
        if scl_high and psda == _HIGH and sda == _LOW:
            # ---- START（可能是重复 START）----
            # START 是帧复位：Sr 前未凑满 9 拍的残余采样位直接丢弃，
            # 不带入新段（真正的半截字节只在流末尾截断时保留为 partial_bits）。
            if cur_seg is not None:
                carried_high = None
                carried_low = None
                carried_header = None
                if (
                    cur_seg.address_mode == "10-bit"
                    and cur_seg.rw == 0
                    and not cur_seg.ten_bit_address_acknowledged
                    and pending_bits
                ):
                    cur_seg.partial_bits = list(pending_bits)
                if (
                    cur_seg.address_mode == "10-bit"
                    and cur_seg.rw == 0
                    and cur_seg.ten_bit_address_acknowledged
                ):
                    carried_high = cur_seg.ten_bit_high
                    carried_low = pending_ten_low
                    carried_header = cur_seg.blocks[0]
                cur_seg.ended_by_repeated_start = True
                new_seg = Segment(start=StartEvent(tick=tick, repeated=True),
                                  starters=starters_sorted)
                cur_tx.segments.append(new_seg)
                cur_seg = new_seg
                cur_seg.ten_bit_high = carried_high
                pending_ten_high = carried_high
                pending_ten_low = carried_low
                pending_ten_write_header = carried_header
            else:
                cur_seg = Segment(
                    start=StartEvent(tick=tick, repeated=False),
                    starters=starters_sorted,
                )
                cur_tx = Transaction(segments=[cur_seg])
                transactions.append(cur_tx)
                pending_ten_high = None
                pending_ten_low = None
                pending_ten_write_header = None
            pending_bits = []
            byte_count = 0
            bit_in_byte = 0
            seg_rw = None
            pulse = None
        elif scl_high and psda == _LOW and sda == _HIGH:
            # ---- STOP（干净边界：字节后的残余采样位不作为截断位保留）----
            if cur_seg is not None:
                cur_seg.stop = StopEvent(tick=tick)
            pending_bits = []
            byte_count = 0
            bit_in_byte = 0
            seg_rw = None
            pulse = None
            cur_seg = None
            cur_tx = None
            pending_ten_low = None
            pending_ten_high = None
            pending_ten_write_header = None
        else:
            # ---- SCL 边沿（与 START/STOP 互斥，因后两者要求 SCL 保持高）----
            if pscl == _LOW and scl == _HIGH:
                # 上升沿：采样。
                if cur_seg is not None:
                    if bit_in_byte < 8:
                        phase = _data_phase(cur_seg, byte_count, bit_in_byte)
                        bit_index = bit_in_byte
                    else:
                        phase = _ack_phase(cur_seg, byte_count)
                        bit_index = 8
                    sda_low = tuple(
                        d for d in driver_order if state[d][1] == _LOW
                    )
                    source = BitSource(
                        scl_low_tick=pulse.fall_tick if pulse else tick,
                        scl_high_tick=tick,
                        scl_low_drivers=(
                            pulse.scl_low_drivers if pulse else ()
                        ),
                        sda_low_drivers=sda_low,
                        drivers=tuple(driver_order),
                    )
                    bit = Bit(
                        index=bit_index, value=sda, tick=tick,
                        phase=phase, source=source,
                    )
                    pending_bits.append(bit)

                    # 仲裁丢失：主机发 1（已释放 SDA）却在**主机发送**的数据位读到 0。
                    # 地址/RW 的 8 位恒由主机发送；数据字节仅写方向（seg_rw==0）由主机发送，
                    # 读方向的数据位由从机驱动 SDA，主机释放是正常行为，不报仲裁丢失。
                    master = cur_seg.starters[0] if cur_seg.starters else None
                    master_sends = byte_count == 0 or seg_rw == 0
                    if (
                        master is not None
                        and bit_index < 8
                        and master_sends
                        and level_of(master, 1) == _HIGH
                        and sda == _LOW
                    ):
                        cur_seg.arbitration_losses.append(
                            ArbitrationLoss(
                                tick=tick,
                                bit_index=bit_index,
                                phase=phase,
                                master=master,
                                observed=sda,
                                driven_low_by=tuple(
                                    d for d in sda_low if d != master
                                ),
                            )
                        )

                    bit_in_byte += 1
                    if bit_in_byte == 9:
                        data_bits = tuple(pending_bits[:8])
                        value = 0
                        for b in data_bits:
                            value = (value << 1) | b.value
                        ack_bit = pending_bits[8]
                        block_phase = (
                            Phase.ADDRESS
                            if byte_count == 0
                            else _data_phase(cur_seg, byte_count, 0)
                        )
                        block = ByteBlock(
                            phase=block_phase,
                            value=value,
                            bits=data_bits,
                            ack=ack_bit,
                            start_tick=data_bits[0].source.scl_low_tick,
                            end_tick=ack_bit.tick,
                        )
                        cur_seg.blocks.append(block)
                        if byte_count == 0:
                            seg_rw = value & 1  # 首字节完成，确定本段方向
                            if (value >> 3) & 0x1F == 0b11110:
                                cur_seg.address_mode = "10-bit"
                                cur_seg.ten_bit_high = (value >> 1) & 0x03
                                if seg_rw == 1:
                                    if (
                                        pending_ten_write_header is not None
                                        and pending_ten_low is not None
                                        and pending_ten_low.ack.value == 0
                                        and pending_ten_high == cur_seg.ten_bit_high
                                    ):
                                        cur_seg.ten_bit_low = pending_ten_low.value
                                        cur_seg.ten_bit_address_acknowledged = (
                                            block.ack.value == 0
                                            and pending_ten_low.ack.value == 0
                                        )
                                        cur_seg.ten_bit_address_complete = (
                                            cur_seg.ten_bit_address_acknowledged
                                        )
                                        cur_seg.address_blocks = [
                                            pending_ten_write_header,
                                            pending_ten_low,
                                            block,
                                        ]
                                    else:
                                        cur_seg.ten_bit_address_complete = False
                                        cur_seg.ten_bit_address_acknowledged = False
                                        cur_seg.address_blocks = [block]
                                else:
                                    cur_seg.address_blocks = [block]
                            else:
                                cur_seg.address_mode = "7-bit"
                                cur_seg.ten_bit_address_complete = True
                                cur_seg.ten_bit_address_acknowledged = block.ack.value == 0
                                cur_seg.address_blocks = [block]
                        elif (
                            cur_seg.address_mode == "10-bit"
                            and cur_seg.rw == 0
                            and not cur_seg.ten_bit_address_acknowledged
                        ):
                            cur_seg.address_blocks.append(block)
                            if byte_count == 1:
                                cur_seg.ten_bit_low = value
                                cur_seg.ten_bit_address_acknowledged = (
                                    cur_seg.blocks[0].ack.value == 0
                                    and block.ack.value == 0
                                )
                                cur_seg.ten_bit_address_complete = (
                                    cur_seg.ten_bit_address_acknowledged
                                )
                                pending_ten_high = cur_seg.ten_bit_high
                                if cur_seg.ten_bit_address_acknowledged:
                                    pending_ten_low = block
                                    pending_ten_write_header = cur_seg.blocks[0]
                        elif (
                            cur_seg.address_mode == "10-bit"
                            and cur_seg.rw == 1
                            and not cur_seg.ten_bit_address_complete
                        ):
                            cur_seg.address_blocks.append(block)
                        pending_bits = []
                        byte_count += 1
                        bit_in_byte = 0
                # START 之前的孤立 SCL 脉冲不属于任何事务，忽略（见 README 已知限制）。
                pulse = None
            elif pscl == _HIGH and scl == _LOW:
                holders = tuple(
                    d for d in driver_order
                    if state[d][0] == _LOW
                )
                pulse = _PendingPulse(fall_tick=tick, scl_low_drivers=holders)

        # ---- 时钟拉伸检查 ----
        # 每个有更新的时刻都检查：低电平持续期间超阈值会立刻报错；
        # 低电平期间没有任何更新、直到上升沿才超阈值时，在上升沿补检。
        if stretch_limit is not None and scl_low_since is not None:
            duration = tick - scl_low_since
            if duration > stretch_limit:
                raise StretchTimeoutError(
                    tick=tick,
                    low_since=scl_low_since,
                    duration=duration,
                    limit=stretch_limit,
                )
        if scl == _HIGH:
            scl_low_since = None
        elif pscl == _HIGH:
            scl_low_since = tick

        pscl, psda = scl, sda

    # ---- 流末尾：未见 STOP 的事务保持未关闭（缺 STOP 是合法输出而非异常）----
    if cur_seg is not None:
        cur_seg.partial_bits = list(pending_bits)

    return ParseResult(
        transactions=tuple(transactions),
        drivers=tuple(driver_order),
        final_scl=pscl,
        final_sda=psda,
        span_ticks=(items[-1].tick - items[0].tick) if items else 0,
    )
