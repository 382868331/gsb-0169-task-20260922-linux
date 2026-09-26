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
    IterationResult,
    ParseResult,
    Segment,
    StartEvent,
    StopEvent,
    StretchEvent,
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


def _check_conflicts(items: list[Update]) -> None:
    """同一 (tick, driver) 只允许一条更新（批处理不得隐含二选一）。"""
    seen_key: dict[tuple[int, str], int] = {}
    for idx, u in enumerate(items):
        key = (u.tick, u.driver)
        if key in seen_key:
            raise ConflictingUpdateError(
                tick=u.tick, driver=u.driver, index=idx, first_index=seen_key[key]
            )
        seen_key[key] = idx


def _data_phase(byte_count: int, bit_in_byte: int) -> BitPhase:
    if byte_count == 0:
        return Phase.RW if bit_in_byte == 7 else Phase.ADDRESS
    return Phase.DATA


def _ack_phase(byte_count: int) -> BitPhase:
    return Phase.ACK_ADDR if byte_count == 0 else Phase.ACK_DATA


def parse_updates(updates: Iterable[Update], *, stretch_limit: int | None = None) -> ParseResult:
    """解析驱动器更新流，返回 :class:`ParseResult`；输入不可解析时抛错，不做部分输出。

    Args:
        updates: 按整数时刻非递减排列的 :class:`Update` 可迭代对象。
        stretch_limit: SCL 允许连续为低的 tick 数；严格超过（``duration > limit``）即抛
            :class:`StretchTimeoutError`。``None`` 表示不检查。
    """
    stretch_limit = require_stretch_limit(stretch_limit)
    items = _validate_updates(updates)
    _check_conflicts(items)

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
                cur_seg.ended_by_repeated_start = True
                new_seg = Segment(start=StartEvent(tick=tick, repeated=True),
                                  starters=starters_sorted)
                cur_tx.segments.append(new_seg)
                cur_seg = new_seg
            else:
                cur_seg = Segment(
                    start=StartEvent(tick=tick, repeated=False),
                    starters=starters_sorted,
                )
                cur_tx = Transaction(segments=[cur_seg])
                transactions.append(cur_tx)
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
        else:
            # ---- SCL 边沿（与 START/STOP 互斥，因后两者要求 SCL 保持高）----
            if pscl == _LOW and scl == _HIGH:
                # 上升沿：采样。
                if cur_seg is not None:
                    if bit_in_byte < 8:
                        phase = _data_phase(byte_count, bit_in_byte)
                        bit_index = bit_in_byte
                    else:
                        phase = _ack_phase(byte_count)
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
                        block = ByteBlock(
                            phase=Phase.ADDRESS if byte_count == 0 else Phase.DATA,
                            value=value,
                            bits=data_bits,
                            ack=ack_bit,
                            start_tick=data_bits[0].source.scl_low_tick,
                            end_tick=ack_bit.tick,
                        )
                        cur_seg.blocks.append(block)
                        if byte_count == 0:
                            seg_rw = value & 1  # 首字节完成，确定本段方向
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


def iterate_masters(
    intents: Iterable[Update], *, stretch_limit: int | None = None
) -> IterationResult:
    """两个（或多个）开漏主机逐拍驱动输入的仲裁迭代。

    输入中每个驱动器都被视为竞争主机，逐拍给出自己对 SCL/SDA 的主张
    （拉低或释放），总线电平取所有主机的线与：

    * 主机在**主机发送**的数据位（地址/RW 八位，以及写方向数据位）上
      释放 SDA（发 1），却在 SCL 上升沿的有效高电平采到 0 时，判它仲裁丢失；
      ACK 位（每字第 9 拍）不按数据位规则判决，读方向数据位也不判决；
    * 失败方从**下一驱动阶段**起被强制释放两条线：其后续的拉低主张不再
      作用于总线，因此无法继续制造 STOP 打断赢家；
    * 时钟低电平取所有主机共同释放后的结果（线与）；部分主机已释放而
      总线仍被拉低的区间记为 StretchEvent，与仲裁丢失分开记录；
    * START / 重复 START / STOP 的识别仍交给原解码逻辑：迭代层只合成
      实际总线更新流，最终事务由 parse_updates 解码得到。

    Args:
        intents: 各主机按整数时刻非递减排列的逐拍驱动主张（Update，
            driver 为主机标识）。至少需要两个不同主机。
        stretch_limit: 透传给 parse_updates 的 SCL 低电平阈值。

    Returns:
        IterationResult：赢家们、每个失败方的首次冲突位置、
        拉伸区间、实际总线更新流及赢家的解码事务。
    """
    items = _validate_updates(intents)
    _check_conflicts(items)

    masters: list[str] = []
    for u in items:
        if u.driver not in masters:
            masters.append(u.driver)
    if len(masters) < 2:
        raise InvalidUpdateError(
            field="driver",
            reason=f"仲裁迭代至少需要两个主机驱动器，实际 {len(masters)} 个",
        )

    state: dict[str, list[int]] = {m: [_HIGH, _HIGH] for m in masters}
    losers: dict[str, ArbitrationLoss] = {}  # 每主机只记首次冲突
    effective: list[Update] = []
    stretches: list[StretchEvent] = []
    open_stretch: list | None = None  # [start_tick, holders, released]

    pscl = psda = _HIGH
    in_seg = False
    bit_in_byte = 0
    byte_count = 0
    seg_rw: int | None = None
    cur_value = 0  # 当前字节已采样数据位（MSB first）的累计值

    def close_stretch(end_tick: int) -> None:
        nonlocal open_stretch
        if open_stretch is not None:
            start, holders, released = open_stretch
            stretches.append(
                StretchEvent(
                    start_tick=start, end_tick=end_tick,
                    holders=holders, released=released,
                )
            )
            open_stretch = None

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

        # 应用本拍主张：失败方自判决后的下一驱动阶段起强制释放。
        for u in group:
            if u.driver in losers:
                eff_scl = eff_sda = _HIGH
            else:
                eff_scl, eff_sda = u.scl.line_level, u.sda.line_level
            state[u.driver][0] = eff_scl
            state[u.driver][1] = eff_sda
            effective.append(
                Update(
                    tick=tick,
                    driver=u.driver,
                    scl=Drive.RELEASE if eff_scl else Drive.LOW,
                    sda=Drive.RELEASE if eff_sda else Drive.LOW,
                )
            )

        scl = min(state[m][0] for m in masters)
        sda = min(state[m][1] for m in masters)

        # ---- 位相位跟踪（只为仲裁判决服务；START/STOP 本身由解码层识别）----
        if pscl == _HIGH and scl == _HIGH and psda == _HIGH and sda == _LOW:
            in_seg = True
            bit_in_byte = 0
            byte_count = 0
            seg_rw = None
            cur_value = 0
        elif pscl == _HIGH and scl == _HIGH and psda == _LOW and sda == _HIGH:
            in_seg = False
        elif pscl == _LOW and scl == _HIGH and in_seg:
            # SCL 上升沿：有效高电平采样。
            master_sends = byte_count == 0 or seg_rw == 0
            if bit_in_byte < 8:
                if master_sends and sda == _LOW:
                    low_by = tuple(m for m in masters if state[m][1] == _LOW)
                    for m in masters:
                        # 发 1（已释放）却采到 0 => 该主机仲裁丢失。
                        if m not in losers and state[m][1] == _HIGH:
                            losers[m] = ArbitrationLoss(
                                tick=tick,
                                bit_index=bit_in_byte,
                                phase=_data_phase(byte_count, bit_in_byte),
                                master=m,
                                observed=sda,
                                driven_low_by=tuple(d for d in low_by if d != m),
                            )
                cur_value = (cur_value << 1) | sda
            # ACK 位（第 9 拍）不按数据位规则判决。
            bit_in_byte += 1
            if bit_in_byte == 9:
                if byte_count == 0:
                    seg_rw = cur_value & 1
                byte_count += 1
                bit_in_byte = 0
                cur_value = 0

        # ---- 时钟拉伸记录（与仲裁分开）：有人已释放而总线仍低 ----
        if scl == _LOW:
            holders = tuple(m for m in masters if state[m][0] == _LOW)
            # 已被判负的主机是强制释放，不算“被拖住”的在赛主机。
            released = tuple(
                m for m in masters if m not in losers and state[m][0] == _HIGH
            )
            if released:
                if open_stretch is not None and open_stretch[1] == holders:
                    pass  # 同一批持有者，区间延续
                else:
                    close_stretch(tick)
                    open_stretch = [tick, holders, released]
        else:
            close_stretch(tick)

        pscl, psda = scl, sda

    close_stretch(items[-1].tick)

    parsed = parse_updates(effective, stretch_limit=stretch_limit)
    winners = tuple(m for m in masters if m not in losers)
    return IterationResult(
        winners=winners,
        losses=tuple(losers[m] for m in masters if m in losers),
        stretch_events=tuple(stretches),
        effective_updates=tuple(effective),
        result=parsed,
    )
