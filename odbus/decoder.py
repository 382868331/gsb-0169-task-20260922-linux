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


class IncrementalDecoder:
    """分批提供更新的增量识别器。

    总线事件有时只能分批到达：调用方反复 :meth:`feed` 若干批更新，
    识别器在内部保留当前未完成的事务状态，下一批到来后继续识别；
    全部输入结束后调用 :meth:`finish` 取得 :class:`ParseResult`。

    语义保证：

    * 分批结果与一次性 :func:`parse_updates` **完全一致**——批次边界即使落在
      START / ACK 位 / STOP 附近，也不改变识别结果；
    * **同一时刻的更新收齐后才采样**：每批最后一个时刻的分组会被暂时挂起，
      直到出现更晚的时刻（或 `finish`）才合并判断边沿，避免把同刻分批到达
      误认为先后发生；
    * `finish` 是调用方对“输入已结束”的显式声明：此时仍未见 STOP 的事务
      按未完成返回（`closed=False`、残余位进 `partial_bits`），
      **不会**补出一个停止条件；
    * 跨批次同样强制：时刻非递减、同一 `(tick, driver)` 只允许一条更新；
      非法批次整体拒绝，不留下未说明的部分变更；
    * `finish` 之后不允许再 `feed`；`finish` 可重复调用并返回同一结果。
    """

    def __init__(self, *, stretch_limit: int | None = None) -> None:
        self._stretch_limit = require_stretch_limit(stretch_limit)
        self._core = _Core(self._stretch_limit)
        # 已收到但尚未确定“同刻更新已收齐”的最后时刻分组。
        self._pending_tick: int | None = None
        self._pending: list[Update] = []
        self._pending_first_index: dict[str, int] = {}
        self._total = 0  # 累计录入条数（用于跨批次错误定位）
        self._finished = False
        self._result: ParseResult | None = None

    def feed(self, updates: Iterable[Update]) -> "IncrementalDecoder":
        """录入一批更新；批次内/跨批次校验失败时整批拒绝，状态不变。"""
        if self._finished:
            raise InvalidUpdateError(
                field="updates",
                reason="finish() 已声明输入结束，不允许继续 feed",
            )
        items = _validate_updates(updates)
        # 先整体校验本批与挂起分组的衔接，再逐条应用，避免部分变更。
        # seen_* 跟踪“当前时刻已见驱动器”：跨批次时以挂起分组为种子，
        # 批内时刻前进后重置（时刻非递减，更早时刻不可能再出现）。
        seen_tick = self._pending_tick
        seen: dict[str, int] = dict(self._pending_first_index)
        base = self._total
        for offset, u in enumerate(items):
            idx = base + offset
            if seen_tick is not None and u.tick < seen_tick:
                raise InvalidUpdateError(
                    field="tick",
                    reason=(
                        f"时刻必须非递减（跨批次）：tick={u.tick} 早于上一批末尾 "
                        f"tick={seen_tick}"
                    ),
                    index=idx,
                    tick=u.tick,
                )
            if u.tick != seen_tick:
                seen_tick = u.tick
                seen = {}
            if u.driver in seen:
                raise ConflictingUpdateError(
                    tick=u.tick,
                    driver=u.driver,
                    index=idx,
                    first_index=seen[u.driver],
                )
            seen[u.driver] = idx
        self._total += len(items)
        # 校验通过，逐条应用：同刻追加到挂起分组；更晚时刻先冲刷旧分组。
        for offset, u in enumerate(items):
            if self._pending_tick is None or u.tick > self._pending_tick:
                self._flush_pending()
                self._pending_tick = u.tick
            self._pending_first_index.setdefault(u.driver, base + offset)
            self._pending.append(u)
        return self

    def _flush_pending(self) -> None:
        if self._pending_tick is not None and self._pending:
            self._core.process_group(self._pending_tick, self._pending)
        self._pending_tick = None
        self._pending = []
        self._pending_first_index = {}

    def finish(self) -> ParseResult:
        """声明输入结束并返回解析结果；未完成事务按未完成返回，不补 STOP。"""
        if self._finished:
            assert self._result is not None
            return self._result
        self._flush_pending()
        self._result = self._core.finalize()
        self._finished = True
        return self._result


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

    # --- 冲突检测：同一 (tick, driver) 只允许一条更新 ---
    seen_key: dict[tuple[int, str], int] = {}
    for idx, u in enumerate(items):
        key = (u.tick, u.driver)
        if key in seen_key:
            raise ConflictingUpdateError(
                tick=u.tick, driver=u.driver, index=idx, first_index=seen_key[key]
            )
        seen_key[key] = idx

    core = _Core(stretch_limit)
    n = len(items)
    i = 0
    while i < n:
        tick = items[i].tick
        j = i + 1
        while j < n and items[j].tick == tick:
            j += 1
        core.process_group(tick, items[i:j])
        i = j
    return core.finalize()


class _Core:
    """逐“时刻分组”推进的解码状态机（`parse_updates` 与增量识别器共用）。"""

    def __init__(self, stretch_limit: int | None) -> None:
        self.stretch_limit = stretch_limit
        # 驱动器状态（1=RELEASE，0=LOW），缺省为释放。
        self.state: dict[str, list[int]] = {}
        self.driver_order: list[str] = []
        self.transactions: list[Transaction] = []
        self.cur_tx: Transaction | None = None
        self.cur_seg: Segment | None = None
        self.pending_bits: list[Bit] = []
        self.byte_count = 0
        self.bit_in_byte = 0  # 0..8：数据位计数；8 表示下一个脉冲是 ACK
        self.pulse: _PendingPulse | None = None
        self.seg_rw: int | None = None  # 当前段方向；读段数据位不查主机仲裁
        self.pscl = _HIGH
        self.psda = _HIGH
        self.scl_low_since: int | None = None
        self.first_tick: int | None = None
        self.last_tick: int | None = None

    def level_of(self, driver: str, line: int) -> int:
        return self.state[driver][line] if driver in self.state else _HIGH

    def process_group(self, tick: int, group: list[Update]) -> None:
        """合并应用同一时刻的全部更新，再统一判断边沿/采样/超时。"""
        state = self.state
        driver_order = self.driver_order
        cur_tx = self.cur_tx
        cur_seg = self.cur_seg
        pending_bits = self.pending_bits
        byte_count = self.byte_count
        bit_in_byte = self.bit_in_byte
        pulse = self.pulse
        seg_rw = self.seg_rw
        pscl = self.pscl
        psda = self.psda
        scl_low_since = self.scl_low_since
        stretch_limit = self.stretch_limit

        if self.first_tick is None:
            self.first_tick = tick
        self.last_tick = tick

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
                self.transactions.append(cur_tx)
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
                        and self.level_of(master, 1) == _HIGH
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

        # 回写状态（cur_tx/cur_seg 为可变对象，引用不变时无需回写，但 STOP 会置 None）。
        self.cur_tx = cur_tx
        self.cur_seg = cur_seg
        self.pending_bits = pending_bits
        self.byte_count = byte_count
        self.bit_in_byte = bit_in_byte
        self.pulse = pulse
        self.seg_rw = seg_rw
        self.pscl = scl
        self.psda = sda
        self.scl_low_since = scl_low_since

    def finalize(self) -> ParseResult:
        # ---- 流末尾：未见 STOP 的事务保持未关闭（缺 STOP 是合法输出而非异常）----
        if self.cur_seg is not None:
            self.cur_seg.partial_bits = list(self.pending_bits)
        first = self.first_tick
        last = self.last_tick
        return ParseResult(
            transactions=tuple(self.transactions),
            drivers=tuple(self.driver_order),
            final_scl=self.pscl,
            final_sda=self.psda,
            span_ticks=(last - first) if first is not None and last is not None else 0,
        )
