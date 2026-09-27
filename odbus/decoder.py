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

import copy
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


class StreamingBusDecoder:
    """分批接收总线更新的有状态识别器。

    每批的最后一个时刻会被暂存，直到下一批出现更晚时刻或调用
    :meth:`finish`。因此同一时刻的更新即使跨越批次边界，也会在全部收齐后
    一次性合并、判断边沿。:meth:`finish` 只表示输入结束，不会补造 STOP。
    """

    def __init__(self, stretch_limit: int | None = None) -> None:
        self._stretch_limit = require_stretch_limit(stretch_limit)
        self._buffered: list[Update] = []
        self._finished = False

        self.state: dict[str, list[int]] = {}
        self.driver_order: list[str] = []
        self.transactions: list[Transaction] = []
        self.cur_tx: Transaction | None = None
        self.cur_seg: Segment | None = None
        self.pending_bits: list[Bit] = []
        self.byte_count = 0
        self.bit_in_byte = 0
        self.pulse: _PendingPulse | None = None
        self.seg_rw: int | None = None
        self.pscl = self.psda = _HIGH
        self.scl_low_since: int | None = None
        self.first_tick: int | None = None
        self.last_tick: int | None = None
        self._update_count = 0

    def feed(self, updates: Iterable[Update]) -> "StreamingBusDecoder":
        """送入一批更新；成功后返回本识别器以便链式调用。"""
        if self._finished:
            raise InvalidUpdateError(
                field="batch",
                reason="识别器已经 finish，不能继续送入更新",
                index=0,
            )

        incoming = _validate_updates(updates)
        combined = self._buffered + incoming
        if self._update_count + len(combined) > MAX_UPDATE_COUNT:
            raise LimitError(
                name="更新条数",
                value=self._update_count + len(combined),
                limit=MAX_UPDATE_COUNT,
            )
        if (
            self.last_tick is not None
            and combined
            and combined[0].tick < self.last_tick
        ):
            raise InvalidUpdateError(
                field="tick",
                reason=(
                    f"时刻必须非递减：tick={combined[0].tick} 早于已处理 "
                    f"tick={self.last_tick}"
                ),
                index=0,
                tick=combined[0].tick,
            )
        combined = _validate_updates(combined)
        self._check_conflicts(combined)
        if len(self.state) + self._incoming_driver_count(combined) > MAX_DRIVER_COUNT:
            raise LimitError(
                name="不同驱动器数量",
                value=len(self.state) + self._incoming_driver_count(combined),
                limit=MAX_DRIVER_COUNT,
            )

        groups = list(_group_by_tick(combined))
        if not groups:
            return self

        snapshot = copy.deepcopy(self.__dict__)
        complete = groups[:-1]
        try:
            self._buffered = list(groups[-1][1])
            for tick, group in complete:
                self._process_tick(tick, group)
                self._update_count += len(group)
        except Exception:
            self.__dict__.update(snapshot)
            raise
        return self

    def finish(self) -> ParseResult:
        """声明输入结束，冲刷最后一个暂存时刻并返回当前识别结果。"""
        if not self._finished:
            pending = self._buffered
            pending = _validate_updates(pending)
            self._check_conflicts(pending)
            if self._update_count + len(pending) > MAX_UPDATE_COUNT:
                raise LimitError(
                    name="更新条数",
                    value=self._update_count + len(pending),
                    limit=MAX_UPDATE_COUNT,
                )
            snapshot = copy.deepcopy(self.__dict__)
            try:
                for tick, group in _group_by_tick(pending):
                    self._process_tick(tick, group)
                    self._update_count += len(group)
                self._buffered = []
                self._finished = True
            except Exception:
                self.__dict__.update(snapshot)
                raise
        return self.result()

    def result(self) -> ParseResult:
        """返回当前已识别内容；未结束事务保持未关闭状态。"""
        if self.cur_seg is not None:
            self.cur_seg.partial_bits = list(self.pending_bits)
        return ParseResult(
            transactions=tuple(self.transactions),
            drivers=tuple(self.driver_order),
            final_scl=self.pscl,
            final_sda=self.psda,
            span_ticks=(self.last_tick - self.first_tick) if self.first_tick is not None else 0,
        )

    def _incoming_driver_count(self, updates: list[Update]) -> int:
        return len({u.driver for u in updates if u.driver not in self.state})

    @staticmethod
    def _check_conflicts(updates: list[Update]) -> None:
        seen_key: dict[tuple[int, str], int] = {}
        for idx, u in enumerate(updates):
            key = (u.tick, u.driver)
            if key in seen_key:
                raise ConflictingUpdateError(
                    tick=u.tick,
                    driver=u.driver,
                    index=idx,
                    first_index=seen_key[key],
                )
            seen_key[key] = idx

    def _level_after(self, group: list[Update], line: int) -> int:
        pending_levels: dict[str, int] = {}
        for update in group:
            pending_levels[update.driver] = (
                update.scl if line == 0 else update.sda
            ).line_level
        level = _HIGH
        for driver in self.driver_order:
            level = min(level, pending_levels.get(driver, self.state[driver][line]))
        for driver, pending_level in pending_levels.items():
            if driver not in self.state:
                level = min(level, pending_level)
        return level

    def _process_tick(self, tick: int, group: list[Update]) -> None:
        new_scl = self._level_after(group, 0)
        new_sda = self._level_after(group, 1)

        if (
            self._stretch_limit is not None
            and self.scl_low_since is not None
            and tick - self.scl_low_since > self._stretch_limit
        ):
            raise StretchTimeoutError(
                tick=tick,
                low_since=self.scl_low_since,
                duration=tick - self.scl_low_since,
                limit=self._stretch_limit,
            )

        starters: list[str] = []
        for u in group:
            if u.driver not in self.state:
                self.state[u.driver] = [_HIGH, _HIGH]
                self.driver_order.append(u.driver)
            old_scl, old_sda = self.state[u.driver]
            self.state[u.driver][0] = u.scl.line_level
            self.state[u.driver][1] = u.sda.line_level
            if old_sda == _HIGH and u.sda.line_level == _LOW:
                starters.append(u.driver)

        scl, sda = new_scl, new_sda
        starters_sorted = tuple(d for d in self.driver_order if d in starters)
        self._handle_edge(tick, scl, sda, starters_sorted)

        if scl == _HIGH:
            self.scl_low_since = None
        elif self.pscl == _HIGH:
            self.scl_low_since = tick

        self.pscl, self.psda = scl, sda
        if self.first_tick is None:
            self.first_tick = tick
        self.last_tick = tick

    def _handle_edge(
        self,
        tick: int,
        scl: int,
        sda: int,
        starters_sorted: tuple[str, ...],
    ) -> None:
        pscl, psda = self.pscl, self.psda
        scl_high = pscl == _HIGH and scl == _HIGH

        if scl_high and psda == _HIGH and sda == _LOW:
            if self.cur_seg is not None:
                self.cur_seg.partial_bits = []
                self.cur_seg.ended_by_repeated_start = True
                new_seg = Segment(
                    start=StartEvent(tick=tick, repeated=True),
                    starters=starters_sorted,
                )
                self.cur_tx.segments.append(new_seg)
                self.cur_seg = new_seg
            else:
                self.cur_seg = Segment(
                    start=StartEvent(tick=tick, repeated=False),
                    starters=starters_sorted,
                )
                self.cur_tx = Transaction(segments=[self.cur_seg])
                self.transactions.append(self.cur_tx)
            self.pending_bits = []
            self.cur_seg.partial_bits = []
            self.byte_count = 0
            self.bit_in_byte = 0
            self.seg_rw = None
            self.pulse = None
            return

        if scl_high and psda == _LOW and sda == _HIGH:
            if self.cur_seg is not None:
                self.cur_seg.partial_bits = []
                self.cur_seg.stop = StopEvent(tick=tick)
            self.pending_bits = []
            self.byte_count = 0
            self.bit_in_byte = 0
            self.seg_rw = None
            self.pulse = None
            self.cur_seg = None
            self.cur_tx = None
            return

        if pscl == _LOW and scl == _HIGH:
            self._sample_rise(tick, sda)
            self.pulse = None
        elif pscl == _HIGH and scl == _LOW:
            holders = tuple(
                d for d in self.driver_order if self.state[d][0] == _LOW
            )
            self.pulse = _PendingPulse(fall_tick=tick, scl_low_drivers=holders)

    def _sample_rise(self, tick: int, sda: int) -> None:
        if self.cur_seg is None:
            return

        if self.bit_in_byte < 8:
            phase = _data_phase(self.byte_count, self.bit_in_byte)
            bit_index = self.bit_in_byte
        else:
            phase = _ack_phase(self.byte_count)
            bit_index = 8

        sda_low = tuple(d for d in self.driver_order if self.state[d][1] == _LOW)
        source = BitSource(
            scl_low_tick=self.pulse.fall_tick if self.pulse else tick,
            scl_high_tick=tick,
            scl_low_drivers=self.pulse.scl_low_drivers if self.pulse else (),
            sda_low_drivers=sda_low,
            drivers=tuple(self.driver_order),
        )
        bit = Bit(
            index=bit_index,
            value=sda,
            tick=tick,
            phase=phase,
            source=source,
        )
        self.pending_bits.append(bit)

        master = self.cur_seg.starters[0] if self.cur_seg.starters else None
        master_sends = self.byte_count == 0 or self.seg_rw == 0
        if (
            master is not None
            and bit_index < 8
            and master_sends
            and self.state[master][1] == _HIGH
            and sda == _LOW
        ):
            self.cur_seg.arbitration_losses.append(
                ArbitrationLoss(
                    tick=tick,
                    bit_index=bit_index,
                    phase=phase,
                    master=master,
                    observed=sda,
                    driven_low_by=tuple(d for d in sda_low if d != master),
                )
            )

        self.bit_in_byte += 1
        if self.bit_in_byte != 9:
            return

        data_bits = tuple(self.pending_bits[:8])
        value = 0
        for sampled in data_bits:
            value = (value << 1) | sampled.value
        ack_bit = self.pending_bits[8]
        block = ByteBlock(
            phase=Phase.ADDRESS if self.byte_count == 0 else Phase.DATA,
            value=value,
            bits=data_bits,
            ack=ack_bit,
            start_tick=data_bits[0].source.scl_low_tick,
            end_tick=ack_bit.tick,
        )
        self.cur_seg.blocks.append(block)
        if self.byte_count == 0:
            self.seg_rw = value & 1
        self.pending_bits = []
        self.cur_seg.partial_bits = []
        self.byte_count += 1
        self.bit_in_byte = 0


def _group_by_tick(items: list[Update]) -> Iterable[tuple[int, list[Update]]]:
    if not items:
        return
    tick = items[0].tick
    group = [items[0]]
    for update in items[1:]:
        if update.tick != tick:
            yield tick, group
            tick = update.tick
            group = []
        group.append(update)
    yield tick, group


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
    """一次性解析完整更新流；失败时整体抛错，不返回部分结果。"""
    decoder = StreamingBusDecoder(stretch_limit=stretch_limit)
    decoder.feed(list(updates))
    return decoder.finish()
