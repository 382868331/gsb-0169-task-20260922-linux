"""核心解码算法。

边沿触发模型（题目契约）：

* 每个驱动器在整数时刻给出它对 SCL/SDA 的**绝对**驱动状态（RELEASE=高阻/LOW=拉低）；
* 同一时刻的多条更新先按驱动器合并（线值 = 所有驱动器的 AND），再统一判断边沿；
* SCL 保持高电平时 SDA 高->低为 START，低->高为 STOP；
* SCL 低->高采样一位；9 个脉冲 = 8 数据位（MSB first）+ 1 ACK；
* 事务内重复 START 不产生 STOP，而是在同一事务内开启新段；
* 数据位（地址/RW/数据，不含 ACK）上：主机已释放 SDA（发 1）但线采样为 0 => 仲裁丢失；
* SCL 持续为低且 ``tick - low_since > stretch_limit`` => :class:`StretchTimeoutError`。

总线恢复（驱动器模型，``recovery=RecoveryRequest(...)``）：

* 从请求时刻起，主机由恢复状态机接管（该主机的输入更新在恢复期间被拒绝）；
* 先释放 SDA，再最多发 ``max_pulses`` 个 SCL 脉冲；**只有真正观察到 SCL 升高
  才计数**，被他人拉住时主机保持释放并等待，等待期间沿用同一个 stretch_timeout
  检查，绝不按计划时刻假装脉冲成功；
* 有效脉冲后采到 SDA 为高 => 走 “SCL 低 + SDA 低 -> 放 SCL -> 放 SDA” 生成合法
  STOP；第 ``max_pulses`` 个脉冲后 SDA 仍低 => :class:`BusStuckError`；
* 恢复期间其他主机的 START 边沿成立 => 立即中止（ABORTED），正常识别继续；
* 恢复期间采到的位记入 ``RecoveryRecord.bits``（``BitPhase.RECOVERY``），
  不拼进之前没收完的数据字节。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .errors import (
    BusStuckError,
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
    LineSample,
    ParseResult,
    RecoveryOutcome,
    RecoveryPulse,
    RecoveryRecord,
    RecoveryRequest,
    Segment,
    StartEvent,
    StopEvent,
    Transaction,
    Update,
)
from .validation import (
    MAX_DRIVER_COUNT,
    MAX_RECOVERY_PULSES,
    MAX_TICK,
    MAX_UPDATE_COUNT,
    require_driver_id,
    require_stretch_limit,
    require_tick,
)

_HIGH = Drive.RELEASE.line_level  # 1
_LOW = Drive.LOW.line_level       # 0

# 恢复状态机阶段
_REC_RELEASE_SDA = "release_sda"  # 主机已释放 SDA，观察总线 SDA
_REC_PULSE_LOW = "pulse_low"      # 主机拉低 SCL（脉冲低半周）
_REC_WAIT_HIGH = "wait_high"      # 主机已释放 SCL，等待真正观察到升高
_REC_STOP_LOW = "stop_low"        # STOP 生成：SCL 低 + SDA 低
_REC_STOP_RISE = "stop_rise"      # STOP 生成：释放 SCL，等待升高
_REC_STOP_SDA = "stop_sda"        # STOP 生成：释放 SDA（SCL 高时 SDA 升 => STOP）


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

    def build(
        self,
        stretch_limit: int | None = None,
        *,
        recovery: RecoveryRequest | None = None,
        trace: bool = False,
    ) -> ParseResult:
        return parse_updates(
            self._updates, stretch_limit=stretch_limit, recovery=recovery, trace=trace
        )


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


def _validate_recovery(recovery: object) -> RecoveryRequest | None:
    if recovery is None:
        return None
    if isinstance(recovery, bool) or not isinstance(recovery, RecoveryRequest):
        raise InvalidUpdateError(
            field="recovery",
            reason=f"必须为 odbus.RecoveryRequest，收到 {type(recovery).__name__}",
        )
    host = require_driver_id(recovery.host)
    tick = require_tick(recovery.tick)
    mp = recovery.max_pulses
    if isinstance(mp, bool) or not isinstance(mp, int):
        raise InvalidUpdateError(
            field="max_pulses",
            reason=f"必须为非布尔 int，收到 {mp!r}",
        )
    if mp < 1 or mp > MAX_RECOVERY_PULSES:
        raise LimitError(name="max_pulses", value=mp, limit=MAX_RECOVERY_PULSES)
    return RecoveryRequest(host=host, tick=tick, max_pulses=mp)


def _data_phase(byte_count: int, bit_in_byte: int) -> BitPhase:
    if byte_count == 0:
        return Phase.RW if bit_in_byte == 7 else Phase.ADDRESS
    return Phase.DATA


def _ack_phase(byte_count: int) -> BitPhase:
    return Phase.ACK_ADDR if byte_count == 0 else Phase.ACK_DATA


def parse_updates(
    updates: Iterable[Update],
    *,
    stretch_limit: int | None = None,
    recovery: RecoveryRequest | None = None,
    trace: bool = False,
) -> ParseResult:
    """解析驱动器更新流，返回 :class:`ParseResult`；输入不可解析时抛错，不做部分输出。

    Args:
        updates: 按整数时刻非递减排列的 :class:`Update` 可迭代对象。
        stretch_limit: SCL 允许连续为低的 tick 数；严格超过（``duration > limit``）即抛
            :class:`StretchTimeoutError`。``None`` 表示不检查。
        recovery: 可选的 :class:`RecoveryRequest`。给出后，从请求时刻起主机由恢复
            驱动器模型接管：先释放 SDA，再最多发 ``max_pulses`` 个 SCL 脉冲
            （必须真正观察到 SCL 升高才计数），SDA 释放后生成合法 STOP；
            打满脉冲仍被拉低抛 :class:`BusStuckError`；其他主机 START 成立时中止。
        trace: 为 True 时在 ``ParseResult.trace`` 中输出逐事件合并后的线值。
    """
    stretch_limit = require_stretch_limit(stretch_limit)
    rec_req = _validate_recovery(recovery)
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

    pscl = psda = _HIGH
    scl_low_since: int | None = None

    # --- 恢复状态机 ---
    recoveries: list[RecoveryRecord] = []
    rec_phase: str | None = None
    rec_action_tick: int | None = None   # 下一个主机动作时刻（None=无待执行动作）
    rec_record: RecoveryRecord | None = None
    rec_started = False                  # 请求时刻已到、首个动作已应用
    rec_pulse_fall = 0
    rec_pulse_stretched = False
    if rec_req is not None:
        rec_record = RecoveryRecord(
            host=rec_req.host,
            request_tick=rec_req.tick,
            max_pulses=rec_req.max_pulses,
        )
        rec_phase = _REC_RELEASE_SDA
        rec_action_tick = rec_req.tick

    samples: list[LineSample] = []

    n = len(items)
    i = 0
    while True:
        next_ext = items[i].tick if i < n else None
        if next_ext is None and rec_action_tick is None:
            break
        if next_ext is None:
            tick = rec_action_tick
        elif rec_action_tick is None:
            tick = next_ext
        else:
            tick = min(next_ext, rec_action_tick)

        group: list[tuple[int, Update]] = []
        while i < n and items[i].tick == tick:
            group.append((i, items[i]))
            i += 1

        # 合并同一时刻的更新（不同驱动器彼此独立；同驱动器重复已在上面拒绝）。
        starters: list[str] = []
        for idx, u in group:
            if (
                rec_phase is not None
                and u.driver == rec_record.host
                and u.tick >= rec_record.request_tick
            ):
                raise InvalidUpdateError(
                    field="driver",
                    reason="恢复进行期间主机由恢复模型驱动，不接受该驱动器的输入更新",
                    index=idx,
                    tick=u.tick,
                    driver=u.driver,
                )
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

        # --- 恢复主机动作（与外部更新同刻合并后再判边沿）---
        if rec_phase is not None and rec_action_tick == tick:
            host = rec_record.host
            if host not in state:
                if len(driver_order) >= MAX_DRIVER_COUNT:
                    raise LimitError(
                        name="不同驱动器数量",
                        value=len(driver_order) + 1,
                        limit=MAX_DRIVER_COUNT,
                    )
                state[host] = [_HIGH, _HIGH]
                driver_order.append(host)
            if rec_phase == _REC_RELEASE_SDA:
                state[host][1] = _HIGH
                rec_started = True
            elif rec_phase == _REC_PULSE_LOW:
                state[host][0] = _LOW
                rec_pulse_fall = tick
                rec_pulse_stretched = False
            elif rec_phase in (_REC_WAIT_HIGH, _REC_STOP_RISE):
                state[host][0] = _HIGH
            elif rec_phase == _REC_STOP_LOW:
                state[host][0] = _LOW
                state[host][1] = _LOW
            elif rec_phase == _REC_STOP_SDA:
                state[host][1] = _HIGH
            rec_action_tick = None  # 动作已应用；下一步由下方观察逻辑调度

        scl = min((d[0] for d in state.values()), default=_HIGH)
        sda = min((d[1] for d in state.values()), default=_HIGH)
        starters_sorted = tuple(d for d in driver_order if d in starters)

        if trace:
            samples.append(
                LineSample(
                    tick=tick,
                    scl=scl,
                    sda=sda,
                    scl_low_drivers=tuple(
                        d for d in driver_order if state[d][0] == _LOW
                    ),
                    sda_low_drivers=tuple(
                        d for d in driver_order if state[d][1] == _LOW
                    ),
                )
            )

        started_this_tick = False
        stopped_this_tick = False

        # 同一时刻合并后再判断边沿。START/STOP 要求 SCL 前后均为高。
        scl_high = pscl == _HIGH and scl == _HIGH
        if scl_high and psda == _HIGH and sda == _LOW:
            # ---- START（可能是重复 START）----
            # START 是帧复位：Sr 前未凑满 9 拍的残余采样位直接丢弃，
            # 不带入新段（真正的半截字节只在流末尾截断时保留为 partial_bits）。
            started_this_tick = True
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
            stopped_this_tick = True
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
                if rec_started and rec_phase is not None:
                    # 恢复期间：采样位单独标记，不拼进之前没收完的数据字节。
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
                    rec_record.bits.append(
                        Bit(
                            index=len(rec_record.bits),
                            value=sda,
                            tick=tick,
                            phase=Phase.RECOVERY,
                            source=source,
                        )
                    )
                elif cur_seg is not None:
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

        # ---- 时钟拉伸检查（恢复等待 SCL 升高期间同样生效）----
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

        # ---- 恢复状态机：观察合并后的实际线值并转移 ----
        if rec_phase is not None and rec_started:
            host = rec_record.host
            if started_this_tick and host not in starters_sorted:
                # 其他主机发起 START：中止本次恢复，正常识别继续，不抢占其事务。
                state[host][0] = _HIGH
                state[host][1] = _HIGH
                rec_record.outcome = RecoveryOutcome.ABORTED
                rec_record.abort_tick = tick
                rec_record.aborted_by = starters_sorted
                recoveries.append(rec_record)
                rec_phase = None
                rec_action_tick = None
                rec_record = None
            elif rec_phase == _REC_RELEASE_SDA:
                # 主机已释放 SDA：总线仍低则开始打脉冲；已为高则直接生成 STOP。
                if sda == _HIGH:
                    rec_phase = _REC_STOP_LOW
                else:
                    rec_phase = _REC_PULSE_LOW
                rec_action_tick = tick + 1
            elif rec_phase == _REC_PULSE_LOW:
                rec_phase = _REC_WAIT_HIGH
                rec_action_tick = tick + 1
            elif rec_phase == _REC_WAIT_HIGH:
                if scl == _HIGH:
                    # 真正观察到 SCL 升高才计数（被拉住时绝不凭计划时间算成功）。
                    rec_record.pulses.append(
                        RecoveryPulse(
                            index=len(rec_record.pulses) + 1,
                            fall_tick=rec_pulse_fall,
                            rise_tick=tick,
                            sda=sda,
                            stretched=rec_pulse_stretched,
                        )
                    )
                    if sda == _HIGH:
                        rec_phase = _REC_STOP_LOW
                        rec_action_tick = tick + 1
                    elif len(rec_record.pulses) >= rec_record.max_pulses:
                        sda_low = tuple(
                            d for d in driver_order if state[d][1] == _LOW
                        )
                        recoveries.append(rec_record)
                        raise BusStuckError(
                            host=host,
                            tick=tick,
                            pulses=len(rec_record.pulses),
                            max_pulses=rec_record.max_pulses,
                            sda_low_drivers=sda_low,
                            record=rec_record,
                        )
                    else:
                        rec_phase = _REC_PULSE_LOW
                        rec_action_tick = tick + 1
                else:
                    # SCL 被他人拉住：保持释放并等待（stretch_timeout 检查在上面）。
                    rec_pulse_stretched = True
            elif rec_phase == _REC_STOP_LOW:
                rec_phase = _REC_STOP_RISE
                rec_action_tick = tick + 1
            elif rec_phase == _REC_STOP_RISE:
                if scl == _HIGH:
                    rec_phase = _REC_STOP_SDA
                    rec_action_tick = tick + 1
            elif rec_phase == _REC_STOP_SDA:
                if stopped_this_tick:
                    rec_record.outcome = RecoveryOutcome.STOPPED
                    rec_record.stop_tick = tick
                else:
                    # 释放 SDA 后总线仍被他人拉住：STOP 未成立。
                    rec_record.outcome = RecoveryOutcome.INCOMPLETE
                recoveries.append(rec_record)
                rec_phase = None
                rec_action_tick = None
                rec_record = None

        pscl, psda = scl, sda

    # ---- 流末尾：恢复未了结是一种输出状态（INCOMPLETE），不是异常 ----
    if rec_phase is not None:
        rec_record.outcome = RecoveryOutcome.INCOMPLETE
        recoveries.append(rec_record)

    # ---- 流末尾：未见 STOP 的事务保持未关闭（缺 STOP 是合法输出而非异常）----
    if cur_seg is not None:
        cur_seg.partial_bits = list(pending_bits)

    return ParseResult(
        transactions=tuple(transactions),
        drivers=tuple(driver_order),
        final_scl=pscl,
        final_sda=psda,
        span_ticks=(items[-1].tick - items[0].tick) if items else 0,
        recoveries=tuple(recoveries),
        trace=tuple(samples),
    )
