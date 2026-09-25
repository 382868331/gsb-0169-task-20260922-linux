"""核心解码算法。

边沿触发模型（题目契约）：

* 每个驱动器在整数时刻给出它对 SCL/SDA 的**绝对**驱动状态（RELEASE=高阻/LOW=拉低）；
* 同一时刻的多条更新先按驱动器合并（线值 = 所有驱动器的 AND），再统一判断边沿；
* SCL 保持高电平时 SDA 高->低为 START，低->高为 STOP；
* SCL 低->高采样一位；9 个脉冲 = 8 数据位（MSB first）+ 1 ACK；
* 事务内重复 START 不产生 STOP，而是在同一事务内开启新段；
* 数据位（地址/RW/数据，不含 ACK）上：主机已释放 SDA（发 1）但线采样为 0 => 仲裁丢失；
* SCL 持续为低且 ``tick - low_since > stretch_limit`` => :class:`StretchTimeoutError`。

显式总线恢复（``recoveries`` 参数）：

* 调用方在某整数时刻请求恢复后，恢复驱动器模型接管该主机：**先释放 SDA**，
  随后最多发 :data:`~odbus.model.MAX_RECOVERY_PULSES` 个 SCL 脉冲；
* 恢复主机按 1 拍/时刻主动推进（它自己的计划时刻），外部驱动器若在同一整数时刻
  有更新，先做线与合并再判边沿——所有翻转都只能来自驱动器合并，没有滤波器；
* 脉冲是否计数只看**合并后实测 SCL 是否真的升高**：主机放行 SCL 后线仍被别的
  驱动器拉住时，该拍不计数，模型继续等待，并沿用 ``stretch_limit`` 报 stretch_timeout，
  绝不按计划时间假装成功；
* SDA 在某个有效脉冲后已释放（采样为 1）=> 模型让出一拍（给其他主机一次 START
  机会）后生成合法 STOP 结束；恢复期间总线自然形成 STOP 也直接采纳；
  发满 9 个有效脉冲 SDA 仍低 => bus_stuck；
* 恢复期间采到的位记录在 :class:`RecoveryBit`（带脉冲序号），**不拼入**此前未收完
  的数据字节（那部分位冻结在 ``Segment.partial_bits``）；
* 恢复期间其他主机发起 START => 中止本次恢复（不抢占它的事务），正常识别继续。
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
    MAX_RECOVERY_PULSES,
    ParseResult,
    RecoveryBit,
    RecoveryEvent,
    RecoveryEventKind as EKind,
    RecoveryOutcome,
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
    MAX_TICK,
    MAX_UPDATE_COUNT,
    require_driver_id,
    require_recovery_requests,
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


# 恢复驱动器模型的内部阶段（每个激活 tick 执行一个阶段步）。
_ST_RELEASE = "release"            # 接管首刻：SCL/SDA 都释放
_ST_PULSE_LOW = "pulse_low"        # 主机拉低 SCL，开始一个脉冲
_ST_PULSE_RELEASE = "pulse_rel"    # 主机放行 SCL，观察是否真的升高（首拍）
_ST_PULSE_WAIT = "pulse_wait"      # SCL 被别人拉住，继续等它升高
_ST_IDLE = "idle"                  # SDA 已空：释放两线观察一拍（让位其他主机 START）
_ST_STOP_LOW = "stop_low"          # 生成 STOP：SCL 低时拉低 SDA
_ST_STOP_RISE = "stop_rise"        # 生成 STOP：放行 SCL（SDA 保持低）
_ST_STOP_WAIT = "stop_wait"        # 生成 STOP 时 SCL 被拉住，继续等
_ST_STOP_RELEASE = "stop_release"  # SCL 高时释放 SDA，形成 STOP 边沿


class _ActiveRecovery:
    """一次正在进行的恢复操作的全部可变状态。"""

    def __init__(self, request: RecoveryRequest, segment_index: int | None) -> None:
        self.request = request
        self.segment_index = segment_index
        self.stage = _ST_RELEASE
        self.next_tick = request.tick   # 模型计划下一个驱动时刻
        self.events: list[RecoveryEvent] = []
        self.bits: list[RecoveryBit] = []
        self.pulses = 0
        self.pulse_low_tick: int | None = None

    def finalize(
        self,
        outcome: RecoveryOutcome,
        *,
        stop: StopEvent | None = None,
        end_tick: int | None = None,
        stuck_tick: int | None = None,
        aborted_by: str | None = None,
    ) -> RecoveryRecord:
        return RecoveryRecord(
            request=self.request,
            outcome=outcome,
            events=tuple(self.events),
            bits=tuple(self.bits),
            stop=stop,
            pulses_sent=self.pulses,
            end_tick=end_tick,
            stuck_tick=stuck_tick,
            aborted_by=aborted_by,
            segment_index=self.segment_index,
        )


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
        recoveries: Iterable[RecoveryRequest] | None = None,
    ) -> ParseResult:
        return parse_updates(
            self._updates, stretch_limit=stretch_limit, recoveries=recoveries
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


def _data_phase(byte_count: int, bit_in_byte: int) -> Phase:
    if byte_count == 0:
        return Phase.RW if bit_in_byte == 7 else Phase.ADDRESS
    return Phase.DATA


def _ack_phase(byte_count: int) -> Phase:
    return Phase.ACK_ADDR if byte_count == 0 else Phase.ACK_DATA


def parse_updates(
    updates: Iterable[Update],
    *,
    stretch_limit: int | None = None,
    recoveries: Iterable[RecoveryRequest] | None = None,
) -> ParseResult:
    """解析驱动器更新流，返回 :class:`ParseResult`；输入不可解析时抛错，不做部分输出。

    Args:
        updates: 按整数时刻非递减排列的 :class:`Update` 可迭代对象。
        stretch_limit: SCL 允许连续为低的 tick 数；严格超过（``duration > limit``）即抛
            :class:`StretchTimeoutError`。``None`` 表示不检查。恢复期间 SCL 被别人拉住
            时沿用同一阈值（``during_recovery=True``）。
        recoveries: 可选的 :class:`RecoveryRequest` 可迭代对象。请求时刻必须是更新流中
            实际存在的整数时刻；恢复模型从该时刻起接管请求主机的线驱动（先释放 SDA），
            并按每整数时刻一拍主动发脉冲；外部驱动器在模型计划时刻的同刻更新照常合并。
    """
    stretch_limit = require_stretch_limit(stretch_limit)
    items = _validate_updates(updates)
    requests = require_recovery_requests(recoveries)

    # --- 冲突检测：同一 (tick, driver) 只允许一条更新 ---
    seen_key: dict[tuple[int, str], int] = {}
    for idx, u in enumerate(items):
        key = (u.tick, u.driver)
        if key in seen_key:
            raise ConflictingUpdateError(
                tick=u.tick, driver=u.driver, index=idx, first_index=seen_key[key]
            )
        seen_key[key] = idx

    # --- 按 tick 分组（外部更新）---
    ext_groups: dict[int, list[Update]] = {}
    ext_ticks: list[int] = []
    for u in items:
        if u.tick not in ext_groups:
            ext_groups[u.tick] = []
            ext_ticks.append(u.tick)
        ext_groups[u.tick].append(u)

    for req in requests:
        group_here = ext_groups.get(req.tick)
        if not group_here:
            raise InvalidUpdateError(
                field="recoveries",
                reason=(
                    f"恢复请求 tick={req.tick} 不是更新流中存在的时刻；"
                    "恢复模型只能在实际有驱动器更新的整数时刻接管"
                ),
                tick=req.tick,
                driver=req.master,
            )
        owners = {u.driver for u in group_here}
        if req.master in owners:
            # 接管刻主机的线状态由模型决定（先释放 SDA）；调用方同刻再给它一条
            # 绝对状态更新会造成“同一驱动器同刻两条驱动声明”，直接拒绝而非二选一。
            raise InvalidUpdateError(
                field="recoveries",
                reason=(
                    f"恢复请求 tick={req.tick} 与驱动器 {req.master!r} 的外部更新同刻："
                    "接管刻该主机由恢复模型驱动（先释放 SDA），不得再提交它的更新；"
                    "请把它的最后一条更新放在请求刻之前"
                ),
                tick=req.tick,
                driver=req.master,
            )
    request_at: dict[int, RecoveryRequest] = {req.tick: req for req in requests}

    # --- 驱动器状态（1=RELEASE，0=LOW），缺省为释放 ---
    state: dict[str, list[int]] = {}
    driver_order: list[str] = []

    def level_of(driver: str, line: int) -> int:
        return state[driver][line] if driver in state else _HIGH

    def holders(line: int) -> tuple[str, ...]:
        return tuple(d for d in driver_order if state[d][line] == _LOW)

    def ensure_driver(driver: str) -> None:
        if driver not in state:
            if len(driver_order) >= MAX_DRIVER_COUNT:
                raise LimitError(
                    name="不同驱动器数量",
                    value=len(driver_order) + 1,
                    limit=MAX_DRIVER_COUNT,
                )
            state[driver] = [_HIGH, _HIGH]
            driver_order.append(driver)

    transactions: list[Transaction] = []
    recovery_records: list[RecoveryRecord] = []
    cur_tx: Transaction | None = None
    cur_seg: Segment | None = None
    pending_bits: list[Bit] = []
    byte_count = 0
    bit_in_byte = 0          # 0..8：数据位计数；8 表示下一个脉冲是 ACK
    pulse: _PendingPulse | None = None
    seg_rw: int | None = None  # 当前段方向（首字节完成后确定）；读段数据位不查主机仲裁

    # 恢复期间被冻结的段（恢复前未走完的事务），恢复的 STOP 会把它合法关闭。
    frozen_seg: Segment | None = None
    active: _ActiveRecovery | None = None

    pscl = psda = _HIGH
    scl_low_since: int | None = None

    ei = 0  # ext_ticks 游标
    while ei < len(ext_ticks) or active is not None:
        next_ext = ext_ticks[ei] if ei < len(ext_ticks) else None
        if active is not None:
            tick = active.next_tick if next_ext is None else min(next_ext, active.next_tick)
            if tick > MAX_TICK:
                raise LimitError(name="恢复模型推进时刻", value=tick, limit=MAX_TICK)
        else:
            tick = next_ext  # type: ignore[assignment]
            if tick is None:
                break
        group = ext_groups.pop(tick, [])
        if group:
            ei += 1
        model_drives = active is not None and active.next_tick == tick
        req = request_at.get(tick)

        # ---- 恢复请求在本刻生效 ----
        if req is not None:
            if active is not None:
                raise InvalidUpdateError(
                    field="recoveries",
                    reason=(
                        f"tick={tick} 上一次恢复（{active.request.master!r} 自 "
                        f"tick={active.request.tick} 起）尚未结束，不能叠加新的恢复请求"
                    ),
                    tick=tick,
                    driver=req.master,
                )
            seg_index: int | None = None
            if cur_seg is not None and cur_tx is not None:
                # 冻结恢复前没收完的数据位；它们不与恢复期间采到的位相混。
                cur_seg.partial_bits = list(pending_bits)
                seg_index = len(cur_tx.segments) - 1
                frozen_seg = cur_seg
            else:
                frozen_seg = None
            # 帧复位（恢复脉冲不进入任何段），但不产生 START/STOP。
            pending_bits = []
            byte_count = 0
            bit_in_byte = 0
            seg_rw = None
            pulse = None
            cur_seg = None
            cur_tx = None
            active = _ActiveRecovery(req, seg_index)
            model_drives = True

        # ---- 应用外部更新 ----
        starters: list[str] = []
        for u in group:
            if active is not None and u.driver == active.request.master:
                raise InvalidUpdateError(
                    field="driver",
                    reason=(
                        f"tick={tick} 处于 {u.driver!r} 的恢复期间（自 "
                        f"tick={active.request.tick} 起）：该驱动器由恢复模型驱动，"
                        "调用方不得再对其提交更新"
                    ),
                    tick=tick,
                    driver=u.driver,
                )
            ensure_driver(u.driver)
            old_scl, old_sda = state[u.driver]
            new_scl, new_sda = u.scl.line_level, u.sda.line_level
            state[u.driver][0] = new_scl
            state[u.driver][1] = new_sda
            # START 来源：本时刻 SDA 由释放变为拉低的驱动器（稍后按注册序排序）。
            if old_sda == _HIGH and new_sda == _LOW:
                starters.append(u.driver)

        # ---- 恢复驱动器模型：像一条虚拟更新一样参与同刻线与合并 ----
        event_kind: EKind | None = None
        event_pulse: int | None = None
        event_note = ""
        master_caused_release = False  # 本刻 STOP 边沿是否由模型释放 SDA 造成
        if model_drives and active is not None:
            master = active.request.master
            ensure_driver(master)
            stage = active.stage
            m_scl = m_sda = _HIGH
            if stage == _ST_RELEASE:
                m_scl, m_sda = _HIGH, _HIGH
                event_kind = EKind.RELEASE_SDA
                # 接管前主机若自己拉着 SDA，本刻的 0->1 边沿由模型释放造成。
                master_caused_release = state[master][1] == _LOW
            elif stage == _ST_PULSE_LOW:
                m_scl, m_sda = _LOW, _HIGH
                event_kind = EKind.PULSE_LOW
            elif stage in (_ST_PULSE_RELEASE, _ST_PULSE_WAIT):
                m_scl, m_sda = _HIGH, _HIGH
                event_kind = EKind.WAIT_SCL_LOW
            elif stage == _ST_IDLE:
                m_scl, m_sda = _HIGH, _HIGH
                event_kind = EKind.BUS_IDLE
            elif stage == _ST_STOP_LOW:
                m_scl, m_sda = _LOW, _LOW
                event_kind = EKind.STOP_SETUP_LOW
            elif stage in (_ST_STOP_RISE, _ST_STOP_WAIT):
                m_scl, m_sda = _HIGH, _LOW
            elif stage == _ST_STOP_RELEASE:
                m_scl, m_sda = _HIGH, _HIGH
                event_kind = EKind.STOP_RELEASE
                master_caused_release = state[master][1] == _LOW
            state[master][0] = m_scl
            state[master][1] = m_sda
            # 放行 SCL 的拍，事件种类按合并后实测是否真的高来标。
            if stage in (_ST_PULSE_RELEASE, _ST_PULSE_WAIT):
                merged_scl = min((d[0] for d in state.values()), default=_HIGH)
                if merged_scl == _LOW:
                    event_kind = EKind.WAIT_SCL_LOW
            elif stage in (_ST_STOP_RISE, _ST_STOP_WAIT):
                merged_scl = min((d[0] for d in state.values()), default=_HIGH)
                event_kind = (
                    EKind.STOP_SETUP_RISE if merged_scl == _HIGH else EKind.STOP_WAIT_SCL
                )

        scl = min((d[0] for d in state.values()), default=_HIGH)
        sda = min((d[1] for d in state.values()), default=_HIGH)
        starters_sorted = tuple(d for d in driver_order if d in starters)

        # ---- 同刻合并后再判边沿 ----
        scl_high = pscl == _HIGH and scl == _HIGH
        is_start = scl_high and psda == _HIGH and sda == _LOW
        # 接管首刻：恢复刚刚开始，任何 SDA 0->1 都不算恢复的 STOP——
        # 契约要求“先释放 SDA，再发脉冲”，STOP 只能在某个**有效脉冲之后**形成，
        # 无论本刻 SDA 是模型放的还是外部持有者同刻放的。
        takeover_tick = active is not None and active.stage == _ST_RELEASE
        is_stop = (
            scl_high and psda == _LOW and sda == _HIGH and not takeover_tick
        )

        terminal = False  # 本刻恢复是否已终结（事件已内联记录）

        if is_start:
            # ---- START（可能是重复 START）----
            if active is not None:
                master = active.request.master
                other_starters = tuple(d for d in starters_sorted if d != master)
                if other_starters:
                    # 其他主机抢得 START：中止恢复，不抢占它的事务。
                    aborted_by = other_starters[0]
                    event_kind = EKind.ABORTED
                    event_note = f"其他主机 {aborted_by!r} 发起 START，恢复中止"
                    state[master][0] = _HIGH
                    state[master][1] = _HIGH
                    # 被冻结的旧段保持未闭合；新事务交给下面的正常识别。
                    frozen_seg = None
                    active.events.append(
                        RecoveryEvent(
                            tick=tick, kind=EKind.ABORTED, scl=scl, sda=sda,
                            scl_low_drivers=holders(0), sda_low_drivers=holders(1),
                            note=event_note,
                        )
                    )
                    rec = active.finalize(
                        RecoveryOutcome.ABORTED, end_tick=tick, aborted_by=aborted_by
                    )
                    recovery_records.append(rec)
                    active = None
                    terminal = True
            if active is None:
                # ---- 正常 START 识别（恢复未激活，或本刻刚中止）----
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
        elif is_stop:
            # ---- STOP ----
            stop_ev = StopEvent(tick=tick)
            if active is not None and not terminal:
                generated = master_caused_release
                kind = EKind.STOP_RELEASE if generated else EKind.STOP_OBSERVED
                event_kind = kind
                active.events.append(
                    RecoveryEvent(
                        tick=tick, kind=kind, scl=scl, sda=sda,
                        scl_low_drivers=holders(0), sda_low_drivers=holders(1),
                        pulse=event_pulse,
                    )
                )
                if frozen_seg is not None:
                    frozen_seg.stop = stop_ev
                frozen_seg = None
                rec = active.finalize(
                    RecoveryOutcome.STOP_GENERATED if generated
                    else RecoveryOutcome.STOP_OBSERVED,
                    stop=stop_ev,
                    end_tick=tick,
                )
                recovery_records.append(rec)
                active = None
                terminal = True
            if cur_seg is not None:
                cur_seg.stop = stop_ev
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
                # 上升沿：恢复模型先看脉冲是否真实成立。
                if active is not None and active.stage in (
                    _ST_PULSE_RELEASE, _ST_PULSE_WAIT
                ):
                    active.pulses += 1
                    pno = active.pulses
                    sda_low = holders(1)
                    active.bits.append(
                        RecoveryBit(
                            pulse=pno,
                            value=sda,
                            tick=tick,
                            scl_low_tick=active.pulse_low_tick or tick,
                            sda_low_drivers=sda_low,
                        )
                    )
                    event_kind = EKind.PULSE_COUNTED
                    event_pulse = pno
                    if sda == _HIGH:
                        # SDA 已释放：让出一拍，再生成 STOP。
                        active.stage = _ST_IDLE
                    elif pno >= MAX_RECOVERY_PULSES:
                        # 九个**有效**脉冲后 SDA 仍被拉低：bus_stuck。
                        active.events.append(
                            RecoveryEvent(
                                tick=tick, kind=EKind.PULSE_COUNTED, scl=scl, sda=sda,
                                scl_low_drivers=holders(0), sda_low_drivers=sda_low,
                                pulse=pno,
                            )
                        )
                        rec = active.finalize(
                            RecoveryOutcome.STUCK, end_tick=tick, stuck_tick=tick
                        )
                        recovery_records.append(rec)
                        active = None
                        terminal = True
                        err = BusStuckError(
                            tick=tick,
                            master=rec.request.master,
                            pulses=rec.pulses_sent,
                            sda_low_drivers=sda_low,
                            record=rec,
                        )
                        pscl, psda = scl, sda  # 仅为状态一致（随即抛错）
                        raise err
                    else:
                        active.stage = _ST_PULSE_LOW
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
                            seg_rw = value & 1
                        pending_bits = []
                        byte_count += 1
                        bit_in_byte = 0
                pulse = None
            elif pscl == _HIGH and scl == _LOW:
                pulse = _PendingPulse(fall_tick=tick, scl_low_drivers=holders(0))

        # ---- 恢复模型：记录普通事件并推进阶段 ----
        if active is not None and model_drives and not terminal:
            stage = active.stage
            if event_kind is EKind.PULSE_LOW:
                # 主机本刻驱动 SCL 低（合并后必为低）：脉冲尝试起点。
                active.pulse_low_tick = tick
            active.events.append(
                RecoveryEvent(
                    tick=tick,
                    kind=event_kind,  # type: ignore[arg-type]
                    scl=scl,
                    sda=sda,
                    scl_low_drivers=holders(0),
                    sda_low_drivers=holders(1),
                    pulse=event_pulse,
                    note=event_note,
                )
            )
            if event_kind is EKind.PULSE_COUNTED:
                pass  # 阶段已在上升沿分支按采样值转好
            elif stage == _ST_RELEASE:
                active.stage = _ST_PULSE_LOW
            elif stage == _ST_PULSE_LOW:
                active.stage = _ST_PULSE_RELEASE
            elif stage in (_ST_PULSE_RELEASE, _ST_PULSE_WAIT):
                # 放行后本刻未观察到升高（否则已计为 PULSE_COUNTED）=> 被别人拉住。
                active.stage = _ST_PULSE_WAIT
            elif stage == _ST_IDLE:
                active.stage = _ST_STOP_LOW
            elif stage == _ST_STOP_LOW:
                active.stage = _ST_STOP_RISE
            elif stage in (_ST_STOP_RISE, _ST_STOP_WAIT):
                active.stage = _ST_STOP_RELEASE if scl == _HIGH else _ST_STOP_WAIT
            elif stage == _ST_STOP_RELEASE:
                if sda == _LOW:
                    # 计划释放时 SDA 又被别人拉住：STOP 没形成。
                    if active.pulses >= MAX_RECOVERY_PULSES:
                        # 九个有效脉冲额度已用尽仍收不出总线：bus_stuck。
                        active.events[-1] = RecoveryEvent(
                            tick=tick,
                            kind=EKind.STOP_BLOCKED,
                            scl=scl,
                            sda=sda,
                            scl_low_drivers=holders(0),
                            sda_low_drivers=holders(1),
                            pulse=event_pulse,
                            note=(
                                "SDA 在计划 STOP 时刻仍被其他驱动器拉低，"
                                "且 9 个有效脉冲额度已用尽"
                            ),
                        )
                        rec = active.finalize(
                            RecoveryOutcome.STUCK, end_tick=tick, stuck_tick=tick
                        )
                        recovery_records.append(rec)
                        active = None
                        raise BusStuckError(
                            tick=tick,
                            master=rec.request.master,
                            pulses=rec.pulses_sent,
                            sda_low_drivers=holders(1),
                            record=rec,
                        )
                    # 回去补脉冲（仍受最多 9 个有效脉冲限制）。
                    active.events[-1] = RecoveryEvent(
                        tick=tick,
                        kind=EKind.STOP_BLOCKED,
                        scl=scl,
                        sda=sda,
                        scl_low_drivers=holders(0),
                        sda_low_drivers=holders(1),
                        pulse=event_pulse,
                        note="SDA 在计划 STOP 时刻仍被其他驱动器拉低，回去补脉冲",
                    )
                    active.stage = _ST_PULSE_LOW

        # ---- 时钟拉伸检查（恢复期间沿用同一口径）----
        if stretch_limit is not None and scl_low_since is not None:
            duration = tick - scl_low_since
            if duration > stretch_limit:
                if active is not None:
                    rec = active.finalize(
                        RecoveryOutcome.STRETCH_TIMEOUT, end_tick=tick
                    )
                    recovery_records.append(rec)
                    active = None
                    err = StretchTimeoutError(
                        tick=tick,
                        low_since=scl_low_since,
                        duration=duration,
                        limit=stretch_limit,
                        during_recovery=True,
                    )
                    err.record = rec  # type: ignore[attr-defined]
                    raise err
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

        # ---- 无超时阈值时，等待不能越过最后一个观测 tick 无限自造时刻 ----
        # SCL 被别人拉住（脉冲放行或 STOP 升 SCL 后实测仍低），后续再无任何外部更新
        # 可能释放它，又没有 stretch_limit 可判定超时：恢复不能凭计划时间成功，报 bus_stuck。
        if (
            active is not None
            and active.stage in (_ST_PULSE_WAIT, _ST_STOP_WAIT)
            and scl == _LOW
            and stretch_limit is None
            and ei >= len(ext_ticks)
        ):
            rec = active.finalize(RecoveryOutcome.STUCK, end_tick=tick, stuck_tick=tick)
            recovery_records.append(rec)
            active = None
            raise BusStuckError(
                tick=tick,
                master=rec.request.master,
                pulses=rec.pulses_sent,
                sda_low_drivers=holders(0),
                record=rec,
                scl_held_low=True,
            )

        if active is not None:
            active.next_tick = tick + 1
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
        recoveries=tuple(recovery_records),
    )
