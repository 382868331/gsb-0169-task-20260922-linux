"""双主机逐拍驱动仲裁仿真（本题增量）。详见 README。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

from .decoder import parse_updates
from .errors import InvalidUpdateError, LimitError
from .model import (
    ArbitrationLoss,
    BitPhase as Phase,
    Drive,
    ParseResult,
    Update,
)
from .validation import MAX_UPDATE_COUNT, require_driver_id, require_stretch_limit

_HIGH = Drive.RELEASE.line_level  # 1
_LOW = Drive.LOW.line_level       # 0

#: 逐拍驱动序列的拍数上限（每拍每台主机一条更新，合计受更新条数上限约束）。
MAX_TICKS = MAX_UPDATE_COUNT // 2


@dataclass(frozen=True)
class DriveTick:
    """一台主机在一拍内对两根线的主张（拉低或释放）。"""

    scl: Drive
    sda: Drive


@dataclass(frozen=True)
class FirstConflict:
    """失败方首次仲裁冲突的位置。"""

    tick: int           # 采样时刻（SCL 上升沿）
    byte_index: int     # 段内第几个字节（0 = 地址/RW 字节）
    bit_index: int      # 字节内位序 0..7（MSB first）
    phase: Phase
    loser: str
    winner: str         # 冲突时刻把 SDA 拉低的另一方


@dataclass(frozen=True)
class StretchEvent:
    """一次时钟拉伸：低电平期间已有主机释放 SCL（在等拉低方），与仲裁分开记录。"""

    start_tick: int
    end_tick: int       # 低电平结束的上升沿时刻
    duration: int
    holders: tuple[str, ...]  # 低电平结束时刻仍拉低 SCL 的主机
    waiters: tuple[str, ...]  # 低电平期间已释放 SCL、被迫等待的主机


@dataclass(frozen=True)
class DualMasterResult:
    """双主机逐拍仿真的完整结果。"""

    winner: str | None          # 完成传输的主机；完全相同序列（无冲突）为 None
    loser: str | None
    first_conflict: FirstConflict | None
    conflicts: tuple[FirstConflict, ...] = ()
    stretches: tuple[StretchEvent, ...] = ()
    parse: ParseResult | None = None   # 仲裁后总线的解码结果（赢家事务在其中）
    updates: tuple[Update, ...] = ()   # 实际生效的更新流（失败方已被强制释放）

    @property
    def arbitration_losses(self) -> tuple[ArbitrationLoss, ...]:
        """原解码器在生效更新流上记录到的仲裁丢失事件。"""
        if self.parse is None:
            return ()
        return tuple(
            loss
            for tx in self.parse.transactions
            for seg in tx.segments
            for loss in seg.arbitration_losses
        )


def _require_drive_tick(value: object, *, master: str, index: int) -> DriveTick:
    if isinstance(value, DriveTick):
        tick = value
    elif isinstance(value, tuple) and len(value) == 2:
        tick = DriveTick(scl=value[0], sda=value[1])
    else:
        raise InvalidUpdateError(
            field="drives",
            reason=f"每拍必须为 DriveTick 或 (scl, sda) 二元组，收到 {value!r}",
            index=index,
            driver=master,
        )
    for field_name, item in (("scl", tick.scl), ("sda", tick.sda)):
        if isinstance(item, bool) or not isinstance(item, Drive):
            raise InvalidUpdateError(
                field=field_name,
                reason=f"必须为 Drive.RELEASE 或 Drive.LOW，收到 {item!r}",
                index=index,
                driver=master,
            )
    return tick


def _validate_drives(drives: Mapping[str, Sequence[object]]) -> dict[str, list[DriveTick]]:
    if not isinstance(drives, Mapping):
        raise InvalidUpdateError(
            field="drives",
            reason=f"需要 dict 映射（主机名 -> 逐拍序列），收到 {type(drives).__name__}",
        )
    names = list(drives.keys())
    if len(names) != 2:
        raise InvalidUpdateError(
            field="drives",
            reason=f"本接口恰好模拟两个开漏主机，收到 {len(names)} 个：{names!r}",
        )
    checked: dict[str, list[DriveTick]] = {}
    lengths: set[int] = set()
    for raw_name in names:
        name = require_driver_id(raw_name)
        seq = drives[raw_name]
        if isinstance(seq, (str, bytes)) or not isinstance(seq, Sequence):
            raise InvalidUpdateError(
                field="drives",
                reason=f"主机 {name!r} 的驱动输入必须为逐拍序列",
                driver=name,
            )
        if len(seq) > MAX_TICKS:
            raise LimitError(name=f"主机 {name!r} 的拍数", value=len(seq), limit=MAX_TICKS)
        checked[name] = [
            _require_drive_tick(v, master=name, index=i) for i, v in enumerate(seq)
        ]
        lengths.add(len(seq))
    if len(lengths) != 1:
        raise InvalidUpdateError(
            field="drives",
            reason="两个主机的逐拍序列必须等长（同一拍网格）；可用 pad_script 补齐",
        )
    if not lengths.pop():
        raise InvalidUpdateError(field="drives", reason="逐拍序列不能为空")
    return checked


def run_dual_master(
    drives: Mapping[str, Sequence[object]],
    *,
    stretch_limit: int | None = None,
) -> DualMasterResult:
    """逐拍合成两个开漏主机的导线电平，判定仲裁并解码赢家事务。

    drives: {主机名: [每拍 (scl, sda) 主张]}，恰好两个主机、序列等长。
    stretch_limit: 透传给 odbus.parse_updates 的 SCL 低电平阈值。
    输入非法时抛 InvalidUpdateError / LimitError。
    """
    stretch_limit = require_stretch_limit(stretch_limit)
    scripts = _validate_drives(drives)
    names = list(scripts)
    n_ticks = len(scripts[names[0]])

    # 各主机当前生效的线状态（1=释放，0=拉低），缺省释放。
    level = {n: [_HIGH, _HIGH] for n in names}
    lost = {n: False for n in names}

    updates: list[Update] = []
    conflicts: list[FirstConflict] = []
    stretches: list[StretchEvent] = []

    # 轻量帧跟踪（仅用于仲裁判定的位计数；正式解码由 parse_updates 完成）。
    in_tx = False
    byte_count = 0
    bit_in_byte = 0
    seg_rw: int | None = None
    byte_bits = 0  # 当前字节已采数据位拼出的值（用于确定 rw）

    pscl = psda = _HIGH
    scl_low_since: int | None = None
    stretch_waiters: set[str] = set()
    stretch_holders: set[str] = set()

    for tick in range(n_ticks):
        # 1) 失败方从下一驱动阶段起强制释放；其余按各自脚本主张。
        for name in names:
            if lost[name]:
                level[name] = [_HIGH, _HIGH]
            else:
                dt = scripts[name][tick]
                level[name] = [dt.scl.line_level, dt.sda.line_level]
            updates.append(
                Update(
                    tick=tick,
                    driver=name,
                    scl=Drive.RELEASE if level[name][0] else Drive.LOW,
                    sda=Drive.RELEASE if level[name][1] else Drive.LOW,
                )
            )

        # 2) 线与合成：任一拉低即为低（时钟低电平 = 共同释放前的结果）。
        scl = min(level[n][0] for n in names)
        sda = min(level[n][1] for n in names)

        # 3) START / STOP 帧边界（与原解码器同一套边沿定义）。
        scl_high = pscl == _HIGH and scl == _HIGH
        if scl_high and psda == _HIGH and sda == _LOW:
            in_tx = True
            byte_count = 0
            bit_in_byte = 0
            seg_rw = None
            byte_bits = 0
        elif scl_high and psda == _LOW and sda == _HIGH:
            in_tx = False
            bit_in_byte = 0
            byte_count = 0
            seg_rw = None
            byte_bits = 0
        elif pscl == _LOW and scl == _HIGH and in_tx:
            # 4) 有效高电平采样点：仲裁判定（ACK 位与读方向数据位除外）。
            if bit_in_byte < 8:
                master_sends = byte_count == 0 or seg_rw == 0
                if master_sends and sda == _LOW:
                    for name in names:
                        if lost[name] or level[name][1] == _LOW:
                            continue
                        # 该主机发 1 却采到 0 => 仲裁丢失，下一拍起释放。
                        other = [n for n in names if n != name and level[n][1] == _LOW]
                        conflicts.append(
                            FirstConflict(
                                tick=tick,
                                byte_index=byte_count,
                                bit_index=bit_in_byte,
                                phase=(
                                    Phase.RW
                                    if byte_count == 0 and bit_in_byte == 7
                                    else (Phase.ADDRESS if byte_count == 0 else Phase.DATA)
                                ),
                                loser=name,
                                winner=other[0] if other else "",
                            )
                        )
                        lost[name] = True
                byte_bits = (byte_bits << 1) | sda
            bit_in_byte += 1
            if bit_in_byte == 9:
                if byte_count == 0:
                    seg_rw = byte_bits & 1
                byte_count += 1
                bit_in_byte = 0
                byte_bits = 0

        # 5) 时钟拉伸记录：仍在总线上的主机中，有人已释放 SCL（被迫等待）
        # 而有人仍拉低 => 记一次拉伸；已失败离场的主机不算等待者。
        if scl == _LOW:
            if scl_low_since is None:
                scl_low_since = tick
                stretch_waiters = set()
                stretch_holders = set()
            for name in names:
                if lost[name]:
                    continue
                if level[name][0] == _HIGH:
                    stretch_waiters.add(name)
                else:
                    stretch_holders.add(name)
        elif pscl == _LOW and scl_low_since is not None:
            if stretch_waiters:
                stretches.append(
                    StretchEvent(
                        start_tick=scl_low_since,
                        end_tick=tick,
                        duration=tick - scl_low_since,
                        holders=tuple(n for n in names if n in stretch_holders),
                        waiters=tuple(n for n in names if n in stretch_waiters),
                    )
                )
            scl_low_since = None
            stretch_waiters = set()
            stretch_holders = set()

        pscl, psda = scl, sda

    result = parse_updates(updates, stretch_limit=stretch_limit)
    loser = conflicts[0].loser if conflicts else None
    winner = next((n for n in names if n != loser), None) if loser else None
    return DualMasterResult(
        winner=winner,
        loser=loser,
        first_conflict=conflicts[0] if conflicts else None,
        conflicts=tuple(conflicts),
        stretches=tuple(stretches),
        parse=result,
        updates=tuple(updates),
    )


# --------------------------- 逐拍脚本构建辅助 ---------------------------


def write_script(address: int, data: Sequence[int] = ()) -> list[DriveTick]:
    """生成一个“主机写”逐拍脚本：START + 地址写字节 + 数据字节 + STOP。

    每拍一个 DriveTick；位脉冲固定 2 拍（SCL 低放数据 / SCL 高采样）。
    ACK 拍主机释放 SDA（总线应答电平由线与结果自然决定）。
    两个主机脚本长度不一致时用 pad_script 补齐。
    """
    R, L = Drive.RELEASE, Drive.LOW
    script: list[DriveTick] = []

    def pulse(sda: Drive) -> None:
        script.append(DriveTick(L, sda))
        script.append(DriveTick(R, sda))

    script.append(DriveTick(R, L))  # START：SCL 高时 SDA 拉低
    for byte in [(address << 1) & 0xFE, *data]:
        for k in range(8):
            pulse(R if (byte >> (7 - k)) & 1 else L)
        pulse(R)  # ACK 拍：主机释放 SDA
    # STOP：先补一拍把 SDA 拉低，再在 SCL 高时释放 => 0->1。
    script.append(DriveTick(L, L))
    script.append(DriveTick(R, L))
    script.append(DriveTick(R, R))
    return script


def pad_script(script: list[DriveTick], length: int) -> list[DriveTick]:
    """把脚本补齐到指定拍数（尾部补释放），便于两主机脚本对齐拍网格。"""
    if len(script) > length:
        raise InvalidUpdateError(
            field="script",
            reason=f"脚本已有 {len(script)} 拍，无法补齐到更短的 {length} 拍",
        )
    return script + [DriveTick(Drive.RELEASE, Drive.RELEASE)] * (length - len(script))
