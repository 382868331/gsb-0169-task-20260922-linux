"""线采样输入（整数时间戳 SCL/SDA）的最小稳定宽度滤波与旧解码器接入。

本模块是第二轮迭代新增能力，**复用** :mod:`odbus.decoder` 的全部事务语义：

1. 输入是逻辑分析仪/ADC 视角的 :class:`~odbus.model.Sample`：整数时刻采到的
    SCL/SDA *线电平*（0/1，已线与，无驱动器归属）；
2. 对两根线**各自独立**做最小稳定宽度滤波；
3. 滤波后的阶跃时刻按 tick 取并集重建联合采样——同一 tick 两线都变化时合并为
   同一条采样（“先合并再判断边沿”，与旧契约一致）；
4. 把联合采样合成为单一虚拟驱动器的 :class:`~odbus.model.Update`，送入旧的
   :func:`odbus.parse_updates`，事务/仲裁/拉伸等规则全部沿用。

宽度口径（确定性，无浮点）：一根线在 ``begin_tick`` 进入某电平、在
``end_tick`` 离开，则该段宽度为 ``end_tick - begin_tick``（整数 tick 差）。
例如 t=0 拉低、t=3 拉高，低电平宽度为 3；``min_width=3`` 时保留（等号边界），
``min_width=4`` 时作为毛刺剔除。

滤波的固定规则（离线，允许查看整条采样流）：

* 把单根线的阶跃序列切成相邻且电平严格交替的恒定段，两端各补一个“宽度无穷”
  的段——流首之前缺省为高的空闲段、流末之后无结束时刻的开放段；
* 反复选取当前**宽度最小**（宽度相同取开始时刻最早）的有限段：
    - 宽度已 ``>= min_width``：连最窄段都达标，剩余段全部达标，结束；
    - 宽度 ``< min_width``：判定为毛刺，记录到 ``filtered`` 并删除；它左右两段
      电平必然相同（交替序列），将二者**合并成一段**并按合并后的新宽度重排。
* 流首空闲段与流末开放段宽度视为无穷，只用于吸收相邻毛刺，自身永不被删除。

这样“真实宽电平中间夹窄毛刺”会跨过毛刺把两侧同电平合并（毛刺桥接）；孤立毛刺
消除后线值保持稳定。删除优先级是全序（宽度→开始时刻），结果唯一、与扫描方向
无关。实现用最小堆 + 双向链表，复杂度 O(n log n)。
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Iterable

from .decoder import parse_updates
from .errors import DuplicateSampleError, InvalidUpdateError, LimitError
from .model import (
    Drive,
    FilteredPulse,
    Line,
    ParseResult,
    Sample,
    SampleFilterResult,
    Update,
)
from .validation import (
    MAX_UPDATE_COUNT,
    require_level,
    require_min_width,
    require_stretch_limit,
    require_tick,
)

#: 线采样合成旧解码器更新时使用的单一虚拟驱动器名。
#:
#: 线采样不携带驱动器归属，滤波后的线值由该虚拟驱动器统一持有；因此位来源中
#: 的 ``scl_low_drivers`` / ``sda_low_drivers`` 只会出现该名字，且采样输入不会
#: 产生仲裁丢失事件（无法判断谁是主机、谁在竞争）。
SAMPLE_DRIVER = "SAMPLE"

_IDLE = 1  # 首条采样之前总线缺省为高（与旧解码器缺省释放一致）
_INF = float("inf")


# ----------------------------- 输入校验 -----------------------------

def _validate_samples(samples: Iterable[Sample]) -> list[Sample]:
    """校验采样流：类型、tick 值域/非递减、电平 0/1、同刻重复；不做静默纠正。"""
    result: list[Sample] = []
    prev_tick: int | None = None
    first_index_at_tick: dict[int, int] = {}
    for idx, s in enumerate(samples):
        if idx >= MAX_UPDATE_COUNT:
            raise LimitError(name="采样条数", value=idx + 1, limit=MAX_UPDATE_COUNT)
        if not isinstance(s, Sample):
            raise InvalidUpdateError(
                field="sample",
                reason=f"必须为 odbus.Sample，收到 {type(s).__name__}",
                index=idx,
            )
        tick = require_tick(s.tick, index=idx)
        scl = require_level(s.scl, field="scl", index=idx, tick=tick)
        sda = require_level(s.sda, field="sda", index=idx, tick=tick)
        if prev_tick is not None and tick < prev_tick:
            raise InvalidUpdateError(
                field="tick",
                reason=f"时刻必须非递减：tick={tick} 早于上一条 tick={prev_tick}",
                index=idx,
                tick=tick,
            )
        if tick in first_index_at_tick:
            raise DuplicateSampleError(
                tick=tick, index=idx, first_index=first_index_at_tick[tick]
            )
        first_index_at_tick[tick] = idx
        prev_tick = tick
        result.append(Sample(tick=tick, scl=scl, sda=sda))
    return result


# ----------------------------- 滤波核心 -----------------------------

def _line_changes(samples: list[Sample], attr: str) -> list[tuple[int, int]]:
    """抽取单根线相对空闲高电平的阶跃序列 ``[(tick, new_level), ...]``（去重保持段）。"""
    changes: list[tuple[int, int]] = []
    prev = _IDLE
    for s in samples:
        value = getattr(s, attr)
        if value != prev:
            changes.append((s.tick, value))
            prev = value
    return changes


class _Seg:
    """滤波过程中一根线上的一个恒定段（双向链表节点）。"""

    __slots__ = ("begin", "end", "level", "prev", "next", "alive", "ver", "sid")

    def __init__(self, begin, end, level, sid: int) -> None:
        self.begin = begin        # 进入时刻（流首哨兵为 -inf）
        self.end = end            # 离开时刻（流末哨兵为 +inf）
        self.level = level
        self.prev: "_Seg | None" = None
        self.next: "_Seg | None" = None
        self.alive = True
        self.ver = 0
        self.sid = sid

    @property
    def width(self):
        if self.begin == _NEG or self.end == _INF:
            return _INF
        return self.end - self.begin


_NEG = float("-inf")


def _filter_line(
    changes: list[tuple[int, int]],
    line: Line,
    min_width: int,
    glitches: list[FilteredPulse],
) -> list[tuple[int, int]]:
    """单线最窄段优先滤波（离线、确定性规则）。

    把阶跃序列切成相邻恒定段（电平严格交替），两端补“宽度无穷”的空闲/末尾
    哨兵段；随后反复取**宽度最小**（等宽取最早开始）的有限段：

    * 其宽度 ``>= min_width``：最窄段已达标，所有剩余段都达标，停止；
    * 其宽度 ``< min_width``：它是毛刺，记录并删除；它左右两段电平必然相同
      （交替序列），将二者**合并**为一段（``begin=左.begin, end=右.end``）并
      重新计算宽度后放回候选堆；被合并的邻段视为该稳定电平的一部分，不单列。

    用最小堆 + 双向链表实现（段合并后旧堆条目以版本号判失效），复杂度 O(n log n)。
    流首空闲段与流末无结束段宽度视为无穷：只吸收邻居，永不被当作毛刺删除。
    """
    if not changes:
        return []

    # 构造 n+1 个段：head=空闲哨兵，tail=末尾哨兵，中间为有限段。
    head = _Seg(_NEG, changes[0][0], _IDLE, 0)
    segs: list[_Seg] = [head]
    for k in range(1, len(changes)):
        segs.append(_Seg(changes[k - 1][0], changes[k][0], changes[k - 1][1], k))
    tail = _Seg(changes[-1][0], _INF, changes[-1][1], len(changes))
    segs.append(tail)
    for a, b in zip(segs, segs[1:]):
        a.next = b
        b.prev = a

    # 有限段入堆：(宽度, 开始时刻, 节点 id, 版本)。等宽按 sid（即开始先后）。
    heap: list[tuple] = []
    for node in segs:
        if node.width != _INF:
            heapq.heappush(heap, (node.width, node.begin, node.sid, 0))

    while heap:
        w, _begin, sid, ver = heapq.heappop(heap)
        node = segs[sid]
        if not node.alive or ver != node.ver or w != node.width:
            continue  # 已被合并的失效条目
        if w >= min_width:
            break     # 最窄存活段已达标 => 全部达标

        # 删除该毛刺段；左、右段电平相同，合并。
        glitches.append(
            FilteredPulse(
                line=line, level=node.level,
                begin_tick=node.begin, end_tick=node.end, duration=w,
            )
        )
        left, right = node.prev, node.next
        node.alive = False
        # 右段并入左段：左段延伸到 right.end，跳过 right。
        left.end = right.end
        left.next = right.next
        if right.next is not None:
            right.next.prev = left
        right.alive = False
        left.ver += 1
        if left.width != _INF:
            heapq.heappush(heap, (left.width, left.begin, left.sid, left.ver))

    # 沿存活链表重建阶跃（跳过与空闲同电平的链头）。
    out: list[tuple[int, int]] = []
    prev_level = _IDLE
    cur = head.next
    while cur is not None:
        if cur.level != prev_level:
            out.append((cur.begin, cur.level))
            prev_level = cur.level
        cur = cur.next
    return out


def _rebuild(
    scl_events: list[tuple[int, int]],
    sda_events: list[tuple[int, int]],
) -> list[Sample]:
    """两线存活阶跃时刻取并集，逐刻合并为联合采样；无边际效果的冗余刻丢弃。"""
    ticks = sorted({t for t, _ in scl_events} | {t for t, _ in sda_events})
    scl = sda = _IDLE
    i = j = 0
    out: list[Sample] = []
    for t in ticks:
        while i < len(scl_events) and scl_events[i][0] == t:
            scl = scl_events[i][1]
            i += 1
        while j < len(sda_events) and sda_events[j][0] == t:
            sda = sda_events[j][1]
            j += 1
        if not out or out[-1].scl != scl or out[-1].sda != sda:
            out.append(Sample(tick=t, scl=scl, sda=sda))
    return out


def filter_samples(samples: Iterable[Sample], min_width: int) -> SampleFilterResult:
    """对线采样做最小稳定宽度滤波（纯函数，不修改输入）。

    Args:
        samples: 非递减的 :class:`Sample` 序列；同一 tick 只允许一条。
        min_width: 最小稳定宽度（正整数）。段宽度严格小于它才被剔除，
            宽度恰好等于它的段保留。

    Returns:
        :class:`SampleFilterResult`：重建后的采样与被剔除的窄脉冲清单。
    """
    width = require_min_width(min_width)
    items = _validate_samples(samples)

    glitches: list[FilteredPulse] = []
    scl_events = _filter_line(_line_changes(items, "scl"), Line.SCL, width, glitches)
    sda_events = _filter_line(_line_changes(items, "sda"), Line.SDA, width, glitches)
    rebuilt = _rebuild(scl_events, sda_events)

    glitches.sort(key=lambda g: (g.begin_tick, 0 if g.line is Line.SCL else 1))
    return SampleFilterResult(
        samples=tuple(rebuilt),
        filtered=tuple(glitches),
        min_width=width,
    )


# ----------------------------- 接入旧解码器 -----------------------------

def samples_to_updates(samples: Iterable[Sample]) -> list[Update]:
    """把（已滤波的）线采样合成为单一虚拟驱动器 :data:`SAMPLE_DRIVER` 的更新。

    每个采样时刻一条更新：线电平 0 => ``Drive.LOW``，1 => ``Drive.RELEASE``。
    独立使用时同样做完整输入校验。
    """
    items = _validate_samples(samples)
    return [
        Update(
            tick=s.tick,
            driver=SAMPLE_DRIVER,
            scl=Drive.LOW if s.scl == 0 else Drive.RELEASE,
            sda=Drive.LOW if s.sda == 0 else Drive.RELEASE,
        )
        for s in items
    ]


@dataclass(frozen=True)
class SampleParseResult:
    """:func:`parse_samples` 的结果：旧解码器结果 + 滤波记录。"""

    result: ParseResult
    filtered: tuple[FilteredPulse, ...]
    min_width: int

    @property
    def transactions(self) -> tuple:
        return self.result.transactions

    @property
    def drivers(self) -> tuple[str, ...]:
        return self.result.drivers

    @property
    def final_scl(self) -> int:
        return self.result.final_scl

    @property
    def final_sda(self) -> int:
        return self.result.final_sda

    @property
    def span_ticks(self) -> int:
        return self.result.span_ticks


def parse_samples(
    samples: Iterable[Sample],
    min_width: int,
    *,
    stretch_limit: int | None = None,
) -> SampleParseResult:
    """滤波线采样并复用旧事务解码器（纯函数；非法输入整体抛错，无部分结果）。

    Args:
        samples: 非递减的 :class:`Sample` 序列；同一 tick 只允许一条。
        min_width: 最小稳定宽度（正整数），``width < min_width`` 的短脉冲被剔除。
        stretch_limit: 透传给 :func:`odbus.parse_updates` 的 SCL 低电平阈值，
            在**滤波后**的波形上检查。
    """
    width = require_min_width(min_width)
    stretch_limit = require_stretch_limit(stretch_limit)
    filtered_result = filter_samples(samples, width)
    updates = samples_to_updates(filtered_result.samples)
    result = parse_updates(updates, stretch_limit=stretch_limit)
    return SampleParseResult(
        result=result,
        filtered=filtered_result.filtered,
        min_width=width,
    )
