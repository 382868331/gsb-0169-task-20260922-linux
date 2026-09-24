"""整数时间戳 SCL/SDA 采样的前级：最小稳定宽度毛刺滤波 + 复用事务解码器。

输入是逻辑分析仪视角的 :class:`~odbus.model.Sample` 流（每个采样给出两根线的
0/1 线电平，相邻采样之间零阶保持），与旧接口的“多驱动器驱动声明”
(:class:`~odbus.model.Update`) 不同。

处理流水线：

1. 严格校验采样（类型/值域/严格递增时刻/同刻重复），失败整体抛错；
2. 对每根线各自做**最小稳定宽度滤波**：宽度严格小于 ``min_width`` 的异电平
   脉冲判为毛刺并剔除（宽度等于阈值必须保留——等号边界）；
3. 把滤波后的逐时刻合并电平转换成单虚拟驱动器 ``BUS`` 的
   :class:`~odbus.model.Update`，交给既有事务解码器 :func:`parse_updates`，
   因此 START/STOP/重复 START/仲裁/拉伸等语义全部沿用旧实现。

脉冲宽度口径（零阶保持）
------------------------

采样序列 Y@t0, X@t1, Y@t2 表示 X 脉冲占据区间 [t1, t2)，宽度
``width = t2 - t1``（X 首次出现到恢复为 Y 的 tick 差），不依赖采样是否密集。

* ``width < min_width``  => 剔除（:class:`FilteredGlitch`）；
* ``width == min_width`` => 保留（等号边界）。

被剔除的脉冲必须两侧都是相同电平（一次“偏离后恢复”）；与首段/末段相邻的
短区间缺少一侧的恢复确认，按观测值保留，不做猜测。连续多个短区间时按
fixpoint 反复合并（内层毛刺剔除后，外层仍不足宽度的偏离继续剔除），算法
用运行段栈在线性时间内完成。

同刻变化的固定规则
------------------

一个 Sample 同时给出两根线，允许 SCL 与 SDA 在同一时刻变化；合并规则与旧
解码器的“同刻先合并再判边沿”严格一致：

* 滤波时两根线**各自独立**判毛刺（只看本线自己的区间宽度），滤波不重排、
  不移动任何时刻；
* 转换后同一 tick 至多一条 BUS 更新（两根线在一条更新里），天然同刻合并；
* SDA 沿与 SCL 沿同刻时，SCL 不满足“前后均高”，因此该 SDA 沿不算
  START/STOP（旧解码器语义），SCL 沿本身照常作为脉冲边沿处理。
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from typing import Iterable

from .decoder import parse_updates
from .errors import DuplicateSampleError, InvalidSampleError, LimitError
from .model import (
    Drive,
    FilteredGlitch,
    Line,
    Sample,
    SampleParseResult,
    Update,
)
from .validation import (
    MAX_SAMPLE_COUNT,
    require_level,
    require_min_width,
    require_sample_tick,
)

#: 滤波后合成的单虚拟驱动器名（采样视角没有多驱动器，线与已在采样器完成）。
BUS_DRIVER = "BUS"


@dataclass
class _Run:
    """单根线上的一个同电平运行段（合并后跨度可能覆盖被剔除的内层区间）。

    Attributes:
        level: 区间电平。
        start: 覆盖起点（用于按 tick 查电平）。
        end: 覆盖终点（最后一个该电平采样时刻）。
        edge: 本区间作为“右侧恢复确认”时，恢复电平实际开始的 tick；
            未经过合并时等于 ``start``；若本区间是合并 ``left/mid/right``
            得到的，则继承原 ``right.edge``（嵌套毛刺剔除时宽度仍正确）。
    """

    level: int
    start: int
    end: int
    edge: int = -1

    def __post_init__(self) -> None:
        if self.edge < 0:
            self.edge = self.start


def _validate_samples(samples: Iterable[Sample]) -> list[Sample]:
    result: list[Sample] = []
    last_tick: int | None = None
    for idx, s in enumerate(samples):
        if idx >= MAX_SAMPLE_COUNT:
            raise LimitError(name="采样条数", value=idx + 1, limit=MAX_SAMPLE_COUNT)
        if not isinstance(s, Sample):
            raise InvalidSampleError(
                field="sample",
                reason=f"必须为 odbus.Sample，收到 {type(s).__name__}",
                index=idx,
            )
        tick = require_sample_tick(s.tick, index=idx)
        scl = require_level(s.scl, field="scl", index=idx, tick=tick)
        sda = require_level(s.sda, field="sda", index=idx, tick=tick)
        if last_tick is not None and tick < last_tick:
            raise InvalidSampleError(
                field="tick",
                reason=f"采样时刻必须严格递增：tick={tick} 早于上一条 tick={last_tick}",
                index=idx,
                tick=tick,
            )
        last_tick = tick
        result.append(Sample(tick=tick, scl=scl, sda=sda))
    return result


def _runs(levels: list[tuple[int, int]]) -> list[_Run]:
    """把 ``[(tick, level), ...]`` 压缩成同电平运行段（输入按 tick 严格递增）。"""
    runs: list[_Run] = []
    for tick, level in levels:
        if runs and runs[-1].level == level:
            runs[-1].end = tick
        else:
            runs.append(_Run(level=level, start=tick, end=tick))
    return runs


def _filter_line(
    levels: list[tuple[int, int]],
    line: Line,
    min_width: int,
) -> tuple[list[_Run], list[FilteredGlitch]]:
    """单根线最小稳定宽度滤波（运行段栈，O(m) fixpoint）。

    每读入一个运行段就检查栈顶三个 ``left/mid/right``：若 ``mid`` 两侧电平
    相同且 ``right.start - mid.start < min_width``（零阶保持口径的脉冲宽度），
    则 ``mid`` 是毛刺——记录并把左右合并为一个连续区间，然后继续检查
    （内层毛刺剔除后外层短偏离会继续被剔除）。首段/末段永远不会成为
    ``mid``，因此缺少恢复确认的区间按观测值保留。
    """
    stack: list[_Run] = []
    glitches: list[FilteredGlitch] = []
    for run in _runs(levels):
        stack.append(run)
        while len(stack) >= 3:
            left, mid, right = stack[-3], stack[-2], stack[-1]
            width = right.edge - mid.start
            if not (
                left.level == right.level
                and mid.level != left.level
                and width < min_width
            ):
                break
            glitches.append(
                FilteredGlitch(
                    line=line,
                    start_tick=mid.start,
                    end_tick=right.edge,
                    width=width,
                    level=mid.level,
                )
            )
            # 合并为一个连续的 left 电平区间；edge 继承 right 的恢复时刻，
            # 使本区间日后充当更外层毛刺的“右侧恢复确认”时宽度口径不变。
            stack[-3:] = [
                _Run(level=left.level, start=left.start, end=right.end,
                     edge=right.edge)
            ]
    return stack, glitches


def _level_at(runs: list[_Run], starts: list[int], tick: int) -> int:
    """ surviving 运行段覆盖下 tick 时刻的电平（tick 必落在某段内）。"""
    return runs[bisect_right(starts, tick) - 1].level


def parse_samples(
    samples: Iterable[Sample],
    *,
    min_width: int = 1,
    stretch_limit: int | None = None,
) -> SampleParseResult:
    """解析整数时间戳 SCL/SDA 采样流：先滤短脉冲，再跑事务解码器。

    Args:
        samples: 按 tick **严格递增**排列的 :class:`Sample` 可迭代对象；
            同一 tick 出现两条（含完全相同）抛 :class:`DuplicateSampleError`。
        min_width: 最小稳定宽度（tick）。异电平脉冲宽度
            ``恢复时刻 - 起始时刻`` **严格小于** ``min_width`` 才作为毛刺剔除；
            宽度等于阈值的脉冲保留。``min_width=1``（默认）不滤除任何脉冲。
        stretch_limit: 透传给事务解码器的 SCL 低电平阈值（语义同
            :func:`parse_updates`）。

    失败时整体抛错，不返回部分结果。
    """
    min_width = require_min_width(min_width)
    items = _validate_samples(samples)

    # 同刻重复采样（严格递增；倒序已在 _validate_samples 拒绝）。
    for idx in range(1, len(items)):
        if items[idx].tick == items[idx - 1].tick:
            raise DuplicateSampleError(
                tick=items[idx].tick, index=idx, first_index=idx - 1
            )

    scl_runs, scl_glitches = _filter_line(
        [(s.tick, s.scl) for s in items], Line.SCL, min_width
    )
    sda_runs, sda_glitches = _filter_line(
        [(s.tick, s.sda) for s in items], Line.SDA, min_width
    )
    scl_starts = [r.start for r in scl_runs]
    sda_starts = [r.start for r in sda_runs]

    # 报告按（起始时刻、线）排序，确定且可定位。
    glitches = sorted(
        scl_glitches + sda_glitches,
        key=lambda g: (g.start_tick, 0 if g.line is Line.SCL else 1),
    )

    # 合成单虚拟驱动器 BUS 的绝对状态更新：**每个采样时刻都生成一条更新**
    # （即使电平未变也作为“保持声明”）。这样：
    #   * 两根线同刻变化天然合并在一条更新里；
    #   * 电平保持期间的采样时刻仍存在于更新流中，旧解码器的拉伸检查
    #     （只在更新时刻触发）能在采样点看到持续低电平，不会因稀疏省略漏检。
    bus_updates: list[Update] = []
    for s in items:
        cur_scl = _level_at(scl_runs, scl_starts, s.tick)
        cur_sda = _level_at(sda_runs, sda_starts, s.tick)
        bus_updates.append(
            Update(
                tick=s.tick,
                driver=BUS_DRIVER,
                scl=Drive.RELEASE if cur_scl == 1 else Drive.LOW,
                sda=Drive.RELEASE if cur_sda == 1 else Drive.LOW,
            )
        )

    result = parse_updates(bus_updates, stretch_limit=stretch_limit)
    return SampleParseResult(
        result=result,
        min_width=min_width,
        filtered_glitches=tuple(glitches),
    )
