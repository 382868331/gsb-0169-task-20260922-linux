# odbus — 双线开漏总线事务识别模块

面向离线嵌入式协议工程的可复用 Python 库，提供两级输入接口：

1. **驱动声明接口**：各驱动器在整数时刻对 **SCL/SDA** 的驱动状态更新
   （`release` / `low`），按“线与”合并同一时刻更新、识别
   START / STOP / 重复 START、在 SCL 上升沿采样 8 数据位 + 1 ACK、跟踪主机
   仲裁丢失，并在 SCL 低电平持续超过阈值时报告 `stretch_timeout`；
2. **采样接口（本次新增）**：输入改为整数时间戳的 **SCL/SDA 采样电平**
   （逻辑分析仪视角的 0/1），先按给定**最小稳定宽度**滤除短脉冲（毛刺），
   再送入上面同一个事务解码器。

这是一个**有限的教学/诊断子集**，只实现本题契约定下的语义，不宣称与任何完整
行业标准（如 I²C 规范的全部模式）兼容。

## 运行环境

- Python **3.14.7**（仅使用标准库，3.10+ 的语法也可兼容，但以 3.14.7 为验收版本）
- Linux 原生运行；安装 Python 后**无需安装任何其他依赖**
- 全部离线：无网络、无账号、无硬件、无数据库/中间件/Docker

## 目录

```
odbus/              本题库（可 import odbus）
  __init__.py        公开接口汇总
  model.py           数据结构（Update/Sample/Bit/ByteBlock/Segment/Transaction/
                     FilteredGlitch/SampleParseResult/...）
  decoder.py         旧事务解码器（parse_updates / BusRecorder）
  sampler.py         采样前级：最小稳定宽度滤波 + 复用 decoder（parse_samples）
  errors.py          错误类型（全部继承 BusTraceError）
  validation.py      输入校验与接口上限
tests/
  reference.py       独立参考实现 + 确定性波形生成器（不 import odbus）
  test_*.py          手工位序列 / 特性 / 差分 / 仲裁 / 校验 / 采样滤波测试
demo.py             固定输入演示（旧接口 + 采样滤波接口）
```

## 运行命令

```bash
# 测试（题目指定）
python -m unittest discover -s tests -v

# 演示（约 0.01 秒，远低于 90 秒；真实计算，无 sleep、无预录结果）
python demo.py
```

## 公开接口

### 输入

```python
from odbus import Update, Drive, parse_updates

Update(tick: int, driver: str, scl: Drive, sda: Drive)
# tick   整数时刻，非递减；范围 [0, 1_000_000_000]
# driver 非空 str，长度 <= 32；一次解析中不同驱动器 <= 64 个
# scl/sda 该驱动器在该时刻之后持续保持的绝对驱动状态：
#          Drive.RELEASE = 释放（上拉为高，线值 1）
#          Drive.LOW     = 拉低（线值 0）
```

线值 = 所有驱动器的 AND（任一拉低即为低）。驱动器缺省为释放。
更新按 `tick` 非递减排列；**同一 `(tick, driver)` 只允许一条更新**，
即使两条完全相同也抛 `ConflictingUpdateError`（批处理不得隐含二选一）。
不同驱动器在同一时刻的更新先合并，再统一判断边沿。

### 解析

```python
result = parse_updates(updates, stretch_limit=None)
```

- `stretch_limit: int | None`：SCL 允许连续为低的 tick 数。
  某时刻 `duration = tick - low_since` **严格大于** `limit` 时抛
  `StretchTimeoutError`（即 `duration == limit` 仍合法）。`None` 表示不检查。
  该检查对总线上任意 SCL 低区间生效（见“设计取舍”）。
- 纯函数：失败时整体抛错，不返回部分结果；`BusRecorder` 在非法 `add` 时拒绝录入，
  不产生未说明的部分变更，且可对同批记录重复 `build()`。

### 采样输入（毛刺滤波前级，本次新增）

```python
from odbus import Sample, parse_samples

Sample(tick: int, scl: int, sda: int)
# tick 整数时刻，严格递增；范围 [0, 1_000_000_000]
# scl/sda 该时刻采样到的总线电平，整数 0/1（1=高，0=低）；相邻采样间零阶保持

out = parse_samples(samples, min_width=4, stretch_limit=None)
# out: SampleParseResult
```

- `Sample` 是逻辑分析仪视角（线与已在采样器完成），与多驱动器的 `Update`
  不同；`scl/sda` 只接受整数 0/1，拒绝 bool、float、NaN/Infinity、其它整数。
- 采样时刻必须**严格递增**：同一 `tick` 两条采样（即使完全相同）抛
  `DuplicateSampleError`；倒序抛 `InvalidSampleError`。
- `min_width`：最小稳定宽度（tick，正整数，默认 1=不滤波）。滤波对 **SCL、
  SDA 各自独立**进行，规则为：电平从 `Y` 偏离到 `X`、再恢复回 `Y` 的脉冲，
  其宽度按零阶保持口径 `width = 恢复时刻 − 偏离时刻`；
  **`width < min_width` 剔除，`width == min_width` 保留（等号边界）**。
  被剔除脉冲在 `filtered_glitches` 中逐条报告（线/起止时刻/宽度/电平），
  剔除即把该脉冲区间填平为 `Y`；内层毛刺填平后外层偏离仍不足宽度时会
  继续剔除（fixpoint，线性时间）。首段/末段因缺少一侧“恢复确认”，
  即使很窄也按观测值保留，不猜测。
- 滤波后的电平转成单虚拟驱动器 `BUS` 的 `Update`（**每个采样时刻一条绝对
  状态更新**，同刻双线变化天然合并为一条），交给同一个 `parse_updates`，
  因此事务语义与旧接口完全一致；`stretch_limit` 原样透传。
- 返回 `SampleParseResult`：`.result`（标准 `ParseResult`）、
  `.transactions`（透传便捷属性）、`.min_width`、
  `.filtered_glitches: tuple[FilteredGlitch, ...]`。
- 纯函数：校验或解码失败时整体抛错，不返回部分结果。

**同刻变化的固定规则**：一个 Sample 同时承载两根线，允许 SCL、SDA 同刻变化；
滤波阶段两线独立判宽、不移动任何时刻；送入解码器时两线处于同一条 BUS 更新，
与旧接口“同刻先合并再判边沿”一致——SDA 沿与 SCL 沿同刻时 SCL 不满足
“前后均高”，该 SDA 沿不算 START/STOP，SCL 沿仍照常作为脉冲边沿。

### 输出（`ParseResult`）

- `transactions: tuple[Transaction, ...]`
  - `Transaction.segments: list[Segment]`：重复 START 在**同一事务**内开启新段；
  - `Segment.start`（含 `repeated`）、`starters`（START 时拉低 SDA 的驱动器，
    第一个即主机）、`blocks`、`partial_bits`、`arbitration_losses`、`stop`、
    `ended_by_repeated_start`；
  - 便捷属性：`address`（首字节高 7 位）、`rw`（0=写/1=读）、
    `data`（后续数据字节）、`acks`（每字节第 9 拍，0=ACK/1=NACK）、`closed`。
- `ByteBlock`：`value`（MSB first 的 8 位）、`bits[8]`、`ack`，每个 `Bit` 带
  `phase`（address/rw/ack_addr/data/ack_data）与 `BitSource` 原始来源：
  SCL 下降/上升时刻、SCL 下降时拉低 SCL 的驱动器、采样时刻拉低 SDA 的驱动器。
- 顶层另有 `drivers`（首次出现顺序）、`final_scl/final_sda`、`span_ticks`。

### 错误语义

| 错误 | 触发 |
|---|---|
| `InvalidUpdateError` | 字段类型/值域错误；带 `field/index/tick/driver/reason` 可定位字段 |
| `ConflictingUpdateError` | 同一驱动器同一时刻出现第二条更新（含完全重复） |
| `LimitError` | 超过公开上限（时刻、条数、驱动器数、ID 长度、采样条数） |
| `StretchTimeoutError` | SCL 低电平持续严格超过 `stretch_limit`；带 `tick/low_since/duration/limit` |
| `InvalidSampleError` | 采样字段/参数非法（时刻、电平、`min_width`）；带 `field/index/tick/reason` |
| `DuplicateSampleError` | 同一整数时刻出现第二条采样；带 `tick/index/first_index` |

数值输入拒绝 `NaN`/`Infinity`（浮点一律拒绝，时刻只接受整数）；
**`bool` 不被当作整数**（`True`/`False` 显式拒绝）。所有错误继承 `BusTraceError`。

本模块没有“搜索预算”类算法：解析是对更新流的单次线性扫描（O(n)），
因此不存在“预算耗尽 vs 证明无解”的区分场景；输入不可解析即明确抛错。

## 关键语义与设计取舍

1. **边沿判定**：START = SCL 前后均高且 SDA 1→0；STOP = SCL 前后均高且 SDA 0→1。
   同一时刻 SCL 也发生跳变时（如 SDA 降与 SCL 降同刻）不算 START/STOP。
2. **采样**：仅在 SCL 低→高的上升沿采样 SDA；9 个脉冲组成
   8 数据位（MSB first）+ 1 ACK。START 之前的孤立 SCL 脉冲不属于任何事务，忽略。
3. **重复 START**：不产生 STOP，当前段以 `ended_by_repeated_start=True` 结束，
   同一 `Transaction` 内开启新段；Sr 是帧复位，Sr 前不足 9 拍的残余采样位丢弃。
4. **缺 STOP**：流末尾未见 STOP 的事务保留在结果中，`closed=False`；
   字节中途截断时已采样的位保存在 `Segment.partial_bits`（不伪造半截字节）。
   缺 STOP 是一种输出状态，不是异常。
5. **仲裁丢失**：段主机 = START 时刻把 SDA 由释放变为拉低的首个驱动器。
   在**主机发送**的数据位上（地址/RW 八位恒为主机发送；写方向的数据字节也是），
   主机已释放 SDA（发 1）而总线采样为 0，记录 `ArbitrationLoss`。
   读方向的数据位由从机驱动 SDA，主机释放读到 0 是正常的，不报仲裁丢失；
   ACK 位也不参与仲裁判定。
6. **绝对状态模型**：更新表示驱动器在该时刻之后的持续状态（边沿触发于状态变化），
   因此测试/演示里每拍为每个相关驱动器各给一条绝对更新。
7. **拉伸检查窗口**：对总线任意 SCL 连续低区间检查（不限事务内），因为“低超过
   阈值”本身就是总线级异常；低电平期间无更新、直到上升沿才超阈值时也会在上升沿补检。
8. **同刻合并**：先按 tick 分组应用全部驱动器状态，再计算 AND、判边沿与采样，
   严格满足“同刻更新先合并再判断边沿”。
9. **采样毛刺滤波（采样接口）**：`parse_samples` 在解码前对每根线独立做
   最小稳定宽度滤波，宽度口径为零阶保持的 `恢复时刻 − 起始时刻`，
   `width < min_width` 剔除、等号保留；滤波不移动任何时刻，滤波后合成单
   驱动器 `BUS` 更新流复用以上全部语义。被剔除脉冲逐条写入
   `filtered_glitches`，因此“被过滤毛刺”可核对、不会静默消失。

## 误差口径

- 时刻、电平、阈值、字节与位值全部是**整数/枚举离散量**，没有浮点运算，
  故不存在数值容差或舍入误差；边沿归属是精确的集合/不等式判断
  （超时报错条件为精确的 `duration > limit`，边界 `duration == limit` 必不报错）。
- “不能用宽容差掩盖边界错误”在本库体现为：边界相等必须通过、超出 1 tick 必须报错
  （`tests/test_features.py` 中有两侧边界测试）。采样滤波同口径：
  `width == min_width` 的脉冲必须保留、小 1 tick 的脉冲必须剔除
  （`tests/test_sampler.py` 有等号两侧的 SCL/SDA 用例）。

## 输入上限（接口边界）

| 参数 | 上限 |
|---|---|
| `tick`（Update / Sample） | 0 … 1_000_000_000 |
| 单次更新条数（`parse_updates`） | 1_000_000 |
| 单次采样条数（`parse_samples`） | 1_000_000 |
| 不同驱动器数量 | 64 |
| 驱动器 ID 长度 | 32（非空） |
| `stretch_limit` | 1 … 2^31−1 |
| `min_width` | 1 … 2^31−1 |

超限抛 `LimitError` 或 `InvalidUpdateError`。短演示不穷举上限规模。

## 测试策略

- `test_hand_reference.py`：手工逐位构造的位序列，地址/数据/ACK/相位/来源全部硬编码。
- `test_features.py`：重复 START、时钟拉伸（边界两侧）、同刻双线变化、缺 STOP/截断。
- `test_arbitration.py`：地址位/写数据位仲裁丢失，读位与主机主动拉低不误报。
- `test_validation.py`：NaN/Infinity/bool/float/类型、冲突更新、非递减、无部分变更。
- `test_differential.py`：确定性 LCG 生成 48 个场景（写/读/ACK/NACK/Sr/拉伸/缺 STOP），
  由 `tests/reference.py` 中**不导入 odbus** 的独立参考解析器
  （密集 tick 扫描 + 位串累加，与核心的事件分组是两套写法）给出期望值，逐字段对照。
  往返一致只作辅助，不作为期望值来源。
- `test_sampler.py`：采样接口。手工用例覆盖干净写事务（地址/数据/ACK 硬编码）、
  SCL 高毛刺不产生额外采样位、SDA 毛刺不改变上升沿采样值、亚宽度 SDA 下降不产生
  START、宽度 `== min_width` 保留与小 1 剔除、同刻双线变化、重复 START 透传、
  首/末段不滤波；失败用例覆盖重复 tick（含完全相同）、倒序、非法电平/时刻/
  `min_width`、拉伸边界等号与超 1。差分部分由**独立的稠密网格 fixpoint 滤波器**
  `ref_filter_grid`（不导入 odbus）在 40 个确定性带毛刺场景上逐 tick、逐毛刺、
  逐事务字段对照。

## 已知限制

- 不实现完整行业标准的全部内容：没有 10 位地址、时钟同步、通用呼叫、
  尖峰滤波、SDA 建立/保持时间的物理时序校验；tick 是逻辑整数时刻而非真实时间单位。
- 多主机模型只识别“START 发起者”为主机；仲裁失败后不模拟该主机停止驱动后的
  完整状态机切换，只如实记录丢失事件与总线真实采样值。
- 未参与任何事务的空闲 SCL 脉冲被忽略；START 前的总线活动不进入事务输出。
- 更新必须非递减且一次性提供（或经 `BusRecorder` 顺序录入）；不支持乱序插入后重排。
- 采样接口以线与后的 0/1 观测电平为输入，因此没有“是哪个驱动器拉低”的信息：
  滤波后合成的事务来自单虚拟驱动器 `BUS`，不跟踪仲裁丢失（仲裁分析仍需使用
  多驱动器的 `Update` 接口）。
- 毛刺滤波只按“偏离后是否在 min_width tick 内恢复到原电平”判据工作，
  不做模拟域去抖/迟滞；宽度不足的真实短脉冲也会被滤除（这是阈值的直接语义，
  调用方需按总线时序选取 `min_width`）。
