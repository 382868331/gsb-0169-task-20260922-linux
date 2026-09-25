# odbus — 双线开漏总线事务识别模块

面向离线嵌入式协议工程的可复用 Python 库：从各驱动器在整数时刻对 **SCL/SDA** 的
驱动状态更新（`release` / `low`）出发，按“线与”合并同一时刻更新、识别
START / STOP / 重复 START、在 SCL 上升沿采样 8 数据位 + 1 ACK、跟踪主机仲裁丢失，
并在 SCL 低电平持续超过阈值时报告 `stretch_timeout`。
另含**总线恢复驱动器模型**：调用方请求恢复后，主机先释放 SDA，再最多发 9 个
SCL 脉冲（必须真正观察到 SCL 升高才计数），SDA 释放后生成合法 STOP，
打满脉冲仍被拉低则报告 `bus_stuck`。

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
  model.py           数据结构（Update/Bit/ByteBlock/Segment/Transaction/...）
  decoder.py         核心算法（parse_updates / BusRecorder）
  errors.py          错误类型（全部继承 BusTraceError）
  validation.py      输入校验与接口上限
tests/
  reference.py       独立参考实现 + 确定性波形生成器（不 import odbus）
  test_*.py          手工位序列 / 特性 / 差分 / 仲裁 / 校验测试
demo.py             固定输入演示
```

## 运行命令

```bash
# 测试（题目指定）
python -m unittest discover -s tests -v

# 演示（约 0.05 秒，远低于 8 秒；真实计算，无 sleep、无预录结果）
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
result = parse_updates(updates, stretch_limit=None, recovery=None, trace=False)
```

- `stretch_limit: int | None`：SCL 允许连续为低的 tick 数。
  某时刻 `duration = tick - low_since` **严格大于** `limit` 时抛
  `StretchTimeoutError`（即 `duration == limit` 仍合法）。`None` 表示不检查。
  该检查对总线上任意 SCL 低区间生效（见“设计取舍”）。
- `recovery: RecoveryRequest | None`：请求一次总线恢复（见“总线恢复”节）。
- `trace: bool`：为 `True` 时 `ParseResult.trace` 输出逐事件合并后的实际线值
  （`LineSample`：tick、SCL/SDA 线值、当时拉低两线的驱动器集合），
  供诊断与测试核对“所有翻转来自驱动器合并”。
- 纯函数：失败时整体抛错，不返回部分结果；`BusRecorder` 在非法 `add` 时拒绝录入，
  不产生未说明的部分变更，且可对同批记录重复 `build()`。

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
- 顶层另有 `drivers`（首次出现顺序）、`final_scl/final_sda`、`span_ticks`、
  `recoveries`（恢复记录，未请求恢复时为空）、`trace`（逐事件线值）。

### 总线恢复（驱动器模型）

```python
from odbus import RecoveryRequest, RecoveryOutcome

result = parse_updates(updates, stretch_limit=L,
                       recovery=RecoveryRequest(host="M", tick=T, max_pulses=9))
```

从请求时刻 `tick` 起，主机 `host` 由恢复驱动器模型接管（恢复进行期间该主机的
输入更新被 `InvalidUpdateError` 拒绝；恢复结束后可恢复输入）。模型与外部更新
在**同一套同刻合并 + 边沿判定**里运行，每 tick 至多一个动作：

1. 主机先释放 SDA；若总线 SDA 已为高，直接进入第 3 步（0 个脉冲）。
2. 否则逐个发 SCL 脉冲（拉低 → 释放）。**只有真正观察到 SCL 升高才计数**；
   SCL 被他人拉住时主机保持释放并等待，等待期间沿用同一个 `stretch_timeout`
   检查（`duration > stretch_limit` 抛 `StretchTimeoutError`），
   绝不凭计划时刻假装脉冲成功。
3. 每个有效脉冲的上升沿采样 SDA：采到 1 表示总线已释放，主机走
   “SCL 低 + SDA 低 → 释放 SCL → 释放 SDA” 生成**合法 STOP** 结束
   （`RecoveryOutcome.STOPPED`，事务按正常 STOP 语义闭合）。
4. 第 `max_pulses`（默认 9，接口上限 64）个有效脉冲后 SDA 仍被拉低
   → 抛 `BusStuckError`（bus_stuck），异常携带已完成的脉冲与恢复记录。
5. 恢复期间其他主机的 START 边沿真正成立时，本次恢复立即中止
   （`RecoveryOutcome.ABORTED`），主机释放两线，该主机的事务由正常识别
   继续处理，不抢占。
6. 更新流结束时恢复未了结 → `RecoveryOutcome.INCOMPLETE`
   （一种输出状态，不是异常，与“缺 STOP”同类）。

恢复期间采到的位记入 `RecoveryRecord.bits`（`phase=BitPhase.RECOVERY`），
**不拼进**之前没收完的数据字节（之前残余的位仍按既有规则留在
`partial_bits` 或随 STOP 丢弃）。每个有效脉冲记为 `RecoveryPulse`
（拉低时刻、**实际**升高时刻、采到的 SDA、是否被拉伸等待过）。
一次解析最多挂一次恢复请求。

### 错误语义

| 错误 | 触发 |
|---|---|
| `InvalidUpdateError` | 字段类型/值域错误；带 `field/index/tick/driver/reason` 可定位字段 |
| `ConflictingUpdateError` | 同一驱动器同一时刻出现第二条更新（含完全重复） |
| `LimitError` | 超过公开上限（时刻、条数、驱动器数、ID 长度、max_pulses） |
| `StretchTimeoutError` | SCL 低电平持续严格超过 `stretch_limit`；带 `tick/low_since/duration/limit` |
| `BusStuckError` | 恢复打满 `max_pulses` 个有效脉冲后 SDA 仍被拉低；带 `host/tick/pulses/sda_low_drivers/record` |

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
   恢复等待 SCL 升高期间沿用同一检查。
8. **同刻合并**：先按 tick 分组应用全部驱动器状态，再计算 AND、判边沿与采样，
   严格满足“同刻更新先合并再判断边沿”。恢复主机的动作也在同一分组里参与合并。
9. **恢复 STOP 的生成**：SDA 已为高时主机不能直接在 SCL 高时拉低 SDA
   （那会是 START），因此 STOP 走 “SCL 低 + SDA 低 → 放 SCL → 放 SDA” 三步；
   其中释放 SCL 的上升沿同样会被采样，该位计入 `RecoveryRecord.bits`。

## 误差口径

- 时刻、电平、阈值、字节与位值全部是**整数/枚举离散量**，没有浮点运算，
  故不存在数值容差或舍入误差；边沿归属是精确的集合/不等式判断
  （超时报错条件为精确的 `duration > limit`，边界 `duration == limit` 必不报错）。
- “不能用宽容差掩盖边界错误”在本库体现为：边界相等必须通过、超出 1 tick 必须报错
  （`tests/test_features.py` 中有两侧边界测试）。

## 输入上限（接口边界）

| 参数 | 上限 |
|---|---|
| `tick` | 0 … 1_000_000_000 |
| 单次更新条数 | 1_000_000 |
| 不同驱动器数量 | 64 |
| 驱动器 ID 长度 | 32（非空） |
| `stretch_limit` | 1 … 2^31−1 |
| `max_pulses`（恢复） | 1 … 64（题面契约默认 9） |

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
- `test_recovery.py`：总线恢复——第 1/8/9 个有效脉冲后释放、SCL 被无限拉低
  （stretch_timeout 生效且脉冲不按计划时间计数）、始终不放 SDA（bus_stuck）、
  其他主机 START 中止恢复、恢复位不拼进残余字节、逐事件线值与驱动器合并一致性
  （`trace=True`）、同刻主机动作与从机更新合并、恢复参数校验。

## 已知限制

- 不实现完整行业标准的全部内容：没有 10 位地址、时钟同步、通用呼叫、
  尖峰滤波、SDA 建立/保持时间的物理时序校验；tick 是逻辑整数时刻而非真实时间单位。
- 多主机模型只识别“START 发起者”为主机；仲裁失败后不模拟该主机停止驱动后的
  完整状态机切换，只如实记录丢失事件与总线真实采样值。
- 未参与任何事务的空闲 SCL 脉冲被忽略；START 前的总线活动不进入事务输出。
- 更新必须非递减且一次性提供（或经 `BusRecorder` 顺序录入）；不支持乱序插入后重排。
- 恢复模型每 tick 至多一个动作；恢复生成的内部时刻不占用输入更新条数上限，
  极端情况下（请求时刻贴近 `MAX_TICK`）内部时刻可略超 `MAX_TICK`。
- 恢复中止只在其他主机的 START 边沿**真正成立**（SCL 前后均高且 SDA 1→0）时触发；
  与主机动作同刻被合并掉的 START 尝试不产生中止。
- 恢复 STOP 生成的最后一步若 SDA 仍被他人拉住（STOP 未成立），
  恢复以 `INCOMPLETE` 了结而不是重试。
- 一次解析只支持一次恢复请求；不串联多次恢复。
- 恢复期间的采样位只进 `RecoveryRecord.bits`，不参与字节拼装与仲裁判定；
  恢复前已残余的位按既有规则处理（STOP 丢弃 / 流末进 `partial_bits`）。
