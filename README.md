# odbus — 双线开漏总线事务识别模块

面向离线嵌入式协议工程的可复用 Python 库：从各驱动器在整数时刻对 **SCL/SDA** 的
驱动状态更新（`release` / `low`）出发，按“线与”合并同一时刻更新、识别
START / STOP / 重复 START、在 SCL 上升沿采样 8 数据位 + 1 ACK、跟踪主机仲裁丢失，
在 SCL 低电平持续超过阈值时报告 `stretch_timeout`，并提供**显式总线恢复驱动器模型**：
调用方请求恢复后，模型接管被卡住的主机，先释放 SDA，再最多发 9 个 SCL 脉冲
（只有实测 SCL 真的升高才计数），SDA 释放后生成合法 STOP，九脉冲后仍低则报 `bus_stuck`。

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
# 测试（题目指定；本机验收解释器为 ./.venv/bin/python，Python 3.14.7）
./.venv/bin/python -m unittest discover -s tests -v

# 演示（约 0.06 秒，远低于 90 秒；真实计算，无 sleep、无预录结果）
./.venv/bin/python demo.py
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
result = parse_updates(updates, stretch_limit=None, recoveries=None)
```

- `stretch_limit: int | None`：SCL 允许连续为低的 tick 数。
  某时刻 `duration = tick - low_since` **严格大于** `limit` 时抛
  `StretchTimeoutError`（即 `duration == limit` 仍合法）。`None` 表示不检查。
  该检查对总线上任意 SCL 低区间生效（见“设计取舍”），**恢复期间等待 SCL 升高时
  沿用同一阈值**（异常上 `during_recovery=True`）。
- `recoveries: Iterable[RecoveryRequest] | None`：可选的显式恢复请求（见下节）。
- 纯函数：失败时整体抛错，不返回部分结果；`BusRecorder` 在非法 `add` 时拒绝录入，
  不产生未说明的部分变更，且可对同批记录重复 `build()`。

### 显式总线恢复（新增契约）

```python
from odbus import RecoveryRequest
result = parse_updates(updates, recoveries=[RecoveryRequest(tick=7, master="M")])
record = result.recoveries[0]
```

`RecoveryRequest(tick, master, start_tick=None)` 表示调用方在整数时刻 `tick` 请求
驱动器 `master` 发起一次恢复。**`tick` 必须是更新流中实际存在的时刻**（该刻至少有
一条其他驱动器的更新，恢复模型借此刻接管；不凭空生成起始时刻），且该刻不能再带
`master` 自己的外部更新——接管后该主机由恢复模型驱动，恢复期间调用方再对它提交
更新会被拒绝。同一时刻至多一个恢复请求；恢复未结束时不能叠加第二个请求。

恢复模型本身**就是一个驱动器**：它在每个整数时刻给出一条对 SCL/SDA 的绝对驱动，
与外部驱动器的同刻更新先做线与合并、再判边沿。因此恢复造成的每一次翻转都能在
`RecoveryEvent` 里追溯到具体驱动器的合并，模型不使用任何滤波器、不接真实 GPIO。

操作顺序（契约）：

1. **先释放 SDA**：接管首刻主机对两线都释放。即使此刻总线 SDA 0→1（主机自己或外部
   持有者刚放手），也**不**算 STOP——STOP 只能在某个有效脉冲之后形成。
2. **最多 9 个 SCL 脉冲**：每拍主机拉低 SCL，下一拍放行 SCL。脉冲计数**只看合并后
   实测 SCL 是否真的升高**：若 SCL 被别的驱动器拉住（实测仍低），该拍**不计**脉冲，
   模型继续在后续时刻等待，直到实测升高才计数并采样 SDA。等待期间 `stretch_limit`
   照常生效（`StretchTimeoutError(during_recovery=True)`）；未配置阈值且后续再无
   外部更新可能释放 SCL 时，报 `BusStuckError(scl_held_low=True)`，绝不按计划时间
   假装脉冲成功。
3. **SDA 在某个有效脉冲上升沿被采样为 1（已释放）**：模型先让出一拍（两线释放，
   给其他主机一次发 START 的机会），随后主动生成合法 STOP（SCL 低时拉低 SDA →
   SCL 升高 → SCL 高时释放 SDA）结束恢复，并把恢复前被冻结的那个未闭合段合法关闭。
4. **9 个有效脉冲后 SDA 仍为 0**：抛 `BusStuckError`（异常对象带完整 `record`）。
5. **其他主机在恢复期间发起 START**（SCL 高时 SDA 1→0 由非恢复主机造成）：立即
   **中止**本次恢复（`outcome=ABORTED`，`aborted_by` 记录发起者），模型释放两线，
   不抢占该事务；START 之后的位由正常识别解码。恢复前未走完的旧段保持未闭合。
6. 恢复期间在有效脉冲上升沿采到的位记为 `RecoveryBit(pulse, value, tick,
   scl_low_tick, sda_low_drivers)`，**单独标记**，绝不拼入恢复前未收完的数据字节
   （那些位冻结在 `Segment.partial_bits`），也不进入任何 `ByteBlock`。

`ParseResult.recoveries: tuple[RecoveryRecord, ...]` 每次恢复一条记录：

| 字段 | 含义 |
|---|---|
| `request` / `outcome` | 请求；结局 `stop_generated` / `stop_observed` / `aborted` / `stuck` / `stretch_timeout` |
| `events` | 逐时刻 `RecoveryEvent`：`tick, kind, scl, sda, scl_low_drivers, sda_low_drivers, pulse` |
| `bits` | 各**有效**脉冲采到的 `RecoveryBit`（按脉冲序号 1..9） |
| `pulses_sent` | 实测 SCL 确实升高的脉冲数（被拉住的拍不计） |
| `stop` / `end_tick` | 成功时的 STOP 事件与结束时刻 |
| `stuck_tick` / `aborted_by` / `segment_index` | bus_stuck 确认刻 / 中止者 / 恢复前段的下标 |

`RecoveryEventKind`：`release_sda, pulse_low, wait_scl_low, pulse_counted, bus_idle,
stop_setup_low, stop_setup_rise, stop_wait_scl, stop_release, stop_blocked,
stop_observed, aborted`。常量 `MAX_RECOVERY_PULSES = 9`。

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
  `recoveries`（按生效顺序排列的 `RecoveryRecord`；无恢复请求时为 `()`）。

### 错误语义

| 错误 | 触发 |
|---|---|
| `InvalidUpdateError` | 字段类型/值域错误；带 `field/index/tick/driver/reason` 可定位字段 |
| `ConflictingUpdateError` | 同一驱动器同一时刻出现第二条更新（含完全重复） |
| `LimitError` | 超过公开上限（时刻、条数、驱动器数、ID 长度） |
| `StretchTimeoutError` | SCL 低电平持续严格超过 `stretch_limit`；带 `tick/low_since/duration/limit`；恢复等待 SCL 期间触发时 `during_recovery=True` 且附 `record` |
| `BusStuckError` | 恢复发满 9 个**有效**脉冲 SDA 仍低（`scl_held_low=False`），或等待 SCL 升高时被无限拉住且无阈值/无后续更新（`scl_held_low=True`）；异常对象携带完整 `record` |

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
9. **恢复即驱动器**：恢复模型不是旁路开关，而是在每个整数时刻给出一条绝对驱动的
   虚拟驱动器，与外部更新同刻 AND 合并；因此脉冲是否成立只认**合并后实测 SCL**，
   被别人拉住时不计脉冲、继续等并沿用 `stretch_limit`，不存在“计划上发了就算成功”。
   模型按每整数时刻一拍主动推进（恢复期间即使没有外部更新也会产生模型自己的
   驱动时刻，否则无法对“SCL 无限拉低”超时），合成时刻仍受 `MAX_TICK` 约束。
10. **STOP 必须在有效脉冲之后**：接管首刻的 SDA 释放（无论谁放手）不产生 STOP；
    SDA 采到 1 后模型先让出一拍（`bus_idle`），把这一拍交给其他主机——它若发
    START 就中止恢复；否则模型才生成 STOP。生成 STOP 时若计划释放刻 SDA 又被别人
    拉低（`stop_blocked`），则回去补发脉冲，受同一 9 个有效脉冲额度约束。
11. **恢复位隔离**：恢复脉冲的采样位是 `RecoveryBit`（带脉冲序号 1..9），只用于
    判断“SDA 是否已释放”和溯源，绝不与恢复前的半截字节或后续事务的字节相拼。
    恢复前事务在恢复成功（生成 STOP）时被该 STOP 合法关闭，半截位仍留在
    `partial_bits`；恢复被中止时旧段保持未闭合。

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
- `test_recovery.py`：恢复驱动器模型（新增，24 个用例）：
  - 第 1 / 8 / 9 个有效脉冲后 SDA 释放 → 完整期望事件序列与 STOP 时刻；
  - 九脉冲后始终不放 SDA → `bus_stuck`（九个位全 0、序号 1..9）；
  - SCL 无限拉低：有阈值 → `stretch_timeout(during_recovery=True)` 且脉冲数为 0；
    无阈值且无后续更新 → `BusStuckError(scl_held_low=True)`；被拉伸后释放 →
    只有实测升高的那一拍计数；
  - 其他主机 START → 恢复中止，随后的地址/ACK/STOP 由正常识别完整解码，恢复位
    不混入新事务；
  - 同刻多驱动器更新（请求刻、放行刻的释放与拉伸同刻）先合并再判边沿；
  - **逐事件不变量**：测试侧独立回放外部更新、按契约叠加恢复主机每拍的驱动后
    重算线与，断言每个 `RecoveryEvent` 的 `scl/sda/scl_low_drivers/sda_low_drivers`
    与重算一致，且线值只在有驱动器状态改变的 tick 翻转（无凭空翻转/无滤波器）；
  - 请求校验：请求刻不存在/与主机更新同刻/同刻两请求/恢复重叠/恢复期间写主机/
    bool 伪整数/非可迭代类型；无恢复请求时行为与旧版完全一致。

## 已知限制

- 不实现完整行业标准的全部内容：没有 10 位地址、时钟同步、通用呼叫、
  尖峰滤波、SDA 建立/保持时间的物理时序校验；tick 是逻辑整数时刻而非真实时间单位。
- 多主机模型只识别“START 发起者”为主机；仲裁失败后不模拟该主机停止驱动后的
  完整状态机切换，只如实记录丢失事件与总线真实采样值。
- 未参与任何事务的空闲 SCL 脉冲被忽略；START 前的总线活动不进入事务输出。
- 更新必须非递减且一次性提供（或经 `BusRecorder` 顺序录入）；不支持乱序插入后重排。
- 恢复模型是本题契约下的**驱动器模型**而非真实主控：它按固定的每整数时刻一拍
  节奏推进（下降一拍、放行一拍，等待 SCL 时每个观测 tick 一拍），不模拟真实 I²C
  主控的时钟同步握手、也不模拟主机在恢复后重新寻址；恢复只负责把总线收回到
  合法 STOP 空闲态或如实报 `bus_stuck`。
- `RecoveryOutcome.STOP_OBSERVED` 与事件 `stop_observed` 是为“恢复期间总线自然
  形成 STOP”保留的出口；在当前严格一拍一驱动的模型下，SDA 一旦在有效脉冲采到 1，
  模型下一拍即接管造 STOP，外部没有窗口抢先形成 STOP，因此正常路径终局为
  `STOP_GENERATED`；该枚举保留以覆盖未来放宽节奏时的语义，不代表当前可达路径。
- 恢复请求只在更新流**已给出**的整数时刻生效（该刻至少一条其他驱动器的更新）；
  恢复期间模型合成的驱动时刻最多推进到 `MAX_TICK`，不支持跨“无更新也无模型活动”
  的真实时间段——等待必须通过后续外部更新（释放者）或 `stretch_limit` 结束。
