# odbus — 双线开漏总线事务识别模块

面向离线嵌入式协议工程的可复用 Python 库：从各驱动器在整数时刻对 **SCL/SDA** 的
驱动状态更新（`release` / `low`）出发，按“线与”合并同一时刻更新、识别
START / STOP / 重复 START、在 SCL 上升沿采样 8 数据位 + 1 ACK、跟踪主机仲裁丢失，
并在 SCL 低电平持续超过阈值时报告 `stretch_timeout`。

这是一个**有限的教学/诊断子集**，只实现本题契约定下的语义，不宣称与任何完整
行业标准（如 I²C 规范的全部模式）兼容。本版本在原七位地址之外，增加题内十位
寻址形式：首字节高五位为 `11110`，随后两位是地址高两位，最低位是方向；写方向
再接收地址低八位。十位读事务必须先完成该写地址阶段，再在同一事务内用重复
START 发送高位相同的读头。

## 运行环境

- Python **3.14.7**（仅使用标准库，3.10+ 的语法也可兼容，但以 3.14.7 为验收版本）
- Linux 原生运行；安装 Python 后**无需安装任何其他依赖**
- 全部离线：无网络、无账号、无硬件、无数据库/中间件/Docker

## venv 准备

项目只依赖标准库，`requirements.txt` 为空；venv 只是建议的隔离方式：

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
```

如果系统 `python` 已经是 3.14.7，也可以不建 venv，直接运行下方命令。

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
  tenbit_reference.py 十位寻址独立字节状态机（不 import odbus）
  test_*.py          手工位序列 / 特性 / 差分 / 仲裁 / 校验测试
demo.py             固定输入演示
demo_iteration.py   十位寻址增量演示
```

## 运行命令

```bash
# 测试（题目指定）
python -m unittest discover -s tests -v

# 十位寻址增量演示（约 0.05 秒，远低于 90 秒；真实计算，无 sleep、无预录结果）
python demo_iteration.py

# 原基线演示（仍保留）
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

### 输出（`ParseResult`）

- `transactions: tuple[Transaction, ...]`
  - `Transaction.segments: list[Segment]`：重复 START 在**同一事务**内开启新段；
  - `Segment.start`（含 `repeated`）、`starters`（START 时拉低 SDA 的驱动器，
    第一个即主机）、`blocks`、`partial_bits`、`arbitration_losses`、`stop`、
    `ended_by_repeated_start`；
  - 便捷属性：`address`（七位地址或完整十位地址）、`rw`（0=写/1=读）、
    `data`（地址阶段之后的完整数据字节）、`acks`（每字节第 9 拍，0=ACK/1=NACK）、
    `closed`。
  - 十位寻址新增字段：`address_mode`（`"7-bit"`/`"10-bit"`）、`ten_bit_high`、
    `ten_bit_low`、`ten_bit_address_complete`、`ten_bit_address_acknowledged`、
    `address_blocks`（本段参与寻址的完整字节；合法十位读段也引用写地址阶段的
    写头和低字节，以便输出三个地址字节的 ACK 来源位置）、`address_acks`。
- `ByteBlock`：`value`（MSB first 的 8 位）、`bits[8]`、`ack`，每个 `Bit` 带
  `phase`（address/rw/ack_addr/address_low/ack_addr_low/data/ack_data）与
  `BitSource` 原始来源：
  SCL 下降/上升时刻、SCL 下降时拉低 SCL 的驱动器、采样时刻拉低 SDA 的驱动器。
- 顶层另有 `drivers`（首次出现顺序）、`final_scl/final_sda`、`span_ticks`。

### 错误语义

| 错误 | 触发 |
|---|---|
| `InvalidUpdateError` | 字段类型/值域错误；带 `field/index/tick/driver/reason` 可定位字段 |
| `ConflictingUpdateError` | 同一驱动器同一时刻出现第二条更新（含完全重复） |
| `LimitError` | 超过公开上限（时刻、条数、驱动器数、ID 长度） |
| `StretchTimeoutError` | SCL 低电平持续严格超过 `stretch_limit`；带 `tick/low_since/duration/limit` |

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
   例外：十位写地址尚未收全时，Sr 前已采样的残余地址位保存在写段
   `partial_bits`，用于呈现不完整地址状态。
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
9. **十位寻址**：`11110xxW` 是写地址头，`xx` 为地址高两位；只有写头和随后的
   低八位都被 ACK 后，完整地址才有效，低八位不会计入 `data`。同一事务内
   重复 START 后收到 `11110xxR` 且 `xx` 与写地址阶段一致时，读段继承完整地址；
   高位不一致、低地址 NACK、STOP 后另起事务，或写地址未收全就 Sr，都使读段
   保持不完整地址状态，后续字节不会被误当数据。七位地址仍按首字节高 7 位
   的旧逻辑解释；`11110` 前缀属于本题十位寻址形式。

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
- `test_ten_bit_addressing.py`：逐拍生成合法十位写、写后 Sr 转读、低地址 NACK、
  错误读头、高位不一致、STOP 后不复用低字节等场景；期望值来自
  `tests/tenbit_reference.py` 的独立字节状态机，并核对每个寻址字节 ACK 的
  采样时刻和 SDA 低驱动来源。

## 已知限制

- 不实现完整行业标准的全部内容：没有时钟同步、通用呼叫、
  尖峰滤波、SDA 建立/保持时间的物理时序校验；tick 是逻辑整数时刻而非真实时间单位。
- 十位寻址只实现本题契约的 `11110` 两阶段形式；不实现完整行业标准中的
  其它保留地址、广播呼叫、高速模式或 SMBus/PMBus 扩展。
- 多主机模型只识别“START 发起者”为主机；仲裁失败后不模拟该主机停止驱动后的
  完整状态机切换，只如实记录丢失事件与总线真实采样值。
- 未参与任何事务的空闲 SCL 脉冲被忽略；START 前的总线活动不进入事务输出。
- 更新必须非递减且一次性提供（或经 `BusRecorder` 顺序录入）；不支持乱序插入后重排。
