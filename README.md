# busdecode — 双线开漏总线的事务识别

离线嵌入式协议工程用可复用模块:输入各驱动器在整数时刻对 SCL/SDA 的
驱动更新(`release`/`low`),按"线与"合成线值,识别 START/STOP/重复 START、
采样 8 位字节加 ACK、跟踪仲裁丢失与时钟拉伸超时,输出地址/读写/数据/ACK
及每个原始位的来源。

* Python 3.14.7,仅标准库,无需安装任何第三方依赖。
* 完全离线:无网络、无硬件、无账号、无数据库、无 Docker。
* 本模块实现的是题目契约规定的协议子集,**不宣称完整兼容 I2C 行业标准**
  (见"已知限制")。

## 运行

```sh
python -m unittest discover -s tests -v   # 单元测试(手工位序列参考)
python demo.py                            # 演示:读事务 + 仲裁失败 + 输入拒绝
```

## 接口

```python
from busdecode import BusDecoder, Update, Level, Line

dec = BusDecoder(master="m", stretch_threshold=50)
dec.apply(t, [("m", "sda", "low"), ("s", "scl", "release")])  # 同刻一批更新
...
dec.finalize()            # 结束输入:报告缺 STOP / 未解除的拉伸
dec.transactions          # list[Transaction]
dec.events                # list[Event]
dec.scl, dec.sda          # 当前线值(Level)
dec.in_transaction
```

* `apply(t, updates)`:`t` 为整数时刻(单调不减);`updates` 为
  `(driver, line, level)` 三元组或 `Update` 的可迭代对象,`line` 取
  `"scl"/"sda"`(或 `Line`),`level` 取 `"release"/"low"`(或 `Level`)。
* `master`:被跟踪仲裁的主机驱动器名;`None` 表示不跟踪。
* `stretch_threshold`:SCL 低电平超时阈值(整数);`None` 表示不检测。
* `finalize(at=None)`:在时刻 `at`(缺省为最后输入时刻)收尾。

### 输出结构

* `Transaction`:`start_time` / `stop_time`(缺 STOP 为 `None`)/
  `segments` / `complete`。
* `Segment`:一次 START(或重复 START)开启的地址段:`address`(7 位)、
  `read`、`address_ack`、`address_bits`、`address_ack_bit`、`data`。
* `DataByte`:`value`、`ack`(线上 0 为 ACK)、`bits`(8 个 `BitSample`)、
  `ack_bit`。
* `BitSample`:原始位来源 —— 采样时刻 `time`、线值 `value`、
  `role`(`"address"/"data"/"ack"`)、采样时把 SDA 拉低的驱动器
  `low_drivers`。
* `Event`:`kind` ∈ `start`(带 `repeated`)、`stop`、`orphan_stop`、
  `orphan_bit`、`stretch_timeout`(带 `low_duration`)、
  `arbitration_lost`(带 `role`)、`truncated_byte`、`missing_stop`。

## 语义契约(确定行为)

* 线值为所有驱动器的"线与":任一驱动器 `low` 则线为低,否则为高。
  未出现过的驱动器视为一直 `release`。
* 同一时刻的一批更新**先全部合并,再判断边沿**。
* START:SDA 下降且 SCL 本拍前后都为高;STOP:SDA 上升且 SCL 本拍前后
  都为高。因此同拍 SCL 下降 + SDA 下降**不是** START。
* SCL 上升沿采样 SDA 合并后的**新值**(同拍双线变化时以此为准),每 8 位
  后跟 1 个 ACK 位;START 可在事务内重复,开启新地址段。
* 时钟低电平持续时间**严格大于**阈值才报 `stretch_timeout`
  (等于阈值不报),每个低电平期只报一次;`finalize` 时仍低且超时也会报。
* 仲裁丢失:被跟踪主机在地址位或写方向数据位上释放 SDA(发 1)却采样到
  0;ACK 位与读方向数据位不检查;每次事务只报第一次,START 后重新武装。
* STOP/重复 START 的建立时钟会多采一个余数位;不足 8 位的余数在
  STOP/重复 START 时按协议惯例丢弃,不算错误。`finalize` 时仍留有
  未组字节的余数位才报 `truncated_byte`;事务未关闭则报 `missing_stop`,
  事务仍进入 `transactions` 且 `stop_time=None`。

## 错误语义

所有错误是 `ValueError` 的子类(`BusDecodeError` 及其子类),消息中带
定位信息(驱动器、线、时刻、冲突值):

* `InvalidTimeError`:时刻非整数(拒绝 bool、float、NaN、Infinity)、
  为负或超过 `MAX_TIME`;`stretch_threshold` 非整数或越界。
* `TimeOrderError`:时刻倒退。
* `InvalidUpdateError`:更新条目结构、驱动器名、线名、电平非法。
* `ConflictingUpdateError`:同一驱动器在同一批次对同一条线给出冲突电平。
  (同值的重复更新是幂等 no-op。)

`apply` 先完整校验整个批次再改变任何状态,**失败的调用不会留下部分变更**
(测试 `test_failed_batch_leaves_no_partial_change` 覆盖)。

## 输入上限(接口边界)

* 时刻:整数,`0 <= t <= MAX_TIME = 2**63 - 1`,单调不减。
* 单批更新条数 ≤ `MAX_BATCH_SIZE = 10000`。
* 驱动器名:非空字符串,≤ 64 字符。
* 驱动器数量、事务/事件数量不设硬上限,受可用内存约束。

## 误差口径

本模块不产生任何浮点结果:所有输出(时刻、字节值、地址、低电平持续
时间)均为精确整数,不存在浮点误差;非法浮点输入一律拒绝而非截断。

## 设计取舍

* 事件与事务都流式追加到公开列表,调用方按自己的节奏 `apply`,无需回调。
* 边沿判定只依赖"上一拍合并后线值",同拍多驱动器、多线变化行为确定。
* 仲裁检查限定在主机应为发送方的位上,避免把从机发的 0 误判为丢裁。

## 已知限制

* 仅 7 位地址;不支持 10 位地址、广播呼叫、SMBus PEC 等扩展。
* 仲裁只跟踪一个指定主机;不解析多主机各自的后续行为。
* 时钟同步(多主机 SCL 低电平期取最长者)由"线与"自然体现,但模块不
  区分拉伸来自从机还是其他主机。
* 不模拟上拉电阻、电平时间等模拟特性;输入必须是已数字化的整数时刻序列。
