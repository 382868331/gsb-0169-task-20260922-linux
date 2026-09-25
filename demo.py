#!/usr/bin/env python3
"""odbus 固定输入演示（确定性波形，真实计算，无网络/硬件/sleep/预录结果）。

运行：python demo.py

四部分：
1. 正常读事务：主机 M 读从机 E（地址 0x53）的数据 0x9E，含每一位的原始来源；
2. 仲裁失败：主机 M 发送 1 却被竞争者 C 拉低，解析出 ArbitrationLoss；
3. 异常边界（实际触发，非模拟）：
   a. 同一驱动器同一时刻两条更新 -> ConflictingUpdateError；
   b. SCL 持续低超过阈值 -> StretchTimeoutError。
4. 显式总线恢复（recoveries）：
   a. SDA 在第 8 个**有效**脉冲后释放 -> 逐事件线值 + 合法 STOP；
   b. 九个有效脉冲后 SDA 仍低 -> BusStuckError；
   c. SCL 被别的驱动器无限拉住 -> 恢复期间沿用 stretch_limit 报 StretchTimeoutError；
   d. 其他主机发 START -> 恢复中止，它的事务被正常识别（不抢占）。
"""

from __future__ import annotations

from odbus import (
    Drive,
    RecoveryEventKind,
    RecoveryRequest,
    Update,
    parse_updates,
)
from odbus.errors import (
    BusStuckError,
    ConflictingUpdateError,
    StretchTimeoutError,
)

R = Drive.RELEASE
D = Drive.LOW


class Wave:
    """固定 2-tick 脉冲节奏的波形构建器（绝对驱动状态模型）。

    每个 tick 为所有驱动器各提交一条绝对状态更新（同刻先合并再判边沿）。
    """

    def __init__(self, *names: str) -> None:
        self.t = 0
        self.names = names
        self.st = {n: [R, R] for n in names}
        self.updates: list[Update] = []

    def commit_all(self) -> None:
        for n in self.names:
            scl, sda = self.st[n]
            self.updates.append(Update(tick=self.t, driver=n, scl=scl, sda=sda))

    def advance(self, n: int = 1) -> None:
        self.t += n

    def bit_pulse(self, owner: str, sda: Drive) -> None:
        """一个 SCL 脉冲：下降沿 owner 放好 SDA（其余驱动器释放 SDA），上升沿采样。

        SCL 始终由 M 控制；非 owner 的驱动器在该拍释放 SDA，避免拉住总线。
        """
        for n in self.names:
            self.st[n][1] = sda if n == owner else R
        self.st["M"][0] = D
        self.commit_all()
        self.advance()
        self.st["M"][0] = R
        self.commit_all()
        self.advance()

    def start(self) -> None:
        self.st["M"] = [R, D]
        for n in self.names:
            if n != "M":
                self.st[n] = [R, R]
        self.commit_all()
        self.advance()

    def stop_after_nack_high(self) -> None:
        """NACK（总线 SDA 高）后：补一个脉冲拉低 SDA，再 SCL 高时释放 => STOP。"""
        self.st["M"] = [D, D]
        self.commit_all()
        self.advance()
        self.st["M"] = [R, D]
        self.commit_all()
        self.advance()
        self.st["M"] = [R, R]
        self.commit_all()
        self.advance()


def build_read_transaction() -> list[Update]:
    """M 读 E(0x53)：START + 地址读字节(从机ACK) + 数据0x9E(主机NACK) + STOP。"""
    w = Wave("M", "E")
    w.start()
    addr = (0x53 << 1) | 1  # 0xA7 读
    for k in range(8):
        w.bit_pulse("M", R if ((addr >> (7 - k)) & 1) else D)
    w.bit_pulse("E", D)     # 从机 ACK=0
    data = 0x9E
    for k in range(8):
        w.bit_pulse("E", R if ((data >> (7 - k)) & 1) else D)
    w.bit_pulse("M", R)     # 主机 NACK=1
    w.stop_after_nack_high()
    return w.updates


def build_arbitration_case() -> list[Update]:
    """M 与 C 同起始；地址首位 M 发 1、C 拉低 => M 仲裁丢失，随后补齐 9 拍到 STOP。"""
    ups: list[Update] = []
    t = 0
    ups += [Update(t, "M", R, D), Update(t, "C", R, D)]  # 同时 START
    t += 1
    # bit7：M 释放 SDA（发1），C 拉低（发0）
    ups += [Update(t, "M", D, R), Update(t, "C", R, D)]
    t += 1
    ups += [Update(t, "M", R, R), Update(t, "C", R, D)]  # 上升采样=0 => 丢失
    t += 1
    # 其余 7 位双方都释放（读到全 1），第 9 拍 C 给 ACK=0
    for _ in range(7):
        ups += [Update(t, "M", D, R), Update(t, "C", R, R)]
        t += 1
        ups += [Update(t, "M", R, R), Update(t, "C", R, R)]
        t += 1
    ups += [Update(t, "M", D, R), Update(t, "C", R, D)]
    t += 1
    ups += [Update(t, "M", R, R), Update(t, "C", R, D)]
    t += 1
    # STOP：C 释放 / M 接管（同刻 AND 保持低），下一拍 M 释放 => STOP
    ups += [Update(t, "M", R, D), Update(t, "C", R, R)]
    t += 1
    ups += [Update(t, "M", R, R)]
    return ups


def print_header(title: str) -> None:
    bar = "=" * 68
    print(f"\n{bar}\n{title}\n{bar}")


# ---------------- 显式恢复波形 ----------------

def stuck_prefix():
    """M 发 START 后地址位收到一半（1,0,0），SDA 被卡死在低电平。返回 (updates, req_tick)。"""
    ups = [Update(0, "M", R, D)]
    t = 1
    for b in (1, 0, 0):
        ups += [Update(t, "M", D, R if b else D)]
        t += 1
        ups += [Update(t, "M", R, R if b else D)]
        t += 1
    # 请求刻必须有（其他驱动器的）外部更新：从机 X 在此刻声明拉住 SDA。
    ups.append(Update(t, "X", R, D))
    return ups, t


def print_recovery_events(rec, *, max_rows=None) -> None:
    print("逐事件（实际线值 = 全部驱动器含恢复模型的线与）：")
    evs = rec.events if max_rows is None else rec.events[:max_rows]
    for e in evs:
        p = f" 脉冲#{e.pulse}" if e.pulse else ""
        scl_who = ",".join(e.scl_low_drivers) or "-"
        sda_who = ",".join(e.sda_low_drivers) or "-"
        print(f"  t{e.tick:<3} {e.kind.value:<15} SCL={e.scl}(低:{scl_who}) "
              f"SDA={e.sda}(低:{sda_who}){p}")
    if max_rows is not None and len(rec.events) > max_rows:
        print(f"  ...（共 {len(rec.events)} 个事件，省略其余）")
    print("恢复期间单独标记的位：", [(b.pulse, b.value) for b in rec.bits])


def main() -> None:
    # ---------- 1. 正常读事务 ----------
    print_header("1) 正常结果：一次读事务（主机 M 读从机 E，地址 0x53）")
    result = parse_updates(build_read_transaction())
    tx = result.transactions[0]
    seg = tx.segments[0]
    print(f"事务数            : {len(result.transactions)}（已闭合 STOP={tx.closed}）")
    print(f"驱动器（注册序）  : {result.drivers}")
    print(f"地址              : 0x{seg.address:02X}  读写位={seg.rw}（1=读）")
    print(f"数据字节          : {' '.join(f'0x{x:02X}' for x in seg.data)}")
    print(f"ACK（每字节第9拍）: {seg.acks}  （0=ACK, 1=NACK）")
    print("原始位来源（数据字节 0x9E = 10011110，由从机 E 驱动 SDA）：")
    data_block = seg.blocks[1]
    for b in data_block.bits:
        who = ",".join(b.source.sda_low_drivers) or "无（上拉为1）"
        print(f"  位{b.index} 值={b.value} 采样@t{b.tick:<2} "
              f"SCL {b.source.scl_low_tick}->{b.source.scl_high_tick} "
              f"SDA低驱动=[{who}]")
    print(f"第 9 拍 NACK      : 值={data_block.ack.value} "
          f"SDA低驱动={data_block.ack.source.sda_low_drivers or '（无，读为1=NACK）'}")

    # ---------- 2. 仲裁失败 ----------
    print_header("2) 仲裁失败：主机 M 发 1，竞争者 C 同刻拉低 SDA")
    res2 = parse_updates(build_arbitration_case())
    seg2 = res2.transactions[0].segments[0]
    print(f"首字节总线实际值  : 0x{seg2.blocks[0].value:02X}（M 期望的首位 1 被裁成 0）")
    for loss in seg2.arbitration_losses:
        print(f"  ArbitrationLoss @t{loss.tick} 字节内位{loss.bit_index} "
              f"阶段={loss.phase.value} 主机={loss.master} "
              f"读到={loss.observed} 拉低者={loss.driven_low_by}")

    # ---------- 3a. 拒绝：同一驱动器同一时刻冲突更新 ----------
    print_header("3a) 异常边界（实际触发）：同一驱动器同刻冲突更新被拒绝")
    conflict = [
        Update(0, "M", R, R),
        Update(1, "M", D, R),
        Update(1, "M", R, D),   # 与上一条同 tick 同驱动器
    ]
    try:
        parse_updates(conflict)
    except ConflictingUpdateError as e:
        print(f"捕获 {type(e).__name__}: {e}")

    # ---------- 3b. 异常：时钟拉伸超时 ----------
    print_header("3b) 异常边界（实际触发）：SCL 低电平超过 stretch_limit")
    stretch_ups = [
        Update(0, "M", R, D),          # START
        Update(1, "M", D, D),          # SCL 降，低电平开始
        Update(2, "M", D, D),
        Update(3, "M", D, D),
        Update(4, "M", D, D),
        Update(5, "M", D, D),          # 到 t=5，duration=4 > limit=3
    ]
    try:
        parse_updates(stretch_ups, stretch_limit=3)
    except StretchTimeoutError as e:
        print(f"捕获 {type(e).__name__}: {e}")

    # ---------- 4a. 恢复：第 8 个有效脉冲后 SDA 释放 ----------
    print_header("4a) 显式恢复：SDA 在第 8 个有效脉冲后释放，生成合法 STOP")
    ups, t0 = stuck_prefix()
    # 从机 X 一直拉住 SDA，直到第 8 个脉冲的上升刻（t0 + 16）同刻释放。
    ups.append(Update(t0 + 16, "X", R, R))
    result = parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
    rec = result.recoveries[0]
    seg = result.transactions[0].segments[0]
    print(f"结局={rec.outcome.value}  实测有效脉冲数={rec.pulses_sent}  "
          f"STOP@t{rec.stop.tick}  事务闭合={result.transactions[0].closed}")
    print_recovery_events(rec, max_rows=20)
    print(f"恢复前半截地址位被冻结（不与恢复位相混）：{[b.value for b in seg.partial_bits]}")

    # ---------- 4b. 恢复：九脉冲后仍不放 SDA -> bus_stuck ----------
    print_header("4b) 显式恢复：九个有效脉冲后 SDA 仍被拉低 -> bus_stuck")
    ups, t0 = stuck_prefix()   # X 永不放 SDA、不碰 SCL
    try:
        parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
    except BusStuckError as e:
        print(f"捕获 {type(e).__name__}: {e}")
        print(f"有效脉冲数={e.pulses}（全 9 个都实测到 SCL 升高），"
              f"确认时刻=t{e.tick}，SDA 拉低者={e.sda_low_drivers}")

    # ---------- 4c. 恢复：SCL 被无限拉住 -> 恢复期间 stretch_timeout ----------
    print_header("4c) 显式恢复：SCL 被其他驱动器无限拉住 -> stretch_timeout")
    ups, t0 = stuck_prefix()
    # 请求刻 X 同时拉住 SCL 且之后不放：恢复主机放行 SCL 也观察不到升高，脉冲计数为 0。
    # 把 X 的请求刻状态改成 SCL=LOW（覆盖 4a 里构造的 RELEASE）。
    ups[-1] = Update(t0, "X", D, D)
    try:
        parse_updates(
            ups, stretch_limit=3, recoveries=[RecoveryRequest(t0, "M")]
        )
    except StretchTimeoutError as e:
        print(f"捕获 {type(e).__name__}: {e}")
        print(f"有效脉冲数={e.record.pulses_sent}（从未实测到 SCL 升高，一个都不计），"
              f"结局={e.record.outcome.value}")

    # ---------- 4d. 恢复：其他主机 START 中止 ----------
    print_header("4d) 显式恢复：第 1 个有效脉冲后其他主机 N 发 START，恢复中止")
    ups, t0 = stuck_prefix()
    # X 在第 1 个脉冲上升刻（t0+2）释放 SDA；模型下一拍 idle（t0+3）时 N 发 START。
    ups += [Update(t0 + 2, "X", R, R), Update(t0 + 3, "N", R, D)]
    t = t0 + 4
    # N 发地址字节 0x42<<1|0 = 0x84，ACK=0 由 E 给，然后 STOP。
    addr = 0x84
    for k in range(8):
        b = (addr >> (7 - k)) & 1
        ups += [Update(t, "N", D, R if b else D), Update(t, "E", R, R)]
        t += 1
        ups += [Update(t, "N", R, R if b else D), Update(t, "E", R, R)]
        t += 1
    ups += [Update(t, "N", D, R), Update(t, "E", R, D)]
    t += 1
    # ACK 上升采样刻 E 仍拉住 SDA => 采样 0（ACK）；N 释放
    ups += [Update(t, "N", R, R), Update(t, "E", R, D)]
    t += 1
    # ACK=0 后 STOP：E 释放 / N 接管（同刻 AND 仍低，无边沿），再 N 释放
    ups += [Update(t, "N", R, D), Update(t, "E", R, R)]
    t += 1
    ups += [Update(t, "N", R, R)]
    result = parse_updates(ups, recoveries=[RecoveryRequest(t0, "M")])
    rec = result.recoveries[0]
    print(f"结局={rec.outcome.value}  中止者={rec.aborted_by}  有效脉冲数={rec.pulses_sent}")
    last = rec.events[-1]
    print(f"末事件 t{last.tick} {last.kind.value} SCL={last.scl} SDA={last.sda}")
    n_tx = result.transactions[-1]
    n_seg = n_tx.segments[0]
    print(f"N 的事务被正常识别：地址=0x{n_seg.address:02X} rw={n_seg.rw} "
          f"ACK={n_seg.acks} STOP={n_tx.closed}；M 的半截事务仍未闭合="
      f"{not result.transactions[0].closed}")

    print("\n演示完成：以上结果均由 odbus.parse_updates 对固定输入真实计算得到。")


if __name__ == "__main__":
    main()
