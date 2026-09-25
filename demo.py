#!/usr/bin/env python3
"""odbus 固定输入演示（确定性波形，真实计算，无网络/硬件/sleep/预录结果）。

运行：python demo.py

五部分：
1. 正常读事务：主机 M 读从机 E（地址 0x53）的数据 0x9E，含每一位的原始来源；
2. 仲裁失败：主机 M 发送 1 却被竞争者 C 拉低，解析出 ArbitrationLoss；
3. 异常边界（实际触发，非模拟）：
   a. 同一驱动器同一时刻两条更新 -> ConflictingUpdateError；
   b. SCL 持续低超过阈值 -> StretchTimeoutError；
4. 总线恢复：从机 S 卡死 SDA，主机 M 请求恢复，第 3 个有效脉冲后 SDA 释放，
   主机生成合法 STOP（逐事件列出实际线值，全部来自驱动器合并）；
5. 恢复失败边界（实际触发）：SDA 始终不放，打满 9 个有效脉冲 -> BusStuckError。
"""

from __future__ import annotations

from odbus import Drive, RecoveryRequest, Update, parse_updates
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


def build_stuck_bus(release_pulse: int | None) -> tuple[list[Update], int]:
    """M 发 START + 3 个地址位后，从机 S 把 SDA 卡死在低电平。

    release_pulse 给出时，S 在第 release_pulse 个恢复脉冲的拉低期
    （与主机动作同刻）释放 SDA；为 None 时 S 永不释放。
    返回 (updates, 恢复请求时刻)。
    """
    ups = [Update(0, "M", R, D)]  # START
    t = 1
    for _ in range(3):
        ups.append(Update(t, "M", D, R))
        t += 1
        ups.append(Update(t, "M", R, R))
        t += 1
    ups.append(Update(t, "M", D, R))
    ups.append(Update(t, "S", R, D))  # S 卡死 SDA；M 停在 SCL 低
    t += 1
    if release_pulse is not None:
        ups.append(Update(t + 2 * release_pulse - 1, "S", R, R))
    return ups, t


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

    # ---------- 4. 总线恢复：卡死的 SDA 在第 3 个有效脉冲后释放 ----------
    print_header("4) 总线恢复：SDA 被从机卡死，主机发 SCL 脉冲后生成合法 STOP")
    ups4, req4 = build_stuck_bus(release_pulse=3)
    res4 = parse_updates(
        ups4, stretch_limit=10, recovery=RecoveryRequest("M", req4), trace=True
    )
    rec4 = res4.recoveries[0]
    print(f"恢复请求时刻      : t{req4}（主机 M 先释放 SDA，再逐个发 SCL 脉冲）")
    print("逐事件实际线值（全部来自同刻驱动器合并，无滤波器、无真实 GPIO）：")
    for s in res4.trace:
        scl_who = ",".join(s.scl_low_drivers) or "-"
        sda_who = ",".join(s.sda_low_drivers) or "-"
        print(f"  t{s.tick:<3} SCL={s.scl} SDA={s.sda}   "
              f"拉低SCL=[{scl_who}] 拉低SDA=[{sda_who}]")
    print("有效脉冲（真正观察到 SCL 升高才计数）：")
    for p in rec4.pulses:
        print(f"  脉冲{p.index} 拉低@t{p.fall_tick} 升高@t{p.rise_tick} "
              f"采到SDA={p.sda} 被拉伸等待={p.stretched}")
    print(f"恢复期间采到的位  : {[b.value for b in rec4.bits]} "
          f"（单独标记 phase=recovery，不拼进没收完的字节）")
    print(f"恢复结果          : {rec4.outcome.value}，STOP 生成于 t{rec4.stop_tick}")
    tx4 = res4.transactions[0]
    print(f"事务闭合          : {tx4.closed}（恢复生成的 STOP 正常结束事务）")

    # ---------- 5. 恢复失败边界（实际触发）：SDA 始终不放 ----------
    print_header("5) 异常边界（实际触发）：9 个有效脉冲后 SDA 仍被拉低 -> bus_stuck")
    ups5, req5 = build_stuck_bus(release_pulse=None)
    try:
        parse_updates(ups5, recovery=RecoveryRequest("M", req5))
    except BusStuckError as e:
        print(f"捕获 {type(e).__name__}: {e}")
        print(f"已发有效脉冲 {e.pulses}/{e.max_pulses}，"
          f"最后仍拉低 SDA 的驱动器={e.sda_low_drivers}")

    print("\n演示完成：以上结果均由 odbus.parse_updates 对固定输入真实计算得到。")


if __name__ == "__main__":
    main()
