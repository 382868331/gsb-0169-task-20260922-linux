#!/usr/bin/env python3
"""两个开漏主机逐拍竞争总线的仲裁迭代演示（真实计算，无 sleep/预录结果）。

运行：python demo_iteration.py

三个场景（逐拍按真值表合成导线，不启动真实 I2C 设备）：
1. 地址阶段分胜负：W 发 0xA0、L 发 0xA2，第 6 位决出输赢；
   失败方 L 后续的拉低主张（含一次会打断赢家的假 STOP）被释放机制吞掉；
2. 数据阶段才分胜负：地址相同，数据字节首位决出输赢；
3. 完全相同发送序列：无仲裁丢失，两家并列完成同一事务。
另附：时钟拉伸与仲裁分开记录的场景。
"""

from __future__ import annotations

from odbus import Drive, Update, iterate_masters

R = Drive.RELEASE
D = Drive.LOW
H, L = 1, 0  # 电平：1=释放（上拉高），0=拉低


class Script:
    """单主机逐拍驱动主张脚本（2-tick 位脉冲节奏）。"""

    def __init__(self, name: str) -> None:
        self.name = name
        self.t = 0
        self.raw: list[tuple[int, str, int, int]] = []

    def emit(self, scl: int, sda: int, n: int = 1) -> None:
        for _ in range(n):
            self.raw.append((self.t, self.name, scl, sda))
            self.t += 1

    def start(self) -> None:
        self.emit(H, L)

    def bit(self, b: int, high: int = 1) -> None:
        self.emit(L, b)
        self.emit(H, b, high)

    def byte(self, value: int, ack: int = 1) -> None:
        for k in range(8):
            self.bit((value >> (7 - k)) & 1)
        self.bit(ack)  # 第 9 拍：释放=1（无从机 => NACK）

    def stop(self) -> None:
        self.emit(L, L)
        self.emit(H, L)
        self.emit(H, H)


def to_updates(*scripts: Script) -> list[Update]:
    raw = sorted((e for s in scripts for e in s.raw), key=lambda e: e[0])
    return [
        Update(tick=t, driver=d,
               scl=R if scl == H else D, sda=R if sda == H else D)
        for (t, d, scl, sda) in raw
    ]


def scenario_address_phase() -> list[Update]:
    w, l = Script("W"), Script("L")
    w.start(); l.start()
    w.byte(0xA0); l.byte(0xA2)          # 0xA0=10100000 vs 0xA2=10100010 -> 第6位分歧
    w.bit(0); l.bit(0)
    w.bit(1, high=2)                    # W 该位 SCL 高电平拉满 2 tick
    l.emit(0, 1); l.emit(1, 0); l.emit(1, 1)  # L 若不释放，将在此制造假 STOP
    for k in range(2, 8):
        b = (0x5A >> (7 - k)) & 1
        w.bit(b); l.bit(b)
    w.bit(1); l.bit(1)
    w.stop(); l.stop()
    return to_updates(w, l)


def scenario_data_phase() -> list[Update]:
    w, l = Script("W"), Script("L")
    w.start(); l.start()
    w.byte(0xA0); l.byte(0xA0)          # 地址完全相同
    w.bit(0); l.bit(1)                  # 数据首位：L 发 1、W 发 0 -> L 负
    for _ in range(7):
        w.bit(0); l.bit(1)
    w.bit(1); l.bit(1)
    w.stop(); l.stop()
    return to_updates(w, l)


def scenario_identical() -> list[Update]:
    w, l = Script("W"), Script("L")
    for s in (w, l):
        s.start()
        s.byte(0xA0)
        s.byte(0x5A)
        s.stop()
    return to_updates(w, l)


def scenario_stretch() -> list[Update]:
    w, l = Script("W"), Script("L")
    w.start(); l.start()
    w.emit(L, 1, 3); w.emit(H, 1)       # W 把 SCL 多拉低 2 拍（拉伸）
    l.emit(L, 1); l.emit(H, 1, 3)       # L 已释放，被拖住
    for k in range(1, 8):
        b = (0xA0 >> (7 - k)) & 1
        w.bit(b); l.bit(b)
    w.bit(1); l.bit(1)
    w.byte(0x11); l.byte(0x11)
    w.stop(); l.stop()
    return to_updates(w, l)


def print_header(title: str) -> None:
    bar = "=" * 68
    print(f"\n{bar}\n{title}\n{bar}")


def show_result(it) -> None:
    print(f"赢家（未丢失仲裁）: {it.winners}  唯一赢家: {it.winner}")
    if it.losses:
        for loss in it.losses:
            print(f"失败方首次冲突    : 主机 {loss.master} @t{loss.tick} "
                  f"字节内第{loss.bit_index}位 阶段={loss.phase.value} "
                  f"发1采到{loss.observed} 拉低者={loss.driven_low_by}")
    else:
        print("失败方首次冲突    : 无（完全相同序列，无人丢失仲裁）")
    for tx in it.result.transactions:
        for seg in tx.segments:
            print(f"赢家解码事务      : 地址=0x{seg.address:02X} "
                  f"读写位={seg.rw} 数据={' '.join(f'0x{x:02X}' for x in seg.data)} "
                  f"ACK={seg.acks} STOP闭合={tx.closed}"
                  + (f" @t{seg.stop.tick}" if seg.stop else ""))
    if it.stretch_events:
        for ev in it.stretch_events:
            print(f"时钟拉伸区间      : t{ev.start_tick}->t{ev.end_tick} "
                  f"拉低者={ev.holders} 被拖住={ev.released}")


def main() -> None:
    print_header("1) 地址阶段分胜负（W=0xA0 胜，L=0xA2 在第6位发1采到0）")
    show_result(iterate_masters(scenario_address_phase()))
    print("说明：L 自判决后的下一驱动阶段起被强制释放，"
          "其后续假 STOP 主张未上线，赢家事务完整闭合。")

    print_header("2) 数据阶段才分胜负（地址相同，数据首位 L 发1采到0）")
    show_result(iterate_masters(scenario_data_phase()))

    print_header("3) 完全相同发送序列（无冲突，两家并列）")
    show_result(iterate_masters(scenario_identical()))

    print_header("4) 时钟拉伸与仲裁分开记录（W 多拉低 SCL 两拍）")
    show_result(iterate_masters(scenario_stretch()))

    print("\n演示完成：以上均为现场逐拍合成并解码的真实计算结果。")


if __name__ == "__main__":
    main()

