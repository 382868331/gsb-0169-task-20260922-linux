#!/usr/bin/env python3
"""odbus 固定输入演示（确定性波形，真实计算，无网络/硬件/sleep/预录结果）。

运行：python demo.py

六部分：
1. 旧接口正常读事务：多驱动器 Update -> parse_updates（地址/数据/位来源）；
2. 旧接口仲裁失败与冲突更新拒绝；
3. 新接口正常结果：整数时间戳 Sample 采样，注入的亚宽度毛刺被滤波剔除后正确解码；
4. 新接口边界：脉冲宽度 == min_width 保留 / 小 1 剔除；
   stretch duration == limit 通过 / 超 1 报错（等号两侧）；
5. 新接口实际错误：同一时刻重复采样、非法电平、SCL 拉伸超时（均真实触发）。
"""

from __future__ import annotations

import time

from odbus import Drive, Sample, Update, parse_samples, parse_updates
from odbus.errors import (
    ConflictingUpdateError,
    DuplicateSampleError,
    InvalidSampleError,
    StretchTimeoutError,
)

R = Drive.RELEASE
D = Drive.LOW


# ============================ 旧接口波形 ============================

class Wave:
    """固定 2-tick 脉冲节奏的波形构建器（绝对驱动状态模型）。"""

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
    addr = (0x53 << 1) | 1
    for k in range(8):
        w.bit_pulse("M", R if ((addr >> (7 - k)) & 1) else D)
    w.bit_pulse("E", D)
    data = 0x9E
    for k in range(8):
        w.bit_pulse("E", R if ((data >> (7 - k)) & 1) else D)
    w.bit_pulse("M", R)
    w.stop_after_nack_high()
    return w.updates


def build_arbitration_case() -> list[Update]:
    """M 与 C 同起始；地址首位 M 发 1、C 拉低 => M 仲裁丢失，随后补齐到 STOP。"""
    ups: list[Update] = []
    t = 0
    ups += [Update(t, "M", R, D), Update(t, "C", R, D)]
    t += 1
    ups += [Update(t, "M", D, R), Update(t, "C", R, D)]
    t += 1
    ups += [Update(t, "M", R, R), Update(t, "C", R, D)]
    t += 1
    for _ in range(7):
        ups += [Update(t, "M", D, R), Update(t, "C", R, R)]
        t += 1
        ups += [Update(t, "M", R, R), Update(t, "C", R, R)]
        t += 1
    ups += [Update(t, "M", D, R), Update(t, "C", R, D)]
    t += 1
    ups += [Update(t, "M", R, R), Update(t, "C", R, D)]
    t += 1
    ups += [Update(t, "M", R, D), Update(t, "C", R, R)]
    t += 1
    ups += [Update(t, "M", R, R)]
    return ups


# ======================== 新接口：采样波形 ========================

HOLD = 10  # 每电平保持 10 tick（min_width=4 时，段中部注入 1-tick 毛刺两侧余宽 >=4）


def sample_write_transaction(addr7: int, data: tuple[int, ...]):
    """稠密整数时刻采样的写事务（ACK 恒为 0），返回 list[Sample]。

    波形（每段 HOLD tick）：空闲 -> START -> 每字节 8 位+ACK -> STOP。
    地址/数据/ACK 是固定常量，用于和解析结果逐项核对。
    """
    grid: list[tuple[int, int]] = [(1, 1)] * HOLD          # 空闲
    grid += [(1, 0)] * HOLD                                 # START
    stream = [(addr7 << 1) & 0xFE, *data]
    for byte in stream:
        for k in range(8):
            b = (byte >> (7 - k)) & 1
            grid += [(0, b)] * HOLD
            grid += [(1, b)] * HOLD
        grid += [(0, 0)] * HOLD                             # ACK=0
        grid += [(1, 0)] * HOLD
    grid += [(1, 1)] * HOLD                                 # STOP
    return [Sample(tick=i, scl=scl, sda=sda)
            for i, (scl, sda) in enumerate(grid)]


def inject(samples, index, *, scl=None, sda=None):
    """在指定采样序号把某线电平改写（确定性“毛刺注入”）。"""
    old = samples[index]
    samples[index] = Sample(
        tick=old.tick,
        scl=old.scl if scl is None else scl,
        sda=old.sda if sda is None else sda,
    )


def print_header(title: str) -> None:
    print(f"\n{'=' * 70}\n{title}\n{'=' * 70}")


def main() -> None:
    t0 = time.perf_counter()

    # ---------- 1. 旧接口正常读事务 ----------
    print_header("1) 旧接口 parse_updates：正常读事务（M 读 E，地址 0x53）")
    result = parse_updates(build_read_transaction())
    seg = result.transactions[0].segments[0]
    print(f"地址 0x{seg.address:02X}  读写位={seg.rw}（1=读）  "
          f"数据={['0x%02X' % x for x in seg.data]}  ACK={seg.acks}  闭合={seg.stop is not None}")

    # ---------- 2. 旧接口仲裁失败 + 冲突更新 ----------
    print_header("2) 旧接口：仲裁丢失 + 同驱动器同刻冲突更新被拒绝")
    seg2 = parse_updates(build_arbitration_case()).transactions[0].segments[0]
    loss = seg2.arbitration_losses[0]
    print(f"仲裁丢失 @t{loss.tick} 位{loss.bit_index} 主机={loss.master} "
          f"读到={loss.observed} 拉低者={loss.driven_low_by}")
    try:
        parse_updates([Update(1, "M", D, R), Update(1, "M", R, D)])
    except ConflictingUpdateError as e:
        print(f"捕获 {type(e).__name__}: {e}")

    # ---------- 3. 新接口正常结果：毛刺滤波后正确解码 ----------
    print_header("3) 新接口 parse_samples：含毛刺采样经 min_width=4 滤波后解码")
    samples = sample_write_transaction(0x50, (0x71,))
    # 手算注入点（HOLD=10，从 0 起编号）：
    #  START 占 [10,20)；地址字节 9 拍从 t=20 起，每拍 20 tick；
    #  数据字节首位(0x71 bit7=0)的 SCL 低段起点 = 20 + 9*20 = 200。
    #  ① t=205：数据位 SCL 低段中部插 1-tick 高毛刺（SDA 保持 0）；
    #  ② 地址首位为 1（0xA0 bit7=1），其 SCL 低段 [20,30)，
    #     t=25 插 1-tick SDA 低毛刺（采样在上升沿 t=30，不应受影响）。
    inject(samples, 205, scl=1)
    inject(samples, 25, sda=0)
    out = parse_samples(samples, min_width=4)
    s3 = out.transactions[0].segments[0]
    print(f"被剔除毛刺 {len(out.filtered_glitches)} 个：")
    for g in out.filtered_glitches:
        print(f"  {g.line.value}: t{g.start_tick}->t{g.end_tick} "
              f"宽{g.width} 电平{g.level} 脉冲（< 4，剔除）")
    print(f"解码：地址 0x{s3.address:02X}（期望 0x50）  读写={s3.rw}（期望 0）")
    print(f"      数据={tuple('0x%02X' % x for x in s3.data)}（期望 ('0x71',)）  "
          f"ACK={s3.acks}（期望 (0, 0)）")
    print(f"      首字节 8 位 = {[b.value for b in s3.blocks[0].bits]}（期望 0xA0 的各位）")

    # ---------- 4. 边界：宽度等号 / 拉伸等号 ----------
    print_header("4) 边界：width == min_width 保留；width 小 1 剔除（等号口径）")
    pulse = [Sample(0, 1, 1), Sample(10, 1, 0), Sample(11, 1, 0), Sample(12, 1, 1)]
    kept = parse_samples(pulse, min_width=2)   # 宽 2 == 2：保留 -> 识别出 START
    gone = parse_samples(pulse, min_width=3)   # 宽 2 < 3：剔除 -> 无事务
    print(f"SDA 低脉冲宽 2：min_width=2 保留，事务数={len(kept.transactions)}；"
          f"min_width=3 剔除，事务数={len(gone.transactions)}，"
          f"毛刺报告={len(gone.filtered_glitches)} 条")

    print_header("4b) 边界：stretch duration == limit 通过；超出 1 tick 报错")
    ok = [Sample(0, 1, 0), Sample(5, 0, 0), Sample(10, 1, 0)]   # 低宽 5
    parse_samples(ok, min_width=1, stretch_limit=5)
    print("SCL t5 降、t10 升：duration=5 == limit=5，通过")
    bad = [Sample(0, 1, 0), Sample(5, 0, 0), Sample(11, 1, 0)]  # 低宽 6
    try:
        parse_samples(bad, min_width=1, stretch_limit=5)
    except StretchTimeoutError as e:
        print(f"t11 升（duration=6 > 5）：捕获 {type(e).__name__}（{e.duration=}, {e.limit=}）")

    # ---------- 5. 实际错误：重复采样 / 非法电平 ----------
    print_header("5) 新接口实际错误：同一时刻重复采样、非法电平")
    try:
        parse_samples([Sample(3, 1, 1), Sample(3, 1, 0)], min_width=2)
    except DuplicateSampleError as e:
        print(f"捕获 {type(e).__name__}: {e}")
    try:
        parse_samples([Sample(0, 1, 2)], min_width=2)
    except InvalidSampleError as e:
        print(f"捕获 {type(e).__name__}: {e}")

    dt = time.perf_counter() - t0
    print(f"\n演示完成：真实计算耗时 {dt:.3f} 秒（远低于 90 秒），无 sleep、无预录结果。")


if __name__ == "__main__":
    main()
