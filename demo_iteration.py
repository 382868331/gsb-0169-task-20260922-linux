#!/usr/bin/env python3
"""十位寻址增量演示（确定性波形，真实计算，无网络/硬件/sleep/预录结果）。

运行：python demo_iteration.py   （实测远低于 90 秒）

题内十位寻址形式：首字节 ``11110`` + 地址高两位 + 方向位；写方向后再接
地址低八位；读取须先在同一事务内完成写地址阶段，再重复 START 发同高位读头。

四部分（波形逐拍生成，对照 tests/reference.py 的独立字节状态机）：
1. 合法十位写事务（含各地址字节的 ACK 来源位置）；
2. 写地址阶段后重复 START 转读；
3. 低八位地址被 NACK：地址未收全，后续字节不算数据；
4. 错误读头：无已完成写地址阶段 => 不匹配。
"""

from __future__ import annotations

import sys

from odbus import Drive, Update, parse_updates

sys.path.insert(0, "tests")
from reference import Generator, H, ref_ten_bit  # noqa: E402


def to_updates(raw):
    return [
        Update(
            tick=t,
            driver=d,
            scl=Drive.RELEASE if scl == H else Drive.LOW,
            sda=Drive.RELEASE if sda == H else Drive.LOW,
        )
        for (t, d, scl, sda) in raw
    ]


def header(high: int, rw: int) -> int:
    return 0xF0 | ((high & 0x03) << 1) | (rw & 1)


def print_header(title: str) -> None:
    bar = "=" * 68
    print(f"\n{bar}\n{title}\n{bar}")


def show_segment(seg, label: str) -> None:
    print(f"[{label}] 寻址形式={seg.addressing}  "
          f"地址={('0x%03X' % seg.address) if seg.address is not None else '未收全'}")
    if seg.ten_bit is not None:
        tb = seg.ten_bit
        print(f"  十位状态: 高两位={tb.high:02b} 低八位="
              f"{('0x%02X' % tb.low) if tb.low is not None else '无'} "
              f"读头={tb.read_header} 收全={tb.complete} 匹配={tb.matched}")
    print(f"  数据字节: {' '.join('0x%02X' % v for v in seg.data) or '（无）'}")
    for i, block in enumerate(seg.address_blocks):
        src = block.ack.source
        who = ",".join(src.sda_low_drivers) or "（无，上拉为1=NACK）"
        kind = "ACK" if block.ack.value == 0 else "NACK"
        print(f"  地址字节{i} 值=0x{block.value:02X} 第9拍={kind} "
              f"采样@t{src.scl_high_tick} SCL {src.scl_low_tick}->{src.scl_high_tick} "
              f"SDA低驱动=[{who}]")


def check_against_reference(txs, result) -> None:
    """现场用独立字节状态机复核核心输出（不导入 odbus 的参考实现）。"""
    expected = ref_ten_bit(txs)
    got = [
        [(s.address, s.data, s.acks) for s in tx.segments]
        for tx in result.transactions
    ]
    want = [
        [(e.address, e.data, e.acks) for e in exp_tx]
        for exp_tx in expected
    ]
    assert got == want, f"核心输出与独立状态机不一致: {got} != {want}"
    print("  独立字节状态机复核: 一致")


def main() -> None:
    # ---------- 1. 合法十位写 ----------
    print_header("1) 合法十位写：地址 0x2AB，两个数据字节")
    g = Generator()
    g.start()
    g.byte_master_sends(header(0b10, 0), 0)   # 写头 11110 10 0
    g.byte_master_sends(0xAB, 0)              # 地址低八位
    g.byte_master_sends(0x11, 0)              # 数据
    g.byte_master_sends(0x22, 1)              # 数据（末字节 NACK）
    g.stop(1)
    txs = [[[(header(0b10, 0), 0), (0xAB, 0), (0x11, 0), (0x22, 1)]]]
    result = parse_updates(to_updates(g.raw))
    show_segment(result.transactions[0].segments[0], "写段")
    check_against_reference(txs, result)

    # ---------- 2. 写地址阶段后转读 ----------
    print_header("2) 写地址阶段 + 重复 START + 同高位读头 => 读 0x134")
    g = Generator()
    g.start()
    g.byte_master_sends(header(0b01, 0), 0)
    g.byte_master_sends(0x34, 0)
    g.repeated_start()
    g.byte_master_sends(header(0b01, 1), 0)   # 读头，同一高位
    g.byte_slave_sends(0x9E, 1)               # 从机数据，主机 NACK
    g.stop(1)
    txs = [[
        [(header(0b01, 0), 0), (0x34, 0)],
        [(header(0b01, 1), 0), (0x9E, 1)],
    ]]
    result = parse_updates(to_updates(g.raw))
    tx = result.transactions[0]
    show_segment(tx.segments[0], "写地址段")
    show_segment(tx.segments[1], "读段（重复 START）")
    check_against_reference(txs, result)

    # ---------- 3. 低八位地址 NACK ----------
    print_header("3) 低八位地址被 NACK：地址未收全，后续字节不算数据")
    g = Generator()
    g.start()
    g.byte_master_sends(header(0b11, 0), 0)
    g.byte_master_sends(0x56, 1)              # 低八位被 NACK
    g.byte_master_sends(0x77, 1)              # 总线上仍有字节，但不是数据
    g.stop(1)
    txs = [[[(header(0b11, 0), 0), (0x56, 1), (0x77, 1)]]]
    result = parse_updates(to_updates(g.raw))
    show_segment(result.transactions[0].segments[0], "写段")
    check_against_reference(txs, result)

    # ---------- 4. 错误读头 ----------
    print_header("4) 错误读头：事务内没有已完成的写地址阶段 => 不匹配")
    g = Generator()
    g.start()
    g.byte_master_sends(header(0b01, 1), 0)   # 直接发读头
    g.byte_slave_sends(0x42, 1)
    g.stop(1)
    txs = [[[(header(0b01, 1), 0), (0x42, 1)]]]
    result = parse_updates(to_updates(g.raw))
    show_segment(result.transactions[0].segments[0], "读头段")
    check_against_reference(txs, result)

    print("\n演示完成：以上结果均由 odbus.parse_updates 对逐拍生成的波形真实计算得到。")


if __name__ == "__main__":
    main()
