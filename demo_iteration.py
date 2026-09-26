#!/usr/bin/env python3
"""双主机逐拍驱动仲裁演示（现场计算，无 sleep / 预录结果 / 真实硬件）。

运行：python demo_iteration.py   （远低于 90 秒）

场景：
1. 地址阶段分胜负：A(0x50) 与 B(0x51) 同时发，B 在地址 bit6 仲裁丢失；
2. 数据阶段才分胜负：同地址 0x34，数据 0x00 vs 0x80，B 在数据首位丢失；
3. 完全相同发送序列：无冲突，两台主机驱动完全一致，事务正常解码；
4. 异常边界（实际触发）：两主机脚本拍数不一致被拒绝。
"""

from __future__ import annotations

from odbus.errors import InvalidUpdateError
from odbus.masters import pad_script, run_dual_master, write_script


def aligned(a, b):
    n = max(len(a), len(b))
    return pad_script(a, n), pad_script(b, n)


def show(tag, res):
    print(f"  赢家              : {res.winner or '（无冲突，序列完全相同）'}")
    print(f"  失败方            : {res.loser or '—'}")
    fc = res.first_conflict
    if fc is not None:
        print(f"  首次冲突位置      : tick={fc.tick} 第{fc.byte_index}字节 "
              f"bit{fc.bit_index} 阶段={fc.phase.value} "
              f"（{fc.loser} 发1却采到0，{fc.winner} 持低）")
    else:
        print("  首次冲突位置      : —（无仲裁冲突）")
    for tx in res.parse.transactions:
        for seg in tx.segments:
            data = " ".join(f"0x{x:02X}" for x in seg.data) or "（无）"
            print(f"  赢家事务解码      : 地址=0x{seg.address:02X} rw={seg.rw} "
                  f"数据=[{data}] ACK={seg.acks} 闭合={seg.closed}")
    if res.stretches:
        for st in res.stretches:
            print(f"  时钟拉伸          : tick {st.start_tick}->{st.end_tick} "
                  f"持续{st.duration} 拉低方={st.holders} 等待方={st.waiters}")
    else:
        print("  时钟拉伸          : 无")


def header(title):
    bar = "=" * 70
    print(f"\n{bar}\n{title}\n{bar}")


def main():
    header("1) 地址阶段分胜负：A 发 0x50 写，B 发 0x51 写（数据相同）")
    sa, sb = aligned(write_script(0x50, [0x11, 0x22]),
                     write_script(0x51, [0x11, 0x22]))
    show("addr", run_dual_master({"A": sa, "B": sb}))

    header("2) 数据阶段才分胜负：同地址 0x34，A 发数据 0x00，B 发 0x80")
    sa, sb = aligned(write_script(0x34, [0x00]), write_script(0x34, [0x80]))
    show("data", run_dual_master({"A": sa, "B": sb}))

    header("3) 完全相同发送序列：两台主机都发 0x50 写 [0x11, 0x22]")
    s = write_script(0x50, [0x11, 0x22])
    show("identical", run_dual_master({"A": s, "B": list(s)}))

    header("4) 异常边界（实际触发）：两主机逐拍序列不等长被拒绝")
    try:
        run_dual_master({"A": write_script(0x50, [0x11]), "B": write_script(0x50)})
    except InvalidUpdateError as e:
        print(f"  捕获 {type(e).__name__}: {e}")

    print("\n演示完成：以上均为 odbus.masters.run_dual_master 现场计算结果。")


if __name__ == "__main__":
    main()

