"""demo:解析一次读事务、一次仲裁失败,并实际触发一次输入拒绝。

全部数据由本文件中的确定性代码生成,离线运行,无 sleep、无预录结果。
运行:python demo.py
"""

from busdecode import (
    BusDecoder,
    ConflictingUpdateError,
    InvalidTimeError,
)


class Script:
    """波形脚本构造器:每个 step 是一拍,同拍多条更新构成一个批次。"""

    def __init__(self):
        self.t = 0
        self.steps = []

    def step(self, *updates, dt=1):
        self.t += dt
        self.steps.append((self.t, list(updates)))


def run(decoder, script):
    for t, updates in script.steps:
        decoder.apply(t, updates)
    return decoder


def build_read_script():
    """主机 m 从地址 0x42 的从机 s 读 2 字节:0x9A(ACK)、0x35(NACK),STOP。"""
    sc = Script()
    sc.step(("m", "sda", "low"))   # START
    sc.step(("m", "scl", "low"))

    def master_bit(b):
        sc.step(("m", "sda", "low" if b == 0 else "release"))
        sc.step(("m", "scl", "release"))
        sc.step(("m", "scl", "low"))

    def slave_bit(b):
        sc.step(("s", "sda", "low" if b == 0 else "release"))
        sc.step(("m", "scl", "release"))
        sc.step(("m", "scl", "low"))

    def master_byte(byte):
        for i in range(7, -1, -1):
            master_bit((byte >> i) & 1)

    def slave_byte(byte):
        for i in range(7, -1, -1):
            slave_bit((byte >> i) & 1)
        sc.step(("s", "sda", "release"))

    def slave_ack():
        sc.step(("m", "sda", "release"), ("s", "sda", "low"))
        sc.step(("m", "scl", "release"))
        sc.step(("m", "scl", "low"))
        sc.step(("s", "sda", "release"))

    def master_ack(ack):
        if ack:
            sc.step(("m", "sda", "low"))
        sc.step(("m", "scl", "release"))
        sc.step(("m", "scl", "low"))
        if ack:
            sc.step(("m", "sda", "release"))

    master_byte((0x42 << 1) | 1)   # 地址 0x42,读
    slave_ack()
    slave_byte(0x9A)
    master_ack(True)               # ACK,继续读
    slave_byte(0x35)
    master_ack(False)              # NACK,结束读
    sc.step(("m", "sda", "low"))   # STOP
    sc.step(("m", "scl", "release"))
    sc.step(("m", "sda", "release"))
    return sc


def build_arbitration_script():
    """m1 发地址 0x55(字节 0xAA),m2 发地址 0x2A(字节 0x54)。

    首位 m1 发 1(释放)、m2 发 0(拉低)-> m1 仲裁丢失并退出,
    m2 完成地址字节,从机 s 应答,m2 发 STOP。
    """
    sc = Script()
    sc.step(("m1", "sda", "low"), ("m2", "sda", "low"))   # 双主机同时 START
    sc.step(("m1", "scl", "low"), ("m2", "scl", "low"))
    sc.step(("m1", "sda", "release"), ("m2", "sda", "low"))  # 首位 1 vs 0
    sc.step(("m1", "scl", "release"), ("m2", "scl", "release"))
    sc.step(("m1", "scl", "low"), ("m2", "scl", "low"))
    # m1 退出:释放 SCL(SDA 已释放),不再驱动
    sc.step(("m1", "scl", "release"))
    # m2 发完 0x54 剩余 7 位
    for i in range(6, -1, -1):
        b = (0x54 >> i) & 1
        sc.step(("m2", "sda", "low" if b == 0 else "release"))
        sc.step(("m2", "scl", "release"))
        sc.step(("m2", "scl", "low"))
    sc.step(("m2", "sda", "release"), ("s", "sda", "low"))  # 从机 ACK
    sc.step(("m2", "scl", "release"))
    sc.step(("m2", "scl", "low"))
    sc.step(("s", "sda", "release"))
    sc.step(("m2", "sda", "low"))   # STOP
    sc.step(("m2", "scl", "release"))
    sc.step(("m2", "sda", "release"))
    return sc


def show_transaction(txn):
    span = f"t={txn.start_time}..{txn.stop_time}"
    print(f"  事务 {span} complete={txn.complete}")
    for seg in txn.segments:
        rw = "读" if seg.read else "写"
        ack = "ACK" if seg.address_ack else "NACK"
        print(f"  段(repeated={seg.repeated}): 地址 0x{seg.address:02X} {rw} {ack}")
        for i, d in enumerate(seg.data):
            bits_t = [b.time for b in d.bits]
            print(f"    数据[{i}] = 0x{d.value:02X} "
                  f"{'ACK' if d.ack else 'NACK'} "
                  f"(8 位采样于 t={bits_t[0]}..{bits_t[-1]},"
                  f"ACK 位来自 {d.ack_bit.low_drivers or '无人拉低'})")


def main():
    print("=== 1. 正常结果:一次读事务 ===")
    dec = run(BusDecoder(master="m", stretch_threshold=50), build_read_script())
    dec.finalize()
    for txn in dec.transactions:
        show_transaction(txn)
    print(f"  事件: {[(e.kind, e.time) for e in dec.events]}")

    print()
    print("=== 2. 仲裁失败:双主机同时发地址 ===")
    dec2 = run(BusDecoder(master="m1"), build_arbitration_script())
    dec2.finalize()
    for e in dec2.events:
        if e.kind == "arbitration_lost":
            print(f"  t={e.time} {e.kind}({e.role}): {e.detail}")
    for txn in dec2.transactions:
        show_transaction(txn)

    print()
    print("=== 3. 实际触发的拒绝/异常边界 ===")
    dec3 = BusDecoder()
    try:
        dec3.apply(1, [("m", "sda", "low"), ("m", "sda", "release")])
    except ConflictingUpdateError as exc:
        print(f"  拒绝同一驱动器同刻冲突更新: {exc}")
    try:
        dec3.apply(float("nan"), [])
    except InvalidTimeError as exc:
        print(f"  拒绝非法时刻(NaN): {exc}")
    print(f"  拒绝后解码器状态未被污染: scl={dec3.scl.value}, "
          f"sda={dec3.sda.value}, events={dec3.events}")


if __name__ == "__main__":
    main()
