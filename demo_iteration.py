#!/usr/bin/env python3
"""十位寻址增量演示：确定性逐拍生成，现场解析，不 sleep、不预录结果。"""

from __future__ import annotations

from odbus import Drive, Update, parse_updates

R = Drive.RELEASE
D = Drive.LOW
HIGH = 1
LOW = 0
ADDRESS = 0x2A1


class Wave:
    def __init__(self) -> None:
        self.tick = 0
        self.master = (HIGH, HIGH)
        self.slave = (HIGH, HIGH)
        self.updates: list[Update] = []

    @property
    def bus_sda(self) -> int:
        return self.master[1] & self.slave[1]

    def commit(self) -> None:
        self.updates.append(
            Update(self.tick, "M", R if self.master[0] else D, R if self.master[1] else D)
        )
        self.updates.append(
            Update(self.tick, "E", R if self.slave[0] else D, R if self.slave[1] else D)
        )

    def advance(self) -> None:
        self.commit()
        self.tick += 1

    def start(self) -> None:
        self.master = (HIGH, LOW)
        self.slave = (HIGH, HIGH)
        self.advance()

    def pulse(self, sender: str, sda: int) -> None:
        self.master = (LOW, sda if sender == "M" else HIGH)
        self.slave = (HIGH, sda if sender == "E" else HIGH)
        self.advance()
        self.master = (HIGH, sda if sender == "M" else HIGH)
        self.slave = (HIGH, sda if sender == "E" else HIGH)
        self.advance()

    def byte(self, sender: str, value: int, ack: int) -> None:
        for position in range(8):
            self.pulse(sender, (value >> (7 - position)) & 1)
        self.pulse("E" if sender == "M" else "M", ack)

    def repeated_start(self) -> None:
        if self.bus_sda == LOW:
            self.master = (HIGH, LOW)
            self.slave = (HIGH, HIGH)
            self.advance()
            self.master = (LOW, LOW)
            self.advance()
            self.master = (LOW, HIGH)
            self.advance()
            self.master = (HIGH, HIGH)
            self.advance()
        self.master = (HIGH, LOW)
        self.advance()

    def stop(self, last_ack: int) -> None:
        if last_ack == 0:
            self.master = (HIGH, LOW)
            self.slave = (HIGH, HIGH)
            self.advance()
            self.master = (HIGH, HIGH)
            self.advance()
        else:
            self.master = (LOW, LOW)
            self.slave = (HIGH, HIGH)
            self.advance()
            self.master = (HIGH, LOW)
            self.advance()
            self.master = (HIGH, HIGH)
            self.advance()


def high_header(high: int, read: bool) -> int:
    return 0b11110_000 | ((high & 0b11) << 1) | int(read)


def legal_write() -> list[Update]:
    w = Wave()
    w.start()
    w.byte("M", high_header(ADDRESS >> 8, False), 0)
    w.byte("M", ADDRESS & 0xFF, 0)
    w.byte("M", 0x3C, 1)
    w.stop(1)
    return w.updates


def write_then_read() -> list[Update]:
    w = Wave()
    w.start()
    w.byte("M", high_header(ADDRESS >> 8, False), 0)
    w.byte("M", ADDRESS & 0xFF, 0)
    w.repeated_start()
    w.byte("M", high_header(ADDRESS >> 8, True), 0)
    w.byte("E", 0x6D, 1)
    w.stop(1)
    return w.updates


def low_address_nack() -> list[Update]:
    w = Wave()
    w.start()
    w.byte("M", high_header(ADDRESS >> 8, False), 0)
    w.byte("M", ADDRESS & 0xFF, 1)
    w.repeated_start()
    w.byte("M", high_header(ADDRESS >> 8, True), 1)
    w.byte("E", 0x6D, 1)
    w.stop(1)
    return w.updates


def wrong_read_header() -> list[Update]:
    w = Wave()
    w.start()
    w.byte("M", high_header(ADDRESS >> 8, False), 0)
    w.byte("M", ADDRESS & 0xFF, 0)
    w.repeated_start()
    w.byte("M", high_header((ADDRESS >> 8) ^ 1, True), 0)
    w.byte("E", 0x6D, 1)
    w.stop(1)
    return w.updates


def show(title: str, updates: list[Update]) -> None:
    result = parse_updates(updates)
    print(f"\n{title}")
    print("-" * len(title))
    for tx_index, tx in enumerate(result.transactions):
        for seg_index, seg in enumerate(tx.segments):
            address = "无完整ACK地址" if seg.address is None else f"0x{seg.address:03X}"
            print(
                f"  tx{tx_index}.seg{seg_index} {seg.address_mode} "
                f"rw={seg.rw} address={address} data={tuple(hex(x) for x in seg.data)}"
            )
            for order, block in enumerate(seg.address_blocks):
                drivers = ",".join(block.ack.source.sda_low_drivers) or "无（上拉为1/NACK）"
                print(
                    f"    寻址字节{order + 1}=0x{block.value:02X} ACK={block.ack.value} "
                    f"脉冲t{block.ack.source.scl_low_tick}->t{block.ack.tick} "
                    f"ACK低驱动=[{drivers}]"
                )


def main() -> None:
    print("odbus 十位寻址增量演示（Python 标准库；现场逐拍生成并解析）")
    print(f"目标十位地址：0x{ADDRESS:03X}")
    show("1. 合法写：低八位属于地址，不计入 data", legal_write())
    show("2. 写地址阶段后重复 START，同高位读头转读", write_then_read())
    show("3. 低地址 NACK：后续字节不能借用或当数据", low_address_nack())
    show("4. 错误读头：高位不一致，读段保持不完整地址", wrong_read_header())


if __name__ == "__main__":
    main()
