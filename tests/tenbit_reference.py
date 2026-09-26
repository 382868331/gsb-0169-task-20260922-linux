"""十位寻址独立字节状态机与测试波形；本模块不导入 odbus。"""

from __future__ import annotations

from dataclasses import dataclass, field

H = 1
L = 0


@dataclass
class RefTenBlock:
    value: int
    ack: int
    ack_tick: int
    role: str


@dataclass
class RefTenSegment:
    repeated: bool
    blocks: list[RefTenBlock] = field(default_factory=list)
    mode: str | None = None
    high: int | None = None
    low: int | None = None
    complete: bool = False
    acknowledged: bool = False
    address: int | None = None
    address_blocks: list[RefTenBlock] = field(default_factory=list)

    @property
    def data(self) -> tuple[int, ...]:
        address_ids = {id(b) for b in self.address_blocks}
        return tuple(b.value for b in self.blocks if id(b) not in address_ids)


def ten_high_byte(high: int, rw: int) -> int:
    return 0b11110_000 | ((high & 0b11) << 1) | rw


class TenWave:
    def __init__(self) -> None:
        self.t = 0
        self.raw: list[tuple[int, str, int, int]] = []
        self.bus_sda = H

    def both(self, m: tuple[int, int], v: tuple[int, int]) -> None:
        self.raw.append((self.t, "M", m[0], m[1]))
        self.raw.append((self.t, "V", v[0], v[1]))
        self.bus_sda = m[1] & v[1]

    def advance(self) -> None:
        self.t += 1

    def start(self) -> None:
        self.both((H, L), (H, H))
        self.advance()

    def pulse(self, owner: str, sda: int) -> None:
        self.both((L, sda if owner == "M" else H), (H, sda if owner == "V" else H))
        self.advance()
        self.both((H, sda if owner == "M" else H), (H, sda if owner == "V" else H))
        self.advance()

    def master_byte(self, value: int, ack: int) -> None:
        for k in range(8):
            self.pulse("M", (value >> (7 - k)) & 1)
        self.pulse("V", ack)

    def slave_byte(self, value: int, master_ack: int) -> None:
        for k in range(8):
            self.pulse("V", (value >> (7 - k)) & 1)
        self.pulse("M", master_ack)

    def repeated_start(self) -> None:
        if self.bus_sda == L:
            self.both((H, L), (H, H))  # ACK 后 M 接管，线值保持低
            self.advance()
            self.both((L, L), (H, H))
            self.advance()
            self.both((L, H), (H, H))
            self.advance()
            self.both((H, H), (H, H))
            self.advance()
        else:
            pass  # SCL 高且 SDA 高，直接拉低 SDA 即 Sr
        self.both((H, L), (H, H))
        self.advance()

    def stop(self, last_ack: int) -> None:
        if last_ack == 0:
            self.both((H, L), (H, H))
            self.advance()
            self.both((H, H), (H, H))
            self.advance()
        else:
            self.both((L, L), (H, H))
            self.advance()
            self.both((H, L), (H, H))
            self.advance()
            self.both((H, H), (H, H))
            self.advance()


def ref_ten_simulate(raw: list[tuple[int, str, int, int]]) -> list[list[RefTenSegment]]:
    """密集 tick 扫描：先恢复字节边界，再用独立的十位寻址状态机分类。"""
    table: dict[int, dict[str, tuple[int, int]]] = {}
    for t, d, scl, sda in raw:
        table.setdefault(t, {})[d] = (scl, sda)
    t0, t1 = min(table), max(table)
    current = {"M": (H, H), "V": (H, H)}
    transactions: list[list[RefTenSegment]] = []
    tx: list[RefTenSegment] | None = None
    seg: RefTenSegment | None = None
    pscl = psda = H
    bits: list[tuple[int, int]] = []
    pending_low: RefTenBlock | None = None
    pending_high: int | None = None
    pending_write_header: RefTenBlock | None = None

    def line(idx: int) -> int:
        return min(current[d][idx] for d in ("M", "V"))

    def add_block(value: int, ack: int, ack_tick: int) -> None:
        nonlocal pending_low, pending_high, pending_write_header
        assert seg is not None
        count = len(seg.blocks)
        if count == 0:
            mode = "10-bit" if (value >> 3) & 0x1F == 0b11110 else "7-bit"
            seg.mode = mode
            seg.high = (value >> 1) & 3 if mode == "10-bit" else None
            rw = value & 1
            block = RefTenBlock(value, ack, ack_tick, "address_header")
            seg.blocks.append(block)
            seg.address_blocks.append(block)
            if mode == "7-bit":
                seg.complete = seg.acknowledged = ack == 0
                seg.address = value >> 1
                return
            if rw == 1:
                if (
                    pending_write_header is not None
                    and pending_low is not None
                    and pending_low.ack == 0
                    and pending_high == seg.high
                ):
                    seg.low = pending_low.value
                    seg.complete = seg.acknowledged = ack == 0
                    seg.address = (seg.high << 8) | seg.low if seg.acknowledged else None
                    seg.address_blocks = [pending_write_header, pending_low, block]
                return
            return

        write_direction = (seg.blocks[0].value & 1) == 0
        if seg.mode == "10-bit" and write_direction and count == 1 and not seg.acknowledged:
            block = RefTenBlock(value, ack, ack_tick, "address_low")
            seg.blocks.append(block)
            seg.address_blocks.append(block)
            seg.low = value
            seg.complete = seg.acknowledged = seg.blocks[0].ack == 0 and ack == 0
            if seg.acknowledged:
                seg.address = (seg.high or 0) << 8 | value
                pending_low = block
                pending_high = seg.high
                pending_write_header = seg.blocks[0]
            return

        if seg.mode == "10-bit" and not (seg.complete and seg.acknowledged):
            block = RefTenBlock(value, ack, ack_tick, "address_low")
            seg.blocks.append(block)
            seg.address_blocks.append(block)
            return

        seg.blocks.append(RefTenBlock(value, ack, ack_tick, "data"))

    for t in range(t0, t1 + 1):
        for d, pair in table.get(t, {}).items():
            current[d] = pair
        scl, sda = line(0), line(1)
        if pscl == H and scl == H and psda == H and sda == L:
            if tx is None:
                tx = []
                transactions.append(tx)
            seg = RefTenSegment(repeated=bool(tx))
            tx.append(seg)
            bits = []
        elif pscl == H and scl == H and psda == L and sda == H:
            tx = None
            seg = None
            bits = []
            pending_low = pending_high = pending_write_header = None
        elif pscl == L and scl == H and seg is not None:
            bits.append((sda, t))
            if len(bits) == 9:
                value = 0
                for v, _ in bits[:8]:
                    value = (value << 1) | v
                add_block(value, bits[8][0], bits[8][1])
                bits = []
        pscl, psda = scl, sda

    return transactions
