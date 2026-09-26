"""odbus: two-wire open-drain bus transaction decoder (teaching/diagnostic subset).

Public surface:
    Update, Line, Drive, Bit, ByteBlock, StartEvent, StopEvent,
    ArbitrationLoss, Segment, Transaction, ParseResult,
    StretchEvent, IterationResult,
    BusRecorder, parse_updates, iterate_masters,
    BusTraceError, InvalidUpdateError, ConflictingUpdateError,
    LimitError, StretchTimeoutError
"""

from .errors import (
    BusTraceError,
    ConflictingUpdateError,
    InvalidUpdateError,
    LimitError,
    StretchTimeoutError,
)
from .model import (
    ArbitrationLoss,
    Bit,
    BitPhase,
    BitSource,
    ByteBlock,
    Drive,
    IterationResult,
    Line,
    ParseResult,
    Segment,
    StartEvent,
    StopEvent,
    StretchEvent,
    Transaction,
    Update,
)
from .decoder import BusRecorder, iterate_masters, parse_updates
from .validation import (
    MAX_DRIVER_COUNT,
    MAX_DRIVER_ID_LEN,
    MAX_TICK,
    MAX_UPDATE_COUNT,
    MIN_TICK,
)

__all__ = [
    "ArbitrationLoss",
    "Bit",
    "BitPhase",
    "BitSource",
    "ByteBlock",
    "BusRecorder",
    "BusTraceError",
    "ConflictingUpdateError",
    "Drive",
    "InvalidUpdateError",
    "IterationResult",
    "LimitError",
    "Line",
    "MAX_DRIVER_COUNT",
    "MAX_DRIVER_ID_LEN",
    "MAX_TICK",
    "MAX_UPDATE_COUNT",
    "MIN_TICK",
    "ParseResult",
    "Segment",
    "StartEvent",
    "StopEvent",
    "StretchEvent",
    "StretchTimeoutError",
    "Transaction",
    "Update",
    "iterate_masters",
    "parse_updates",
]
