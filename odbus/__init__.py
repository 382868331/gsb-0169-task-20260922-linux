"""odbus: two-wire open-drain bus transaction decoder (teaching/diagnostic subset).

Public surface:
    Update, Line, Drive, Bit, ByteBlock, StartEvent, StopEvent,
    ArbitrationLoss, Segment, Transaction, ParseResult,
    BusRecorder, StreamingBusDecoder, parse_updates,
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
    Line,
    ParseResult,
    Segment,
    StartEvent,
    StopEvent,
    Transaction,
    Update,
)
from .decoder import BusRecorder, StreamingBusDecoder, parse_updates
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
    "LimitError",
    "Line",
    "MAX_DRIVER_COUNT",
    "MAX_DRIVER_ID_LEN",
    "MAX_TICK",
    "MAX_UPDATE_COUNT",
    "MIN_TICK",
    "ParseResult",
    "Segment",
    "StreamingBusDecoder",
    "StartEvent",
    "StopEvent",
    "StretchTimeoutError",
    "Transaction",
    "Update",
    "parse_updates",
]
