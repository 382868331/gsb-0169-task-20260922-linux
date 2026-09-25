"""odbus: two-wire open-drain bus transaction decoder (teaching/diagnostic subset).

Public surface:
    Update, Line, Drive, Bit, ByteBlock, StartEvent, StopEvent,
    ArbitrationLoss, Segment, Transaction, ParseResult,
    RecoveryRequest, RecoveryRecord, RecoveryPulse, RecoveryOutcome,
    LineSample,
    BusRecorder, parse_updates,
    BusTraceError, InvalidUpdateError, ConflictingUpdateError,
    LimitError, StretchTimeoutError, BusStuckError
"""

from .errors import (
    BusStuckError,
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
    LineSample,
    ParseResult,
    RecoveryOutcome,
    RecoveryPulse,
    RecoveryRecord,
    RecoveryRequest,
    Segment,
    StartEvent,
    StopEvent,
    Transaction,
    Update,
)
from .decoder import BusRecorder, parse_updates
from .validation import (
    MAX_DRIVER_COUNT,
    MAX_DRIVER_ID_LEN,
    MAX_RECOVERY_PULSES,
    MAX_TICK,
    MAX_UPDATE_COUNT,
    MIN_TICK,
)

__all__ = [
    "ArbitrationLoss",
    "Bit",
    "BitPhase",
    "BitSource",
    "BusRecorder",
    "BusStuckError",
    "BusTraceError",
    "ByteBlock",
    "ConflictingUpdateError",
    "Drive",
    "InvalidUpdateError",
    "LimitError",
    "Line",
    "LineSample",
    "MAX_DRIVER_COUNT",
    "MAX_DRIVER_ID_LEN",
    "MAX_RECOVERY_PULSES",
    "MAX_TICK",
    "MAX_UPDATE_COUNT",
    "MIN_TICK",
    "ParseResult",
    "RecoveryOutcome",
    "RecoveryPulse",
    "RecoveryRecord",
    "RecoveryRequest",
    "Segment",
    "StartEvent",
    "StopEvent",
    "StretchTimeoutError",
    "Transaction",
    "Update",
    "parse_updates",
]
