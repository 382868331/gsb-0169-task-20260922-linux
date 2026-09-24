"""odbus: two-wire open-drain bus transaction decoder (teaching/diagnostic subset).

Public surface:
    Update, Sample, Line, Drive, Bit, ByteBlock, StartEvent, StopEvent,
    ArbitrationLoss, Segment, Transaction, ParseResult,
    FilteredGlitch, SampleParseResult,
    BusRecorder, parse_updates, parse_samples,
    BusTraceError, InvalidUpdateError, ConflictingUpdateError,
    InvalidSampleError, DuplicateSampleError,
    LimitError, StretchTimeoutError
"""

from .errors import (
    BusTraceError,
    ConflictingUpdateError,
    DuplicateSampleError,
    InvalidSampleError,
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
    FilteredGlitch,
    Line,
    ParseResult,
    Sample,
    SampleParseResult,
    Segment,
    StartEvent,
    StopEvent,
    Transaction,
    Update,
)
from .decoder import BusRecorder, parse_updates
from .sampler import BUS_DRIVER, parse_samples
from .validation import (
    MAX_DRIVER_COUNT,
    MAX_DRIVER_ID_LEN,
    MAX_MIN_WIDTH,
    MAX_SAMPLE_COUNT,
    MAX_TICK,
    MAX_UPDATE_COUNT,
    MIN_TICK,
)

__all__ = [
    "ArbitrationLoss",
    "Bit",
    "BitPhase",
    "BitSource",
    "BUS_DRIVER",
    "ByteBlock",
    "BusRecorder",
    "BusTraceError",
    "ConflictingUpdateError",
    "Drive",
    "DuplicateSampleError",
    "FilteredGlitch",
    "InvalidSampleError",
    "InvalidUpdateError",
    "LimitError",
    "Line",
    "MAX_DRIVER_COUNT",
    "MAX_DRIVER_ID_LEN",
    "MAX_MIN_WIDTH",
    "MAX_SAMPLE_COUNT",
    "MAX_TICK",
    "MAX_UPDATE_COUNT",
    "MIN_TICK",
    "ParseResult",
    "Sample",
    "SampleParseResult",
    "Segment",
    "StartEvent",
    "StopEvent",
    "StretchTimeoutError",
    "Transaction",
    "Update",
    "parse_samples",
    "parse_updates",
]
