"""odbus: two-wire open-drain bus transaction decoder (teaching/diagnostic subset).

Public surface:
    Update, Sample, Line, Drive, Bit, ByteBlock, StartEvent, StopEvent,
    ArbitrationLoss, Segment, Transaction, ParseResult,
    FilteredPulse, SampleFilterResult, SampleParseResult,
    BusRecorder, parse_updates,
    filter_samples, samples_to_updates, parse_samples, SAMPLE_DRIVER,
    BusTraceError, InvalidUpdateError, ConflictingUpdateError,
    DuplicateSampleError, LimitError, StretchTimeoutError
"""

from .errors import (
    BusTraceError,
    ConflictingUpdateError,
    DuplicateSampleError,
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
    FilteredPulse,
    Line,
    ParseResult,
    Sample,
    SampleFilterResult,
    Segment,
    StartEvent,
    StopEvent,
    Transaction,
    Update,
)
from .decoder import BusRecorder, parse_updates
from .sampling import (
    SAMPLE_DRIVER,
    SampleParseResult,
    filter_samples,
    parse_samples,
    samples_to_updates,
)
from .validation import (
    MAX_DRIVER_COUNT,
    MAX_DRIVER_ID_LEN,
    MAX_MIN_WIDTH,
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
    "BusTraceError",
    "ByteBlock",
    "ConflictingUpdateError",
    "Drive",
    "DuplicateSampleError",
    "FilteredPulse",
    "InvalidUpdateError",
    "LimitError",
    "Line",
    "MAX_DRIVER_COUNT",
    "MAX_DRIVER_ID_LEN",
    "MAX_MIN_WIDTH",
    "MAX_TICK",
    "MAX_UPDATE_COUNT",
    "MIN_TICK",
    "ParseResult",
    "SAMPLE_DRIVER",
    "Sample",
    "SampleFilterResult",
    "SampleParseResult",
    "Segment",
    "StartEvent",
    "StopEvent",
    "StretchTimeoutError",
    "Transaction",
    "Update",
    "filter_samples",
    "parse_samples",
    "parse_updates",
    "samples_to_updates",
]
