"""busdecode:双线开漏总线(I2C 风格)事务识别库。仅依赖 Python 标准库。"""

from .core import (
    MAX_BATCH_SIZE,
    MAX_DRIVER_NAME_LENGTH,
    MAX_TIME,
    BitSample,
    BusDecodeError,
    BusDecoder,
    ConflictingUpdateError,
    DataByte,
    Event,
    InvalidTimeError,
    InvalidUpdateError,
    Level,
    Line,
    Segment,
    TimeOrderError,
    Transaction,
    Update,
)

__version__ = "1.0.0"
