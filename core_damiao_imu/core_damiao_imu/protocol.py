"""DM-IMU-L1 USB protocol parsing and conversion helpers."""

from dataclasses import dataclass
import math
import struct
from typing import Iterable, List, Sequence, Tuple


HEADER = b"\x55\xaa"
TAIL = 0x0A
FRAME_LENGTH = 19

RID_ACCELERATION = 1
RID_ANGULAR_VELOCITY = 2
RID_EULER = 3
VALID_RIDS = frozenset(
    (RID_ACCELERATION, RID_ANGULAR_VELOCITY, RID_EULER)
)

ENTER_SETTINGS = b"\xaa\x06\x01\x0d"
ENABLE_ACCELERATION = b"\xaa\x01\x14\x0d"
ENABLE_ANGULAR_VELOCITY = b"\xaa\x01\x15\x0d"
ENABLE_EULER = b"\xaa\x01\x16\x0d"
DISABLE_QUATERNION = b"\xaa\x01\x07\x0d"
EXIT_SETTINGS = b"\xaa\x06\x00\x0d"
SAVE_SETTINGS = b"\xaa\x03\x01\x0d"

_OUTPUT_INTERVAL_MS = {
    100: 10,
    125: 8,
    200: 5,
    250: 4,
    500: 2,
    1000: 1,
}


def _make_crc16_table() -> Tuple[int, ...]:
    table = []
    for value in range(256):
        crc = value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC16_TABLE = _make_crc16_table()


def crc16(data: Iterable[int]) -> int:
    """Return the CRC16 used by the DM-IMU-L1 data frames."""
    crc = 0xFFFF
    for value in data:
        index = ((crc >> 8) ^ value) & 0xFF
        crc = ((crc << 1) ^ _CRC16_TABLE[index]) & 0xFFFF
    return crc


@dataclass(frozen=True)
class DamiaoFrame:
    """One validated DM-IMU-L1 data frame."""

    slave_id: int
    rid: int
    values: Tuple[float, float, float]


class DamiaoFrameParser:
    """Incremental parser that re-synchronizes after malformed input."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.valid_frame_count = 0
        self.crc_error_count = 0
        self.format_error_count = 0
        self.nonfinite_value_count = 0

    def reset(self) -> None:
        """Discard buffered bytes and reset stream synchronization."""
        self._buffer.clear()

    def feed(self, data: bytes) -> List[DamiaoFrame]:
        """Consume arbitrary bytes and return every complete valid frame."""
        if data:
            self._buffer.extend(data)

        frames = []
        while True:
            header_index = self._buffer.find(HEADER)
            if header_index < 0:
                if self._buffer[-1:] == HEADER[:1]:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break

            if header_index:
                del self._buffer[:header_index]

            if len(self._buffer) < FRAME_LENGTH:
                break

            candidate = bytes(self._buffer[:FRAME_LENGTH])
            rid = candidate[3]
            if candidate[-1] != TAIL or rid not in VALID_RIDS:
                self.format_error_count += 1
                del self._buffer[0]
                continue

            expected_crc = candidate[16] | (candidate[17] << 8)
            if crc16(candidate[:16]) != expected_crc:
                self.crc_error_count += 1
                del self._buffer[0]
                continue

            values = struct.unpack_from("<fff", candidate, 4)
            if not all(math.isfinite(value) for value in values):
                self.nonfinite_value_count += 1
                del self._buffer[:FRAME_LENGTH]
                continue

            frames.append(DamiaoFrame(candidate[2], rid, values))
            self.valid_frame_count += 1
            del self._buffer[:FRAME_LENGTH]

        return frames


def configuration_commands(output_rate_hz: int) -> Sequence[bytes]:
    """Return volatile configuration commands for the selected output rate."""
    try:
        interval_ms = _OUTPUT_INTERVAL_MS[output_rate_hz]
    except KeyError as error:
        supported = ", ".join(str(rate) for rate in _OUTPUT_INTERVAL_MS)
        raise ValueError(
            f"unsupported output rate {output_rate_hz}; choose one of {supported} Hz"
        ) from error

    return (
        ENTER_SETTINGS,
        ENABLE_ACCELERATION,
        ENABLE_ANGULAR_VELOCITY,
        ENABLE_EULER,
        DISABLE_QUATERNION,
        bytes((0xAA, 0x02, interval_ms & 0xFF, interval_ms >> 8, 0x0D)),
        EXIT_SETTINGS,
    )


def euler_zyx_degrees_to_quaternion(
    roll_degrees: float, pitch_degrees: float, yaw_degrees: float
) -> Tuple[float, float, float, float]:
    """Convert roll/pitch/yaw degrees to a normalized ZYX quaternion."""
    roll = math.radians(roll_degrees)
    pitch = math.radians(pitch_degrees)
    yaw = math.radians(yaw_degrees)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)
