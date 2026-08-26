import math
import struct

import pytest

from core_damiao_imu.protocol import configuration_commands
from core_damiao_imu.protocol import crc16
from core_damiao_imu.protocol import DamiaoFrameParser
from core_damiao_imu.protocol import euler_zyx_degrees_to_quaternion
from core_damiao_imu.protocol import SAVE_SETTINGS


def make_frame(rid, values=(1.0, 2.0, 3.0), slave_id=1):
    payload = bytearray((0x55, 0xAA, slave_id, rid))
    payload.extend(struct.pack("<fff", *values))
    checksum = crc16(payload)
    payload.extend((checksum & 0xFF, checksum >> 8, 0x0A))
    return bytes(payload)


def test_crc_matches_official_reference_vector():
    payload = bytes.fromhex("55aa0103000020410000a0410000f041")

    assert crc16(payload) == 0x5E84


@pytest.mark.parametrize("rid", [1, 2, 3])
def test_parser_accepts_valid_rids(rid):
    parser = DamiaoFrameParser()

    frames = parser.feed(make_frame(rid))

    assert len(frames) == 1
    assert frames[0].slave_id == 1
    assert frames[0].rid == rid
    assert frames[0].values == pytest.approx((1.0, 2.0, 3.0))


def test_parser_handles_noise_fragmentation_and_concatenated_frames():
    parser = DamiaoFrameParser()
    first = make_frame(1, (4.0, 5.0, 6.0))
    second = make_frame(3, (10.0, 20.0, 30.0))

    assert parser.feed(b"noise\x55" + first[:8]) == []
    frames = parser.feed(first[8:] + b"junk" + second)

    assert [frame.rid for frame in frames] == [1, 3]
    assert frames[1].values == pytest.approx((10.0, 20.0, 30.0))


def test_parser_rejects_bad_crc_tail_rid_and_nonfinite_values_then_resyncs():
    parser = DamiaoFrameParser()
    bad_crc = bytearray(make_frame(1))
    bad_crc[16] ^= 0x80
    bad_tail = bytearray(make_frame(2))
    bad_tail[-1] = 0x0D
    bad_rid = make_frame(7)
    nan_frame = make_frame(3, (0.0, float("nan"), 0.0))

    frames = parser.feed(
        bytes(bad_crc) + bytes(bad_tail) + bad_rid + nan_frame + make_frame(3)
    )

    assert len(frames) == 1
    assert frames[0].rid == 3
    assert parser.crc_error_count == 1
    assert parser.format_error_count == 2
    assert parser.nonfinite_value_count == 1


def test_configuration_enables_required_data_without_saving_to_flash():
    commands = configuration_commands(200)

    assert commands == (
        b"\xaa\x06\x01\x0d",
        b"\xaa\x01\x14\x0d",
        b"\xaa\x01\x15\x0d",
        b"\xaa\x01\x16\x0d",
        b"\xaa\x01\x07\x0d",
        b"\xaa\x02\x05\x00\x0d",
        b"\xaa\x06\x00\x0d",
    )
    assert SAVE_SETTINGS not in commands


def test_configuration_rejects_unsupported_output_rate():
    with pytest.raises(ValueError):
        configuration_commands(60)


def test_euler_uses_degrees_and_zyx_order():
    quaternion = euler_zyx_degrees_to_quaternion(0.0, 0.0, 90.0)

    assert quaternion == pytest.approx(
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), abs=1e-7
    )
    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)
