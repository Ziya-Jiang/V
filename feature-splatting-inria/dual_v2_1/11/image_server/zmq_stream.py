"""ZMQ frame codec: color JPEG + optional depth visualization JPEG."""

import struct

MAGIC = b"RSD1"
_HEADER = struct.Struct(">4sII")  # magic, color_len, depth_len


def pack_frame(color_jpg: bytes, depth_jpg: bytes | None = None) -> bytes:
    depth_jpg = depth_jpg or b""
    return _HEADER.pack(MAGIC, len(color_jpg), len(depth_jpg)) + color_jpg + depth_jpg


def unpack_frame(message: bytes) -> tuple[bytes | None, bytes | None]:
    """Return (color_jpg, depth_jpg). Legacy raw JPEG returns (message, None)."""
    if len(message) >= 3 and message[0:3] == b"\xff\xd8\xff":
        return message, None

    if len(message) < _HEADER.size:
        return None, None

    magic, color_len, depth_len = _HEADER.unpack_from(message)
    if magic != MAGIC:
        return None, None

    offset = _HEADER.size
    color_end = offset + color_len
    depth_end = color_end + depth_len
    if depth_end > len(message):
        return None, None

    color_jpg = message[offset:color_end] if color_len else None
    depth_jpg = message[color_end:depth_end] if depth_len else None
    return color_jpg, depth_jpg


_UNIT_TEST_HEADER = struct.calcsize("dI")


def parse_zmq_message(message: bytes) -> tuple[bytes | None, bytes | None]:
    """Parse ZMQ payload: RSD1 packet, legacy JPEG, or unit-test header + payload."""
    if len(message) >= 4 and message[:4] == MAGIC:
        return unpack_frame(message)
    if len(message) >= 3 and message[:3] == b"\xff\xd8\xff":
        return unpack_frame(message)
    if len(message) > _UNIT_TEST_HEADER:
        return unpack_frame(message[_UNIT_TEST_HEADER:])
    return None, None
