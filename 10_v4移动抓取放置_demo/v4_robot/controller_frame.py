from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CameraFrame:
    subsystem_name: str
    state: int
    data_type: str
    data: bytes


def unpack_envelope(raw: bytes) -> tuple[str, bytes]:
    end = raw.index(b"\0")
    topic = raw[:end].decode("utf-8")
    offset = end + 1
    metadata_size = struct.unpack_from(">H", raw, offset)[0]
    offset += 2 + metadata_size
    payload_size = struct.unpack_from(">I", raw, offset)[0]
    offset += 4
    payload = raw[offset:offset + payload_size]
    if len(payload) != payload_size:
        raise ValueError("truncated controller payload")
    return topic, payload


def _varint(data: bytes, offset: int) -> tuple[int, int]:
    value = shift = 0
    while True:
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7f) << shift
        if not byte & 0x80:
            return value, offset
        shift += 7


def _fields(data: bytes):
    offset = 0
    while offset < len(data):
        tag, offset = _varint(data, offset)
        number, wire = tag >> 3, tag & 7
        if wire == 0:
            value, offset = _varint(data, offset)
        elif wire == 1:
            value, offset = data[offset:offset + 8], offset + 8
        elif wire == 2:
            size, offset = _varint(data, offset)
            value, offset = data[offset:offset + size], offset + size
        elif wire == 5:
            value, offset = data[offset:offset + 4], offset + 4
        else:
            raise ValueError(f"unsupported protobuf wire type {wire}")
        yield number, wire, value


def camera_frames(payload: bytes) -> list[CameraFrame]:
    result: list[CameraFrame] = []
    for number, wire, value in _fields(payload):
        if number != 2 or wire != 2:
            continue
        values: dict[int, Any] = {}
        for field, _, item in _fields(value):
            values[field] = item
        result.append(CameraFrame(
            bytes(values.get(1, b"")).decode("utf-8", errors="replace"),
            int(values.get(3, 0)),
            bytes(values.get(4, b"")).decode("utf-8", errors="replace"),
            bytes(values.get(5, b"")),
        ))
    return result
