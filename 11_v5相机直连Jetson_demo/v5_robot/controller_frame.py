"""机械臂控制器二进制帧的解析模块。

控制器（Jetson 上的相机服务订阅它发布的 ZMQ 话题）下发的是自描述二进制包：
先用 ``unpack_envelope`` 剥掉「话题名 + 元数据」的信封，剩下的 payload 是一段
protobuf 风格的编码数据，再用 ``camera_frames`` 按字段号取出其中的相机帧
（子系统名、状态码、载荷类型、图像字节）。

编码约定：
- 定长整数一律大端（network byte order），例如 ``>H`` 为 2 字节、``>I`` 为 4 字节；
- 变长整数用 protobuf 的 varint：每字节低 7 位有效，最高位为 1 表示还有后续字节；
- 字段标签 tag 的高位是字段号，低 3 位是 wire type（0=varint、1=64 位定长、
  2=长度前缀、5=32 位定长）。

本模块只做解析，不负责网络收发与 JPEG 解码。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CameraFrame:
    """一帧相机数据的元信息（不可变数据结构）。

    字段说明：
        subsystem_name —— 相机子系统名，例如 "RsCameraSensor#0#0"
        state          —— 控制器给出的状态码（整数，具体含义由控制器定义）
        data_type      —— 载荷类型，例如 "img/jpeg"
        data           —— 图像原始字节（未做解码）
    """

    subsystem_name: str
    state: int
    data_type: str
    data: bytes


def unpack_envelope(raw: bytes) -> tuple[str, bytes]:
    """拆开一条控制器消息的信封，返回 (话题名, payload 字节)。

    信封布局（各整数均为大端）：

        [话题名 UTF-8] \\0 | [元数据长度 uint16] | [元数据] | [payload 长度 uint32] | [payload]

    元数据本次解析用不到，但仍要按其长度跳过，才能定位到 payload 的起始位置。
    若实际收到的字节数不足声明的 payload 长度（网络截断），抛 ValueError。
    """
    # 话题名以 \0 结尾
    end = raw.index(b"\0")
    topic = raw[:end].decode("utf-8")
    offset = end + 1
    metadata_size = struct.unpack_from(">H", raw, offset)[0]
    # 跳过 2 字节的长度字段本身，加上其后的元数据内容
    offset += 2 + metadata_size
    payload_size = struct.unpack_from(">I", raw, offset)[0]
    offset += 4
    payload = raw[offset:offset + payload_size]
    # 声明长度与实际长度不一致，说明数据被截断，宁可报错也不要解析出半帧图像
    if len(payload) != payload_size:
        raise ValueError("truncated controller payload")
    return topic, payload


def _varint(data: bytes, offset: int) -> tuple[int, int]:
    """按 protobuf varint 规则从 offset 处解出一个整数。

    每字节的低 7 位参与数值、最高位为 1 表示后面还有字节，拼接时逐字节左移 7 位。
    返回 (解出的值, 下一个字节的偏移)。本函数不做越界与超长校验，
    输入异常时由调用方的字节索引错误暴露出来。
    """
    value = shift = 0
    while True:
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7f) << shift
        # 最高位为 0 说明这是本整数的最后一个字节
        if not byte & 0x80:
            return value, offset
        shift += 7


def _fields(data: bytes):
    """迭代解析 protobuf 风格消息，逐字段产出 (字段号, wire type, 值)。

    wire type 与取出的值类型对应关系：
        0 —— varint，值为 int
        1 —— 64 位定长，值为 8 字节 bytes
        2 —— 长度前缀（字符串或嵌套消息），值为 bytes
        5 —— 32 位定长，值为 4 字节 bytes
    其他类型（含已废弃的 3/4 组）直接抛 ValueError。
    """
    offset = 0
    while offset < len(data):
        tag, offset = _varint(data, offset)
        # 字段号在高位，wire type 占最低 3 位
        number, wire = tag >> 3, tag & 7
        if wire == 0:
            value, offset = _varint(data, offset)
        elif wire == 1:
            # 64 位定长：直接切 8 字节
            value, offset = data[offset:offset + 8], offset + 8
        elif wire == 2:
            # 长度前缀：先读长度，再按长度切出内容
            size, offset = _varint(data, offset)
            value, offset = data[offset:offset + size], offset + size
        elif wire == 5:
            # 32 位定长：直接切 4 字节
            value, offset = data[offset:offset + 4], offset + 4
        else:
            raise ValueError(f"unsupported protobuf wire type {wire}")
        yield number, wire, value


def camera_frames(payload: bytes) -> list[CameraFrame]:
    """从 payload 中提取所有相机帧。

    外层消息只需关心字段号 2（且必须是长度前缀类型），它重复出现即代表一帧。
    帧内字段号映射关系：
        1 -> subsystem_name（UTF-8 字符串）
        3 -> state（varint 整数）
        4 -> data_type（UTF-8 字符串）
        5 -> data（原始字节）

    缺失的字段用空字节 / 0 兜底；文本解码失败时用替换字符而不是抛异常，
    这样单帧数据异常不会让整个取流服务中断。
    """
    result: list[CameraFrame] = []
    for number, wire, value in _fields(payload):
        # 只取字段号 2 的重复消息（即相机帧），其余字段直接忽略
        if number != 2 or wire != 2:
            continue
        values: dict[int, Any] = {}
        # 帧内按字段号收集；同一帧内字段不重复，后值覆盖前值即可
        for field, _, item in _fields(value):
            values[field] = item
        # 字段缺失时回退为空字符串 / 0，避免 KeyError 打断取流
        result.append(CameraFrame(
            bytes(values.get(1, b"")).decode("utf-8", errors="replace"),
            int(values.get(3, 0)),
            bytes(values.get(4, b"")).decode("utf-8", errors="replace"),
            bytes(values.get(5, b"")),
        ))
    return result
