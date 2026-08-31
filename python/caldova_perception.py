from __future__ import annotations

import ctypes
from dataclasses import dataclass
from pathlib import Path


class _FrameInfo(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("channels", ctypes.c_uint8),
        ("lidar_points", ctypes.c_uint32),
        ("timestamp_ms", ctypes.c_uint64),
    ]


@dataclass(frozen=True)
class FrameInfo:
    width: int
    height: int
    channels: int
    lidar_points: int
    timestamp_ms: int


class PerceptionLibrary:
    def __init__(self, library_path: str | Path):
        self._lib = ctypes.CDLL(str(library_path))
        self._lib.rb_decode_frame.argtypes = [
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_size_t,
            ctypes.POINTER(_FrameInfo),
        ]
        self._lib.rb_decode_frame.restype = ctypes.c_int
        self._lib.rb_last_error.argtypes = []
        self._lib.rb_last_error.restype = ctypes.c_char_p

    def decode_frame(self, packet: bytes) -> FrameInfo:
        buf = (ctypes.c_uint8 * len(packet)).from_buffer_copy(packet)
        native = _FrameInfo()
        rc = self._lib.rb_decode_frame(buf, len(packet), ctypes.byref(native))
        if rc != 0:
            message = self._lib.rb_last_error()
            raise RuntimeError(message.decode("utf-8") if message else "decode failed")
        return FrameInfo(
            width=native.width,
            height=native.height,
            channels=native.channels,
            lidar_points=native.lidar_points,
            timestamp_ms=native.timestamp_ms,
        )
