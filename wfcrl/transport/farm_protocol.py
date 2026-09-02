"""
farm_protocol.py — FarmController ↔ farm_control_runtime 帧协议（Phase 7 冻结）

二进制帧 = C 结构体（fcr_state_types.h）的原始字节（little-endian，与
x64 Windows/Linux 布局一致）。ZMQ 消息即帧。
"""
from __future__ import annotations

import ctypes
from typing import Optional

# ---------------------------------------------------------------------------
# 常量（与 fcr_protocol.h 同步）
# ---------------------------------------------------------------------------
FCR_PROTOCOL_VERSION = 1
FCR_MAX_TURBINES = 64

# command_status_flags
FCR_CMD_FLAG_VALID = 1 << 0
FCR_CMD_FLAG_YAW_VALID = 1 << 1
FCR_CMD_FLAG_INDUCTION_VALID = 1 << 2
FCR_CMD_FLAG_YAW_STALE = 1 << 3
FCR_CMD_FLAG_INDUCTION_STALE = 1 << 4
FCR_CMD_FLAG_YAW_TTL_EXPIRED = 1 << 5
FCR_CMD_FLAG_INDUCTION_TTL_EXPIRED = 1 << 6
FCR_CMD_FLAG_INVALID_SEQ = 1 << 7
FCR_CMD_FLAG_FALLBACK_ACTIVE = 1 << 8

# ---------------------------------------------------------------------------
# C 结构镜像（字节序/对齐与 C 一致）
# ---------------------------------------------------------------------------

class FcrTurbineCommand(ctypes.Structure):
    _fields_ = [
        ("yaw_delta_rad", ctypes.c_double),
        ("yaw_seq", ctypes.c_uint64),
        ("yaw_valid", ctypes.c_uint32),
        ("yaw_effective_low_step", ctypes.c_int64),
        ("yaw_ttl_s", ctypes.c_double),
        ("induction_ref", ctypes.c_double),
        ("induction_seq", ctypes.c_uint64),
        ("induction_valid", ctypes.c_uint32),
        ("induction_effective_low_step", ctypes.c_int64),
        ("induction_ttl_s", ctypes.c_double),
        ("source_time_s", ctypes.c_double),
    ]

class FarmCommandFrame(ctypes.Structure):
    _fields_ = [
        ("protocol_version", ctypes.c_uint32),
        ("frame_seq", ctypes.c_uint64),
        ("commit_seq", ctypes.c_uint64),
        ("n_turbines", ctypes.c_uint32),
        ("receive_time_s", ctypes.c_double),
        ("turbines", FcrTurbineCommand * FCR_MAX_TURBINES),
    ]

assert ctypes.sizeof(FcrTurbineCommand) == 88
assert ctypes.sizeof(FarmCommandFrame) == 5672

def frame_to_bytes(frame: FarmCommandFrame) -> bytes:
    return bytes(ctypes.cast(ctypes.byref(frame), ctypes.POINTER(ctypes.c_ubyte))[0:ctypes.sizeof(frame)])

def bytes_to_frame(data: bytes) -> FarmCommandFrame:
    if len(data) < ctypes.sizeof(FarmCommandFrame):
        raise ValueError(f"short frame: {len(data)}")
    return FarmCommandFrame.from_buffer_copy(data[:ctypes.sizeof(FarmCommandFrame)])

def new_command_frame(n_turbines: int, frame_seq: int, receive_time_s: float = 0.0) -> FarmCommandFrame:
    frame = FarmCommandFrame()
    frame.protocol_version = FCR_PROTOCOL_VERSION
    frame.frame_seq = frame_seq
    frame.commit_seq = frame_seq
    frame.n_turbines = n_turbines
    frame.receive_time_s = receive_time_s
    return frame

def set_yaw_cmd(frame: FarmCommandFrame, turbine_id: int, delta_rad: float,
                seq: int, ttl_s: float = 60.0, effective_low_step: int = -1) -> None:
    """填入单机 yaw 通道（不影响其他机/其他通道）"""
    if not (1 <= turbine_id <= FCR_MAX_TURBINES):
        raise ValueError(turbine_id)
    t = frame.turbines[turbine_id - 1]
    t.yaw_valid = 1
    t.yaw_seq = seq
    t.yaw_delta_rad = float(delta_rad)
    t.yaw_ttl_s = float(ttl_s)
    t.yaw_effective_low_step = effective_low_step

def set_induction_cmd(frame: FarmCommandFrame, turbine_id: int, induction_ref: float,
                      seq: int, ttl_s: float = 1.0, effective_low_step: int = -1) -> None:
    """填入单机 induction 通道"""
    if not (1 <= turbine_id <= FCR_MAX_TURBINES):
        raise ValueError(turbine_id)
    t = frame.turbines[turbine_id - 1]
    t.induction_valid = 1
    t.induction_seq = seq
    t.induction_ref = float(induction_ref)
    t.induction_ttl_s = float(ttl_s)
    t.induction_effective_low_step = effective_low_step

__all__ = [
    "FCR_PROTOCOL_VERSION", "FCR_MAX_TURBINES",
    "FcrTurbineCommand", "FarmCommandFrame",
    "frame_to_bytes", "bytes_to_frame", "new_command_frame",
    "set_yaw_cmd", "set_induction_cmd",
]
