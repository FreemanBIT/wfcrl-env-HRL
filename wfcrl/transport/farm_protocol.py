"""
farm_protocol.py — FarmController ↔ farm_control_runtime 帧协议（ICD v1.5）

二进制帧 = C 结构体（fcr_state_types.h）的原始字节（little-endian，与
x64 Windows/Linux 布局一致）。ZMQ 消息即帧。

ICD v1.5 变更:
  - 命令帧: 删除 yaw_delta_rad（仅使用绝对目标 yaw_target_heading_rad）
  - 命令帧: 新增 control_enable, control_mode, requested_effective_sim_time_s
  - 命令帧: source_time_s → command_basis_sim_time_s; frame_seq → command_frame_seq
  - 状态帧: frame_seq → state_frame_seq; sim_time_s → state_sample_sim_time_s
  - 状态帧: FcrFarmTurbineState 新增 S1011/S1012/S1015-S1017 → 360 → 400 B
  - 状态帧: FarmStateFrame 23096 → 25664 B（头 64 B + 400×64）

本模块是 C 侧的**逐字节镜像**：尺寸与关键偏移均有 assert 冻结，
并由 farm_control_runtime/tools/check_abi.py 与 C 侧自动交叉校验。
任何 assert 触发都表示 ABI 破坏（历史上曾导致 ZMQ 帧被全部拒收，见 WIN-05）。
"""
from __future__ import annotations

import ctypes
import math
from typing import Optional

# ---------------------------------------------------------------------------
# 常量（与 fcr_protocol.h 同步）
# ---------------------------------------------------------------------------
FCR_PROTOCOL_VERSION = 2  # ICD v1.5
FCR_MAX_TURBINES = 64

# control_mode (S0111)
FCR_CONTROL_MODE_BASELINE = 0
FCR_CONTROL_MODE_PRC = 1
FCR_CONTROL_MODE_CT_FEED = 2
FCR_CONTROL_MODE_CLOSEDLOOP = 3

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
FCR_CMD_FLAG_CONTROL_DISABLED = 1 << 9

# ---------------------------------------------------------------------------
# C 结构镜像（字节序/对齐与 C 一致）
# ---------------------------------------------------------------------------

class FcrTurbineCommand(ctypes.Structure):
    """ICD v1.5 S0103-S0112"""
    _fields_ = [
        # 偏航通道 (S0108-S0110)
        ("yaw_target_heading_rad", ctypes.c_double),    # S0108
        ("yaw_seq", ctypes.c_uint64),                   # S0109
        ("yaw_valid", ctypes.c_uint32),                 # S0110
        # 诱导通道 (S0105-S0107)
        ("induction_ref", ctypes.c_double),             # S0105
        ("induction_seq", ctypes.c_uint64),             # S0106
        ("induction_valid", ctypes.c_uint32),           # S0107
        # 通用控制字段
        ("control_enable", ctypes.c_uint32),            # S0104
        ("control_mode", ctypes.c_uint16),              # S0111
        ("requested_effective_sim_time_s", ctypes.c_double),  # S0112
        # 时间戳
        ("command_basis_sim_time_s", ctypes.c_double),  # S0102
        # 内部实现字段
        ("yaw_ttl_s", ctypes.c_double),
        ("induction_ttl_s", ctypes.c_double),
    ]

class FarmCommandFrame(ctypes.Structure):
    _fields_ = [
        ("protocol_version", ctypes.c_uint32),
        ("command_frame_seq", ctypes.c_uint64),         # S0101
        ("n_turbines", ctypes.c_uint32),
        ("receive_time_s", ctypes.c_double),            # 内部
        ("turbines", FcrTurbineCommand * FCR_MAX_TURBINES),
    ]


class FcrSignalStats(ctypes.Structure):
    """1 s 统计窗口（mean/rms/min/max + 瞬时值）"""
    _fields_ = [
        ("instant", ctypes.c_double),
        ("mean", ctypes.c_double),
        ("rms", ctypes.c_double),
        ("min", ctypes.c_double),
        ("max", ctypes.c_double),
        ("n_samples", ctypes.c_uint32),
        ("valid", ctypes.c_uint32),
    ]


class FcrFarmTurbineState(ctypes.Structure):
    """ICD v1.5 S1001-S1017 单机状态（400 B）"""
    _fields_ = [
        ("turbine_id", ctypes.c_int32),                        # S1003
        ("valid", ctypes.c_uint32),
        ("source_time_s", ctypes.c_double),
        # 高速量：瞬时 + 1 s 统计
        ("gen_power_w", FcrSignalStats),                       # S1004/S1005
        ("rotor_aero_thrust_n", FcrSignalStats),               # S1010
        ("tower_base_fa_moment_nm", FcrSignalStats),
        ("tower_base_ss_moment_nm", FcrSignalStats),
        ("rotor_speed_rad_s", ctypes.c_double),                # S1006
        ("gen_speed_rad_s", ctypes.c_double),                  # S1007
        ("collective_pitch_actual_rad", ctypes.c_double),      # S1008
        ("nacelle_heading_actual_rad", ctypes.c_double),       # S1009
        # 风估计 (S1011, S1016, S1017)
        ("estimated_wind_mps", ctypes.c_double),               # S1011
        ("wind_source", ctypes.c_uint16),                      # S1016
        ("wind_quality_flags", ctypes.c_uint16),               # S1017
        ("ct_hat", ctypes.c_double),                           # S1012
        # 状态确认
        ("yaw_seq_applied", ctypes.c_uint64),                  # S1013
        ("induction_seq_applied", ctypes.c_uint64),            # S1014
        ("turbine_status_flags", ctypes.c_uint32),             # S1015
        # 扩展字段
        ("nacelle_vane_rad", ctypes.c_double),
        ("hub_wind_speed_mps", ctypes.c_double),
        ("rotor_ct", ctypes.c_double),
        ("rotor_cp", ctypes.c_double),
        ("blade_pitch_rad", ctypes.c_double * 3),
        ("yaw_target_heading_rad", ctypes.c_double),
        ("controller_status_flags", ctypes.c_uint32),
        # 低速量（AWAE，1 s 之间 ZOH）
        ("disk_ambient_wind_mps", ctypes.c_double),
        ("disk_disturbed_wind_mps", ctypes.c_double),
        ("disk_ambient_ti", ctypes.c_double),
        ("flow_source_time_s", ctypes.c_double),
        ("flow_seq", ctypes.c_uint64),
    ]


class FarmStateFrame(ctypes.Structure):
    """ICD v1.5 S1001-S1017 场站状态帧（25664 B）"""
    _fields_ = [
        ("protocol_version", ctypes.c_uint32),
        ("state_frame_seq", ctypes.c_uint64),           # S1001
        ("state_sample_sim_time_s", ctypes.c_double),   # S1002
        ("fast_step", ctypes.c_uint64),
        ("low_step", ctypes.c_uint64),
        ("rt_health_flags", ctypes.c_uint32),
        ("n_turbines", ctypes.c_uint32),
        ("flow_source_time_s", ctypes.c_double),
        ("flow_seq", ctypes.c_uint64),
        ("turbines", FcrFarmTurbineState * FCR_MAX_TURBINES),
    ]


# ---------------------------------------------------------------------------
# ABI 冻结断言（必须与 tests/unit/test_abi.c 一致）
# ---------------------------------------------------------------------------
# FcrTurbineCommand: 8+8+4(+4pad)+8+8+4+4+2(+6pad)+8+8+8+8 = 88
# FarmCommandFrame: 4(+4pad)+8+4(+4pad)+8 = 32 header, then 88*64
assert ctypes.sizeof(FcrTurbineCommand) == 88, ctypes.sizeof(FcrTurbineCommand)
assert ctypes.sizeof(FarmCommandFrame) == 5664, ctypes.sizeof(FarmCommandFrame)
assert ctypes.sizeof(FcrSignalStats) == 48, ctypes.sizeof(FcrSignalStats)
assert ctypes.sizeof(FcrFarmTurbineState) == 400, ctypes.sizeof(FcrFarmTurbineState)
assert ctypes.sizeof(FarmStateFrame) == 25664, ctypes.sizeof(FarmStateFrame)

# 关键偏移（任何漂移都会让 Python 解析错位）
assert FarmCommandFrame.turbines.offset == 32, FarmCommandFrame.turbines.offset
assert FarmCommandFrame.command_frame_seq.offset == 8
assert FcrTurbineCommand.control_enable.offset == 44
assert FcrTurbineCommand.command_basis_sim_time_s.offset == 64
assert FarmStateFrame.n_turbines.offset == 44, FarmStateFrame.n_turbines.offset
assert FarmStateFrame.turbines.offset == 64, FarmStateFrame.turbines.offset
assert FcrFarmTurbineState.nacelle_heading_actual_rad.offset == 232
assert FcrFarmTurbineState.estimated_wind_mps.offset == 240
assert FcrFarmTurbineState.yaw_seq_applied.offset == 264
assert FcrFarmTurbineState.turbine_status_flags.offset == 280

# 帧尺寸常量（供传输层使用，禁止在别处硬编码）
FCR_COMMAND_FRAME_SIZE = ctypes.sizeof(FarmCommandFrame)
FCR_STATE_FRAME_SIZE = ctypes.sizeof(FarmStateFrame)
FCR_STATE_FRAME_HEADER_SIZE = FarmStateFrame.turbines.offset
FCR_TURBINE_STATE_SIZE = ctypes.sizeof(FcrFarmTurbineState)


# ---------------------------------------------------------------------------
# 角度约定换算（与 bridge/yaw_adapter.c 语义严格一致）
#   外部动作语义（RCP/SCADA）：yaw_delta，CW+（罗盘方向）
#   内部正式通道（ICD v1.5 S0108）：绝对机舱方位，CCW+，0=+X
#   换算：target = wrap_pm_pi(heading_now - yaw_delta)
# ---------------------------------------------------------------------------
def wrap_pm_pi(a: float) -> float:
    """把角度归一化到 (-pi, pi]"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_delta_cw_to_absolute_ccw(delta_rad: float, heading_now_rad: float) -> float:
    """CW+ 增量（动作指令语义）→ CCW+ 绝对目标（S0108 正式通道）。

    与 bridge/yaw_adapter.c: fcr_yaw_adapter_delta_to_target 等价。
    """
    return wrap_pm_pi(heading_now_rad - delta_rad)


def yaw_absolute_ccw_to_delta_cw(target_rad: float, heading_now_rad: float) -> float:
    """CCW+ 绝对目标 → CW+ 增量（诊断/回读用，逆换算）"""
    return -wrap_pm_pi(target_rad - heading_now_rad)


# ---------------------------------------------------------------------------
# 命令帧编解码
# ---------------------------------------------------------------------------
def frame_to_bytes(frame: FarmCommandFrame) -> bytes:
    return bytes(ctypes.cast(ctypes.byref(frame), ctypes.POINTER(ctypes.c_ubyte))[0:ctypes.sizeof(frame)])

def bytes_to_frame(data: bytes) -> FarmCommandFrame:
    if len(data) < ctypes.sizeof(FarmCommandFrame):
        raise ValueError(f"short frame: {len(data)}")
    return FarmCommandFrame.from_buffer_copy(data[:ctypes.sizeof(FarmCommandFrame)])

def new_command_frame(n_turbines: int, command_frame_seq: int, receive_time_s: float = 0.0) -> FarmCommandFrame:
    frame = FarmCommandFrame()
    frame.protocol_version = FCR_PROTOCOL_VERSION
    frame.command_frame_seq = command_frame_seq
    frame.n_turbines = n_turbines
    frame.receive_time_s = receive_time_s
    # Default: enable external control
    for i in range(n_turbines):
        frame.turbines[i].control_enable = 1
        frame.turbines[i].control_mode = FCR_CONTROL_MODE_PRC
    return frame

def set_yaw_cmd(frame: FarmCommandFrame, turbine_id: int, target_heading_rad: float,
                seq: int, ttl_s: float = 60.0, 
                requested_effective_sim_time_s: float = 0.0) -> None:
    """填入单机 yaw 通道 (S0108-S0110)。

    ⚠ ICD v1.5：target_heading_rad 是**绝对机舱目标方位**（CCW+，0=+X），
    不是增量。CW+ 动作语义请先经 yaw_delta_cw_to_absolute_ccw() 换算。
    """
    if not (1 <= turbine_id <= FCR_MAX_TURBINES):
        raise ValueError(turbine_id)
    import time as _time
    t = frame.turbines[turbine_id - 1]
    t.yaw_valid = 1
    t.yaw_seq = seq
    t.yaw_target_heading_rad = float(target_heading_rad)
    t.yaw_ttl_s = float(ttl_s)
    t.requested_effective_sim_time_s = float(requested_effective_sim_time_s)
    # WIN-05: set command_basis_sim_time_s for ZMQ frame validation
    if t.command_basis_sim_time_s <= 0.0:
        t.command_basis_sim_time_s = _time.time()

def set_induction_cmd(frame: FarmCommandFrame, turbine_id: int, induction_ref: float,
                      seq: int, ttl_s: float = 1.0,
                      requested_effective_sim_time_s: float = 0.0,
                      control_mode: int = FCR_CONTROL_MODE_PRC) -> None:
    """填入单机 induction 通道 (S0105-S0107)"""
    if not (1 <= turbine_id <= FCR_MAX_TURBINES):
        raise ValueError(turbine_id)
    import time as _time
    t = frame.turbines[turbine_id - 1]
    t.induction_valid = 1
    t.induction_seq = seq
    t.induction_ref = float(induction_ref)
    t.induction_ttl_s = float(ttl_s)
    t.requested_effective_sim_time_s = float(requested_effective_sim_time_s)
    t.control_mode = control_mode
    # WIN-05: set command_basis_sim_time_s for ZMQ frame validation
    if t.command_basis_sim_time_s <= 0.0:
        t.command_basis_sim_time_s = _time.time()

def set_control_enable(frame: FarmCommandFrame, turbine_id: int, enable: bool) -> None:
    """设置 control_enable (S0104)"""
    if not (1 <= turbine_id <= FCR_MAX_TURBINES):
        raise ValueError(turbine_id)
    frame.turbines[turbine_id - 1].control_enable = 1 if enable else 0

def set_control_mode(frame: FarmCommandFrame, turbine_id: int, mode: int) -> None:
    """设置 control_mode (S0111)"""
    if not (1 <= turbine_id <= FCR_MAX_TURBINES):
        raise ValueError(turbine_id)
    frame.turbines[turbine_id - 1].control_mode = mode


# ---------------------------------------------------------------------------
# 状态帧解析
# ---------------------------------------------------------------------------
def state_frame_from_bytes(data: bytes) -> FarmStateFrame:
    """把 ZMQ 状态帧原始字节解析为 FarmStateFrame。

    注意：必须做**尺寸校验**——尺寸不符说明 C/Python ABI 不一致，
    宁可显式报错也不要按错误偏移解析出垃圾数据。
    """
    if len(data) != FCR_STATE_FRAME_SIZE:
        raise ValueError(
            f"state frame size mismatch: got {len(data)}, expected {FCR_STATE_FRAME_SIZE}")
    return FarmStateFrame.from_buffer_copy(data)

def state_frame_from_bytes_lenient(data: bytes) -> Optional[FarmStateFrame]:
    """尺寸不符时返回 None（接收循环用，避免抛异常打断线程）。"""
    if len(data) != FCR_STATE_FRAME_SIZE:
        return None
    return FarmStateFrame.from_buffer_copy(data)

def turbine_state_from_slot(slot: bytes) -> FcrFarmTurbineState:
    """从单机槽位字节（400 B）解析单机状态"""
    if len(slot) != FCR_TURBINE_STATE_SIZE:
        raise ValueError(
            f"turbine slot size mismatch: got {len(slot)}, expected {FCR_TURBINE_STATE_SIZE}")
    return FcrFarmTurbineState.from_buffer_copy(slot)
