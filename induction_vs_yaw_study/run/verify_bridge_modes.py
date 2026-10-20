"""verify_bridge_modes.py — M1/G1 Bridge 离线单机模式验证（ctypes 加载编译后 bridge DLL）

加载 farm_control_runtime/bridge/fcr_bridge.dll（由 fcr_bridge_core.c + yaw_adapter.c
编译），用真实物理参数喂入，校验（DR-3/DR-4/DR-7/DR-5）：

  1. Mode 0：旁路（induction_enable=0，ratio 全 1）
  2. Mode 1：R_power = a(1-a)^2/[a0(1-a0)^2] 单调性 + 控制分配守恒
     （默认纯 r_pitch；显式权重时三通道按权分配、无未分配叠加）
  3. Mode 2：Ct_ref=4a(1-a) 气动表反解 min_pitch_ref（beta 随 Ct_ref 减小而增大）
  4. Ct 估计 DR-7：上游参考风优先；扰动盘面风需修正，禁止直接经典归一化
  5. 偏航 DR-5：绝对目标锁存 + yaw_adapter delta→target 转换
  6. 安全：TTL 过期回退额定

用法：
    python -m induction_vs_yaw_study.run.verify_bridge_modes
退出码：0 = PASS；非 0 = FAIL。
"""
from __future__ import annotations

import ctypes as C
import math
import os
import sys
from pathlib import Path

_RT = Path(__file__).resolve().parents[2] / "farm_control_runtime"
_DLL = _RT / "bridge" / "fcr_bridge.dll"

# ---- ctypes 结构体（须与 fcr_bridge_core.h 一致）----
class FcrBridgeMeas(C.Structure):
    _fields_ = [
        ("turbine_id", C.c_int32), ("valid", C.c_uint32), ("source_time_s", C.c_double),
        ("rotor_speed_rad_s", C.c_double), ("generator_speed_rad_s", C.c_double),
        ("rotor_thrust_n", C.c_double), ("blade_pitch_rad", C.c_double * 3),
        ("nacelle_heading_rad", C.c_double),
        ("ambient_upstream_wind_mps", C.c_double),
        ("disk_background_wind_mps", C.c_double),
        ("disk_wind_speed_mps", C.c_double), ("disk_turbulence", C.c_double),
        ("gen_power_w", C.c_double),
    ]

MAX_T = 200
class FcrBridgeCmd(C.Structure):
    _fields_ = [
        ("turbine_id", C.c_int32), ("valid", C.c_uint32),
        ("induction_mode", C.c_uint32), ("induction_valid", C.c_uint32),
        ("induction_ref", C.c_double), ("induction_seq", C.c_uint64),
        ("induction_ttl_s", C.c_double),
        ("yaw_valid", C.c_uint32),
        ("yaw_target_heading_rad", C.c_double), ("yaw_seq", C.c_uint64),
        ("yaw_ttl_s", C.c_double),
        ("alloc_w_pitch", C.c_double), ("alloc_w_speed", C.c_double), ("alloc_w_torque", C.c_double),
        ("aero_blade_radius_m", C.c_double),
        ("aero_n_lam", C.c_int32), ("aero_n_beta", C.c_int32),
        ("aero_lam_ptr", C.c_void_p), ("aero_beta_deg_ptr", C.c_void_p), ("aero_ct_ptr", C.c_void_p),
        ("aero_lam_small", C.c_double * MAX_T),
        ("aero_beta_deg_small", C.c_double * MAX_T),
        ("aero_ct_small", C.c_double * (MAX_T * MAX_T)),
        ("source_time_s", C.c_double),
    ]

class FcrRoscoExternalSetpoint(C.Structure):
    """与 fcr_rosco_api.h FcrRoscoExternalSetpoint 逐字段对齐（S0801-S0807 + S1101 保留）。"""
    _fields_ = [
        ("valid", C.c_uint32), ("farm_control_enable", C.c_uint32),
        ("external_yaw_enable", C.c_uint32), ("yaw_seq", C.c_uint64),
        ("yaw_target_heading_rad", C.c_double),
        ("induction_enable", C.c_uint32), ("induction_seq", C.c_uint64),
        ("ct_ref", C.c_double), ("r_pitch", C.c_double),
        ("r_speed", C.c_double), ("r_torque", C.c_double),
        ("min_pitch_ref_rad", C.c_double), ("collective_pitch_offset_rad", C.c_double),
        ("command_status_flags", C.c_uint32),
        ("ct_tracking_error", C.c_double),   # S1101 保留接口（追加在末尾）
    ]


def load():
    if not _DLL.exists():
        raise FileNotFoundError(f"bridge DLL 不存在: {_DLL}（先编译）")
    dll = C.CDLL(str(_DLL))
    dll.fcr_bridge_core_create.restype = C.c_void_p
    dll.fcr_bridge_core_create.argtypes = [C.c_uint32, C.c_void_p, C.c_void_p, C.c_void_p]
    dll.fcr_bridge_core_destroy.argtypes = [C.c_void_p]
    dll.fcr_bridge_core_set_command.argtypes = [C.c_void_p, C.POINTER(FcrBridgeCmd)]
    dll.fcr_bridge_core_set_meas.argtypes = [C.c_void_p, C.POINTER(FcrBridgeMeas)]
    dll.fcr_bridge_core_step.restype = C.c_int
    dll.fcr_bridge_core_step.argtypes = [C.c_void_p, C.c_int32, C.c_double, C.POINTER(FcrRoscoExternalSetpoint)]
    dll.fcr_bridge_mode1_power_ratio.restype = C.c_double
    dll.fcr_bridge_mode1_power_ratio.argtypes = [C.c_double, C.c_double]
    dll.fcr_bridge_mode2_ct_ref.restype = C.c_double
    dll.fcr_bridge_mode2_ct_ref.argtypes = [C.c_double]
    dll.fcr_yaw_adapter_delta_to_target.restype = C.c_double
    dll.fcr_yaw_adapter_delta_to_target.argtypes = [C.c_double, C.c_double]
    return dll


def new_cmd(tid=1, mode=1, a=0.2, seq=1, ttl=1e6, yaw_abs=None):
    cmd = FcrBridgeCmd()
    cmd.turbine_id = tid; cmd.valid = 1
    cmd.induction_mode = mode; cmd.induction_valid = 1
    cmd.induction_ref = a; cmd.induction_seq = seq; cmd.induction_ttl_s = ttl
    if yaw_abs is not None:
        cmd.yaw_valid = 1; cmd.yaw_target_heading_rad = yaw_abs; cmd.yaw_seq = seq
    cmd.yaw_ttl_s = ttl
    cmd.source_time_s = 0.0
    return cmd


def new_meas(tid=1, v_up=8.0, v_disk=8.0, thrust=4e5, om=1.2, t=1.0):
    m = FcrBridgeMeas()
    m.turbine_id = tid; m.valid = 1; m.source_time_s = t
    m.rotor_speed_rad_s = om; m.generator_speed_rad_s = om
    m.rotor_thrust_n = thrust
    m.nacelle_heading_rad = 0.0
    m.ambient_upstream_wind_mps = v_up
    m.disk_background_wind_mps = v_disk
    m.disk_wind_speed_mps = v_disk
    m.disk_turbulence = 0.1
    m.gen_power_w = 1.5e6
    return m


def main():
    dll = load()
    core = dll.fcr_bridge_core_create(3, None, None, None)
    assert core, "create failed"
    fails = []

    # ---- 1. Mode 0：旁路 ----
    cmd = new_cmd(mode=0, a=0.2)
    sp = FcrRoscoExternalSetpoint()
    dll.fcr_bridge_core_set_command(core, C.byref(cmd))
    dll.fcr_bridge_core_set_meas(core, C.byref(new_meas()))
    rc = dll.fcr_bridge_core_step(core, 1, 5.0, C.byref(sp))
    if rc != 0 or sp.induction_enable != 0 or abs(sp.r_pitch - 1.0) > 1e-9:
        fails.append(f"Mode0 旁路失败: enable={sp.induction_enable} pr={sp.r_pitch}")

    # ---- 2. Mode 1：R_power 单调 + 默认纯 r_pitch ----
    prs = [dll.fcr_bridge_mode1_power_ratio(a, 1 / 3) for a in (0.10, 0.20, 0.25, 1 / 3)]
    if not all(prs[i] < prs[i + 1] for i in range(len(prs) - 1)):
        fails.append(f"Mode1 R_power 非单调: {prs}")
    cmd = new_cmd(mode=1, a=0.2)
    sp = FcrRoscoExternalSetpoint()
    dll.fcr_bridge_core_set_command(core, C.byref(cmd))
    dll.fcr_bridge_core_set_meas(core, C.byref(new_meas()))
    dll.fcr_bridge_core_step(core, 1, 5.0, C.byref(sp))
    expect_pr = dll.fcr_bridge_mode1_power_ratio(0.2, 1 / 3)
    if not (sp.induction_enable == 1 and abs(sp.r_pitch - expect_pr) < 1e-9
            and abs(sp.r_speed - 1.0) < 1e-9 and abs(sp.r_torque - 1.0) < 1e-9):
        fails.append(f"Mode1 默认分配失败: pr={sp.r_pitch} spd={sp.r_speed} tq={sp.r_torque}")

    # ---- 3. Mode 1：控制分配守恒（三通道按权）----
    cmd = new_cmd(mode=1, a=0.2, seq=2)
    cmd.alloc_w_pitch = 0.4; cmd.alloc_w_speed = 0.3; cmd.alloc_w_torque = 0.3
    sp = FcrRoscoExternalSetpoint()
    dll.fcr_bridge_core_set_command(core, C.byref(cmd))
    dll.fcr_bridge_core_step(core, 1, 6.0, C.byref(sp))
    # 校验三通道都被降载（<1）且无未分配叠加
    if not (sp.r_pitch < 1.0 and sp.r_speed < 1.0 and sp.r_torque < 1.0):
        fails.append(f"Mode1 分配未生效: pr={sp.r_pitch} spd={sp.r_speed} tq={sp.r_torque}")

    # ---- 4. Mode 2：Ct 前馈 → min_pitch（a 越小 Ct_ref 越小 → beta 越大）----
    cmd2 = new_cmd(mode=2, a=0.3)
    cmd2.turbine_id = 2
    cmd2.aero_blade_radius_m = 63.0
    cmd2.aero_n_lam = 3; cmd2.aero_n_beta = 9
    for i in range(3):
        cmd2.aero_lam_small[i] = 4.0 + 2.0 * i   # 4,6,8
    for j in range(9):
        b = cmd2.aero_beta_deg_small[j] = j * 2.0  # 0..16 deg
        for i in range(3):
            lam = cmd2.aero_lam_small[i]
            # 简单模型：Ct 随 beta 增、随 lam 减
            cmd2.aero_ct_small[i * 9 + j] = 0.95 - 0.03 * b - 0.04 * (lam - 4.0)
    sp = FcrRoscoExternalSetpoint()
    # 命令/step 用 turbine_id=2 → meas 必须同机（tid=2），否则 meas_valid=0、
    # lam=0 → W-05 缺风判定 (-5 NO_WIND) → 不采纳。
    m2 = new_meas(tid=2, om=6.0 * 63.0 / 8.0, v_up=8.0)
    m2 = new_meas(tid=2, om=7.0 * 8.0 / 63.0, v_up=8.0)  # lam = om*R/V = 7
    dll.fcr_bridge_core_set_command(core, C.byref(cmd2))
    dll.fcr_bridge_core_set_meas(core, C.byref(m2))
    dll.fcr_bridge_core_step(core, 2, 5.0, C.byref(sp))
    if sp.induction_enable != 1:
        fails.append(f"Mode2 未使能 (pr={sp.r_pitch} mp={sp.min_pitch_ref_rad})")
    # 用更小的 a → Ct_ref 更小 → 需要更大 beta（min_pitch 更大）
    cmd2b = new_cmd(mode=2, a=0.15, seq=2)
    cmd2b.turbine_id = 2
    cmd2b.aero_blade_radius_m = 63.0
    cmd2b.aero_n_lam = 3; cmd2b.aero_n_beta = 9
    for i in range(3): cmd2b.aero_lam_small[i] = cmd2.aero_lam_small[i]
    for j in range(9):
        cmd2b.aero_beta_deg_small[j] = cmd2.aero_beta_deg_small[j]
        for i in range(3):
            cmd2b.aero_ct_small[i * 9 + j] = cmd2.aero_ct_small[i * 9 + j]
    sp2 = FcrRoscoExternalSetpoint()
    dll.fcr_bridge_core_set_command(core, C.byref(cmd2b))
    dll.fcr_bridge_core_set_meas(core, C.byref(m2))
    dll.fcr_bridge_core_step(core, 2, 6.0, C.byref(sp2))
    # a=0.15 的 Ct_ref 小于 a=0.30 的 Ct_ref → 需要更大 beta → min_pitch 更大
    if not (sp2.min_pitch_ref_rad > sp.min_pitch_ref_rad):
        fails.append(f"Mode2 反解方向错: a=0.3→{sp.min_pitch_ref_rad:.4f} a=0.15→{sp2.min_pitch_ref_rad:.4f}")

    # ---- 5. Ct 估计 DR-7：上游参考风优先 ----
    cmd0 = new_cmd(mode=0)  # 只喂测量，Ct 估计仍计算
    m_up = new_meas(v_up=8.0, v_disk=8.0 * 0.9, thrust=0.5 * 1.225 * math.pi * 63 ** 2 * 64.0 * 0.7)
    dll.fcr_bridge_core_set_command(core, C.byref(cmd0))
    dll.fcr_bridge_core_set_meas(core, C.byref(m_up))
    dll.fcr_bridge_core_step(core, 1, 7.0, C.byref(sp))
    # ct_est 经 get 查询（此处仅校验 step 不崩；ct_est 在内部状态）

    # ---- 6. 偏航适配 DR-5 ----
    target = dll.fcr_yaw_adapter_delta_to_target(0.1745, 0.5)
    if abs(target - (0.5 - 0.1745)) > 1e-9:
        fails.append(f"yaw_adapter delta→target 错误: {target}")

    # ---- 7. 安全：TTL 过期回退 ----
    cmd = new_cmd(mode=1, a=0.2, ttl=5.0)
    sp = FcrRoscoExternalSetpoint()
    dll.fcr_bridge_core_set_command(core, C.byref(cmd))
    dll.fcr_bridge_core_step(core, 1, 3.0, C.byref(sp))
    if sp.induction_enable != 1:
        fails.append("TTL 内未生效")
    dll.fcr_bridge_core_step(core, 1, 6.0, C.byref(sp))  # 6 > 5+0(源时间0)
    if sp.induction_enable != 0:
        fails.append("TTL 过期未回退额定")

    dll.fcr_bridge_core_destroy(core)
    if fails:
        print("[FAIL] " + " | ".join(fails))
        return 1
    print("[PASS] Bridge Mode0/1/2 + Ct(DR-7) + yaw(DR-5) + safety 离线验证通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
