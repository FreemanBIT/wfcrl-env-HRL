"""
fastfarm_replay.py — FAST.Farm 回放 (Stage 6)
=============================================
用工程的 ContinuousFastFarmInterface（一次启动、流场连续）按 schedule 回放。
静态 LUT：每个控制步下发相同命令；前 t_settle 让尾流稳定，后段取稳态窗口。

v2 — 使用 stop() 读取 .outb 文件，避免 DISCON 轮询的竞态条件。
"""

from __future__ import annotations

from typing import Dict

import numpy as np

from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_fastfarm_config
from induction_vs_yaw_study.replay.schedule import (
    build_control_input, control_inputs_for_recording,
)

from wfcrl.config import SimulationOutput

# (REFACTOR_PLAN 附录 A) 旧文件桥驱动已移除，本入口代理到 ZMQ 驱动。
from induction_vs_yaw_study.replay.fastfarm_zmq_replay import replay_fastfarm_zmq as _zmq_driver


def _attach_control_inputs(out, yaw_cmd, ratio, ptgt):
    """把每台风机的控制输入广播到所有时间步，写入 SimulationOutput。"""
    if out is None or out.time is None:
        return out
    n_steps = len(out.time)
    n_t = len(yaw_cmd)
    out.yaw_cmd_deg = np.tile(np.asarray(yaw_cmd, dtype=float), (n_steps, 1))
    out.ratio = np.tile(np.asarray(ratio, dtype=float), (n_steps, 1))
    out.power_target_mw = np.tile(np.asarray(ptgt, dtype=float), (n_steps, 1))
    return out


def _downsample_from_outb(
    full: SimulationOutput,
    dt: float,
    n_steps: int,
) -> SimulationOutput:
    """
    从 stop() 返回的完整 .outb 时序中，按控制步间隔 dt 降采样。

    .outb 包含高密度时序（每 DT_low 一个点），我们只取 n_steps 个控制步的数据。
    提取每个控制周期结束时（t = dt, 2*dt, ..., n_steps*dt）的仿真状态。
    时间标签沿用旧的 0, dt, 2*dt, ... 习惯。
    """
    t = full.time
    if t is None or len(t) == 0:
        return full

    # 目标仿真时刻：第 1 个控制步结束在 dt，第 k 个在 k*dt
    target_sim_t = np.arange(1, n_steps + 1) * dt

    # 在 .outb 时间轴中找最接近每个目标时刻的索引
    idx = np.array([np.argmin(np.abs(t - tt)) for tt in target_sim_t])
    idx = np.clip(idx, 0, len(t) - 1)

    def _take(arr):
        if arr is None:
            return None
        return arr[idx]

    return SimulationOutput(
        time=target_sim_t,                     # 真实仿真时刻 dt,2dt,...（诚实标签）
        power_mw=_take(full.power_mw),
        wind_speed=_take(full.wind_speed),
        wind_direction=_take(full.wind_direction),
        yaw_deg=_take(full.yaw_deg),
        pitch_deg=_take(full.pitch_deg),
        torque_nm=_take(full.torque_nm),
        rotor_speed_rpm=_take(full.rotor_speed_rpm),
        blade_loads=_take(full.blade_loads),
        metadata=full.metadata,
    )

def replay_fastfarm(
    case, control_type, settings, *, output_dir=None,
    collect_stepwise=True, wind_time_series=None, yaw_mode="static",
    cmd_port=24040, state_port=24041,
):
    """ZMQ 版回放驱动（替代旧文件桥实现，见 fastfarm_zmq_replay.py）。

    参数与旧签名兼容；新增 wind_time_series / yaw_mode。
    返回 SimulationOutput（time/power_mw/... + yaw_cmd/ratio/power_target）。
    """
    return _zmq_driver(
        case, control_type, settings, output_dir=output_dir,
        wind_time_series=wind_time_series, yaw_mode=yaw_mode,
        cmd_port=cmd_port, state_port=state_port,
    )
