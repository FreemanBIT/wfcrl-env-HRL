"""
replay_steady.py — 稳态风回放驱动
==================================
对某稳态工况、某控制类型，在 FAST.Farm（稳态风）或 FLORIS 中回放 LUT。
逻辑与主工程 replay/fastfarm_replay.py、floris_replay.py 一致，仅：
  * 使用 cases_steady 的稳态配置构建（FAST.Farm 走 WindType=1，无 .bts）；
  * 控制输入仍由主工程 replay/schedule.build_control_input 生成（同源）。
"""

from __future__ import annotations

from typing import Dict

import numpy as np

from induction_vs_yaw_study.steady.cases_steady import (
    SteadyCase, make_fastfarm_config_steady, make_floris_config_steady,
)
from induction_vs_yaw_study.replay.schedule import (
    build_control_input, control_inputs_for_recording,
)

from wfcrl.interface import ContinuousFastFarmInterface, FastFarmAborted
from wfcrl.config import SimulationOutput


# -------------------------------------------------------------------------
# FAST.Farm 稳态回放
# -------------------------------------------------------------------------

def replay_fastfarm_steady(
    case: SteadyCase,
    control_type: str,
    settings: Dict[int, Dict[str, float]],
    *,
    output_dir: str = None,
    collect_stepwise: bool = True,
):
    """
    稳态风下回放：setup → reset → set_fixed_yaw → start → wait_step×N → stop（读 .outb）。
    偏航经锁定 DOF 实现（见 ContinuousFastFarmInterface.set_fixed_yaw）；
    FAST.Farm 中途 abort 时保存部分结果并快速返回。
    """
    cfg = make_fastfarm_config_steady(case, output_dir=output_dir)
    ff = ContinuousFastFarmInterface(cfg)
    ff.setup()
    ff.reset(cfg.wind)

    ci = build_control_input(
        control_type, "fastfarm", settings,
        wind_speed_ms=case.wind_speed_ms, n_turbines=3,
    )
    yaw_cmd, ratio, ptgt = control_inputs_for_recording(
        control_type, settings, n_turbines=3,
    )

    # 偏航：写入 ED + 锁定偏航 DOF（start 之前）
    ff.set_fixed_yaw(yaw_cmd)
    ff.start()

    n_total = cfg.max_iter
    final = None
    aborted = False
    try:
        for _ in range(n_total):
            ff.wait_step(ci)
    except FastFarmAborted as e:
        aborted = True
        print(f"    [warn] FAST.Farm aborted mid-run: {e}")
    finally:
        try:
            final = ff.stop(allow_partial=True)
        except Exception as e:  # noqa
            print(f"    [warn] stop() failed: {e}")
            final = None
        ff.close()

    if final is None:
        return None

    if collect_stepwise:
        out = _downsample_from_outb(final, cfg.dt, n_total)
    else:
        out = final
    out = _attach_control_inputs(out, yaw_cmd, ratio, ptgt)
    if out is not None and aborted:
        md = dict(out.metadata or {})
        md["aborted"] = True
        out.metadata = md
    return out


def _attach_control_inputs(out, yaw_cmd, ratio, ptgt):
    """把每台风机的控制输入广播到所有时间步，写入 SimulationOutput。"""
    if out is None or out.time is None:
        return out
    n_steps = len(out.time)
    out.yaw_cmd_deg = np.tile(np.asarray(yaw_cmd, dtype=float), (n_steps, 1))
    out.ratio = np.tile(np.asarray(ratio, dtype=float), (n_steps, 1))
    out.power_target_mw = np.tile(np.asarray(ptgt, dtype=float), (n_steps, 1))
    return out


def _downsample_from_outb(full: SimulationOutput, dt: float, n_steps: int) -> SimulationOutput:
    """从 .outb 完整时序按控制步降采样（与主工程逻辑一致）。"""
    t = full.time
    if t is None or len(t) == 0:
        return full
    target_sim_t = np.arange(1, n_steps + 1) * dt
    idx = np.array([np.argmin(np.abs(t - tt)) for tt in target_sim_t])
    idx = np.clip(idx, 0, len(t) - 1)

    def _take(arr):
        return None if arr is None else arr[idx]

    return SimulationOutput(
        time=target_sim_t,  # 真实仿真时刻（诚实标签）
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


# -------------------------------------------------------------------------
# FLORIS 稳态回放
# -------------------------------------------------------------------------

def replay_floris_steady(
    case: SteadyCase,
    control_type: str,
    settings: Dict[int, Dict[str, float]],
    *,
    output_dir: str = None,
):
    """
    FLORIS 稳态回放：跑少数稳态步、下发同源控制，合并输出。
    与主工程 replay_floris 完全一致的步进/合并约定，只是换用稳态配置。
    """
    from wfcrl.interface import FlorisInterface, SimulatorInterface

    cfg = make_floris_config_steady(case, output_dir=output_dir)
    fl = FlorisInterface(cfg)
    fl.setup()
    fl.reset(cfg.wind)

    ci = build_control_input(
        control_type, "floris", settings,
        wind_speed_ms=case.wind_speed_ms, n_turbines=3,
    )
    yaw_cmd, ratio, ptgt = control_inputs_for_recording(
        control_type, settings, n_turbines=3,
    )

    outputs = []
    for _ in range(cfg.max_iter):
        outputs.append(fl.step(ci))
    try:
        fl.close()
    except Exception:
        pass

    out = SimulatorInterface._merge_outputs(outputs)
    return _attach_control_inputs(out, yaw_cmd, ratio, ptgt)
