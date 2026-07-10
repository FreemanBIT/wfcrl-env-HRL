"""
floris_replay.py — FLORIS 回放 (Stage 5)
========================================
用工程的 FlorisInterface 按 schedule 逐步 step，收集 SimulationOutput。
静态 LUT：每步控制相同，跑少量步取稳态即可。
"""

from __future__ import annotations

from typing import Dict, List

import numpy as np

from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_floris_config
from induction_vs_yaw_study.replay.schedule import (
    build_control_input, control_inputs_for_recording,
)

from wfcrl.interface import FlorisInterface, SimulatorInterface


def replay_floris(
    case: Case,
    control_type: str,
    settings: Dict[int, Dict[str, float]],
    *,
    output_dir: str = None,
):
    """
    在 FLORIS 中回放某工况某控制类型。

    Returns SimulationOutput（合并所有步），并附带每台风机的控制输入
    (yaw_cmd_deg / ratio / power_target_mw) 列。
    """
    cfg = make_floris_config(case, output_dir=output_dir)
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
    fl.close()

    out = SimulatorInterface._merge_outputs(outputs)
    # 把控制输入广播到所有时间步
    if out is not None and out.time is not None:
        n_steps = len(out.time)
        out.yaw_cmd_deg = np.tile(np.asarray(yaw_cmd, dtype=float), (n_steps, 1))
        out.ratio = np.tile(np.asarray(ratio, dtype=float), (n_steps, 1))
        out.power_target_mw = np.tile(np.asarray(ptgt, dtype=float), (n_steps, 1))
    return out
