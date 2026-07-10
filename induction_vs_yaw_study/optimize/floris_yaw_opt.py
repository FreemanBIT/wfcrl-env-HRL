"""
floris_yaw_opt.py — 偏航 LUT 求解 (Stage 3A)
============================================
对每个工况，最大化三机总功率的最优偏航组合。

优先用 FLORIS 自带的串行精炼优化器 YawOptimizationSR；不可用时回退到
scipy.optimize（SLSQP）。末机偏航可固定 0 降维（grid.yaml: fix_last_turbine_zero）。
"""

from __future__ import annotations

from typing import Dict, List

import numpy as np

from induction_vs_yaw_study import constants as C
from induction_vs_yaw_study.cases.three_nrel5mw import Case
from induction_vs_yaw_study.optimize._floris_model import (
    build_model, greedy_farm_and_turbine_power,
)


def optimize_yaw(
    case: Case,
    *,
    yaw_min_deg: float = -30.0,
    yaw_max_deg: float = 30.0,
    fix_last_turbine_zero: bool = True,
) -> Dict:
    """
    求该工况最优偏航。

    Returns dict:
      yaw_deg: List[float] 长度 3
      farm_power_opt_mw, farm_power_baseline_mw, gain_vs_baseline_pct
    """
    fmodel = build_model(case)
    n = 3

    baseline_farm_w, _ = greedy_farm_and_turbine_power(case)

    yaw_opt = None
    # --- 路径 1：FLORIS 内置 SR 优化器 ---
    try:
        from floris.optimization.yaw_optimization.yaw_optimizer_sr import (
            YawOptimizationSR,
        )
        # 限制末机偏航范围
        minimum = np.array([[yaw_min_deg, yaw_min_deg,
                             0.0 if fix_last_turbine_zero else yaw_min_deg]])
        maximum = np.array([[yaw_max_deg, yaw_max_deg,
                             0.0 if fix_last_turbine_zero else yaw_max_deg]])
        yaw_opt_obj = YawOptimizationSR(
            fmodel,
            minimum_yaw_angle=minimum,
            maximum_yaw_angle=maximum,
            Ny_passes=[5, 4],
            exclude_downstream_turbines=True,
        )
        df_opt = yaw_opt_obj.optimize()
        yaw_opt = np.asarray(df_opt["yaw_angles_opt"].iloc[0], dtype=float).ravel()
    except Exception:
        yaw_opt = None

    # --- 路径 2：scipy 回退 ---
    if yaw_opt is None:
        from scipy.optimize import minimize

        n_free = 2 if fix_last_turbine_zero else 3

        def neg_power(x):
            yaw = np.zeros((1, n))
            yaw[0, :n_free] = x
            fmodel.set(yaw_angles=yaw)
            fmodel.run()
            return -float(fmodel.get_turbine_powers().sum())

        x0 = np.full(n_free, 10.0)
        bounds = [(yaw_min_deg, yaw_max_deg)] * n_free
        res = minimize(neg_power, x0, method="SLSQP", bounds=bounds,
                       options={"maxiter": 50, "ftol": 1e-3})
        yaw_opt = np.zeros(n)
        yaw_opt[:n_free] = res.x

    # --- 评估最优 ---
    yaw_arr = np.zeros((1, n))
    yaw_arr[0, :] = yaw_opt
    fmodel.set(yaw_angles=yaw_arr)
    fmodel.run()
    farm_opt_w = float(fmodel.get_turbine_powers().sum())

    gain = 100.0 * (farm_opt_w - baseline_farm_w) / baseline_farm_w if baseline_farm_w > 0 else 0.0

    return {
        "yaw_deg": [float(v) for v in yaw_opt],
        "farm_power_opt_mw": farm_opt_w / 1e6,
        "farm_power_baseline_mw": baseline_farm_w / 1e6,
        "gain_vs_baseline_pct": gain,
    }
