"""
floris_derating_opt.py — 降额(诱导)功率设定点 LUT 求解 (Stage 3B) ★核心★
========================================================================
对每个工况，搜索使三机总功率最大的每台机降额比组合（末机固定贪婪 1.0）。

关键：降额机制必须与 wfcrl/interface.py 的 FlorisInterface 完全一致，否则
优化结果无法忠实迁移到回放。FlorisInterface 的做法是：
    对 power_thrust_table 整体乘以 ratio，并用一维制动盘关系由缩放后的 Cp
    反解新 a，写回 Ct=4a(1-a)。
本模块复用 **同一套** _apply_curtailment 逻辑（从 interface 复制，保持一致），
作用在 FlorisModel 的 turbine_definitions 上。

决策变量：每台上游机的 ratio ∈ [ratio_min, 1.0]。3 机 → 2 个自由变量。
搜索：先粗网格扫描（鲁棒、可画响应面），再可选 scipy 细化。

输出：最优 ratio → 通过 conversion 层换算成绝对目标功率(MW)与诊断 a，写 LUT。
绝对功率 = ratio × 自由来流贪婪功率(W)（与 FAST.Farm 口径一致）。
"""

from __future__ import annotations

import copy
from itertools import product
from typing import Dict, List, Optional

import numpy as np

from induction_vs_yaw_study import constants as C
from induction_vs_yaw_study.cases.three_nrel5mw import Case
from induction_vs_yaw_study.optimize._floris_model import (
    build_model, greedy_farm_and_turbine_power, freestream_greedy_power_w,
)

from floris import FlorisModel


# ---- 与 wfcrl/interface.py FlorisInterface 一致的限功率逻辑 ----

def _a_from_ct(ct: float) -> float:
    ct = float(np.clip(ct, 0.0, 0.999))
    return 0.5 * (1.0 - np.sqrt(1.0 - ct))


def _solve_new_a(a_old: float, ratio: float) -> float:
    cp_old = 4.0 * a_old * (1.0 - a_old) ** 2
    if cp_old <= 0.0:
        return 0.0
    cp_new = float(ratio) * cp_old
    # 在 [0, 1/3] 上二分（替代 fsolve，更稳）
    lo, hi = 0.0, 1.0 / 3.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if 4.0 * mid * (1.0 - mid) ** 2 < cp_new:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _apply_curtailment(base_table: dict, ratio: float) -> dict:
    """对 power_thrust_table 施加限功率比（与 interface 一致）。"""
    new_table = copy.deepcopy(base_table)
    powers = np.array(new_table["power"], dtype=np.float64)
    cts = np.array(new_table["thrust_coefficient"], dtype=np.float64)
    new_table["power"] = (powers * ratio).tolist()
    new_cts = np.zeros_like(cts)
    for i, ct in enumerate(cts):
        if ct > 0.0:
            a_old = _a_from_ct(float(ct))
            a_new = _solve_new_a(a_old, ratio)
            new_cts[i] = 4.0 * a_new * (1.0 - a_new)
    new_table["thrust_coefficient"] = new_cts.tolist()
    return new_table


def _build_curtailed_model(case: Case, ratios: np.ndarray):
    """
    构建对每台机施加各自 ratio 的 FlorisModel。

    Returns (FlorisModel, lib_dir)。调用方负责在用完后 shutil.rmtree(lib_dir)。
    """
    fmodel = build_model(case)
    base_td = copy.deepcopy(fmodel.core.farm.turbine_definitions[0])
    base_table = base_td["power_thrust_table"]
    base_name = base_td.get("turbine_type", "nrel_5MW")

    # 采用与 wfcrl/interface.py FlorisInterface._apply_turbine_curtailment 完全一致
    # 的方式：为每台机写一个临时 turbine yaml 到 turbine_library_path，再用 dict
    # 重建 FlorisModel。这是工程里已验证可用的路径，避免 FLORIS 版本对 dict-list
    # turbine_type 支持差异。
    import os
    import yaml as _yaml
    import tempfile

    lib_dir = tempfile.mkdtemp(prefix="floris_cr_")
    new_types = []
    for i in range(3):
        r = float(np.clip(ratios[i], 0.01, 1.0))
        tname = f"{base_name}_cr{i}"
        td_i = copy.deepcopy(base_td)
        td_i["power_thrust_table"] = _apply_curtailment(base_table, r)
        td_i["turbine_type"] = tname
        # 移除不可序列化字段（pathlib.Path 等）
        for k in list(td_i.keys()):
            v = td_i[k]
            if not isinstance(v, (str, int, float, bool, list, dict, type(None))):
                del td_i[k]
        with open(os.path.join(lib_dir, f"{tname}.yaml"), "w") as f:
            _yaml.dump(td_i, f)
        new_types.append(tname)

    # 取出基准配置 dict 并改 farm 段（与 interface 同构）
    cfg = fmodel.core.as_dict() if hasattr(fmodel.core, "as_dict") else None
    xs = list(fmodel.core.farm.layout_x)
    ys = list(fmodel.core.farm.layout_y)

    # 用一个干净的 FlorisModel 重建（defaults 基底 + 覆盖）
    new_fmodel = build_model(case)
    new_cfg = (new_fmodel.core.as_dict()
               if hasattr(new_fmodel.core, "as_dict") else None)
    if new_cfg is not None:
        new_cfg["farm"]["layout_x"] = xs
        new_cfg["farm"]["layout_y"] = ys
        new_cfg["farm"]["turbine_type"] = new_types
        new_cfg["farm"]["turbine_library_path"] = lib_dir
        new_cfg["flow_field"]["wind_speeds"] = [case.wind_speed_ms]
        new_cfg["flow_field"]["wind_directions"] = [case.wind_direction_deg]
        new_cfg["flow_field"]["turbulence_intensities"] = [case.turbulence_intensity]
        out = FlorisModel(new_cfg)
        out.set(yaw_angles=np.zeros((1, 3)))
        return out, lib_dir

    # 回退：直接 set turbine_type 名 + library path
    new_fmodel.set(
        layout_x=xs, layout_y=ys,
        turbine_type=new_types,
        turbine_library_path=lib_dir,
        wind_speeds=[case.wind_speed_ms],
        wind_directions=[case.wind_direction_deg],
        turbulence_intensities=[case.turbulence_intensity],
        yaw_angles=np.zeros((1, 3)),
    )
    return new_fmodel, lib_dir


def _farm_power_for_ratios(case: Case, ratios: np.ndarray) -> float:
    import shutil
    fmodel, lib_dir = _build_curtailed_model(case, ratios)
    try:
        fmodel.run()
        return float(fmodel.get_turbine_powers().sum())
    finally:
        shutil.rmtree(lib_dir, ignore_errors=True)


def optimize_derating(
    case: Case,
    *,
    ratio_min: float = 0.4,
    ratio_grid_step: float = 0.05,
    refine: bool = True,
    fix_last_turbine_greedy: bool = True,
) -> Dict:
    """
    求该工况最优降额比组合。

    Returns dict:
      ratios: List[float] 长度 3
      power_target_mw: List[float]  (= ratio × 自由来流贪婪功率)
      a_diag: List[float]
      greedy_power_w: float (自由来流单机贪婪功率)
      farm_power_opt_mw, farm_power_baseline_mw, gain_vs_baseline_pct
    """
    baseline_farm_w, _ = greedy_farm_and_turbine_power(case)
    p_greedy_w = freestream_greedy_power_w(case)

    n_free = 2 if fix_last_turbine_greedy else 3
    grid = np.arange(ratio_min, 1.0 + 1e-9, ratio_grid_step)

    # --- 粗网格扫描 ---
    best_val = -1.0
    best_ratios = np.ones(3)
    for combo in product(grid, repeat=n_free):
        ratios = np.ones(3)
        ratios[:n_free] = combo
        val = _farm_power_for_ratios(case, ratios)
        if val > best_val:
            best_val = val
            best_ratios = ratios.copy()

    # --- scipy 细化（可选）---
    if refine:
        try:
            from scipy.optimize import minimize

            def neg(x):
                ratios = np.ones(3)
                ratios[:n_free] = np.clip(x, ratio_min, 1.0)
                return -_farm_power_for_ratios(case, ratios)

            res = minimize(neg, best_ratios[:n_free], method="Nelder-Mead",
                           options={"maxiter": 60, "xatol": 0.01, "fatol": 1e3})
            cand = np.ones(3)
            cand[:n_free] = np.clip(res.x, ratio_min, 1.0)
            cand_val = _farm_power_for_ratios(case, cand)
            if cand_val > best_val:
                best_val = cand_val
                best_ratios = cand
        except Exception:
            pass

    # --- 换算成绝对功率与诊断 a（conversion 层口径）---
    from induction_vs_yaw_study.conversion.power_setpoint_tools import (
        ratio_to_power_mw, induction_from_power,
    )
    power_mw = [ratio_to_power_mw(best_ratios[i], case.wind_speed_ms,
                                  greedy_power_w_value=p_greedy_w)
                for i in range(3)]
    a_diag = [induction_from_power(power_mw[i] * 1e6, case.wind_speed_ms)
              for i in range(3)]

    gain = (100.0 * (best_val - baseline_farm_w) / baseline_farm_w
            if baseline_farm_w > 0 else 0.0)

    return {
        "ratios": [float(r) for r in best_ratios],
        "power_target_mw": [float(p) for p in power_mw],
        "a_diag": [float(a) for a in a_diag],
        "greedy_power_w": float(p_greedy_w),
        "farm_power_opt_mw": best_val / 1e6,
        "farm_power_baseline_mw": baseline_farm_w / 1e6,
        "gain_vs_baseline_pct": gain,
    }
