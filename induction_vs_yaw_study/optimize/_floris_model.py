"""
_floris_model.py — 构建用于优化的 FlorisModel
=============================================
直接用 FLORIS Python API 构建模型（绕过逐步 step 接口，优化更快），但布局/
风况/湍流/机型与 cases.make_floris_config 完全一致，确保优化与回放同源。

间距通过 layout_x 注入（不改模板）。机型用 FLORIS 内置 nrel_5MW。
"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np

from induction_vs_yaw_study import constants as C
from induction_vs_yaw_study.cases.three_nrel5mw import Case, layout_for_spacing

from floris import FlorisModel


def build_model(case: Case) -> FlorisModel:
    """
    构建与该 case 对应的 FlorisModel（单工况，3 机单列）。

    使用 FLORIS 自带 GCH 默认输入作为基底，再覆盖 layout/wind/TI/turbine。
    """
    xs, ys = layout_for_spacing(case.spacing_D, n_turbines=3)

    # 用 FLORIS 内置默认配置文件名（v4 提供 defaults）；若不可用，
    # coding agent 可改为指向 wfcrl/simulators/floris/inputs/template/case.yaml。
    try:
        fmodel = FlorisModel("defaults")
    except Exception:
        from pathlib import Path
        tmpl = (Path(__file__).resolve().parents[2] / "wfcrl" / "simulators" /
                "floris" / "inputs" / "template" / "case.yaml")
        fmodel = FlorisModel(str(tmpl))

    fmodel.set(
        layout_x=xs,
        layout_y=ys,
        turbine_type=["nrel_5MW"],
        wind_speeds=[case.wind_speed_ms],
        wind_directions=[case.wind_direction_deg],
        turbulence_intensities=[case.turbulence_intensity],
    )
    return fmodel


def greedy_farm_and_turbine_power(case: Case) -> Tuple[float, np.ndarray]:
    """
    基准（贪婪/无降额、零偏航）下的全场功率与各机功率 (W)。

    用于：
      * baseline 对照；
      * 标定 derating 的 ratio（自由来流贪婪功率）。
    """
    fmodel = build_model(case)
    fmodel.set(yaw_angles=np.zeros((1, 3)))
    fmodel.run()
    turbine_powers = fmodel.get_turbine_powers().flatten()  # W
    farm_power = float(turbine_powers.sum())
    return farm_power, turbine_powers


def freestream_greedy_power_w(case: Case) -> float:
    """
    单机自由来流贪婪功率 (W)，作为 ratio 的统一分母。

    用一台孤立风机在该风速/TI 下的功率（无尾流），使 ratio 的物理含义为
    "相对该机自身满发能力的降额比"，与 FAST.Farm 端绝对功率口径一致。
    """
    fmodel = build_model(case)
    fmodel.set(layout_x=[0.0], layout_y=[0.0],
               turbine_type=["nrel_5MW"],
               wind_speeds=[case.wind_speed_ms],
               wind_directions=[case.wind_direction_deg],
               turbulence_intensities=[case.turbulence_intensity],
               yaw_angles=np.zeros((1, 1)))
    fmodel.run()
    return float(fmodel.get_turbine_powers().flatten()[0])
