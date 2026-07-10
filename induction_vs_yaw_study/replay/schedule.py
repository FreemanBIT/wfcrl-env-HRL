"""
schedule.py — 控制输入单一来源 (Stage 4)
========================================
把 LUT 中某工况某 control_type 的三机设定，转成指定仿真器的 ControlInput。
**所有回放都从这里取控制输入**，确保 FLORIS 与 FAST.Farm 完全同源。
"""

from __future__ import annotations

from typing import Dict, List

import numpy as np

from induction_vs_yaw_study.conversion.power_setpoint_tools import (
    build_yaw_control, build_derating_control,
)

from wfcrl.config import ControlInput


def build_control_input(
    control_type: str,
    sim_kind: str,
    settings: Dict[int, Dict[str, float]],
    *,
    wind_speed_ms: float,
    n_turbines: int = 3,
) -> ControlInput:
    """
    构造单步控制输入（静态 LUT → 每步相同）。

    Parameters
    ----------
    control_type : {"yaw", "derating", "baseline"}
    sim_kind : {"floris", "fastfarm"}
    settings : Dict[turbine_id -> {yaw_deg, power_target_mw, ratio,
                                   min_pitch_deg, greedy_power_w}]
        来自 lut.lookup() 或直接由优化结果构造。
    wind_speed_ms : 用于 FLORIS 的 ratio 换算。
    """
    ids = sorted(settings.keys())[:n_turbines]

    if control_type == "baseline":
        return build_yaw_control([0.0] * n_turbines)

    if control_type == "yaw":
        yaw = [settings[i]["yaw_deg"] for i in ids]
        return build_yaw_control(yaw)

    if control_type == "derating":
        power_mw = [settings[i]["power_target_mw"] for i in ids]
        min_pitch = [settings[i].get("min_pitch_deg", 0.0) for i in ids]
        greedy = [settings[i].get("greedy_power_w") for i in ids]
        greedy = None if any(g is None or np.isnan(g) for g in greedy) else greedy
        return build_derating_control(
            sim_kind,
            power_target_mw=power_mw,
            wind_speed_ms=wind_speed_ms,
            min_pitch_deg=min_pitch,
            greedy_power_w_per_turbine=greedy,
        )

    raise ValueError(f"Unknown control_type {control_type!r}")


def control_inputs_for_recording(
    control_type: str,
    settings: Dict[int, Dict[str, float]],
    *,
    n_turbines: int = 3,
):
    """
    从 settings 提取**用于记录到 timeseries** 的每台风机控制输入。

    返回 (yaw_cmd_deg, ratio, power_target_mw)，均为长度 n_turbines 的 list；
    缺失项用合理默认（baseline：yaw=0, ratio=1.0, power_target=NaN）。

    说明：
      * yaw_cmd_deg —— 偏航控制指令（相对来流失准角，deg）。
      * ratio       —— 限电比例 = 目标功率/贪婪功率（1.0 表示不限电）。
      * power_target_mw —— 目标功率（MW）；偏航/基准工况无意义，记为 NaN。
    """
    import numpy as _np
    ids = sorted(settings.keys())[:n_turbines]

    if control_type == "baseline":
        yaw = [0.0] * n_turbines
        ratio = [1.0] * n_turbines
        ptgt = [float("nan")] * n_turbines
        return yaw, ratio, ptgt

    if control_type == "yaw":
        yaw = [float(settings[i].get("yaw_deg", 0.0)) for i in ids]
        # 纯偏航不限电
        ratio = [float(settings[i].get("ratio", 1.0)) for i in ids]
        ptgt = [float(settings[i].get("power_target_mw", float("nan"))) for i in ids]
        return yaw, ratio, ptgt

    if control_type == "derating":
        yaw = [float(settings[i].get("yaw_deg", 0.0)) for i in ids]
        ratio = []
        ptgt = []
        for i in ids:
            p = settings[i].get("power_target_mw", float("nan"))
            g = settings[i].get("greedy_power_w", None)
            r = settings[i].get("ratio", None)
            if r is None or (isinstance(r, float) and _np.isnan(r)):
                # 由 目标功率/贪婪功率 反算 ratio（贪婪功率单位 W）
                if g is not None and not (isinstance(g, float) and _np.isnan(g)) and g > 0:
                    r = float(p) * 1e6 / float(g)
                else:
                    r = float("nan")
            ratio.append(float(r))
            ptgt.append(float(p))
        return yaw, ratio, ptgt

    raise ValueError(f"Unknown control_type {control_type!r}")
