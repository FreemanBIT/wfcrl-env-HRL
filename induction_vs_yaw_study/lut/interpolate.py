"""
interpolate.py — LUT 多维插值查询
=================================
按 (wind_speed, wind_direction_offset, turbulence_intensity, spacing_D) 对某
control_type 查询三机控制设定。网格点处返回原值；非网格点用最近邻 +（可选）
线性插值。

设计为对"逐台风机的 yaw_deg / power_target_mw"做插值。
"""

from __future__ import annotations

from typing import Dict, List

import numpy as np
import pandas as pd

_KEYS = ["wind_speed_ms", "wind_direction_offset_deg",
         "turbulence_intensity", "spacing_D"]


def lookup(
    lut: pd.DataFrame,
    control_type: str,
    *,
    wind_speed_ms: float,
    wind_direction_offset_deg: float,
    turbulence_intensity: float,
    spacing_D: float,
    method: str = "nearest",
) -> Dict[int, Dict[str, float]]:
    """
    查询某工况某控制类型的三机设定。

    Returns
    -------
    Dict[turbine_id -> {yaw_deg, power_target_mw, ratio, min_pitch_deg, greedy_power_w}]

    method:
      * "nearest": 取最近网格点（鲁棒、默认）。
      * "linear" : 对每台机的标量做 4 维反距离加权（简化线性）。
    """
    sub = lut[lut["control_type"] == control_type].copy()
    if sub.empty:
        raise ValueError(f"No rows for control_type={control_type!r}")

    query = np.array([wind_speed_ms, wind_direction_offset_deg,
                      turbulence_intensity, spacing_D], dtype=float)

    # 归一化各维（避免量纲差异主导距离）
    scales = {}
    for k in _KEYS:
        vals = sub[k].astype(float).values
        rng = (vals.max() - vals.min()) or 1.0
        scales[k] = rng

    def _dist(row) -> float:
        d2 = 0.0
        for i, k in enumerate(_KEYS):
            d2 += ((float(row[k]) - query[i]) / scales[k]) ** 2
        return np.sqrt(d2)

    if method == "nearest":
        # 找到距离最近的 (case)：按 case_id 分组取整组
        sub["_d"] = sub.apply(_dist, axis=1)
        best_case = sub.loc[sub["_d"].idxmin(), "case_id"]
        rows = sub[sub["case_id"] == best_case]
        return _rows_to_dict(rows)

    elif method == "linear":
        # 反距离加权：对每台机的标量分别加权
        out: Dict[int, Dict[str, float]] = {}
        for tid, grp in sub.groupby("turbine_id"):
            d = grp.apply(_dist, axis=1).values
            w = 1.0 / np.maximum(d, 1e-9)
            w = w / w.sum()
            out[int(tid)] = {
                "yaw_deg": float(np.sum(w * grp["yaw_deg"].astype(float).values)),
                "power_target_mw": float(np.sum(w * grp["power_target_mw"].astype(float).values)),
                "ratio": float(np.sum(w * grp["ratio"].astype(float).values)),
                "min_pitch_deg": float(np.sum(w * grp["min_pitch_deg"].astype(float).fillna(0).values)),
                "greedy_power_w": float(np.sum(w * grp["greedy_power_w"].astype(float).values)),
            }
        return out
    else:
        raise ValueError(f"Unknown method {method!r}")


def _rows_to_dict(rows: pd.DataFrame) -> Dict[int, Dict[str, float]]:
    out: Dict[int, Dict[str, float]] = {}
    for _, r in rows.iterrows():
        out[int(r["turbine_id"])] = {
            "yaw_deg": float(r["yaw_deg"]) if not pd.isna(r["yaw_deg"]) else 0.0,
            "power_target_mw": float(r["power_target_mw"]) if not pd.isna(r["power_target_mw"]) else np.nan,
            "ratio": float(r["ratio"]) if not pd.isna(r["ratio"]) else 1.0,
            "min_pitch_deg": float(r["min_pitch_deg"]) if not pd.isna(r["min_pitch_deg"]) else 0.0,
            "greedy_power_w": float(r["greedy_power_w"]) if not pd.isna(r["greedy_power_w"]) else np.nan,
        }
    return out
