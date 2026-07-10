"""
schema.py — LUT 数据结构与读写
==============================
两套 LUT（yaw / derating）合并存在一张表，用 control_type 区分。
每行 = 一个工况 × 一种控制 × 一台风机。

标准存储量：
  * yaw_deg          : 该机偏航角 (deg)
  * power_target_mw  : 该机绝对目标功率 (MW) —— derating 的标准量
  * ratio            : 相对自由来流贪婪功率的限功率比（FLORIS 下发用）
  * a_diag           : 诊断轴向诱导因子（由 power+U 反算）
存绝对功率(MW)是关键：FAST.Farm 直接用，FLORIS 用 ratio（见 conversion 层）。
"""

from __future__ import annotations

import os
from typing import List

import pandas as pd

LUT_COLUMNS: List[str] = [
    # 工况键
    "case_id",
    "wind_speed_ms",
    "wind_direction_offset_deg",
    "turbulence_intensity",
    "spacing_D",
    "control_type",        # "yaw" | "derating" | "baseline"
    "turbine_id",          # 1..N
    # 控制量
    "yaw_deg",
    "power_target_mw",
    "ratio",
    "min_pitch_deg",
    "a_diag",
    # 优化结果（每工况重复，便于分析）
    "farm_power_opt_mw",       # FLORIS 优化得到的全场功率
    "farm_power_baseline_mw",  # 基准全场功率
    "gain_vs_baseline_pct",
    # 该机贪婪(自由来流)功率，供回放 ratio 标定
    "greedy_power_w",
]


def empty_lut() -> pd.DataFrame:
    return pd.DataFrame(columns=LUT_COLUMNS)


def append_row(df: pd.DataFrame, **kwargs) -> pd.DataFrame:
    """追加一行（缺失列填 NaN）。返回新 DataFrame。"""
    row = {c: kwargs.get(c) for c in LUT_COLUMNS}
    return pd.concat([df, pd.DataFrame([row])], ignore_index=True)


def save_lut(df: pd.DataFrame, path: str) -> None:
    """保存为 parquet（带 csv 备份）。"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    try:
        df.to_parquet(path, index=False)
    except Exception:
        # 无 pyarrow 时退回 csv
        path = os.path.splitext(path)[0] + ".csv"
    df.to_csv(os.path.splitext(path)[0] + ".csv", index=False)


def load_lut(path: str) -> pd.DataFrame:
    if path.endswith(".parquet") and os.path.exists(path):
        try:
            return pd.read_parquet(path)
        except Exception:
            pass
    csv = os.path.splitext(path)[0] + ".csv"
    return pd.read_csv(csv)
