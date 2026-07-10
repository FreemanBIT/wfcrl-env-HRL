"""
metrics.py — 指标计算 (Stage 7)
===============================
从回放时序中提取稳态指标，构建对比 summary 表。
"""

from __future__ import annotations

import glob
import os
from typing import Dict, List, Optional

import numpy as np
import pandas as pd


def steady_window_mean(values: np.ndarray, frac: float = 0.5) -> float:
    """取后 frac 段的均值（默认后 50% 作为稳态窗口）。"""
    v = np.asarray(values, dtype=float).ravel()
    v = v[~np.isnan(v)]
    if v.size == 0:
        return np.nan
    k = max(1, int(v.size * (1 - frac)))
    return float(np.mean(v[k:]))


def _load_timeseries_csv(path: str) -> pd.DataFrame:
    return pd.read_csv(path)


def summarize_timeseries(df: pd.DataFrame, frac: float = 0.5) -> Dict[str, float]:
    """
    从单条时序 DataFrame（SimulationOutput.to_dataframe 格式）提取稳态指标。

    期望列：farm_power_MW, T{i}_power_MW, 以及可选 T{i}_blade_loads* / pitch。
    """
    out: Dict[str, float] = {}
    if "farm_power_MW" in df.columns:
        out["farm_power_mw"] = steady_window_mean(df["farm_power_MW"].values, frac)
    # 各机功率
    pcols = sorted([c for c in df.columns if c.endswith("_power_MW") and c != "farm_power_MW"])
    for c in pcols:
        out[c.replace("_power_MW", "_power_mw").lower()] = steady_window_mean(df[c].values, frac)
    # 载荷代理：叶根弯矩标准差（若存在 blade load 列）
    bcols = [c for c in df.columns if "blade" in c.lower() or "RootM" in c]
    if bcols:
        # 用首通道的 std 作为载荷波动代理（DEL 的简化）
        loads = df[bcols].values.astype(float)
        out["blade_load_std"] = float(np.nanstd(loads))
    return out


def build_summary(
    timeseries_dir: str,
    *,
    frac: float = 0.5,
) -> pd.DataFrame:
    """
    扫描 timeseries 目录，构建 summary 表。

    文件命名约定（见 run/run_replay.py）：
        {sim}_{control}_{case_id}.csv
        sim ∈ {floris, fastfarm}; control ∈ {yaw, derating, baseline}

    Returns 每行 = (case_id, sim, control) 的稳态指标 + baseline 增益。
    """
    rows: List[Dict] = []
    files = sorted(glob.glob(os.path.join(timeseries_dir, "*.csv")))
    for f in files:
        base = os.path.splitext(os.path.basename(f))[0]
        parts = base.split("_")
        if len(parts) < 3:
            continue
        sim = parts[0]
        control = parts[1]
        case_id = "_".join(parts[2:])
        try:
            df = _load_timeseries_csv(f)
        except Exception:
            continue
        metrics = summarize_timeseries(df, frac)
        rows.append({"case_id": case_id, "sim": sim, "control": control, **metrics})

    summary = pd.DataFrame(rows)
    if summary.empty:
        return summary

    # 计算相对 baseline 的功率增益（同 sim 内）
    summary["gain_vs_baseline_pct"] = np.nan
    for (case_id, sim), grp in summary.groupby(["case_id", "sim"]):
        base = grp[grp["control"] == "baseline"]
        if base.empty or "farm_power_mw" not in base.columns:
            continue
        p_base = base["farm_power_mw"].iloc[0]
        if not p_base or np.isnan(p_base):
            continue
        for idx in grp.index:
            p = summary.loc[idx, "farm_power_mw"]
            summary.loc[idx, "gain_vs_baseline_pct"] = 100.0 * (p - p_base) / p_base

    # 计算模型偏差 (FAST.Farm vs FLORIS)，按 (case, control) 对齐
    summary["model_bias_pct"] = np.nan
    for (case_id, control), grp in summary.groupby(["case_id", "control"]):
        flo = grp[grp["sim"] == "floris"]
        ff = grp[grp["sim"] == "fastfarm"]
        if flo.empty or ff.empty:
            continue
        p_flo = flo["farm_power_mw"].iloc[0]
        p_ff = ff["farm_power_mw"].iloc[0]
        if not p_flo or np.isnan(p_flo):
            continue
        bias = 100.0 * (p_ff - p_flo) / p_flo
        for idx in ff.index:
            summary.loc[idx, "model_bias_pct"] = bias

    return summary
