"""metrics.py — 指标计算（Phase 2：显式统计窗口）
=========================================
方案 V2 §8.1 / 开发计划 §8.1：废弃 "last 50%"（steady_window_mean(frac)），
改为显式 [t_settle_s, t_sim_s] 时间窗（默认 600..1200 s，与 grid 的 t_settle 一致）。

兼容：
  * steady_window_mean 保留但标记 deprecated（历史脚本仍可调用）；
  * summarize_timeseries / build_summary 新签名优先用显式窗口；
    未提供 t_settle_s 时回退 frac 逻辑（旧 CSV 无 time_s 列的场景）。
"""
from __future__ import annotations

import glob
import os
import warnings
from typing import Dict, List, Optional

import numpy as np
import pandas as pd


def steady_window_mean(values: np.ndarray, frac: float = 0.5) -> float:
    """[Deprecated] 取后 frac 段的均值（旧 "last 50%" 口径，禁止用于新研究）。"""
    warnings.warn("steady_window_mean 已废弃：请使用显式统计窗口 [t_settle, t_sim]（方案 V2 §8.1）",
                  DeprecationWarning, stacklevel=2)
    v = np.asarray(values, dtype=float).ravel()
    v = v[~np.isnan(v)]
    if v.size == 0:
        return np.nan
    k = max(1, int(v.size * (1 - frac)))
    return float(np.mean(v[k:]))


def steady_window(df: pd.DataFrame, t_settle_s: float = 600.0,
                  t_sim_s: Optional[float] = None) -> pd.DataFrame:
    """
    显式统计窗口：time_s ∈ [t_settle_s, t_sim_s]（若无 time_s 列则原样返回）。

    方案 V2 §8.2：baseline/yaw/induction 必须使用完全相同的时间窗口。
    """
    if "time_s" not in df.columns:
        return df
    mask = df["time_s"] >= t_settle_s
    if t_sim_s is not None:
        mask &= df["time_s"] <= t_sim_s
    return df[mask]


def _load_timeseries_csv(path: str) -> pd.DataFrame:
    return pd.read_csv(path)


def summarize_timeseries(df: pd.DataFrame, frac: float = 0.5,
                         t_settle_s: Optional[float] = None,
                         t_sim_s: Optional[float] = None) -> Dict[str, float]:
    """
    从单条时序 DataFrame 提取稳态指标。

    期望列：farm_power_MW, T{i}_power_MW, time_s, 以及可选 T{i}_blade_loads* / pitch。
    * 提供 t_settle_s -> 显式窗口（推荐）；
    * 否则回退 frac（旧 CSV 无 time_s）。
    """
    if t_settle_s is not None:
        df = steady_window(df, t_settle_s=t_settle_s, t_sim_s=t_sim_s)

    pcols = sorted([c for c in df.columns if c.endswith("_power_MW") and c != "farm_power_MW"])
    out: Dict[str, float] = {}
    if "farm_power_MW" in df.columns:
        v = _window_mean(df["farm_power_MW"].values, frac)
        out["farm_power_mw"] = v
        out["n_samples"] = float(len(df))
    for c in pcols:
        out[c.replace("_power_MW", "_power_mw").lower()] = _window_mean(df[c].values, frac)
    # 载荷代理：叶根弯矩标准差（若存在 blade load 列）
    bcols = [c for c in df.columns if "blade" in c.lower() or "RootM" in c]
    if bcols:
        loads = df[bcols].values.astype(float)
        out["blade_load_std"] = float(np.nanstd(loads))
    return out


def _window_mean(values, frac: float) -> float:
    if frac is None:
        # 纯显式窗口：窗口内均值（含 NaN 剔除）
        v = np.asarray(values, dtype=float).ravel()
        v = v[~np.isnan(v)]
        return float(np.mean(v)) if v.size else float("nan")
    return steady_window_mean(values, frac)


def build_summary(
    timeseries_dir: str,
    *,
    frac: Optional[float] = None,
    t_settle_s: Optional[float] = 600.0,
    t_sim_s: Optional[float] = None,
    case_manifest: Optional[str] = None,
) -> pd.DataFrame:
    """
    扫描 timeseries 目录构建 summary（每行 = case_id, sim, control + 稳态指标 + 增益）。

    * 默认显式窗口 [600, ...]（方案 V2 §8）；
    * frac 非 None -> 回退旧口径（兼容历史）；
    * case_manifest 提供时只统计其中的 case_id（结果隔离，开发计划 §9）。
    """
    if frac is not None:
        t_settle_s = None  # 旧口径优先
    rows: List[Dict] = []
    only = None
    if case_manifest:
        mf = pd.read_csv(case_manifest)
        if "case_id" in mf.columns:
            only = set(mf["case_id"].dropna())

    files = sorted(glob.glob(os.path.join(timeseries_dir, "*.csv")))
    for f in files:
        base = os.path.splitext(os.path.basename(f))[0]
        parts = base.split("_")
        if len(parts) < 3:
            continue
        sim = parts[0]
        control = parts[1]
        case_id = "_".join(parts[2:])
        if only is not None and case_id not in only:
            continue
        try:
            df = _load_timeseries_csv(f)
        except Exception:
            continue
        metrics = summarize_timeseries(df, frac=(frac if frac is not None else 0.5),
                                       t_settle_s=t_settle_s, t_sim_s=t_sim_s)
        rows.append({"case_id": case_id, "sim": sim, "control": control, **metrics})

    summary = pd.DataFrame(rows)
    if summary.empty:
        return summary

    # 相对 baseline 的功率增益（同 sim 内）
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

    # 模型偏差 (FAST.Farm vs FLORIS)，按 (case, control) 对齐
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
