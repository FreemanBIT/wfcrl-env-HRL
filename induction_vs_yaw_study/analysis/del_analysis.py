"""del.py — Damage Equivalent Load (DEL) 管线（Phase 7 前置 / 开发计划 §Phase 7）
================================================================================

从 FAST.Farm .outb 的载荷通道（0.1s 高频）计算损伤等效载荷（DEL），用于
yaw/induction 控制的载荷对比与 Power-Load Pareto。

方法（风电场疲劳标准惯例）：
  * Rainflow 循环计数（ASTM E1049-85 三点法）
  * S-N 曲线：N = K * S^-m，损伤按 Miner 线性累积 D = Σ n_i / N_i
  * DEL = ( (Σ n_i * S_i^m) / N_eq )^(1/m)
    即：等效于 N_eq 次等幅循环的载荷幅值，损伤等效于实际载荷谱。
  * 本实现取**幅值基** DEL（忽略均值修正/Goodman），与大多数风电 DEL 对比口径一致；
    需含均值效应时另行扩展（留接口）。

组件 S-N 斜率（NREL 5MW 惯例）：
  * 叶片（复合材料）: m = 10
  * 塔架/塔基（钢）:   m = 4
  * 主轴（钢）:        m = 8
  * 偏航轴承:          m = 6

用法（CLI）：
    python -m induction_vs_yaw_study.analysis.del \
        --outb <Case.T1.outb> [--channel RootMOoP1] [--m 10] \
        [--n-eq 600] [--t-settle 60] [--out del.csv]
"""
from __future__ import annotations

import argparse
import os
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---- 组件 S-N 斜率（NREL 5MW 工程惯例）----
M_BLADE = 10      # 叶片根弯矩（复合材料，S-N 缓）
M_TOWER = 4       # 塔基（钢）
M_SHAFT = 8       # 主轴（钢）
M_YAW = 6         # 偏航轴承

# 载荷通道 → 组件 S-N 斜率映射（outb 列名子串 → m）
CHANNEL_M: Dict[str, int] = {
    "RootMIP1": M_BLADE, "RootMOoP1": M_BLADE, "RootMzb1": M_BLADE,
    "TwrBsMyt": M_TOWER, "TwrBsMxt": M_TOWER, "TwrBsFxt": M_TOWER,
    "LSSGagMxa": M_SHAFT, "LSSGagFxa": M_SHAFT,
    "YawBrMyp": M_YAW, "YawBrFxn": M_YAW,
}

# 默认等效循环数：10 分钟（600 s）等效循环（比较用固定基准）
DEFAULT_N_EQ = 600


# =========================================================================
# Rainflow 计数（ASTM E1049-85 三点法）
# =========================================================================

def _extrema(x: np.ndarray) -> np.ndarray:
    """保留转折点（局部极值），去除单调段中间点与连续重复值。"""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 2:
        return x
    keep = np.ones(n, dtype=bool)
    for i in range(1, n - 1):
        d1 = x[i] - x[i - 1]
        d2 = x[i + 1] - x[i]
        # 若 x[i] 不在 d1 与 d2 之间 -> 非转折点，去掉
        if (d1 > 0 and d2 > 0) or (d1 < 0 and d2 < 0) or d1 == 0 or d2 == 0:
            keep[i] = False
    out = x[keep]
    # 去除连续重复值（相邻相等只留一个，避免三点判定歧义）
    if len(out) > 1:
        mask = np.ones(len(out), dtype=bool)
        mask[1:] = out[1:] != out[:-1]
        out = out[mask]
    return out


def rainflow(series: np.ndarray) -> List[Tuple[float, float]]:
    """Rainflow 循环计数，返回 [(range, mean), ...]。

    输入：载荷时序（N）。输出：每个闭合循环的 (range, mean)；
    残余半循环按 0.5 权重计入（与 NREL/风电场惯例一致）。

    实现：ASTM E1049-85 三点法。先取转折点序列，再以最大幅值点
    环化并首尾相接（使谱闭合，避免端点半个循环污染），然后栈式
    三点配对计数。闭合后理论上无残余；防御性保留 0.5 权重分支。
    """
    x = _extrema(np.asarray(series, dtype=float))
    if len(x) < 3:
        return []
    # ASTM E1049：以最大幅值点断开环化（起点落在极值上，避免谱中部
    # 起点造成的伪配对），再首尾相接闭合，三点配对。残差按 0.5 权重。
    idx = int(np.argmax(np.abs(x)))
    x_rot = np.concatenate([x[idx:], x[:idx]])
    x_rot = np.append(x_rot, x_rot[0])  # 首尾相接（闭合）
    stack: List[float] = []
    cycles: List[Tuple[float, float]] = []
    for v in x_rot:
        stack.append(v)
        # 三点配对：a,b,c 三相邻点，若 b 是严格局部极值且 |a-b| <= |b-c|
        # 则计数循环 (a,b)（range=|a-b|, mean=(a+b)/2）并移除 b
        while len(stack) >= 3:
            a, b, c = stack[-3], stack[-2], stack[-1]
            if (b > a and b > c) or (b < a and b < c):
                if abs(b - a) <= abs(c - b):
                    cycles.append((abs(b - a), 0.5 * (a + b)))
                    stack.pop(-2)
                else:
                    break
            else:
                break
    # 残余路径：成对取半循环（0.5 权重）
    for i in range(0, len(stack) - 1, 2):
        rng = abs(stack[i + 1] - stack[i])
        if rng > 1e-12 * (max(abs(x)) + 1.0):
            cycles.append((rng, 0.5 * (stack[i] + stack[i + 1])))
    return cycles


# =========================================================================
# S-N 损伤与 DEL
# =========================================================================

def damage_from_cycles(cycles: List[Tuple[float, float]], m: float) -> float:
    """Miner 线性损伤 D = Σ n_i * (S_i^m)，比例常数归一（比较用相对量）。"""
    if not cycles:
        return 0.0
    dmg = sum(rng ** m for rng, _ in cycles)
    return float(dmg)


def del_from_cycles(cycles: List[Tuple[float, float]], m: float,
                    n_eq: float = DEFAULT_N_EQ) -> float:
    """由循环谱计算 DEL = ( Σ S_i^m / N_eq )^(1/m)。"""
    if not cycles or n_eq <= 0:
        return float("nan")
    num = sum(rng ** m for rng, _ in cycles)
    return float((num / n_eq) ** (1.0 / m))


# =========================================================================
# 从 outb 计算
# =========================================================================

def _col_substring(cols: List[str], key: str) -> Optional[str]:
    """outb 列名模糊匹配：返回含 key 的列名（首个）。"""
    for c in cols:
        if key in c:
            return c
    return None


def compute_del_from_outb(outb_path: str, channel: str = "RootMOoP1",
                          m: Optional[int] = None,
                          n_eq: float = DEFAULT_N_EQ,
                          t_settle_s: float = 0.0,
                          settle_col: str = "Time_[s]") -> Dict:
    """从 .outb 计算指定通道的 DEL。

    Parameters
    ----------
    outb_path : str
        Case.T{i}.outb 路径（openfast_toolbox 可解析）。
    channel : str
        载荷通道名子串（如 "RootMOoP1" / "TwrBsMyt" / "LSSGagMxa"）。
    m : int | None
        S-N 斜率；None 则按 CHANNEL_M 自动推断（未知通道默认 10）。
    n_eq : float
        等效循环数（默认 600 = 10 分钟基准）。
    t_settle_s : float
        跳过前 t_settle 秒（过渡段），默认 0。
    """
    from openfast_toolbox.io.fast_output_file import FASTOutputFile
    df = FASTOutputFile(outb_path).toDataFrame()
    cols = list(df.columns)
    # 时间列
    tcol = _col_substring(cols, "Time") or cols[0]
    t = df[tcol].to_numpy(float)
    # 载荷列
    lcol = _col_substring(cols, channel)
    if lcol is None:
        raise KeyError(f"channel {channel!r} not found in outb columns; "
                       f"available load cols: {[c for c in cols if any(k in c for k in CHANNEL_M)]}")
    series = df[lcol].to_numpy(float)
    # 单位换算：outb kN-m / kN -> N-m / N
    unit = "kN" if "kN" in lcol else "raw"
    if unit == "kN":
        series = series * 1e3
    # 窗口
    if t_settle_s > 0:
        mask = t >= t_settle_s
        series = series[mask]
        t = t[mask]
    series = series[~np.isnan(series)]
    if len(series) < 10:
        return {"channel": lcol, "unit": unit, "m": m, "n_eq": n_eq,
                "n_samples": len(series), "DEL": float("nan")}
    if m is None:
        m = next((v for k, v in CHANNEL_M.items() if k in lcol), M_BLADE)
    cycles = rainflow(series)
    del_val = del_from_cycles(cycles, float(m), n_eq=n_eq)
    dmg = damage_from_cycles(cycles, float(m))
    span = len(series) * (df[tcol].iloc[1] - df[tcol].iloc[0]) if len(df) > 1 else 0.0
    return {
        "channel": lcol, "unit": unit, "m": m, "n_eq": n_eq,
        "n_samples": len(series), "span_s": float(span),
        "n_cycles": len(cycles),
        "load_mean": float(np.mean(series)),
        "load_std": float(np.std(series, ddof=1)),
        "load_max": float(np.max(series)),
        "DEL": float(del_val), "damage_idx": float(dmg),
    }


def main():
    ap = argparse.ArgumentParser(description="DEL 计算（rainflow + S-N）")
    ap.add_argument("--outb", required=True, help="Case.T{i}.outb 路径")
    ap.add_argument("--channel", default="RootMOoP1")
    ap.add_argument("--m", type=int, default=None, help="S-N 斜率（默认按通道推断）")
    ap.add_argument("--n-eq", type=float, default=DEFAULT_N_EQ)
    ap.add_argument("--t-settle", type=float, default=0.0)
    ap.add_argument("--out", default=None, help="结果写入 csv（可选）")
    a = ap.parse_args()
    res = compute_del_from_outb(a.outb, a.channel, m=a.m, n_eq=a.n_eq,
                                t_settle_s=a.t_settle)
    print(f"channel={res['channel']} ({res['unit']})  m={res['m']}  "
          f"n_eq={res['n_eq']:.0f}  n_cycles={res['n_cycles']}  "
          f"DEL={res['DEL']:.4e}")
    if a.out:
        import csv
        with open(a.out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(res.keys()))
            w.writeheader()
            w.writerow(res)
        print(f"-> {a.out}")


if __name__ == "__main__":
    main()
