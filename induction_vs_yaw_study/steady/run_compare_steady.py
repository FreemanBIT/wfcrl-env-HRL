"""
run_compare_steady.py — 稳态风对比汇总入口
============================================
读取 results/timeseries_steady/ 下的回放时序，构建 FAST.Farm vs FLORIS、
yaw vs derating 的对比 summary，并按 (风速, 风向偏移, 间距) 输出表格与图。

复用主工程 analysis.metrics.build_summary（命名约定兼容稳态 case_id）。

用法：
    python -m induction_vs_yaw_study.steady.run_compare_steady
产物：
    results/steady_summary.csv         # 明细（每 case×sim×control 一行）
    results/steady_summary_by_dim.csv  # 按维度透视
    results/figs_steady/*.png          # 可选图（matplotlib 可用时）
"""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pandas as pd

from induction_vs_yaw_study.analysis.metrics import build_summary


# 稳态 case_id 形如：stU8_wd-10_s4D_ab12cd
_ID_RE = re.compile(
    r"stU(?P<U>[-0-9.]+)_wd(?P<wd>[-0-9.]+)_s(?P<s>[-0-9.]+)D_[0-9a-f]+"
)


def _parse_steady_id(case_id: str):
    m = _ID_RE.match(case_id)
    if not m:
        return None
    return {
        "wind_speed_ms": float(m.group("U")),
        "wind_direction_offset_deg": float(m.group("wd")),
        "spacing_D": float(m.group("s")),
    }


def main():
    root = Path(__file__).resolve().parents[2]
    ts_dir = root / "results" / "timeseries_steady"
    if not ts_dir.exists():
        raise SystemExit(f"未找到稳态时序目录：{ts_dir}\n请先运行 run_replay_steady。")

    summary = build_summary(str(ts_dir), frac=0.5)
    if summary.empty:
        raise SystemExit("稳态时序为空，无法汇总。请先成功回放至少若干工况。")

    # 解析维度列
    dims = summary["case_id"].apply(_parse_steady_id)
    dim_df = pd.DataFrame([d if d else {} for d in dims])
    summary = pd.concat([summary.reset_index(drop=True), dim_df], axis=1)

    out_csv = root / "results" / "steady_summary.csv"
    summary.to_csv(out_csv, index=False)
    print(f"[STEADY] 明细汇总 -> {out_csv}  ({len(summary)} 行)")

    # ---- 透视：按 (风速, 风向偏移, 间距) 看 yaw/derating 的增益与模型偏差 ----
    keep = summary.dropna(subset=["wind_speed_ms"]).copy()
    if not keep.empty:
        piv = keep.pivot_table(
            index=["wind_speed_ms", "wind_direction_offset_deg", "spacing_D"],
            columns=["sim", "control"],
            values="farm_power_mw",
            aggfunc="mean",
        )
        piv_csv = root / "results" / "steady_summary_by_dim.csv"
        piv.to_csv(piv_csv)
        print(f"[STEADY] 维度透视 -> {piv_csv}")

        # 增益与模型偏差透视
        gain = keep.pivot_table(
            index=["wind_speed_ms", "wind_direction_offset_deg", "spacing_D"],
            columns=["sim", "control"],
            values="gain_vs_baseline_pct",
            aggfunc="mean",
        )
        gain_csv = root / "results" / "steady_gain_by_dim.csv"
        gain.to_csv(gain_csv)
        print(f"[STEADY] 增益透视 -> {gain_csv}")

    # ---- 可选作图 ----
    try:
        _plots(summary, root)
    except Exception as e:  # noqa
        print(f"[STEADY] 作图跳过（matplotlib 不可用或出错）：{e}")


def _plots(summary: pd.DataFrame, root: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figs = root / "results" / "figs_steady"
    figs.mkdir(parents=True, exist_ok=True)

    df = summary.dropna(subset=["wind_speed_ms"]).copy()
    if df.empty:
        return

    # 图1：每个 sim 下，yaw vs derating vs baseline 的全场功率随间距变化
    #       （在 wd_offset=0、各风速分面）
    sub0 = df[np.isclose(df["wind_direction_offset_deg"], 0.0)]
    speeds = sorted(sub0["wind_speed_ms"].dropna().unique())
    if speeds:
        fig, axes = plt.subplots(1, len(speeds), figsize=(5 * len(speeds), 4),
                                 squeeze=False)
        for j, U in enumerate(speeds):
            ax = axes[0][j]
            d = sub0[np.isclose(sub0["wind_speed_ms"], U)]
            for sim in ["floris", "fastfarm"]:
                for control in ["baseline", "yaw", "derating"]:
                    dd = d[(d["sim"] == sim) & (d["control"] == control)]
                    dd = dd.sort_values("spacing_D")
                    if dd.empty:
                        continue
                    ls = "-" if sim == "fastfarm" else "--"
                    ax.plot(dd["spacing_D"], dd["farm_power_mw"],
                            ls=ls, marker="o", label=f"{sim}/{control}")
            ax.set_title(f"U={U} m/s, wd_offset=0°")
            ax.set_xlabel("spacing (D)")
            ax.set_ylabel("farm power (MW)")
            ax.grid(True, alpha=0.3)
            if j == 0:
                ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(figs / "power_vs_spacing.png", dpi=120)
        plt.close(fig)

    # 图2：模型偏差 (FAST.Farm vs FLORIS) 随风向偏移（在 spacing=4D 各风速）
    sub4 = df[np.isclose(df["spacing_D"], 4.0) & (df["sim"] == "fastfarm")]
    if not sub4.empty and "model_bias_pct" in sub4.columns:
        fig, ax = plt.subplots(figsize=(6, 4))
        for control in ["yaw", "derating"]:
            dd = sub4[sub4["control"] == control].sort_values("wind_direction_offset_deg")
            if dd.empty:
                continue
            ax.plot(dd["wind_direction_offset_deg"], dd["model_bias_pct"],
                    marker="s", label=control)
        ax.axhline(0, color="k", lw=0.8)
        ax.set_title("FAST.Farm vs FLORIS farm-power bias (spacing=4D)")
        ax.set_xlabel("wind direction offset (deg)")
        ax.set_ylabel("model bias (%)")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(figs / "model_bias_vs_direction.png", dpi=120)
        plt.close(fig)

    print(f"[STEADY] 图已保存到 {figs}")


if __name__ == "__main__":
    main()
