"""plot_openfast_yaw_state.py — 用 OpenFAST 逐机输出（Case.T<n>.out 的 NacYaw）验证偏航执行

生成各机 NacYaw 时序图（权威 OpenFAST 状态，非 ZMQ 状态帧），判定偏航指令真正执行。
"""
from __future__ import annotations

import glob
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = ["Microsoft YaHei", "Noto Sans SC", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, r"D:\HR_Project\wfcrl-env-HRL")
from openfast_toolbox.io import FASTOutputFile

_STUDY_ROOT = Path(__file__).resolve().parents[1]
CTRL = [3, 4]
STEP_AT = 200.0


def main() -> int:
    runs = sorted(glob.glob(str(_STUDY_ROOT / "results" / "_six_turbine_yaw" / "*/FarmInputs/Case.T1.out")))
    if not runs:
        print("[skip] 无 OpenFAST 逐机输出")
        return 1
    farm_dir = Path(runs[-1]).parent
    run_dir = farm_dir.parents[1]
    out_dir = run_dir / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(6, 1, figsize=(11, 2.1 * 6), sharex=True)
    for tid, ax in zip(range(1, 7), axes):
        f = FASTOutputFile(str(farm_dir / f"Case.T{tid}.out"))
        df = f.toDataFrame()
        t = df["Time_[s]"].to_numpy()
        ny = df["NacYaw_[deg]"].to_numpy()
        ax.plot(t, ny, "-", color="#2ca02c", lw=1.2)
        ax.axvline(STEP_AT, color="red", ls="--", lw=0.8)
        if tid in CTRL:
            ax.set_facecolor("#fff5f5")
        ax.set_ylabel(f"T{tid} NacYaw (deg)")
        ax.set_ylim(-15, 5)
        ax.grid(alpha=0.3)
        if tid == 1:
            ax.set_title("各机 NacYaw 时序（OpenFAST 逐机输出权威状态；T3/T4 200s 阶跃 +10°，红色=受控机）")
    axes[-1].set_xlabel("时间 (s)")
    fig.tight_layout()
    p = out_dir / "openfast_nacyaw_execution.png"
    fig.savefig(p, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  -> {p}")

    # 汇总判定表
    print("=== 偏航执行判定（OpenFAST NacYaw，均值 260-340s）===")
    for tid in range(1, 7):
        f = FASTOutputFile(str(farm_dir / f"Case.T{tid}.out"))
        df = f.toDataFrame()
        t = df["Time_[s]"].to_numpy()
        ny = df["NacYaw_[deg]"].to_numpy()
        m = (t >= 260) & (t <= 340)
        mean_ny = float(np.mean(ny[m]))
        print(f"  T{tid}: NacYaw 均值={mean_ny:+.2f} deg  "
              f"{'(受控机，应≈-10°)' if tid in CTRL else '(未受控，应≈0°)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
