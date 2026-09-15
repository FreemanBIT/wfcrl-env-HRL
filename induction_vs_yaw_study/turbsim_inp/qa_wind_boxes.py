"""qa_wind_boxes.py — 风盒 QA（方案 V2 §17 / 开发计划 §4.3）

对 (U, TI, seed) 唯一键盒在 (z=90, y=0) 实际轮毂位计算：
  realized U、TI_u、sigma_u/v/w；对照目标（从文件名解析或 --config）输出 pass/fail。

验收（方案 V2 §17.2）：
  |U_real - U_tar| / U_tar <= 5%   且    |TI_real - TI_tar| <= 0.02

用法：
    python -m induction_vs_yaw_study.turbsim_inp.qa_wind_boxes \
        --input induction_vs_yaw_study/FarmInputs/coarse \
        --config induction_vs_yaw_study/config/wind_conditions.yaml \
        --out induction_vs_yaw_study/FarmInputs/qa
"""
from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

import numpy as np

STUDY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = STUDY_ROOT / "FarmInputs" / "coarse"
DEFAULT_CONFIG = STUDY_ROOT / "config" / "wind_conditions.yaml"
DEFAULT_QA_OUT = STUDY_ROOT / "FarmInputs" / "qa"

U_TOL_FRAC = 0.05
TI_TOL_ABS = 0.02

_NAME_RE = re.compile(r"inflow_U(?P<U>\d+)_TI(?P<TI>\d+)_seed(?P<seed>\d+)\.bts$")


def parse_targets(fname: str) -> tuple | None:
    m = _NAME_RE.search(fname)
    if not m:
        return None
    return float(m.group("U")), float(m.group("TI")) / 100.0, int(m.group("seed"))


def _series(ts: dict, spec: str):
    """从 TurbSimFile dict 取 (y=0, z=90) 位置的时间序列。

    openfast_toolbox TurbSimFile 的 u/v/w 数组维度可能为 (t, z, y) 或 (z, y, t)，
    这里按坐标向量长度做防御性判断。
    """
    y = np.asarray(ts["y"], dtype=float)
    z = np.asarray(ts["z"], dtype=float)
    arr = np.asarray(ts[spec], dtype=float)
    iy = int(np.argmin(np.abs(y - 0.0)))
    iz = int(np.argmin(np.abs(z - 90.0)))
    if arr.ndim == 3:
        if len(y) == arr.shape[2] and len(z) == arr.shape[1]:
            return arr[:, iz, iy]          # (t, z, y)
        if len(y) == arr.shape[1] and len(z) == arr.shape[0]:
            return arr[iz, iy, :]          # (z, y, t)
    raise ValueError(f"unexpected {spec} shape {arr.shape} (y={y.shape}, z={z.shape})")


def analyze_bts(bts: Path):
    """返回 dict(file,...,realized_*)；失败时 realized 为 None 并置 error。"""
    from openfast_toolbox.io.turbsim_file import TurbSimFile

    out = {"file": bts.name, "seed_id": None, "target_U": None, "target_TI": None,
           "realized_U": None, "realized_TI": None, "sigma_u": None,
           "sigma_v": None, "sigma_w": None, "pass": False, "error": ""}
    tgt = parse_targets(bts.name)
    if tgt is None:
        out["error"] = "filename not in inflow_U{02d}_TI{02d}_seed{02d} format"
        return out
    out["target_U"], out["target_TI"], out["seed_id"] = tgt[0], tgt[1], tgt[2]

    try:
        ts = TurbSimFile(str(bts))
        u = _series(ts, "u")
        v = _series(ts, "v")
        w = _series(ts, "w")
    except Exception as e:  # noqa
        out["error"] = repr(e)[:200]
        return out

    if u.size < 2:
        out["error"] = "too few samples"
        return out
    u_mean = float(np.mean(u))
    su = float(np.std(u, ddof=1))
    out["realized_U"] = u_mean
    out["realized_TI"] = su / u_mean if u_mean > 0 else np.nan
    out["sigma_u"] = su
    out["sigma_v"] = float(np.std(v, ddof=1))
    out["sigma_w"] = float(np.std(w, ddof=1))
    out["pass"] = (
        abs(u_mean - out["target_U"]) / out["target_U"] <= U_TOL_FRAC
        and abs(out["realized_TI"] - out["target_TI"]) <= TI_TOL_ABS
    )
    return out


def main():
    ap = argparse.ArgumentParser(description="QA unified TurbSim boxes")
    ap.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--out", type=Path, default=DEFAULT_QA_OUT)
    ap.add_argument("--pattern", default="inflow_*_seed*.bts")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    bts_files = sorted(args.input.glob(args.pattern))
    if not bts_files:
        print(f"No .bts matched {args.pattern} in {args.input}")
        return 1
    print(f"QA {len(bts_files)} boxes in {args.input}")

    rows = [analyze_bts(p) for p in bts_files]
    n_pass = sum(1 for r_ in rows if r_["pass"])

    args.out.mkdir(parents=True, exist_ok=True)
    metrics_csv = args.out / "wind_box_metrics.csv"
    with open(metrics_csv, "w", newline="", encoding="utf-8") as f:
        cols = ["file", "seed_id", "target_U", "target_TI", "realized_U",
                "realized_TI", "sigma_u", "sigma_v", "sigma_w", "pass", "error"]
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r_ in rows:
            w.writerow({c: r_.get(c) for c in cols})
    print(f"metrics -> {metrics_csv}")

    failed = [r_ for r_ in rows if not r_["pass"]]
    if failed:
        fail_csv = args.out / "wind_box_failures.csv"
        with open(fail_csv, "w", newline="", encoding="utf-8") as f:
            cols = ["file", "target_U", "target_TI", "realized_U",
                    "realized_TI", "error"]
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            for r_ in failed:
                w.writerow({c: r_.get(c) for c in cols})
        print(f"FAILURES -> {fail_csv} ({len(failed)} boxes)")

    if args.plot and rows:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 2, figsize=(11, 4))
        du = [100 * (r_["realized_U"] - r_["target_U"]) / r_["target_U"]
              for r_ in rows if r_["realized_U"] is not None]
        dti = [r_["realized_TI"] - r_["target_TI"]
               for r_ in rows if r_["realized_TI"] is not None]
        axs[0].hist(du, bins=20); axs[0].set_title("dU/U (%)")
        axs[1].hist(dti, bins=20); axs[1].set_title("dTI (abs)")
        fig.tight_layout()
        png = args.out / "plots" / "wind_box_errors.png"
        png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(png, dpi=130)
        print(f"plot -> {png}")

    print(f"QA done: {n_pass}/{len(rows)} pass")
    return 0 if n_pass == len(rows) else 2


if __name__ == "__main__":
    raise SystemExit(main())
