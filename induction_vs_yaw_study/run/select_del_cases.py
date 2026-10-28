"""select_del_cases.py — 选取 DEL（疲劳载荷）代表工况
================================================================================

背景：DOE case 完成后会清理 FarmInputs（bts/outb/DLL）以控磁盘（每 case 231MB→1.8MB），
因此 DEL 阶段必须对**代表工况**用 WFCRL_STUDY_KEEP_CASE_FILES=1 重跑以保留 .outb。

本工具从已完成的 DOE 分析结果中挑选代表工况：
  1. 覆盖 2–3 个间距（默认 S4D、S6D）
  2. 覆盖 2 个湍流（默认 TI05、TI15）
  3. 每个 (spacing, TI) 下取 baseline / yaw / induction 各 1 seed（默认取该组接近中位增益的 seed）
  4. 输出 case 列表 CSV + 可直接执行的批跑命令

输出：
  <analysis>/del_cases.csv       选取的 case（case_id, control, U, TI, WD, spacing_D, seed）
  <analysis>/del_run_command.txt 建议的批跑命令（含 keep-files 环境变量）

用法：
    python -m induction_vs_yaw_study.run.select_del_cases --analysis results/run_full_doe/analysis
    python -m induction_vs_yaw_study.run.select_del_cases --analysis ... --spacings 4 8 --tis 0.05 0.15
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def _load(path: Path) -> list:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--analysis", type=Path, required=True)
    ap.add_argument("--spacings", nargs="+", type=float, default=[4.0, 6.0])
    ap.add_argument("--tis", nargs="+", type=float, default=[0.05, 0.15])
    ap.add_argument("--wind-speed", type=float, default=8.0,
                    help="代表风速（DEL 通常取额定附近；默认 8 m/s）")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()

    gains = _load(a.analysis / "gains.csv")
    cases = _load(a.analysis / "case_summary.csv")

    # 索引 case： (case_id, control) -> 记录
    idx = {}
    for c in cases:
        idx[(c["case_id"], c["control"])] = c

    picked = []
    for s in a.spacings:
        for ti in a.tis:
            # 先在该 (U,TI,S) 的 yaw 增益中选"接近中位"的工况 -> 得到共同的 (WD, seed)
            ysub = [g for g in gains if abs(float(g["U"]) - a.wind_speed) < 1e-6
                    and abs(float(g["TI"]) - ti) < 1e-6
                    and abs(float(g["spacing_D"]) - s) < 1e-6
                    and g["control"] == "yaw"]
            if not ysub:
                continue
            # 选择依据 = **FLORIS 预测增益**最大的 WD（基于模型，不看实测结果，
            # 避免按结果挑 case）；无预测时退回"最大 |偏航角| 组"即 WD 最大者。
            def _pred(g):
                try:
                    v = float(g.get("floris_pred_pct", "nan"))
                    return v if v == v else -1e9
                except ValueError:
                    return -1e9
            ref = max(ysub, key=lambda g: (_pred(g), abs(float(g["WD"]))))
            print(f"  [select] S{s:g}D TI{ti:.2f}: pick WD{int(float(ref['WD']))} "
                  f"(FLORIS pred {_pred(ref):+.2f}%, measured {float(ref['gain_pct']):+.2f}%)")
            wd = int(float(ref["WD"]))
            # 三 control 共用同一 (TI, WD, spacing)；seed 取该组第一个 stable 的
            seed = None
            for (cid, ctl2), c in sorted(idx.items()):
                if ctl2 != "baseline":
                    continue
                if not cid.startswith(f"U{int(round(a.wind_speed)):02d}_TI{int(round(ti*100)):02d}_seed"):
                    continue
                if not cid.endswith(f"_wd{wd}_s{s:g}D"):
                    continue
                if str(c.get("stable", "")).lower() in ("true", "1"):
                    seed = cid.split("_seed")[1][:2]
                    break
            if seed is None:
                continue
            for ctl in ("baseline", "yaw", "induction"):
                cid = f"U{int(round(a.wind_speed)):02d}_TI{int(round(ti*100)):02d}_seed{seed}_wd{wd}_s{s:g}D"
                c = idx.get((cid, ctl))
                if c is None:
                    continue
                picked.append({
                    "case_id": cid, "control": ctl,
                    "U": c["U"], "TI": c["TI"], "seed": c["seed"],
                    "WD": c["WD"], "spacing_D": c["spacing_D"],
                    "farm_kw": c.get("farm_kw", ""),
                })

    # 去重
    seen = set()
    uniq = []
    for p in picked:
        k = (p["case_id"], p["control"])
        if k in seen:
            continue
        seen.add(k)
        uniq.append(p)

    out_dir = a.out or a.analysis
    csv_path = out_dir / "del_cases.csv"
    if uniq:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(uniq[0].keys()))
            w.writeheader()
            w.writerows(uniq)

    # 生成命令（keep-files + 仅选中的 case 需单独跑：用 --filter 按 seed/spacing/TI 组合过多，
    # 故建议逐 case 调 _case_worker；此处给出批量脚本模板）
    cmd_lines = [
        "# DEL 代表工况重跑（保留 outb）——PowerShell",
        "$env:WFCRL_STUDY_OMP_NUM_THREADS = '3'",
        "$env:WFCRL_STUDY_KEEP_CASE_FILES = '1'",
        "# 对每个 case 调 _case_worker（row json 可由 del_cases.csv 生成）：",
        "#   python -m induction_vs_yaw_study.run._case_worker --row <row.json> --out <run_del>",
        f"# 共 {len(uniq)} 个代表工况，见 {csv_path.name}",
        "# 跑完后执行 DEL 计算：",
        "#   python -m induction_vs_yaw_study.run.compute_del_batch --run <run_del> --t-settle 60",
    ]
    (out_dir / "del_run_command.txt").write_text("\n".join(cmd_lines) + "\n", encoding="utf-8")

    print(f"selected {len(uniq)} representative cases -> {csv_path}")
    for p in uniq:
        print(f"  {p['case_id']}_{p['control']}  ({p['farm_kw']} kW)")


if __name__ == "__main__":
    main()
