"""_fill_boxes.py — 按 (U,TI,seed) 缺失清单补齐风盒（与孤儿 TurbSim 并存不冲突）

规则：.bts 不存在 -> 生成 .inp + 跑 TurbSim；size==0（可能有孤儿正在写）或
>=100MB（已完成）均跳过。仅用于补齐；正式批量仍用 generate_wind_boxes。
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

_STUDY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_STUDY_ROOT))
import os
os.environ.setdefault("WFCRL_FARMINPUTS_DIR", str(_STUDY_ROOT / "FarmInputs"))

from induction_vs_yaw_study.turbsim_inp.generate_wind_boxes import (
    load_config, build_inp_text, inp_stem, run_turbsim,
)
from induction_vs_yaw_study.turbsim_inp.generate_wind_boxes import DEFAULT_CONFIG

COARSE = _STUDY_ROOT / "FarmInputs" / "coarse"
DONE_MIN_BYTES = 100_000_000


def missing_boxes(cfg, speeds, tis, seeds):
    out = []
    for seed_id in seeds:
        for speed in speeds:
            for ti in tis:
                stem = inp_stem(speed, ti, seed_id)
                bts = COARSE / f"{stem}.bts"
                if bts.exists() and bts.stat().st_size >= DONE_MIN_BYTES:
                    continue  # 已完成
                if bts.exists() and bts.stat().st_size == 0:
                    continue  # 0 字节：可能孤儿正在写，跳过
                out.append((speed, ti, seed_id, stem))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", nargs="+", type=float, default=[8.0])
    ap.add_argument("--tis", nargs="+", type=float, default=[0.05, 0.10, 0.15])
    ap.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--turbsim", type=Path, default=None)
    args = ap.parse_args()

    cfg = load_config(DEFAULT_CONFIG)
    missing = missing_boxes(cfg, args.speeds, args.tis, args.seeds)
    print(f"missing: {len(missing)}")
    if not missing:
        return
    exe = args.turbsim or Path(
        r"D:\HR_Project\wfcrl-env-HRL\FarmInputs\TurbSim_x64.exe")
    print(f"TurbSim: {exe}  jobs={args.jobs}")

    def work(item):
        speed, ti, seed_id, stem = item
        inp = COARSE / f"{stem}.inp"
        inp.write_text(build_inp_text(cfg, speed, ti, seed_id), encoding="ascii")
        print(f"== TurbSim {stem} ==", flush=True)
        rc = run_turbsim(exe, inp)
        ok = (COARSE / f"{stem}.bts").exists() and (COARSE / f"{stem}.bts").stat().st_size >= DONE_MIN_BYTES
        print(f"   rc={rc} ok={ok}", flush=True)
        return ok

    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        res = list(pool.map(work, missing))
    print(f"done ok={sum(res)}/{len(res)}")


if __name__ == "__main__":
    main()
