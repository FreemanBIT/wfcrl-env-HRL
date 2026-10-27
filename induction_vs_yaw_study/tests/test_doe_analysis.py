"""test_doe_analysis.py — DOE 分析工具单测（解析/判据/配对增益）"""
from __future__ import annotations

import csv
import os

import numpy as np
import pytest

from induction_vs_yaw_study.run.analyze_doe import (
    _parse_case_dir, _case_metrics, _gains, _agg,
)
from induction_vs_yaw_study.run.check_doe_quality import _parse as _qparse


def _write_ts(path, n=400, dt=3.0, farm_mw=3.0, pitch_deg=0.0, rpm=12.0):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        cols = ["time_s", "farm_power_MW"] + [
            f"T{i}_{k}" for i in (1, 2, 3) for k in ("power_MW", "rotor_speed_rpm", "pitch_deg")
        ]
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for i in range(n):
            row = {"time_s": (i + 1) * dt, "farm_power_MW": farm_mw}
            for t in (1, 2, 3):
                row[f"T{t}_power_MW"] = farm_mw / 3.0
                row[f"T{t}_rotor_speed_rpm"] = rpm
                row[f"T{t}_pitch_deg"] = pitch_deg
            w.writerow(row)


def test_parse_case_dir():
    meta, ctl = _parse_case_dir("U08_TI10_seed01_wd20_s4D_yaw")
    assert ctl == "yaw"
    assert meta["U"] == 8.0 and meta["TI"] == pytest.approx(0.10)
    assert meta["seed"] == 1 and meta["WD"] == 20.0 and meta["spacing_D"] == 4.0
    assert _parse_case_dir("garbage") == (None, "unknown")


def test_quality_parse_matches():
    meta, ctl = _qparse("U12_TI15_seed06_wd20_s8D_induction")
    assert ctl == "induction"
    assert meta["U"] == 12.0 and meta["TI"] == pytest.approx(0.15)
    assert meta["WD"] == 20.0 and meta["spacing_D"] == 8.0


def test_case_metrics_stable(tmp_path):
    p = tmp_path / "timeseries.csv"
    _write_ts(str(p), farm_mw=3.0, pitch_deg=5.0, rpm=12.0)
    m = _case_metrics(p, 600.0, 1200.0)
    assert m is not None
    assert m["stable"] is True
    assert m["farm_kw"] == pytest.approx(3000.0)


def test_case_metrics_unstable_feather(tmp_path):
    p = tmp_path / "timeseries.csv"
    _write_ts(str(p), farm_mw=0.0, pitch_deg=90.0, rpm=0.5)
    m = _case_metrics(p, 600.0, 1200.0)
    assert m["stable"] is False
    assert m["unstable_turbines"] == "1;2;3"


def test_case_metrics_flags_half_feather(tmp_path):
    """半顺桨（pitch 60°, rpm 5）也必须判失稳（旧判据 pitch>80 会漏）。"""
    p = tmp_path / "timeseries.csv"
    _write_ts(str(p), farm_mw=1.0, pitch_deg=60.0, rpm=5.0)
    m = _case_metrics(p, 600.0, 1200.0)
    assert m["stable"] is False


def test_seed_paired_gains():
    """配对增益：同 seed 的 test/baseline 比值；不同 seed 不混用。"""
    case_rows = []
    for seed, (b, t) in {1: (100.0, 110.0), 2: (200.0, 210.0)}.items():
        for ctl, kw in (("baseline", b), ("yaw", t)):
            case_rows.append({
                "case_id": "U08_TI10_seed0%d_wd0_s4D" % seed, "U": 8.0, "TI": 0.10,
                "seed": seed, "WD": 0.0, "spacing_D": 4.0, "control": ctl,
                "farm_kw": kw, "stable": True, "T1_kw": kw / 3, "T2_kw": kw / 3, "T3_kw": kw / 3,
            })
    groups = _agg(case_rows, include_unstable=False)
    gains = _gains(groups, case_rows, include_unstable=False)
    yaw = [g for g in gains if g["control"] == "yaw"]
    assert len(yaw) == 1
    # seed1: +10%, seed2: +5% -> 均值 7.5%
    assert yaw[0]["gain_pct"] == pytest.approx(7.5, abs=1e-6)
    assert yaw[0]["n_seeds"] == 2
