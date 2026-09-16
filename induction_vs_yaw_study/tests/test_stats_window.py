"""test_stats_window.py — 显式统计窗口（方案 V2 §8.2 / §31.5）
"""
from __future__ import annotations

import numpy as np
import pandas as pd
import pytest

from induction_vs_yaw_study.analysis.metrics import (
    steady_window, summarize_timeseries, build_summary,
)


def _toy_df():
    t = np.arange(1, 121) * 3.0          # 3..360 s
    farm = np.full(len(t), 10.0)
    farm[t < 600 if False else 0:] = 10.0
    # 前段 8 MW、后段 12 MW，方便区分窗口
    p = np.where(t < 600, 8.0, 12.0)
    return pd.DataFrame({"time_s": t, "farm_power_MW": p, "T1_power_MW": p})


def test_steady_window_filters_explicit():
    df = _toy_df()
    w = steady_window(df, t_settle_s=600.0)
    assert len(w) == 0  # 全部 < 600（样例只有 360s）—— 需用更长样例另行验证
    df2 = pd.concat([df, pd.DataFrame({"time_s": [600, 603],
                                        "farm_power_MW": [12.0, 12.0],
                                        "T1_power_MW": [12.0, 12.0]})])
    w2 = steady_window(df2, t_settle_s=600.0)
    assert (w2["time_s"] >= 600).all()
    assert len(w2) == 2


def test_summarize_uses_window():
    df = pd.concat([_toy_df(),
                    pd.DataFrame({"time_s": [600, 603, 606],
                                  "farm_power_MW": [12.0, 12.0, 12.0],
                                  "T1_power_MW": [12.0, 12.0, 12.0]})])
    m = summarize_timeseries(df, t_settle_s=600.0)
    assert m["farm_power_mw"] == pytest.approx(12.0)  # 窗口内全部 12 MW
    assert m["n_samples"] == 3


def test_build_summary_respects_window():
    import tempfile, os
    with tempfile.TemporaryDirectory() as d:
        base = pd.concat([_toy_df(),
                          pd.DataFrame({"time_s": [600, 603],
                                        "farm_power_MW": [12.0, 12.0],
                                        "T1_power_MW": [12.0, 12.0]})])
        base.to_csv(os.path.join(d, "fastfarm_baseline_caseA.csv"), index=False)
        s = build_summary(d, t_settle_s=600.0)
        assert s["farm_power_mw"].iloc[0] == pytest.approx(12.0)
        s_legacy = build_summary(d, frac=0.5)
        assert 8.0 <= s_legacy["farm_power_mw"].iloc[0] <= 12.0
