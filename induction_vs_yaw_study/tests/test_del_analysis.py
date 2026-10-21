"""test_del_analysis.py — DEL/rainflow 单测"""
from __future__ import annotations

import numpy as np
import pytest

from induction_vs_yaw_study.analysis.del_analysis import (
    rainflow, del_from_cycles, compute_del_from_outb, CHANNEL_M,
)


def test_rainflow_sine():
    """0.5Hz 正弦 10s：5 个满循环 range=2000, mean=500。"""
    t = np.linspace(0, 10, 1001)
    sin = 1000.0 * np.sin(2 * np.pi * 0.5 * t) + 500.0
    cyc = rainflow(sin)
    assert len(cyc) == 5
    for rng, mean in cyc:
        assert abs(rng - 2000.0) < 1.0
        assert abs(mean - 500.0) < 1.0


def test_rainflow_flat():
    """常数序列 -> 无循环。"""
    assert rainflow(np.full(50, 7.0)).__len__() == 0


def test_del_scales_with_n():
    """相同循环谱，样本数翻倍 -> DEL 按 n^(1/m) 增长。"""
    cyc1 = [(1000.0, 0.0)] * 10
    cyc2 = [(1000.0, 0.0)] * 20
    d1 = del_from_cycles(cyc1, m=4, n_eq=10)
    d2 = del_from_cycles(cyc2, m=4, n_eq=10)
    assert abs(d2 / d1 - 2.0 ** (1.0 / 4.0)) < 1e-9


def test_del_channel_slopes():
    """通道 -> S-N 斜率映射完整覆盖已注入载荷通道。"""
    for ch in ("RootMIP1", "RootMOoP1", "RootMzb1", "TwrBsMyt", "TwrBsMxt",
               "TwrBsFxt", "LSSGagMxa", "LSSGagFxa", "YawBrMyp", "YawBrFxn"):
        assert any(k in ch for k in CHANNEL_M)
