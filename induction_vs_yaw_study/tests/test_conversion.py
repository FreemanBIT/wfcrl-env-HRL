"""
test_conversion.py — Stage 1 转换层单元测试
============================================
运行：pytest induction_vs_yaw_study/tests/test_conversion.py -v
（需要能 import wfcrl；若仅测纯数学，可单独跑 _selftest() 见文件末尾。）
"""

from __future__ import annotations

import numpy as np
import pytest

from induction_vs_yaw_study import constants as C
from induction_vs_yaw_study.conversion.power_setpoint_tools import (
    cp_of_a, ct_of_a, a_of_ct,
    induction_from_power, power_from_induction, ct_from_power,
    power_mw_to_ratio, ratio_to_power_mw, greedy_power_w,
    make_setpoint, build_yaw_control, build_derating_control,
)


# ---- 制动盘基本关系 ----

def test_betz_point():
    a = C.A_GREEDY
    assert cp_of_a(a) == pytest.approx(16.0 / 27.0, rel=1e-9)
    assert ct_of_a(a) == pytest.approx(8.0 / 9.0, rel=1e-9)


def test_ct_roundtrip():
    for a in np.linspace(0.05, 0.45, 20):
        assert a_of_ct(ct_of_a(a)) == pytest.approx(a, abs=1e-6)


# ---- 功率 ⇄ 诱导 往返 ----

@pytest.mark.parametrize("U", [6.0, 8.0, 10.0, 11.0])
def test_power_induction_roundtrip(U):
    for a in np.linspace(0.05, C.A_GREEDY, 15):
        p = power_from_induction(a, U)
        a_back = induction_from_power(p, U)
        assert a_back == pytest.approx(a, abs=1e-3)


def test_induction_monotonic_in_power():
    U = 8.0
    p_grid = np.linspace(0.2e6, greedy_power_w(U) * 0.999, 25)
    a_vals = [induction_from_power(p, U) for p in p_grid]
    assert np.all(np.diff(a_vals) >= -1e-9)  # 单调不减


def test_ct_monotonic_in_power():
    U = 8.0
    p_grid = np.linspace(0.2e6, greedy_power_w(U) * 0.999, 25)
    ct_vals = [ct_from_power(p, U) for p in p_grid]
    assert np.all(np.diff(ct_vals) >= -1e-9)


# ---- 功率(MW) ⇄ ratio 往返 ----

@pytest.mark.parametrize("U", [6.0, 8.0, 10.0])
def test_power_ratio_roundtrip(U):
    pg = greedy_power_w(U)
    for ratio in np.linspace(0.1, 1.0, 10):
        p_mw = ratio_to_power_mw(ratio, U, greedy_power_w_value=pg)
        r_back = power_mw_to_ratio(p_mw, U, greedy_power_w_value=pg)
        assert r_back == pytest.approx(ratio, abs=1e-6)


def test_ratio_clipped():
    U = 8.0
    # 超过贪婪功率 → ratio 封顶 1.0
    big = greedy_power_w(U) / 1e6 * 2.0
    assert power_mw_to_ratio(big, U) == pytest.approx(1.0)
    # 极低功率 → ratio 下限 0.01
    assert power_mw_to_ratio(1e-6, U) == pytest.approx(0.01)


# ---- Setpoint 构造 ----

def test_make_setpoint_diagnostics():
    U = 8.0
    pg = greedy_power_w(U)
    sp = make_setpoint(pg / 1e6, U, greedy_power_w_value=pg)
    # 贪婪点：ratio≈1，a≈1/3
    assert sp.ratio == pytest.approx(1.0, abs=1e-6)
    assert sp.induction == pytest.approx(C.A_GREEDY, abs=2e-3)


# ---- ControlInput 单一来源构造 ----

def test_build_yaw_control():
    ci = build_yaw_control([10.0, -5.0, 0.0])
    assert np.all(ci.mode == 0)
    assert ci.yaw.tolist() == [10.0, -5.0, 0.0]


def test_build_derating_fastfarm_is_absolute_mw():
    U = 8.0
    p_mw = [3.0, 4.0, 5.0]
    ci = build_derating_control("fastfarm", power_target_mw=p_mw, wind_speed_ms=U)
    assert np.all(ci.mode == 1)
    # FAST.Farm: power 字段就是绝对 MW
    assert ci.power.tolist() == pytest.approx(p_mw)


def test_build_derating_floris_is_ratio():
    U = 8.0
    pg = greedy_power_w(U)
    p_mw = [pg / 1e6 * 0.5, pg / 1e6 * 0.8, pg / 1e6]
    ci = build_derating_control(
        "floris", power_target_mw=p_mw, wind_speed_ms=U,
        greedy_power_w_per_turbine=[pg, pg, pg],
    )
    assert np.all(ci.mode == 1)
    # FLORIS: power 字段是 ratio = P/Pg
    assert ci.power.tolist() == pytest.approx([0.5, 0.8, 1.0], abs=1e-6)


def test_build_derating_with_yaw_is_mode4():
    U = 8.0
    ci = build_derating_control(
        "fastfarm", power_target_mw=[3.0, 4.0, 5.0],
        wind_speed_ms=U, yaw_deg=[10.0, 0.0, 0.0],
    )
    assert np.all(ci.mode == 4)
    assert ci.yaw.tolist() == [10.0, 0.0, 0.0]


def test_floris_fastfarm_same_physical_point():
    """同一个绝对功率，FAST.Farm 与 FLORIS 控制输入应代表同一物理工作点。"""
    U = 8.0
    pg = greedy_power_w(U)
    p_target_mw = pg / 1e6 * 0.6  # 降到贪婪的 60%

    ff = build_derating_control("fastfarm", power_target_mw=[p_target_mw],
                                wind_speed_ms=U)
    fl = build_derating_control("floris", power_target_mw=[p_target_mw],
                                wind_speed_ms=U, greedy_power_w_per_turbine=[pg])
    # FAST.Farm 端绝对 MW
    assert ff.power[0] == pytest.approx(p_target_mw)
    # FLORIS 端 ratio，换算回 MW 应一致
    assert ratio_to_power_mw(fl.power[0], U, greedy_power_w_value=pg) == pytest.approx(
        p_target_mw, rel=1e-6)


if __name__ == "__main__":
    # 允许不装 pytest 时快速自检纯数学部分
    import sys
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            # 跳过需要参数的 parametrized 用例
            import inspect
            if inspect.signature(fn).parameters:
                continue
            fn()
            print(f"PASS {fn.__name__}")
        except Exception as e:  # noqa
            failed += 1
            print(f"FAIL {fn.__name__}: {e}")
    sys.exit(1 if failed else 0)
