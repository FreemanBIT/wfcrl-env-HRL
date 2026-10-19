#!/usr/bin/env python3
"""test_icd_v15_abi_python.py — Python↔C 协议镜像与 ICD v1.5 语义测试

覆盖（对应任务计划 T-04 / T-05 / T-12 的 Python 侧）：
  * 命令帧尺寸/字段往返（S0101-S0112）
  * 状态帧尺寸与解析（S1001-S1017）
  * yaw 增量(CW+) → 绝对目标(CCW+) 换算与 C 侧 yaw_adapter 语义一致
  * ZMQ 客户端不再硬编码 v1.4 帧几何

运行（需要项目 venv，因为 wfcrl.transport 依赖 numpy/zmq）：
    .venv\\Scripts\\python.exe farm_control_runtime\\tests\\integration\\test_icd_v15_abi_python.py
或 pytest：
    .venv\\Scripts\\python.exe -m pytest farm_control_runtime\\tests\\integration\\test_icd_v15_abi_python.py
"""
from __future__ import annotations

import ctypes
import math
import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from wfcrl.transport import farm_protocol as fp  # noqa: E402

DEG = math.pi / 180.0


def test_command_frame_roundtrip():
    """S0101-S0112 全字段 Python→bytes→Python 往返一致"""
    fr = fp.new_command_frame(3, 42, receive_time_s=1.5)
    assert fr.protocol_version == fp.FCR_PROTOCOL_VERSION == 2
    assert fr.command_frame_seq == 42
    assert fr.n_turbines == 3
    assert abs(fr.receive_time_s - 1.5) < 1e-12

    t = fr.turbines[1]
    t.control_enable = 1
    t.control_mode = fp.FCR_CONTROL_MODE_PRC
    t.requested_effective_sim_time_s = 12.0
    t.command_basis_sim_time_s = 9.0
    t.yaw_valid = 1
    t.yaw_seq = 7
    t.yaw_target_heading_rad = -10.0 * DEG
    t.yaw_ttl_s = 60.0
    t.induction_valid = 1
    t.induction_seq = 5
    t.induction_ref = 0.2
    t.induction_ttl_s = 5.0

    raw = fp.frame_to_bytes(fr)
    assert len(raw) == fp.FCR_COMMAND_FRAME_SIZE == 5664

    back = fp.bytes_to_frame(raw)
    b = back.turbines[1]
    assert back.command_frame_seq == 42
    assert b.control_enable == 1
    assert b.control_mode == fp.FCR_CONTROL_MODE_PRC
    assert abs(b.requested_effective_sim_time_s - 12.0) < 1e-12
    assert abs(b.command_basis_sim_time_s - 9.0) < 1e-12
    assert b.yaw_valid == 1 and b.yaw_seq == 7
    assert abs(b.yaw_target_heading_rad - (-10.0 * DEG)) < 1e-15
    assert abs(b.yaw_ttl_s - 60.0) < 1e-12
    assert b.induction_valid == 1 and b.induction_seq == 5
    assert abs(b.induction_ref - 0.2) < 1e-15
    assert abs(b.induction_ttl_s - 5.0) < 1e-12


def test_command_frame_has_no_legacy_fields():
    """ICD v1.5 已删除 yaw_delta_rad / commit_seq / frame_seq(命令侧)"""
    fr = fp.new_command_frame(1, 1)
    for legacy in ("yaw_delta_rad",):
        assert not hasattr(fr.turbines[0], legacy), legacy
    for legacy in ("commit_seq", "frame_seq"):
        assert not hasattr(fr, legacy), legacy
    assert hasattr(fr, "command_frame_seq")


def test_state_frame_roundtrip():
    """S1001-S1017 状态帧构造→序列化→解析一致"""
    st = fp.FarmStateFrame()
    st.protocol_version = fp.FCR_PROTOCOL_VERSION
    st.state_frame_seq = 11
    st.state_sample_sim_time_s = 12.5
    st.fast_step = 1250
    st.low_step = 4
    st.rt_health_flags = 1
    st.n_turbines = 2
    st.flow_source_time_s = 12.0
    st.flow_seq = 7

    t = st.turbines[0]
    t.turbine_id = 1
    t.valid = 1
    t.source_time_s = 12.5
    t.gen_power_w.instant = 4.05e6
    t.gen_power_w.mean = 4.0e6
    t.rotor_aero_thrust_n.instant = 4.5e5
    t.nacelle_heading_actual_rad = 0.1
    t.estimated_wind_mps = 8.1
    t.wind_source = 1
    t.wind_quality_flags = 1
    t.ct_hat = 0.78
    t.yaw_seq_applied = 3
    t.induction_seq_applied = 2
    t.turbine_status_flags = 0x0003
    t.flow_source_time_s = 12.0
    t.flow_seq = 7

    raw = bytes(st)
    assert len(raw) == fp.FCR_STATE_FRAME_SIZE == 25664

    back = fp.state_frame_from_bytes(raw)
    assert back.state_frame_seq == 11
    assert abs(back.state_sample_sim_time_s - 12.5) < 1e-12
    assert back.n_turbines == 2
    assert back.flow_seq == 7
    b = back.turbines[0]
    assert b.turbine_id == 1 and b.valid == 1
    assert abs(b.gen_power_w.instant - 4.05e6) < 1e-3
    assert abs(b.rotor_aero_thrust_n.instant - 4.5e5) < 1e-3
    assert abs(b.nacelle_heading_actual_rad - 0.1) < 1e-12
    assert abs(b.estimated_wind_mps - 8.1) < 1e-12
    assert b.wind_source == 1 and b.wind_quality_flags == 1
    assert abs(b.ct_hat - 0.78) < 1e-12
    assert b.yaw_seq_applied == 3 and b.induction_seq_applied == 2
    assert b.turbine_status_flags == 0x0003

    # 尺寸不符必须显式报错（不得按错误偏移解析）
    try:
        fp.state_frame_from_bytes(raw[:-1])
    except ValueError:
        pass
    else:
        raise AssertionError("size mismatch must raise ValueError")
    assert fp.state_frame_from_bytes_lenient(raw[:-1]) is None


def test_yaw_delta_to_absolute_matches_c_semantics():
    """CW+ 增量 → CCW+ 绝对目标，语义与 bridge/yaw_adapter.c 一致：
       target = wrap_pm_pi(heading_now - delta)
    """
    cases = [
        (0.0, 5.0 * DEG, -5.0 * DEG),
        (0.0, -5.0 * DEG, 5.0 * DEG),
        (170.0 * DEG, 30.0 * DEG, 140.0 * DEG),
        # 机舱 -170°，+30° CW → -200° → wrap → +160°
        (-170.0 * DEG, 30.0 * DEG, 160.0 * DEG),
    ]
    for heading, delta, expect in cases:
        got = fp.yaw_delta_cw_to_absolute_ccw(delta, heading)
        assert abs(fp.wrap_pm_pi(got - expect)) < 1e-12, (heading, delta, got, expect)
        # 逆换算自洽
        back = fp.yaw_absolute_ccw_to_delta_cw(got, heading)
        assert abs(fp.wrap_pm_pi(back - delta)) < 1e-12, (heading, delta, back)

    # 结果必须落在 (-pi, pi]
    for heading in (-4 * math.pi, -math.pi, 0.0, math.pi, 4 * math.pi):
        for delta in (-3 * math.pi, -1.0, 0.0, 1.0, 3 * math.pi):
            v = fp.yaw_delta_cw_to_absolute_ccw(delta, heading)
            assert -math.pi < v <= math.pi, (heading, delta, v)


def test_zmq_client_frame_geometry_not_hardcoded():
    """ZMQ 客户端必须从镜像派生帧几何（v1.4 的 56/360/23096 曾丢弃全部状态帧）"""
    from wfcrl.transport.farm_zmq_client import FarmZmqClient
    assert FarmZmqClient._FRAME_HEADER == 64 == fp.FCR_STATE_FRAME_HEADER_SIZE
    assert FarmZmqClient._TURBINE_SIZE == 400 == fp.FCR_TURBINE_STATE_SIZE
    assert FarmZmqClient._FRAME_SIZE == 25664 == fp.FCR_STATE_FRAME_SIZE
    assert FarmZmqClient._N_TURBINES_OFF == 44
    assert FarmZmqClient._HEADING_OFF == 232


def test_yaw_delta_uses_latest_heading():
    """send_yaw_delta 必须把 CW+ 增量换算为绝对目标（而非直传）"""
    from wfcrl.transport.farm_zmq_client import FarmZmqClient

    c = FarmZmqClient.__new__(FarmZmqClient)     # 不启动 socket
    sent = []
    c._cmd_sock = type("S", (), {"send": staticmethod(lambda b: sent.append(b))})()
    c._started = True
    c._n_turbines = 1
    c._frame_seq = 0
    c._yaw_seq = 0
    c._ind_seq = 0
    c._last_heading = {}

    # 尚未收到状态：按机舱初始 0° 处理 → +10° CW 应下发 -10°
    c.send_yaw_delta(1, 10.0 * DEG)
    fr = fp.bytes_to_frame(sent[-1])
    assert abs(fr.turbines[0].yaw_target_heading_rad - (-10.0 * DEG)) < 1e-12
    assert fr.turbines[0].yaw_valid == 1
    assert fr.command_frame_seq == 1

    # 已收到机舱 170°：+30° CW → 140°
    c._last_heading[1] = 170.0 * DEG
    c.send_yaw_delta(1, 30.0 * DEG)
    fr = fp.bytes_to_frame(sent[-1])
    assert abs(fr.turbines[0].yaw_target_heading_rad - 140.0 * DEG) < 1e-12

    # 绝对目标通道直传
    c.send_yaw_target(1, -5.0 * DEG)
    fr = fp.bytes_to_frame(sent[-1])
    assert abs(fr.turbines[0].yaw_target_heading_rad - (-5.0 * DEG)) < 1e-12


def _main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"  [PASS] {fn.__name__}")
        except Exception as e:                                   # noqa: BLE001
            failed += 1
            print(f"  [FAIL] {fn.__name__}: {type(e).__name__}: {e}")
    print(f"== python protocol tests: {len(tests) - failed}/{len(tests)} passed ==")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_main())
