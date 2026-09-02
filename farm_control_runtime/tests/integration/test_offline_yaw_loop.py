"""test_offline_yaw_loop.py — 偏航闭环集成测试（Phase 8）"""
import math
import time

from conftest import load_trace, replay_trace, wait_seq_applied, REPO


def test_yaw_plus5_cw_direction(farm):
    eng, st0 = farm
    rows = [{"time_rel": 2.0, "turbine_id": 1, "channel": "yaw", "seq": 1, "value": 5.0}]
    th, stop = replay_trace(eng, rows, start_sim_time=st0["sim_time_s"])
    st = wait_seq_applied(eng, 1, "yaw", 1, timeout_s=40)
    stop.set(); th.join(timeout=2)
    assert st is not None, "yaw seq=1 未生效"
    assert abs(st["turbines"][0]["yaw_target_heading_rad"]) > 0.05, "目标未被锁存"
    assert st["turbines"][0]["yaw_target_heading_rad"] < 0, "+5°CW 目标应为负（内部约定）"


def test_yaw_minus5_cw_direction(farm):
    eng, st0 = farm
    rows = [{"time_rel": 2.0, "turbine_id": 1, "channel": "yaw", "seq": 1, "value": -5.0}]
    th, stop = replay_trace(eng, rows, start_sim_time=st0["sim_time_s"])
    st = wait_seq_applied(eng, 1, "yaw", 1, timeout_s=40)
    stop.set(); th.join(timeout=2)
    assert st is not None
    assert st["turbines"][0]["yaw_target_heading_rad"] > 0, "-5°CW 目标应为正（内部）"


def test_yaw_wrap_and_duplicate_seq(farm):
    eng, st0 = farm
    rows = [
        {"time_rel": 2.0, "turbine_id": 1, "channel": "yaw", "seq": 1, "value": 170.0},
        {"time_rel": 4.0, "turbine_id": 1, "channel": "yaw", "seq": 2, "value": 30.0},
        {"time_rel": 6.0, "turbine_id": 1, "channel": "yaw", "seq": 2, "value": 90.0},
    ]
    th, stop = replay_trace(eng, rows, start_sim_time=st0["sim_time_s"])
    st2 = wait_seq_applied(eng, 1, "yaw", 2, timeout_s=40)
    stop.set(); th.join(timeout=2)
    assert st2 is not None, "seq=2 未生效"
    t2 = st2["turbines"][0]["yaw_target_heading_rad"]
    assert -math.pi - 1e-9 <= t2 <= math.pi + 1e-9
    t3 = None
    for _ in range(40):
        st = eng.get_state(timeout_s=1.0)
        if st and st["turbines"][0]["yaw_seq_applied"] == 2:
            t3 = st["turbines"][0]["yaw_target_heading_rad"]
            if t3 != t2:
                break
        time.sleep(0.5)
    if t3 is not None:
        assert abs(t3 - t2) < 1e-9, "重复 seq 后目标发生变化（幂等失效）"


def test_yaw_target_fixed_during_execution(farm):
    eng, st0 = farm
    rows = [{"time_rel": 2.0, "turbine_id": 1, "channel": "yaw", "seq": 1, "value": 20.0}]
    th, stop = replay_trace(eng, rows, start_sim_time=st0["sim_time_s"])
    st = wait_seq_applied(eng, 1, "yaw", 1, timeout_s=40)
    stop.set(); th.join(timeout=2)
    assert st is not None
    t_fixed = st["turbines"][0]["yaw_target_heading_rad"]
    changed = False
    for _ in range(30):
        s2 = eng.get_state(timeout_s=1.0)
        if s2 and s2["turbines"][0]["yaw_seq_applied"] == 1:
            if s2["turbines"][0]["yaw_target_heading_rad"] != t_fixed:
                changed = True
                break
        time.sleep(0.5)
    assert not changed, "执行过程中目标发生变化（latch 失效）"

__all__ = []
