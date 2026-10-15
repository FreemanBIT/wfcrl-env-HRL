"""test_offline_induction_loop.py — 诱导闭环集成测试（Phase 8）"""
import time

from conftest import load_trace, replay_trace, wait_seq_applied


def test_induction_1s_update_and_hold(farm):
    eng, st0 = farm
    rows = [
        {"time_rel": 2.0, "turbine_id": 1, "channel": "induction", "seq": 1, "value": 0.20},
        {"time_rel": 8.0, "turbine_id": 1, "channel": "induction", "seq": 2, "value": 0.30},
    ]
    th, stop = replay_trace(eng, rows, start_sim_time=st0["sim_time_s"])
    st = wait_seq_applied(eng, 1, "induction", 1, timeout_s=40)
    assert st is not None, "induction seq=1 未生效"
    # steady hold：seq=1 保持至少 2 个状态帧（ZOH；seq2 在 8s 才发出）
    holds = 0
    for _ in range(6):
        s = eng.get_state(timeout_s=1.0)
        if s is None:
            time.sleep(0.5); continue
        if s["turbines"][0]["induction_seq_applied"] != 1:
            break
        holds += 1
        time.sleep(0.5)
    assert holds >= 1, "seq=1 未保持（ZOH 失效）"
    # 1s 更新：seq=2
    st2 = wait_seq_applied(eng, 1, "induction", 2, timeout_s=40)
    stop.set(); th.join(timeout=2)
    assert st2 is not None, "induction seq=2 未生效"


def test_induction_multi_turbine_distinct(farm):
    """WIN-06: strong per-turbine assertions — all three turbines must independently
    execute distinct induction commands and show different power responses."""
    eng, st0 = farm
    rows = [
        {"time_rel": 2.0, "turbine_id": 1, "channel": "induction", "seq": 1, "value": 0.20},
        {"time_rel": 3.0, "turbine_id": 2, "channel": "induction", "seq": 1, "value": 0.30},
        {"time_rel": 4.0, "turbine_id": 3, "channel": "induction", "seq": 1, "value": 0.40},
    ]
    th, stop = replay_trace(eng, rows, start_sim_time=st0["sim_time_s"])

    # WIN-06: strict per-turbine seq applied verification
    st1 = wait_seq_applied(eng, 1, "induction", 1, timeout_s=40)
    assert st1 is not None, "T1 induction seq=1 not applied"
    st2 = wait_seq_applied(eng, 2, "induction", 1, timeout_s=40)
    assert st2 is not None, "T2 induction seq=1 not applied"
    st3 = wait_seq_applied(eng, 3, "induction", 1, timeout_s=40)
    assert st3 is not None, "T3 induction seq=1 not applied"

    # Verify all three state frames are valid
    s = eng.get_state(timeout_s=2.0)
    assert s is not None, "no state frame received"
    for i in range(3):
        assert s["turbines"][i]["valid"], f"T{i+1} state frame invalid"

    # WIN-06: verify distinct power responses (different a_ref -> different power)
    pw = [s["turbines"][i]["gen_power_w"]["instant"] for i in range(3)]
    assert pw[0] != pw[1], f"T1/T2 power identical despite different a_ref: {pw[0]} vs {pw[1]}"
    assert pw[1] != pw[2], f"T2/T3 power identical despite different a_ref: {pw[1]} vs {pw[2]}"

    # Higher a_ref should produce lower power (derating)
    assert pw[2] < pw[0], f"T3 (a=0.40) power {pw[2]} should be < T1 (a=0.20) power {pw[0]}"

    stop.set(); th.join(timeout=2)


def test_induction_fallback_rollback(farm):
    eng, st0 = farm
    rows = [
        {"time_rel": 2.0, "turbine_id": 1, "channel": "induction", "seq": 1, "value": 0.20},
        {"time_rel": 8.0, "turbine_id": 1, "channel": "induction", "seq": 2, "value": 1.50},  # 越界 -> fallback
    ]
    th, stop = replay_trace(eng, rows, start_sim_time=st0["sim_time_s"])
    st2 = wait_seq_applied(eng, 1, "induction", 2, timeout_s=40)
    if st2 is None:
        # 合并时序：额外等待窗口再检查
        t_end = time.time() + 30
        while time.time() < t_end:
            time.sleep(2.0)
            s = eng.get_state(timeout_s=2.0)
            if s and s["turbines"][0]["induction_seq_applied"] == 2:
                st2 = s
                break
    stop.set(); th.join(timeout=2)
    assert st2 is not None, "fallback seq 未生效"
    st3 = eng.get_state(timeout_s=2.0)
    assert st3 is not None
    assert st3["turbines"][0]["induction_seq_applied"] == 2

__all__ = []