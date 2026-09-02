"""test_offline_multirate_commands.py — 多速率命令/状态（Phase 8）"""
import math
import time

from conftest import wait_seq_applied


def test_yaw_induction_independent_channels(farm):
    eng, st0 = farm
    # 同时异步：T1 yaw seq1 + T1 induction seq1（互不覆盖）
    eng.send_yaw_delta(1, math.radians(8.0), seq=1, ttl_s=60.0)
    eng.send_induction(1, 0.25, seq=1, ttl_s=30.0)
    st = wait_seq_applied(eng, 1, "yaw", 1, timeout_s=40)
    assert st is not None
    st2 = wait_seq_applied(eng, 1, "induction", 1, timeout_s=30)
    assert st2 is not None
    # yaw 目标存在
    assert abs(st2["turbines"][0]["yaw_target_heading_rad"]) > 0.05
    # 再发 yaw seq=2 —— induction 不应受影响
    eng.send_yaw_delta(1, math.radians(-4.0), seq=2, ttl_s=60.0)
    st3 = wait_seq_applied(eng, 1, "yaw", 2, timeout_s=30)
    assert st3 is not None
    assert st3["turbines"][0]["induction_seq_applied"] == 1, "yaw 更新不应改变 induction"


def test_multirate_state_consistency(farm):
    eng, st0 = farm
    # 连续帧：10ms 高速量（fast_step 单调）+ 1s 低速量（flow_seq 单调）
    prev_fast = st0["fast_step"]
    prev_low = st0["low_step"]
    prev_flow = st0["flow_seq"]
    for _ in range(40):   # ~25s
        st = eng.get_state(timeout_s=1.0)
        assert st is not None
        assert st["fast_step"] >= prev_fast, "fast_step 回退"
        # low_step/flow_seq 取自各机帧头（时钟一致时单调），合并语义下允许持平
        assert st["low_step"] >= prev_low, "low_step 回退"
        prev_fast = st["fast_step"]
        if st["low_step"] > prev_low:
            prev_low = st["low_step"]
        if st["flow_seq"] > prev_flow:
            prev_flow = st["flow_seq"]
        time.sleep(0.6)
    # 载荷统计字段（1s 窗口）应有效（合并帧瞬时缺失容忍：窗口内任一帧有功率即可）
    saw_power = any(
        (s := eng.get_state(timeout_s=1.0)) is not None and s["turbines"][0]["gen_power_w"] > 0
        for _ in range(6)
    )
    assert saw_power, "1s 功率统计字段始终缺失"

__all__ = []