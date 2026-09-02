"""test_offline_provider_states.py — Provider 状态与 outb 校核（Phase 8）"""
import os

from conftest import REPO


def test_provider_fast_states_present(farm):
    eng, st0 = farm
    t = st0["turbines"][0]
    assert t["valid"] == 1
    assert t["rotor_thrust_n"] > 1e4, "thrust 缺失"
    assert t["hub_wind_speed_mps"] > 3.0
    assert t["rotor_speed_rad_s"] > 0.3


def test_provider_crosscheck_with_outb(farm):
    """Web 校核：StateFrame 的 hub wind 与 outb Wind1VelX 趋势一致。"""
    eng, st0 = farm
    import time
    samples = []
    for _ in range(8):   # ~24s
        st = eng.get_state(timeout_s=1.0)
        if st:
            samples.append((st["sim_time_s"], st["turbines"][0]["hub_wind_speed_mps"]))
        time.sleep(3.0)
    assert len(samples) >= 2
    # 结束仿真（仅本测试内提前停止，outb 在仿真结束后完整写出）
    eng.stop()
    from openfast_toolbox.io.fast_output_file import FASTOutputFile
    outb = os.path.join(os.path.dirname(eng._ff._fstf_file), "Case.T1.outb")
    if not os.path.exists(outb):
        outb = os.path.join(os.path.dirname(eng._ff._fstf_file), "Case.T1.out")
        assert os.path.exists(outb), f"输出文件不存在"
    df = FASTOutputFile(outb).toDataFrame()
    import numpy as np
    t_out = df["Time_[s]"].to_numpy()
    w_out = df["Wind1VelX_[m/s]"].to_numpy()
    # 对比：provider hub wind 与 outb 在同时刻应接近
    diffs = []
    for ts, w in samples:
        i = int(np.argmin(np.abs(t_out - ts)))
        if abs(t_out[i] - ts) < 1.0:
            diffs.append(abs(w - w_out[i]))
    assert len(diffs) >= 1
    mean_diff = float(np.mean(diffs))
    assert mean_diff < 3.0, f"provider hub wind 与 outb 偏差过大: {mean_diff:.2f} m/s"

__all__ = []