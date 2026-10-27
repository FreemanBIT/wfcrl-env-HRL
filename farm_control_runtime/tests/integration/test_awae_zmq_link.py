#!/usr/bin/env python3
"""test_awae_zmq_link.py — W-08 集成测试：ZMQ 侧 AWAE 二跳发布（GG-12/GG-13）

验证 Scheduler → RCP 的 AWAE 观测/目录帧真正到达 Python RCP：
  * 离线环境下由 Provider 注入合成测点（FCR_CFG_AWAE_SYNTH_POINTS=N>0，
    带 FCR_AWAE_QUALITY_SIMULATED 质量位，可追溯，用户指示"不删字段"）；
  * runtime step_low 组装 AWAE 观测/目录帧并经 ZMQ 发布；
  * FarmZmqClient 按冻结尺寸分派并缓存，get_awae_* 返回原始字节；
  * 断言内容：观测点数>0、quality_flags 含 OK|SIMULATED、目录点数>0；
  * G-4：S0502 observation_sample_sim_time_s 必须是仿真时刻（0..Tsim），
    不得被墙钟替换（墙钟 ~1.77e9 会立即暴露）。

运行（需项目 venv + FAST.Farm + 重建后的 DLL，逐文件避免 libzmq teardown abort）：
    .venv\\Scripts\\python.exe -m pytest farm_control_runtime\\tests\\integration\\test_awae_zmq_link.py -q
"""
from __future__ import annotations

import os
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from wfcrl.transport import farm_protocol as fp  # noqa: E402
from wfcrl.transport.farm_zmq_client import FarmZmqClient  # noqa: E402

from conftest import _next_ports, _unlock_yawdoff, REPO as _REPO  # noqa: E402
from conftest import ZMQ_DLL, RUN_ROOT  # noqa: E402

import shutil  # noqa: E402

from wfcrl.config import WindConfig  # noqa: E402
from wfcrl.config.layout import LayoutRegistry  # noqa: E402
from wfcrl.config.simulator import FastFarmConfig  # noqa: E402
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine  # noqa: E402

# 合成测点数：>0 才注入（用户指示保留接口 + 外部注入验证发布链路）
AWAE_POINTS = 8


def _start_engine():
    """启动一次 FAST.Farm + ZMQ 主闭环，开启合成 AWAE 测点注入。"""
    out_root = os.path.join(RUN_ROOT, "test_awae_zmq_link")
    os.makedirs(out_root, exist_ok=True)
    reg = LayoutRegistry.from_builtin()
    layout = reg.get("3T")
    wind = WindConfig(speed=8.0, direction=270.0)
    cfg = FastFarmConfig(case_name=layout.name, num_turbines=3,
                         xcoords=layout.xcoords, ycoords=layout.ycoords,
                         dt=3.0, max_iter=30, wind=wind, output_dir=out_root)
    cmd_port, state_port = _next_ports()
    eng = FastFarmZmqEngine(cfg, cmd_port=cmd_port, state_port=state_port)
    eng.setup()
    _unlock_yawdoff(eng)
    shutil.copy(ZMQ_DLL, os.path.join(eng._ff._farm_base, "libzmq.dll"))
    os.environ["FCR_ZMQ_CMD_PORT"] = str(cmd_port)
    os.environ["FCR_ZMQ_STATE_PORT"] = str(state_port)
    os.environ["FCR_CFG_DT_LOW"] = "3.0"
    os.environ["FCR_DIAG_FILE"] = os.path.join(out_root, "zmq_diag.log")
    # W-08：开启 Provider 合成 AWAE 测点注入（默认关闭，见 offline_fastfarm_provider.c）
    os.environ["FCR_CFG_AWAE_SYNTH_POINTS"] = str(AWAE_POINTS)
    eng.start()
    return eng


def test_awae_zmq_link_reaches_python_rcp():
    """W-08：AWAE 观测/目录帧经 ZMQ 二跳到达 Python RCP，内容与 G-4 正确。"""
    eng = None
    try:
        eng = _start_engine()
        client: FarmZmqClient = eng._client

        # 等待状态帧首帧（确认闭环已起）
        st = None
        t0 = time.time()
        while time.time() - t0 < 60:
            st = eng.get_state(timeout_s=2.0)
            if st is not None and st.get("fast_step", 0) > 0:
                break
            time.sleep(0.5)
        assert st is not None, "FAST.Farm 未在 60s 内发送状态帧"

        # 等待 AWAE 观测帧（GG-12：S1201-S1207）
        obs_raw = None
        t0 = time.time()
        while time.time() - t0 < 60:
            obs_raw = client.get_awae_observation(timeout_s=2.0)
            if obs_raw is not None:
                break
            time.sleep(0.5)
        assert obs_raw is not None, "60s 内未收到 AWAE 观测帧（检查 FCR_CFG_AWAE_SYNTH_POINTS 与 DLL）"
        assert len(obs_raw) == fp.FCR_AWAE_OBSERVATION_FRAME_SIZE == 48024

        obs = fp.awae_observation_frame_from_bytes(obs_raw)
        assert obs.protocol_version == fp.FCR_PROTOCOL_VERSION == 2
        assert obs.n_observations > 0, "合成测点注入应产生 >0 观测"
        o0 = obs.observations[0]
        # G-4：S0502 源仿真时刻，绝不可能是墙钟（~1.77e9）
        assert 0.0 < o0.observation_sample_sim_time_s < 1.0e6, (
            f"S0502 疑似墙钟泄漏: {o0.observation_sample_sim_time_s}"
        )
        # 质量位：合成数据必须带 SIMULATED（可追溯，用户指示）
        assert o0.observation_quality_flags & fp.FCR_AWAE_QUALITY_OK
        assert o0.observation_quality_flags & fp.FCR_AWAE_QUALITY_SIMULATED
        # 观测点 ID 非空（目录缺失时退化为 index+1）
        assert o0.point_id >= 1
        # wind_uvw 非零（合成场有数据）
        assert o0.wind_uvw_mps[0] > 0.0

        # 等待 AWAE 目录帧（GG-13：S1301-S1303）
        cat_raw = None
        t0 = time.time()
        while time.time() - t0 < 60:
            cat_raw = client.get_awae_catalog(timeout_s=2.0)
            if cat_raw is not None:
                break
            time.sleep(0.5)
        assert cat_raw is not None, "60s 内未收到 AWAE 目录帧"
        assert len(cat_raw) == fp.FCR_AWAE_CATALOG_FRAME_SIZE == 40024

        cat = fp.awae_catalog_frame_from_bytes(cat_raw)
        assert cat.protocol_version == fp.FCR_PROTOCOL_VERSION == 2
        assert cat.n_points > 0, "目录注入应产生 >0 测点"
        assert cat.entries[0].point_id >= 1
        # S0402 测点坐标非零（合成点均匀布置在半径 300m 圆上）
        assert abs(cat.entries[0].point_position_xyz_m[0]) > 1.0 or \
            abs(cat.entries[0].point_position_xyz_m[1]) > 1.0

        # 观测点 ID 与目录一致（同一注入目录）
        assert obs.observations[0].point_id == cat.entries[0].point_id
    finally:
        if eng is not None:
            eng.stop()
        for k in ("FCR_ZMQ_CMD_PORT", "FCR_ZMQ_STATE_PORT", "FCR_DIAG_FILE",
                  "FCR_CFG_DT_LOW", "FCR_CFG_AWAE_SYNTH_POINTS"):
            os.environ.pop(k, None)
