"""conftest.py — 离线端到端集成测试共享 fixture（Phase 8）"""
import os
import shutil
import sys
import time

import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from wfcrl.config import WindConfig
from wfcrl.config.layout import LayoutRegistry
from wfcrl.config.simulator import FastFarmConfig
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine

ZMQ_DLL = os.path.join(REPO, "farm_control_runtime", "transport", "zmq", "bin", "libzmq.dll")
RUN_ROOT = os.path.join(REPO, "__simul__", "fastfarm", "p8_tests")

# 每个测试使用独立端口（避免 Windows 端口复用/残留冲突）
_port_counter = [24000]


def _next_ports():
    _port_counter[0] += 2
    return _port_counter[0], _port_counter[0] + 1


def _unlock_yawdoff(eng):
    """解锁 YawDOF（Phase 3 惯例：DLL 偏航命令经 ServoDyn YCMode=5 生效）"""
    import glob
    from wfcrl.engine._outlist import _set_fast_scalar
    for ed in glob.glob(os.path.join(eng._ff._farm_base, "*ElastoDyn*.dat")):
        try:
            _set_fast_scalar(ed, "YawDOF", "True")
        except Exception:
            pass


@pytest.fixture(scope="function")
def farm(request):
    """启动一次 FAST.Farm + ZMQ 主闭环；测试结束自动停止。"""
    name = request.node.name.replace("/", "_").replace("::", "_")
    out_root = os.path.join(RUN_ROOT, name)
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
    eng.start()
    # 等待首帧
    st = None
    t0 = time.time()
    while time.time() - t0 < 30:
        st = eng.get_state(timeout_s=2.0)
        if st is not None and st.get("fast_step", 0) > 0:
            break
        time.sleep(0.5)
    assert st is not None, "FAST.Farm 未在 30s 内发送状态帧"
    yield eng, st
    eng.stop()
    os.environ.pop("FCR_ZMQ_CMD_PORT", None)
    os.environ.pop("FCR_ZMQ_STATE_PORT", None)
    os.environ.pop("FCR_DIAG_FILE", None)
    os.environ.pop("FCR_CFG_DT_LOW", None)


def wait_seq_applied(eng, turbine_id, channel, seq, timeout_s=30):
    """等待某机某通道 seq 生效"""
    key = f"{channel}_seq_applied"
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        st = eng.get_state(timeout_s=2.0)
        if st and st["turbines"][turbine_id - 1].get(key) == seq:
            return st
        time.sleep(0.5)
    return None


def load_trace(path):
    """读取 command trace csv: time_rel,turbine_id,channel,seq,value"""
    import csv
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({"time_rel": float(r["time_rel"]),
                        "turbine_id": int(r["turbine_id"]),
                        "channel": r["channel"].strip(),
                        "seq": int(r["seq"]),
                        "value": float(r["value"])})
    return rows


def replay_trace(eng, rows, start_sim_time=0.0):
    """按 trace 时间（仿真秒）发送命令：以状态帧 sim_time 为时钟源。"""
    import math
    import threading
    stop = threading.Event()

    def _worker():
        last = 0
        while not stop.is_set():
            try:
                st = eng.get_state(timeout_s=2.0)
            except RuntimeError:
                return
            if st is None:
                time.sleep(0.2)
                continue
            now = st["sim_time_s"] - start_sim_time
            for r in rows[last:]:
                if now >= r["time_rel"]:
                    if r["channel"] == "yaw":
                        eng.send_yaw_delta(r["turbine_id"], math.radians(r["value"]), seq=r["seq"], ttl_s=60.0)
                    elif r["channel"] == "induction":
                        eng.send_induction(r["turbine_id"], r["value"], seq=r["seq"], ttl_s=30.0)
                    last += 1
                else:
                    break
            time.sleep(0.3)

    th = threading.Thread(target=_worker, daemon=True)
    th.start()
    return th, stop

__all__ = ["farm", "wait_seq_applied", "load_trace", "replay_trace", "REPO", "RUN_ROOT"]