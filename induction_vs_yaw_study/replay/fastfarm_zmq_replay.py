"""fastfarm_zmq_replay.py — FAST.Farm 回放（ZMQ 驱动，REFACTOR_PLAN 附录 A）

替代旧文件桥驱动的 ZMQ 版回放：
  * 仿真：FastFarmZmqEngine（复用 closedloop.zmq_pipeline 的案例/风况配置）
  * 下发：偏航（绝对目标经桥换算 yaw_delta，动作指令语义）与诱导因子 a
    （LUT 的 power_target_mw -> ratio -> a = power_to_induction）
  * 偏航执行模式：
      yaw_mode="static"（默认）: 首个控制步下发一次，runtime 锁存保持（与旧
        "锁定 DOF 静态失准"的稳态等效，研究口径一致）
      yaw_mode="control"    : 每个控制步重发目标（ROSCO 偏航动态）
  * 测量：仿真结束后解析 .outb（保持研究统计口径，与旧结果可比）
"""
from __future__ import annotations

import os as _os
import sys as _sys
_ccc = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), '..', '..', 'closed-loop-control'))
if _ccc not in _sys.path:
    _sys.path.insert(0, _ccc)

from typing import Dict, Optional

import numpy as np

from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_fastfarm_config
from induction_vs_yaw_study.replay.schedule import control_inputs_for_recording

from wfcrl.config import WindConfig, SimulationOutput
from wfcrl.config.types import WindType
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine

from closedloop.types import Cmd
from closedloop.bridge import ZmqFarmBridge
from closedloop.induction import power_to_induction


def replay_fastfarm_zmq(
    case: Case,
    control_type: str,
    settings: Dict[int, Dict[str, float]],
    *,
    output_dir: str = None,
    wind_time_series: Optional[str] = None,
    yaw_mode: str = 'static',
    cmd_port: int = 24040,
    state_port: int = 24041,
):
    """在 FAST.Farm（ZMQ 链路）中回放某工况某控制类型。

    返回结构与旧 replay_fastfarm 兼容（SimulationOutput + yaw_cmd/ratio/power_target）。
    """
    import time as _time

    cfg = make_fastfarm_config(case, output_dir=output_dir)
    if wind_time_series:
        w0 = cfg.wind
        cfg.wind = WindConfig(speed=w0.speed, direction=w0.direction,
                              wind_type=WindType.TURBSIM_BTS, wind_file=wind_time_series)

    n_turbines = 3
    n_total = max(1, int(getattr(case, "fastfarm_n_control_steps", cfg.max_iter)))
    wind_dir = float(cfg.wind.direction)

    # ---- 命令构造（业务语义：绝对偏航 + 诱导因子）----
    cmds = {}
    if control_type == "derating":
        for tid in range(1, n_turbines + 1):
            st = settings.get(tid) or {}
            p_mw = st.get("power_target_mw", float("nan"))
            a = 1.0 / 3.0
            if np.isfinite(p_mw) and p_mw > 0:
                a = power_to_induction(p_mw * 1e6, case.wind_speed_ms, yaw_deg=0.0)
            yaw_mis = st.get("yaw_deg", 0.0) or 0.0
            abs_yaw = float(((270.0 - wind_dir) + yaw_mis) % 360.0)
            cmds[tid] = Cmd(turbine_id=tid, yaw_deg=abs_yaw, induction_a=a)
    elif control_type == "yaw":
        for tid in range(1, n_turbines + 1):
            st = settings.get(tid) or {}
            yaw_mis = st.get("yaw_deg", 0.0) or 0.0
            abs_yaw = float(((270.0 - wind_dir) + yaw_mis) % 360.0)
            cmds[tid] = Cmd(turbine_id=tid, yaw_deg=abs_yaw, induction_a=None)
    elif control_type != "baseline":
        raise ValueError(f"unknown control_type {control_type!r}")

    yaw_cmd, ratio, ptgt = control_inputs_for_recording(control_type, settings,
                                                      n_turbines=n_turbines)

    engine = FastFarmZmqEngine(cfg, cmd_port=cmd_port, state_port=state_port)
    engine.setup()
    engine.start()
    bridge = ZmqFarmBridge(engine, n_turbines, wind_direction=wind_dir)

    aborted = False
    final = None
    try:
        br_ok = False
        t0 = _time.time()
        while _time.time() - t0 < 90:
            st = bridge.get_state(timeout_s=5.0)
            if st is not None and st.get("fast_step", 0) > 0:
                br_ok = True
                break
        if not br_ok:
            raise TimeoutError("FAST.Farm 未在 90s 内发出状态帧")

        for k in range(1, n_total + 1):
            bridge.wait_step(k, timeout=180.0)
            if yaw_mode == "control" or (yaw_mode == "static" and k == 1):
                bridge.write(k, cmds)

        # 控制步已走完；仿真 TMax 含平流余量（engine.setup 扩展），等它自然完成，
        # 以免提前终止导致 .outb 未落盘（研究统计口径依赖完整 outb）。
        _proc = getattr(engine, "_proc", None)
        if _proc is not None:
            try:
                _proc.wait(timeout=300)
            except Exception:
                pass
    except Exception as e:
        import traceback as _tb
        aborted = True
        print(f"    [warn] replay aborted mid-run: {e}")
        _tb.print_exc()
    finally:
        try:
            engine.stop()
        except Exception:
            pass
        try:
            final = engine._ff.stop(allow_partial=True)
        except Exception as e:
            import traceback as _tb
            print(f"    [warn] stop()/outb parse failed: {e}")
            _tb.print_exc()
            final = None

    if final is None:
        return None

    from induction_vs_yaw_study.replay.fastfarm_replay import (
        _downsample_from_outb, _attach_control_inputs,
    )
    out = _downsample_from_outb(final, cfg.dt, n_total)
    out = _attach_control_inputs(out, yaw_cmd, ratio, ptgt)
    if out is not None and aborted:
        md = dict(out.metadata or {})
        md["aborted"] = True
        out.metadata = md
    return out