"""fastfarm_zmq_replay.py — FAST.Farm 回放（ZMQ 驱动，REFACTOR_PLAN 附录 A）

替代旧文件桥驱动的 ZMQ 版回放：
  * 仿真：FastFarmZmqEngine（复用 closedloop.zmq_pipeline 的案例/风况配置）
  * 下发：偏航（绝对目标经桥换算 yaw_delta，动作指令语义）与诱导因子 a
    （LUT 的 power_target_mw -> ratio -> a = power_to_induction）
  * 偏航执行模式：
      yaw_mode="static"（默认）: 首个控制步下发一次，runtime 锁存保持（与旧
        "锁定 DOF 静态失准"的稳态等效，研究口径一致）
      yaw_mode="control"    : 每个控制步重发目标（注意：每次重发 seq 递增会
        重新锁存 target=当前方位−delta，机舱会持续转动；静态失准语义勿用）
  * P1 根因修复（2026-09-21 诊断）：ZmqFarmBridge 硬编码 ttl（yaw 60s /
    induction 30s），static 模式单次下发后通道过期，runtime 回退风向对齐，
    ROSCO 将机舱转回 0° —— 1200s 统计窗内 yaw 恒等于未偏航。此处绕过桥，
    直接经 engine 发送长 TTL（1e6 s；0 会被 runtime 当默认值）命令。
  * 测量：以 ZMQ 状态帧为准（每控制步采集，含全部 3 台风机的 P/yaw/pitch/
    rotor_speed/轮毂风速）。注意：本集成的 FAST.Farm 仅写 Case.T1.outb
    （T2/T3 无 .outb），若用 .outb 统计会把 farm 功率错算成仅 T1 —— 因此
    统计口径改用 ZMQ 状态（对每步 3s 降采样完全足够），.outb 仅作诊断保留。
"""
from __future__ import annotations

import math as _math
import os as _os
import sys as _sys
_ccc = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), '..', '..', 'closed-loop-control'))
if _ccc not in _sys.path:
    _sys.path.insert(0, _ccc)

from typing import Dict, Optional

import numpy as np

from induction_vs_yaw_study.cases.three_nrel5mw import (
    Case, make_fastfarm_config,
)
from induction_vs_yaw_study.replay.schedule import control_inputs_for_recording

from wfcrl.config import WindConfig, SimulationOutput
from wfcrl.config.types import WindType
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine

from closedloop.types import Cmd
from closedloop.bridge import ZmqFarmBridge
from closedloop.induction import power_to_induction, A_GREEDY

# 命令通道 TTL：静态失准/持续诱导语义要求通道保持到仿真结束。
# 注意 0 会被 runtime 解释为默认值（yaw 60s / induction 5s），故显式长 TTL。
_CMD_TTL_S = 1.0e6


def _send_cmds(engine, cmds, seq_ctr) -> None:
    """经 engine 直发长 TTL 命令（绕过 ZmqFarmBridge 的 60s/30s 硬编码）。

    P1 根因（2026-09-21 diag_yaw_effect 实测）：桥的短 TTL 使 static 模式
    下发一次后通道过期，runtime 回退风向对齐目标，ROSCO 把机舱转回 0°，
    统计窗内 yaw 效应被完全抹掉。seq 按通道按机自增（runtime 拒绝乱序）。
    """
    for tid, cmd in cmds.items():
        if getattr(cmd, "yaw_delta_deg", None) is not None:
            n = seq_ctr["yaw"].get(tid, 0) + 1
            seq_ctr["yaw"][tid] = n
            engine.send_yaw_delta(tid, _math.radians(float(cmd.yaw_delta_deg)),
                                  seq=n, ttl_s=_CMD_TTL_S)
        elif getattr(cmd, "yaw_deg", None) is not None:
            raise ValueError(
                "replay 只支持 yaw_delta_deg 动作语义（绝对 yaw_deg 需可靠的 "
                "nacelle_heading 反馈，当前状态帧单位不可靠）")
        if getattr(cmd, "induction_a", None) is not None:
            a = float(cmd.induction_a)
            if a >= A_GREEDY - 1e-3:
                continue   # 贪婪哨兵：满发不下令
            n = seq_ctr["ind"].get(tid, 0) + 1
            seq_ctr["ind"][tid] = n
            engine.send_induction(tid, a, seq=n, ttl_s=_CMD_TTL_S)


def replay_fastfarm_zmq(
    case: Case,
    control_type: str,
    settings: Dict[int, Dict[str, float]],
    *,
    output_dir: str = None,
    wind_time_series: Optional[str] = None,
    yaw_mode: str = 'static',
    cmd_port: Optional[int] = None,
    state_port: Optional[int] = None,
):
    """在 FAST.Farm（ZMQ 链路）中回放某工况某控制类型。

    cmd_port/state_port 默认 None -> 按 PID 自动选择（避免多进程/多次调用间的
    TIME_WAIT 端口冲突）；显式传值（批跑器按 worker 分配）。
    返回结构与旧 replay_fastfarm 兼容（SimulationOutput + yaw_cmd/ratio/power_target）。
    """
    if cmd_port is None or state_port is None:
        base = 24000 + (_os.getpid() % 1500) * 2
        cmd_port = cmd_port or base
        state_port = state_port or base + 1
    import time as _time

    cfg = make_fastfarm_config(case, output_dir=output_dir)
    if wind_time_series:
        w0 = cfg.wind
        cfg.wind = WindConfig(speed=w0.speed, direction=w0.direction,
                              wind_type=WindType.TURBSIM_BTS, wind_file=wind_time_series)

    n_turbines = int(getattr(case, "n_turbines", 3))
    # 总控制步数由 t_sim/control_dt 派生（审查 P0-5）
    n_total = max(1, int(getattr(case, "n_total_control_steps", cfg.max_iter)))
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
            # 动作通道（CW+ delta）：static 模式仅首步下发一次，机舱由初始对齐转到失准角。
            # 不使用绝对 yaw 目标：那需要状态帧 nacelle_heading 反馈（runtime 未可靠填充）。
            cmds[tid] = Cmd(turbine_id=tid, yaw_delta_deg=float(yaw_mis), induction_a=a)
    elif control_type == "yaw":
        for tid in range(1, n_turbines + 1):
            st = settings.get(tid) or {}
            yaw_mis = st.get("yaw_deg", 0.0) or 0.0
            cmds[tid] = Cmd(turbine_id=tid, yaw_delta_deg=float(yaw_mis), induction_a=None)
    elif control_type != "baseline":
        raise ValueError(f"unknown control_type {control_type!r}")

    yaw_cmd, ratio, ptgt = control_inputs_for_recording(control_type, settings,
                                                      n_turbines=n_turbines)

    engine = FastFarmZmqEngine(cfg, cmd_port=cmd_port, state_port=state_port)
    engine.setup()
    # Phase 7 铺路：DEL 载荷通道（塔根/主轴/偏航轴承）追加到已物化 case 的
    # ElastoDyn OutList（幂等；.outb 才会含这些通道，乙模式/在线模式通用）
    try:
        from induction_vs_yaw_study.run.add_load_channels import add_load_channels
        _ff_base = getattr(engine._ff, "_farm_base", None)
        if _ff_base:
            add_load_channels(_ff_base, verbose=False)
    except Exception as _e:
        print(f"[warn] load channels: {_e}")
    # ---- P2 非确定性修复：固定 OMP 线程数 ----
    # FAST.Farm 的 OMP 浮点归约顺序差异造成运行间 ~1.9% farm 功率差（STATUS §三.2），
    # 并疑似参与偶发顺桨事件。默认钉 1 线程；WFCRL_STUDY_OMP_NUM_THREADS 可覆盖，
    # "0" = 不干预（旧行为）。必须在 engine.start() 前设置（子进程继承父 env）。
    _omp = _os.environ.get("WFCRL_STUDY_OMP_NUM_THREADS", "1")
    if _omp not in ("", "0"):
        _os.environ["OMP_NUM_THREADS"] = _omp
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

        state_rows = []
        seq_ctr = {"yaw": {}, "ind": {}}
        for k in range(1, n_total + 1):
            bridge.wait_step(k, timeout=180.0)
            if yaw_mode == "control" or (yaw_mode == "static" and k == 1):
                _send_cmds(engine, cmds, seq_ctr)
            # ZMQ 状态帧（全机）——统计唯一口径（.outb 仅 T1，不可用）
            # 瞬时断连（Connection reset）不中断回放：失败步补 NaN 行
            try:
                st = engine.get_state(timeout_s=5.0)
            except Exception:
                st = None
            if st and st.get("turbines"):
                state_rows.append(st)
            else:
                state_rows.append({"turbines": []})

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

    from induction_vs_yaw_study.replay.fastfarm_replay import _attach_control_inputs

    out = _state_frame_output(state_rows, cfg.dt, n_total, final, aborted)
    out = _attach_control_inputs(out, yaw_cmd, ratio, ptgt)
    if state_rows and output_dir:
        # 保存原始 ZMQ 状态（含 rotor_ct/cp 等），供标定/诊断读取
        try:
            import json as _json
            slim = []
            for st in state_rows:
                slim.append([{k: t.get(k) for k in (
                    "turbine_id", "gen_power_w", "rotor_ct", "rotor_cp",
                    "nacelle_heading_rad", "blade_pitch_rad", "rotor_speed_rad_s",
                    "hub_wind_speed_mps", "disk_ambient_wind_mps")}
                    for t in st.get("turbines", [])])
            with open(_os.path.join(output_dir, "zmq_state_rows.json"), "w",
                      encoding="utf-8") as f:
                _json.dump(slim, f)
        except Exception:
            pass
    if out is not None and aborted:
        md = dict(out.metadata or {})
        md["aborted"] = True
        out.metadata = md
    return out


def _state_frame_output(state_rows, dt, n_total, final, aborted):
    """由 ZMQ 状态帧构建 SimulationOutput（T1..T3 全机）。

    状态帧含 per-turbine：gen_power_w、nacelle_heading_rad、blade_pitch_rad、
    rotor_speed_rad_s、hub_wind_speed_mps。无 state 行（早退）时回退 .outb 解析。
    """
    from wfcrl.config import SimulationOutput

    if len(state_rows) < max(1, n_total):
        from induction_vs_yaw_study.replay.fastfarm_replay import _downsample_from_outb
        out = _downsample_from_outb(final, dt, n_total)
        if out is not None and aborted:
            md = dict(out.metadata or {})
            md["zmq_state_incomplete"] = True
            out.metadata = md
        return out

    time = np.arange(1, len(state_rows) + 1) * dt
    n = len(state_rows)
    # 列数 = 状态帧中实际出现的最大 turbine_id（3T=3 列，1T=1 列；避免恒 NaN 列）
    nt_cols = 1
    for st in state_rows:
        for t in st.get("turbines", []):
            nt_cols = max(nt_cols, int(t.get("turbine_id", 1)))
    p_mw = np.full((n, nt_cols), np.nan)
    yaw = np.full((n, nt_cols), np.nan)
    pitch = np.full((n, nt_cols), np.nan)
    rps = np.full((n, nt_cols), np.nan)
    wnd = np.full((n, nt_cols), np.nan)
    for i, st in enumerate(state_rows):
        for t in st.get("turbines", []):
            tid = int(t.get("turbine_id", 0)) - 1
            if not (0 <= tid < nt_cols):
                continue
            p_mw[i, tid] = float(t.get("gen_power_w", np.nan)) / 1e6
            yaw[i, tid] = float(t.get("nacelle_heading_rad", np.nan)) * 180.0 / np.pi
            bp = t.get("blade_pitch_rad") or [np.nan] * 3
            pitch[i, tid] = float(np.mean(bp)) * 180.0 / np.pi
            rps[i, tid] = float(t.get("rotor_speed_rad_s", np.nan)) * 60.0 / (2.0 * np.pi)
            wnd[i, tid] = float(t.get("hub_wind_speed_mps", np.nan))
    # P3 补丁：运行时不填 rotor_ct/rotor_cp（恒 0），此处由已发布字段回填
    #   Ct = Thrust / (0.5 rho A V^2)，Cp_aero = P_gen/eta / (0.5 rho A V^3)
    #   NREL 5MW: R=63 m, A=pi R^2, rho=1.225, eta_gen=0.944（ROSCO VS_GenEff）。
    #   V 取 disk_disturbed_wind_mps（盘面扰动风）；注意：均匀盒（无湍流）下该字段
    #   恒等于环境风、不反映尾流亏损，受遮挡机组（T2/T3）的 Ct/Cp 会因此被高估
    #   （实测均匀盒 T2 Cp≈0.52>T2 物理上限 0.48）；标定/对比 FLORIS 时须用真盒
    #   湍流工况并用 outb 的盘面风校核。V<=0.5/NaN 或 P<=0 时留空（NaN），不伪造。
    _rho, _eta = 1.225, 0.944
    _area = np.pi * 63.0 ** 2
    for i, st in enumerate(state_rows):
        for t in st.get("turbines", []):
            v = float(t.get("disk_disturbed_wind_mps",
                            t.get("hub_wind_speed_mps", np.nan)) or np.nan)
            pw = float(t.get("gen_power_w", np.nan) or np.nan)
            th = float(t.get("rotor_thrust_n", np.nan) or np.nan)
            if not (v == v) or v <= 0.5 or not (th == th):
                continue
            q = 0.5 * _rho * _area * v * v
            t["rotor_ct"] = th / q
            if (pw == pw) and pw > 0.0:
                t["rotor_cp"] = (pw / _eta) / (q * v)
            else:
                t["rotor_cp"] = np.nan

    md = dict((final.metadata or {}) if final is not None else {})
    md["measurement"] = "zmq_state_frames"
    # 非命令性顺桨检测（P2 偶发数值故障防护）：pitch>80° 且 rotor<1 rpm 连续 ≥30s
    # 记入 metadata["feather_events"]，批跑器据此剔除污染统计
    feather_events = []
    min_len = int(30.0 / dt)
    for tid in range(nt_cols):
        bad = (pitch[:, tid] > 80.0) & (rps[:, tid] < 1.0)
        run_start = None
        for i in range(n + 1):
            on = i < n and bool(bad[i])
            if on and run_start is None:
                run_start = i
            elif not on and run_start is not None:
                if i - run_start >= min_len:
                    feather_events.append({
                        "turbine_id": tid + 1,
                        "t_start_s": float(time[run_start]),
                        "t_end_s": float(time[i - 1]),
                    })
                run_start = None
    if feather_events:
        md["feather_events"] = feather_events
        md["feather_invalid"] = True
    return SimulationOutput(
        time=time,
        power_mw=p_mw,
        wind_speed=wnd,
        wind_direction=np.full_like(p_mw, np.nan),
        yaw_deg=yaw,
        pitch_deg=pitch,
        torque_nm=None,
        rotor_speed_rpm=rps,
        blade_loads=None,
        metadata=md,
    )