"""
fastfarm_replay.py — FAST.Farm 回放 (Stage 6)
=============================================
用工程的 ContinuousFastFarmInterface（一次启动、流场连续）按 schedule 回放。
静态 LUT：每个控制步下发相同命令；前 t_settle 让尾流稳定，后段取稳态窗口。

v2 — 使用 stop() 读取 .outb 文件，避免 DISCON 轮询的竞态条件。
"""

from __future__ import annotations

from typing import Dict

import numpy as np

from induction_vs_yaw_study.cases.three_nrel5mw import Case, make_fastfarm_config
from induction_vs_yaw_study.replay.schedule import (
    build_control_input, control_inputs_for_recording,
)

from wfcrl.engine import ContinuousFastFarmInterface, FastFarmAborted
from wfcrl.config import SimulationOutput


def replay_fastfarm(
    case: Case,
    control_type: str,
    settings: Dict[int, Dict[str, float]],
    *,
    output_dir: str = None,
    collect_stepwise: bool = True,
):
    """
    在 FAST.Farm 中回放某工况某控制类型。

    流程：setup → reset → set_fixed_yaw（偏航经锁定 DOF 实现）→ start
          → wait_step×N（推进）→ stop（拿 .outb 数据）。

    偏航实现说明：yaw 控制通过在 ElastoDyn 中设固定 NacYaw + 锁定偏航 DOF 实现
    （见 ContinuousFastFarmInterface.set_fixed_yaw），因为模板 ServoDyn 的 YCMode=0
    会让 DLL 的偏航速率指令被忽略。derating 仍走 DLL 转矩通道（VSContrl=5）。
    """
    cfg = make_fastfarm_config(case, output_dir=output_dir)
    ff = ContinuousFastFarmInterface(cfg)
    ff.setup()
    ff.reset(cfg.wind)

    ci = build_control_input(
        control_type, "fastfarm", settings,
        wind_speed_ms=case.wind_speed_ms, n_turbines=3,
    )
    # 控制输入（用于记录到 timeseries）
    yaw_cmd, ratio, ptgt = control_inputs_for_recording(
        control_type, settings, n_turbines=3,
    )

    # 偏航：把每台风机的失准角写入 ED 并锁定偏航 DOF（在 start 之前）。
    # 对 baseline/derating，yaw_cmd 全为 0 → 机舱对准来流并锁定，行为与之前一致。
    ff.set_fixed_yaw(yaw_cmd)

    ff.start()

    final = None

    n_total = cfg.max_iter
    aborted = False
    try:
        # ---- 只推进仿真，不收集 wait_step 输出 ----
        for _ in range(n_total):
            ff.wait_step(ci)
    except FastFarmAborted as e:
        # FAST.Farm 中途 abort：不再逐步空等，立即收尾并尽量保存已推进的部分结果。
        aborted = True
        print(f"    [warn] FAST.Farm aborted mid-run: {e}")
    finally:
        # stop() 会解析已写出的 .outb（可能是部分时序），allow_partial 防止再抛。
        try:
            final = ff.stop(allow_partial=True)
        except Exception as e:  # noqa
            print(f"    [warn] stop() failed: {e}")
            final = None
        ff.close()

    if final is None:
        return None

    if collect_stepwise:
        out = _downsample_from_outb(final, cfg.dt, n_total)
    else:
        out = final
    out = _attach_control_inputs(out, yaw_cmd, ratio, ptgt)
    if out is not None and aborted:
        md = dict(out.metadata or {})
        md["aborted"] = True
        out.metadata = md
    return out


def _attach_control_inputs(out, yaw_cmd, ratio, ptgt):
    """把每台风机的控制输入广播到所有时间步，写入 SimulationOutput。"""
    if out is None or out.time is None:
        return out
    n_steps = len(out.time)
    n_t = len(yaw_cmd)
    out.yaw_cmd_deg = np.tile(np.asarray(yaw_cmd, dtype=float), (n_steps, 1))
    out.ratio = np.tile(np.asarray(ratio, dtype=float), (n_steps, 1))
    out.power_target_mw = np.tile(np.asarray(ptgt, dtype=float), (n_steps, 1))
    return out


def _downsample_from_outb(
    full: SimulationOutput,
    dt: float,
    n_steps: int,
) -> SimulationOutput:
    """
    从 stop() 返回的完整 .outb 时序中，按控制步间隔 dt 降采样。

    .outb 包含高密度时序（每 DT_low 一个点），我们只取 n_steps 个控制步的数据。
    提取每个控制周期结束时（t = dt, 2*dt, ..., n_steps*dt）的仿真状态。
    时间标签沿用旧的 0, dt, 2*dt, ... 习惯。
    """
    t = full.time
    if t is None or len(t) == 0:
        return full

    # 目标仿真时刻：第 1 个控制步结束在 dt，第 k 个在 k*dt
    target_sim_t = np.arange(1, n_steps + 1) * dt

    # 在 .outb 时间轴中找最接近每个目标时刻的索引
    idx = np.array([np.argmin(np.abs(t - tt)) for tt in target_sim_t])
    idx = np.clip(idx, 0, len(t) - 1)

    def _take(arr):
        if arr is None:
            return None
        return arr[idx]

    return SimulationOutput(
        time=target_sim_t,                     # 真实仿真时刻 dt,2dt,...（诚实标签）
        power_mw=_take(full.power_mw),
        wind_speed=_take(full.wind_speed),
        wind_direction=_take(full.wind_direction),
        yaw_deg=_take(full.yaw_deg),
        pitch_deg=_take(full.pitch_deg),
        torque_nm=_take(full.torque_nm),
        rotor_speed_rpm=_take(full.rotor_speed_rpm),
        blade_loads=_take(full.blade_loads),
        metadata=full.metadata,
    )