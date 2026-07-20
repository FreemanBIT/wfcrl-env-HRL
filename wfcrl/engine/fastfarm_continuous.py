"""
FAST.Farm 连续仿真接口
=====================
提供 ContinuousFastFarmInterface — 启动一次，流场持续演化的在线接口。

与 FastFarmInterface 的关键区别：
- FastFarmInterface            : 每步重启 FAST.Farm（流场均重置，物理不连续）
- ContinuousFastFarmInterface  : 一次启动，通过 DISCON bridge DLL
  在每个底层 timestep 交换 控制命令 ↔ 测量值（流场连续演化）

架构
----
Python 控制器              FAST.Farm 子进程（持续运行）
     │                              │
     ├── 写 controls.txt ──────────→│ DISCON.dll 每步读取
     │                              │ 应用 yaw/pitch
     │                              │ 写 measurements_T*.txt
     │←── 读 measurements ─────────┤
     │    轮询等 step 匹配            │
     │    计算下步控制 → 重复          │

用法
----
from wfcrl.engine import ContinuousFastFarmInterface
from wfcrl.config import ControlInput

ff = ContinuousFastFarmInterface(config)
ff.setup(); ff.reset(wind); ff.start()
for step in range(N):
    controls = controller.compute(prev_output)
    output = ff.wait_step(controls)
final = ff.stop()
"""

from __future__ import annotations

import os
import subprocess as _sp
import time
import warnings
from typing import Dict, Optional, Sequence

import numpy as np
from openfast_toolbox.io.fast_input_file import FASTInputFile

from wfcrl.config.control import ControlInput
from wfcrl.config.output import SimulationOutput
from wfcrl.config.types import WindConfig
from wfcrl.engine._ff_case import (
    create_ff_case,
    create_dll,
    write_inflow_info,
)
from wfcrl.engine.base import (
    FastFarmAborted,
)
from wfcrl.engine.angle_utils import (
    nacyaw_from_misalignment,
    misalignment_from_nacyaw,
)
from wfcrl.engine.fastfarm_step import FastFarmInterface
from wfcrl.engine._outlist import (
    _set_fast_scalar,
)
from wfcrl.engine._outb import (
    _parse_all_outb,
)


# =========================================================================
# ContinuousFastFarmInterface — 流场连续型 FAST.Farm 在线接口
# =========================================================================

class ContinuousFastFarmInterface(FastFarmInterface):
    """
    FAST.Farm 连续仿真接口 — 启动一次，流场持续演化。

    与 FastFarmInterface 的关键区别：
    - FastFarmInterface：每步重启 FAST.Farm（流场均重置，物理不连续）
    - ContinuousFastFarmInterface：一次启动，通过 DISCON bridge DLL
      在每个底层 timestep 交换 控制命令 ↔ 测量值（流场连续演化）

    架构
    ----
    Python 控制器              FAST.Farm 子进程（持续运行）
         │                              │
         ├── 写 controls.txt ──────────→│ DISCON.dll 每步读取
         │                              │ 应用 yaw/pitch
         │                              │ 写 measurements_T*.txt
         │←── 读 measurements ─────────┤
         │    轮询等 step 匹配            │
         │    计算下步控制 → 重复          │

    用法
    ----
    ff = ContinuousFastFarmInterface(config)
    ff.setup(); ff.reset(wind); ff.start()
    for step in range(N):
        controls = controller.compute(prev_output)
        output = ff.wait_step(controls)
    final = ff.stop()
    """

    def __init__(self, config):
        super().__init__(config)
        self._process: Optional[_sp.Popen] = None
        self._controls_file: Optional[str] = None
        self._discon_initialized = False

    # ========== 核心 API ==========

    def setup(self) -> None:
        """生成完整仿真文件（TMax = max_iter * dt），部署 DISCON bridge。"""
        full_config = self._template_config.copy()
        full_config["max_iter"] = self.config.max_iter
        full_config["dt"] = self.config.dt

        self._fstf_file = create_ff_case(full_config, output_dir=self.config.output_dir)
        self._farm_base = os.path.dirname(self._fstf_file)

        self._add_outlist()
        self._fix_inflow_setup()
        self._fix_initial_yaw()

        # 部署 DISCON bridge DLL + DISCON.IN
        create_dll(self._fstf_file)

        # controls.txt 放在 FarmInputs 同级（DISCON 从 cwd 读取）
        self._controls_file = os.path.join(self._farm_base, "controls.txt")

        print(f"ContinuousFastFarmInterface ready: {self.n_turbines} turbines")

    def set_fixed_yaw(self, yaw_misalign_deg) -> None:
        """设置各风机的**固定偏航失准角**（相对来流，度）并锁定偏航自由度。

        为什么用"锁定 DOF + 固定 NacYaw"而不是 DLL 偏航速率控制：
        --------------------------------------------------------------------
        本研究是**静态 LUT**对比——每台风机保持一个恒定偏航失准角，与 FLORIS
        稳态偏航语义完全一致（FLORIS 偏航也是固定失准角、无动态）。

        原模板 ServoDyn 的 YCMode=0（无偏航控制），DLL 写的偏航速率指令
        avrSWap(48) 被 ServoDyn **完全忽略** —— 这正是"yaw 结果和 baseline 一样"
        的根因。这里直接把每台风机 NacYaw 设为目标失准角并**锁定偏航 DOF
        （YawDOF=False）**，机舱在整个仿真精确保持该角度、零整定、与 FLORIS 一致。
        """
        if self._fstf_file is None or self._farm_base is None:
            return
        wdir = self._current_wind.direction if self._current_wind else self.config.wind.direction

        arr = np.atleast_1d(np.asarray(yaw_misalign_deg, dtype=float))
        if arr.size == 1:
            arr = np.full(self.n_turbines, float(arr[0]))

        fstf = FASTInputFile(self._fstf_file)
        wt_refs = [row[3].replace('"', "") for row in fstf["WindTurbines"]]
        for i, wt_ref in enumerate(wt_refs):
            wt_path = os.path.join(self._farm_base, wt_ref)
            if not os.path.exists(wt_path):
                continue
            wt = FASTInputFile(wt_path)
            ed_rel = wt["EDFile"].replace('"', "")
            ed_path = os.path.join(self._farm_base, ed_rel)
            if not os.path.exists(ed_path):
                continue
            mis = float(arr[i]) if i < arr.size else 0.0
            nacyaw_abs = nacyaw_from_misalignment(wdir, mis)
            _set_fast_scalar(ed_path, "NacYaw", f"{nacyaw_abs:.4f}")
            _set_fast_scalar(ed_path, "YawDOF", "False")
        self._fixed_yaw_applied = arr.copy()

    def _fix_initial_yaw(self) -> None:
        """兼容旧接口：默认所有风机偏航对准来流（失准角 0）、锁定偏航 DOF。"""
        self.set_fixed_yaw(0.0)

    def _write_initial_controls(self) -> None:
        """在 FAST.Farm 启动前写入初始 controls.txt（5-mode protocol）。
        初始用 mode=0 零偏航增量，让 ROSCO 自行对风。"""
        if self._controls_file is None:
            return
        lines = ["step=-1"]
        for t in range(self.n_turbines):
            lines.append(f"T{t+1} mode=0 yaw=0.000 pitch=0.000 power=0.000 minpitch=0.000")
        lines.append("END")
        with open(self._controls_file, 'w') as f:
            f.write('\n'.join(lines) + '\n')

    def start(self) -> None:
        """后台启动 FAST.Farm（非阻塞）。"""
        if self._farm_base is None:
            raise RuntimeError("Call setup() first")

        # 预写初始 controls.txt（step=0），使 DLL 第一时间获得正确偏航
        self._write_initial_controls()

        # 将 FAST.Farm 输出重定向到日志文件（防止 pipe 缓冲区满导致死锁）
        log_path = os.path.join(self.config.output_dir or ".", "fastfarm_continuous.log")
        log_file = open(log_path, 'w', buffering=1)
        self._process = _sp.Popen(
            [self._fastfarm_exe, self._fstf_file],
            cwd=self._farm_base,
            stdout=log_file,
            stderr=_sp.STDOUT,
            text=True,
        )
        self._proc_log = log_file
        self._step_idx = 0
        print(f"FAST.Farm started (PID {self._process.pid}), log: {log_path}")

    def wait_step(self, controls: ControlInput) -> SimulationOutput:
        """
        发送控制命令并等待当前步的测量值。

        1. 写 controls.txt（含 step 号 + per-turbine 命令）
        2. 轮询 measurements_T1.txt 直到 step 号匹配
        3. 读所有风机测量文件 → SimulationOutput
        """
        if self._process is None:
            raise RuntimeError("Call start() first")
        if self._farm_base is None:
            raise RuntimeError("Call setup() first")

        self._write_controls_file(controls)
        self._step_idx += 1

        # 轮询等待（DISCON 每 DT_low ≈ 0.05s 写一次）
        output = self._read_measurements_with_poll()
        self._cumulative_time += self.config.dt
        return output

    def stop(self, *, allow_partial: bool = True) -> SimulationOutput:
        """发停止信号，等进程结束，解析 .outb 获取完整输出。

        allow_partial=True 时，即使进程已 abort/退出，也尽量解析已写出的 .outb
        （部分时序），使上层仍能保存部分结果 CSV，而不是整个工况丢失。
        """
        if self._process is not None:
            # 写 END 标记（若进程还活着，让它优雅收尾）
            if self._controls_file and self._process.poll() is None:
                try:
                    with open(self._controls_file, 'w') as f:
                        f.write('step=-1\nEND\n')
                except OSError:
                    pass
            # 等进程结束；已 abort 的会立即返回。设超时避免永久阻塞。
            try:
                self._process.wait(timeout=60)
            except Exception:
                try:
                    self._process.terminate()
                    self._process.wait(timeout=10)
                except Exception:
                    pass
            self._process = None

        if hasattr(self, '_proc_log') and self._proc_log is not None:
            try:
                self._proc_log.close()
            except Exception:
                pass
            self._proc_log = None

        # 解析 .outb（可能是完整或部分）
        try:
            return self._parse_step_output()
        except Exception as e:
            if allow_partial:
                warnings.warn(f"stop(): 解析 .outb 失败，返回空输出：{e}")
                return SimulationOutput(
                    time=np.array([self._cumulative_time]),
                    power_mw=np.zeros((1, self.n_turbines)),
                    metadata={"warning": f"outb parse failed: {e}"},
                )
            raise

    def close(self) -> None:
        if self._process is not None:
            try:
                self._process.terminate()
                self._process.wait(timeout=10)
            except Exception:
                pass
            self._process = None
        if hasattr(self, '_proc_log') and self._proc_log is not None:
            self._proc_log.close()
            self._proc_log = None
        self._step_idx = 0
        self._cumulative_time = 0.0

    # ========== 内部方法 ==========

    def _write_controls_file(self, controls: ControlInput) -> None:
        """写 controls.txt (5-mode farm protocol)。
        格式: step=N / T{id} mode=M yaw=X pitch=Y power=Z minpitch=W / END"""
        if self._controls_file is None:
            return

        lines = [f"step={self._step_idx}"]
        wdir = (self._current_wind.direction
                if self._current_wind else self.config.wind.direction)
        for t in range(self.n_turbines):
            m = int(controls.mode[t]) if t < len(controls.mode) else 0
            y_misalign = controls.yaw[t] if t < len(controls.yaw) else 0.0
            y = nacyaw_from_misalignment(wdir, float(y_misalign))
            p = controls.pitch[t] if t < len(controls.pitch) else 0.0
            pw = controls.power[t] if controls.power is not None and t < len(controls.power) else 0.0
            mp = controls.min_pitch[t] if controls.min_pitch is not None and t < len(controls.min_pitch) else 0.0
            lines.append(f"T{t+1} mode={m} yaw={y:.3f} pitch={p:.3f} power={pw:.3f} minpitch={mp:.3f}")
        lines.append("END")

        with open(self._controls_file, 'w') as f:
            f.write('\n'.join(lines) + '\n')

    def _read_measurements_with_poll(self, timeout: float = 120.0) -> SimulationOutput:
        """
        轮询读取 DISCON 输出的测量文件。

        修改后的 DISCON_bridge.f90 使用 accINFILE 读取各风机独立的
        DISCON_T{i}.IN 文件，因此每台风机有独立的 my_id，
        写入独立的 measurements_T{i}.txt。
        本方法收集所有存在的测量文件，缺失的风机用 0 填充。

        关键逻辑：不仅要等待 step 号匹配（DISCON 已应用本步控制），
        还要等待仿真时间 t 推进到 target_time = step_idx * dt
        （当前控制周期结束），确保测量值反映的是 dt 秒仿真后的稳态结果。

        文件格式（由 DISCON_bridge.f90 写入）:
            step=N t=1.05 genpwr=XXX genspd=XXX gentq=XXX rotspd=XXX
            wind_x=XXX
            blpitch=XXX nacyaw=XXX
            mip1=XXX moop1=XXX mzb1=XXX
        """
        n = self.n_turbines
        expected_step = self._step_idx - 1
        target_time = self._step_idx * self.config.dt

        def _read_all() -> Optional[Dict[int, Dict[str, float]]]:
            """收集所有存在的 measurements_T*.txt，返回 step+time 都匹配的数据。"""
            results: Dict[int, Dict[str, float]] = {}
            for t_id in range(1, n + 1):
                fpath = os.path.join(self._farm_base or ".", f"measurements_T{t_id}.txt")
                if not os.path.exists(fpath):
                    continue
                vals: Dict[str, float] = {}
                last_label: Optional[str] = None
                try:
                    with open(fpath, 'r') as f:
                        for line in f:
                            for part in line.strip().split():
                                if '=' in part:
                                    k, v = part.split('=', 1)
                                    v = v.strip()
                                    if v:
                                        try:
                                            vals[k] = float(v)
                                        except ValueError:
                                            pass
                                        last_label = None
                                    else:
                                        last_label = k
                                elif last_label is not None:
                                    try:
                                        vals[last_label] = float(part)
                                    except ValueError:
                                        pass
                                    last_label = None
                except (OSError, IOError):
                    continue
                if 'step' not in vals or int(vals['step']) != expected_step:
                    continue
                sim_time = vals.get('t', 0.0)
                if sim_time < target_time - self.config.dt * 0.5:
                    continue
                results[t_id] = vals
            return results if results else None

        # 轮询
        start_t = time.time()
        data = None
        while time.time() - start_t < timeout:
            data = _read_all()
            if data is not None:
                break
            # 快速失败：检查 FAST.Farm 进程是否已退出/崩溃
            if self._process is not None and self._process.poll() is not None:
                rc = self._process.returncode
                raise FastFarmAborted(
                    f"FAST.Farm process exited (returncode={rc}) at step={self._step_idx} "
                    f"while waiting for measurements (likely internal abort; see "
                    f"fastfarm_continuous.log)."
                )
            time.sleep(0.1)

        if data is None:
            raise FastFarmAborted(
                f"Timed out ({timeout:.0f}s) waiting for DISCON measurements at "
                f"step={self._step_idx} (process may be hung or aborting)."
            )

        # 初始化数组（缺失的风机保持 0）
        power_mw = np.zeros(n)
        wind_speed = np.zeros(n)
        yaw_deg = np.zeros(n)
        pitch_deg = np.zeros(n)
        torque_nm = np.zeros(n)
        rotor_speed = np.zeros(n)
        blade_loads = np.zeros((n, 3))

        _wdir = (self._current_wind.direction
                 if self._current_wind else self.config.wind.direction)

        def _to_misalign(nac_abs_deg: float) -> float:
            return misalignment_from_nacyaw(_wdir, nac_abs_deg)

        # 填充所有风机数据
        for t_id, vals in data.items():
            i = t_id - 1
            power_mw[i] = vals.get('genpwr', 0.0) / 1000.0       # kW → MW
            wind_speed[i] = vals.get('wind_x', 0.0)
            yaw_deg[i] = _to_misalign(vals.get('nacyaw', 0.0))
            pitch_deg[i] = vals.get('blpitch', 0.0)
            torque_nm[i] = vals.get('gentq', 0.0)
            rotor_speed[i] = vals.get('rotspd', 0.0)
            blade_loads[i, 0] = vals.get('mip1', 0.0)
            blade_loads[i, 1] = vals.get('moop1', 0.0)
            blade_loads[i, 2] = vals.get('mzb1', 0.0)

        n_found = len(data)
        metadata = {"step": self._step_idx, "n_measurements_found": n_found}
        if n_found < n:
            metadata["warning"] = f"only {n_found}/{n} turbine measurements found"

        return SimulationOutput(
            time=np.array([self._cumulative_time]),
            power_mw=power_mw.reshape(1, -1),
            wind_speed=wind_speed.reshape(1, -1),
            yaw_deg=yaw_deg.reshape(1, -1),
            pitch_deg=pitch_deg.reshape(1, -1),
            torque_nm=torque_nm.reshape(1, -1),
            rotor_speed_rpm=rotor_speed.reshape(1, -1),
            blade_loads=blade_loads.reshape(1, n, 3),
            metadata=metadata,
        )
