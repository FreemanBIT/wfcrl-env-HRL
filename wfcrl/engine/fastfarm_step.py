"""
FAST.Farm 步进式仿真接口
=======================
提供 FastFarmInterface — 每步重启 FAST.Farm 的在线接口（流场均重置）。

与 ContinuousFastFarmInterface 的区别：
- FastFarmInterface       : 每步重启 FAST.Farm（流场均重置，物理不连续）
- ContinuousFastFarmInterface : 一次启动，流场持续演化

用法
----
from wfcrl.engine import FastFarmInterface
from wfcrl.config import ControlInput

config = FastFarmConfig(case_name="demo", num_turbines=3, ...)
ff = FastFarmInterface(config)
ff.setup()
ff.reset(wind)
output = ff.step(ControlInput.scalar(3, yaw_deg=10, pitch_deg=0))
"""

from __future__ import annotations

import os
import subprocess as _sp
import warnings
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np
import yaml
from openfast_toolbox.io.fast_input_file import FASTInputFile
from openfast_toolbox.io import FASTOutputFile

from wfcrl.config.control import ControlInput
from wfcrl.config.output import SimulationOutput
from wfcrl.config.types import WindConfig, WindType
from wfcrl.engine._ff_case import (
    create_ff_case,
    write_inflow_info,
)
from wfcrl.engine.base import (
    SimulatorInterface,
    FastFarmAborted,
)
from wfcrl.engine.capabilities import (
    FASTFARM_CONTROL_MODES,
    FidelityLevel,
    SimulatorCapabilities,
    StepSynchronization,
    TimeModel,
)
from wfcrl.engine.angle_utils import (
    wrap180,
    nacyaw_face_wind,
    nacyaw_from_misalignment,
    misalignment_from_nacyaw,
)
from wfcrl.engine._outb import (
    _parse_outb_file,
    _parse_all_outb,
    CHANNEL_TO_OUTPUT,
    DEFAULT_OUTLIST_CHANNELS,
)
from wfcrl.engine._outlist import (
    _set_fast_scalar,
    _inject_outlist_channels,
    ED_OUTS,
    SRV_OUTS,
    IFW_OUTS,
)


# =========================================================================
# 项目根目录
# =========================================================================

_FF_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


# =========================================================================
# FastFarmInterface — 基于 subprocess 的步进式接口
# =========================================================================

class FastFarmInterface(SimulatorInterface):
    """
    FAST.Farm 步进式仿真接口（基于 subprocess，兼容 v5.0.0+）。

    每步仿真生成独立的 .fstf（max_iter=1），运行 FAST.Farm 子进程，
    解析所有风机的 .outb 文件，返回 SimulationOutput。

    用法
    ----
    config = FastFarmConfig(case_name="demo", num_turbines=3, ...)
    ff = FastFarmInterface(config)
    ff.setup()
    ff.reset(wind)
    output = ff.step(ControlInput.scalar(3, yaw_deg=10, pitch_deg=0))
    """

    capabilities = SimulatorCapabilities(
        simulator_id="fastfarm_segmented",
        fidelity=FidelityLevel.MID_FIDELITY,
        time_model=TimeModel.RESET_EACH_STEP,
        synchronization=StepSynchronization.SUBPROCESS_BLOCKING,
        control_modes=FASTFARM_CONTROL_MODES,
        measurements=frozenset({
            "time", "power_mw", "wind_speed", "wind_direction", "yaw_deg",
            "pitch_deg", "torque_nm", "rotor_speed_rpm", "blade_loads",
        }),
        strict_step=False,
        supports_flow_field=True,
        notes=(
            "The subprocess blocks, but every control step restarts the flow field.",
            "Returned data can contain solver substeps rather than one controller sample.",
        ),
    )

    def __init__(self, config):
        self.config = config
        self.n_turbines = config.num_turbines
        self._fastfarm_exe = config.fastfarm_exe or os.environ.get(
            "FAST_FARM_EXE",
            str(_FF_PROJECT_ROOT / "wfcrl/simulators/fastfarm/bin/FAST.Farm_OpenMP.exe"),
        )
        self._template_config = config.to_legacy_dict()
        self._step_idx = 0
        self._cumulative_time = 0.0
        self._current_wind: Optional[WindConfig] = None
        self._fstf_file: Optional[str] = None
        self._farm_base: Optional[str] = None

        if not os.path.exists(self._fastfarm_exe):
            warnings.warn(f"FAST.Farm executable not found: {self._fastfarm_exe}")

    # ========== SimulatorInterface 实现 ==========

    def setup(self) -> None:
        """生成 FAST.Farm 输入文件。"""
        farm_base_dir = os.path.join(self.config.output_dir, "FarmInputs")
        os.makedirs(farm_base_dir, exist_ok=True)

        self._fstf_file = create_ff_case(
            self._template_config,
            output_dir=self.config.output_dir,
        )
        self._farm_base = os.path.dirname(self._fstf_file)

        self._add_outlist()
        self._fix_inflow_setup()

    def reset(self, wind: WindConfig) -> None:
        """
        根据风况配置重置仿真器。

        写入 InflowWind.dat。
        """
        self._current_wind = wind
        self._step_idx = 0
        self._cumulative_time = 0.0
        self._reset_contract_state()

        if self._fstf_file is None:
            self.setup()

        self._write_wind_config(wind)

    def _step_impl(self, controls: ControlInput) -> SimulationOutput:
        """
        执行一步 FAST.Farm 仿真。

        Parameters
        ----------
        controls : ControlInput
            偏航/变桨/转矩命令 (per-turbine arrays)。

        Returns
        -------
        SimulationOutput
            当前步的完整输出。
        """
        if self._fstf_file is None:
            raise RuntimeError("Call setup() and reset() before step()")

        seg_dir = os.path.join(self.config.output_dir, f"step_{self._step_idx:04d}")
        os.makedirs(seg_dir, exist_ok=True)

        step_config = self._template_config.copy()
        step_config["max_iter"] = 1
        step_config["dt"] = self.config.dt
        if self._current_wind:
            step_config["speed"] = self._current_wind.speed
            step_config["direction"] = self._current_wind.direction
            if self._current_wind.wind_file:
                step_config["wind_time_series"] = self._current_wind.wind_file

        from wfcrl.engine._ff_case import create_ff_case as _create_ff

        self._fstf_file = _create_ff(step_config, output_dir=seg_dir)
        self._farm_base = os.path.dirname(self._fstf_file)
        self._add_outlist()
        self._fix_inflow_setup()
        self._set_controls_in_files(controls)
        self._run_subprocess()

        output = self._parse_step_output()
        self._step_idx += 1
        self._cumulative_time += self.config.dt
        return output

    def run(self, controls_list: Sequence[ControlInput]) -> SimulationOutput:
        """批量运行多步。"""
        return self._merge_outputs([self.step(c) for c in controls_list])

    def close(self) -> None:
        self._step_idx = 0
        self._cumulative_time = 0.0
        self._reset_contract_state()

    # ========== 内部方法 ==========

    def _add_outlist(self) -> None:
        """把测量通道注入各风机的**模块** OutList（ElastoDyn/ServoDyn/InflowWind）。

        这样 .outb 才会包含**执行结果**：实测机舱偏航(YawPzn)、变桨(BldPitch1)、
        转速、功率、转矩、叶根载荷、风速。
        """
        if self._fstf_file is None or self._farm_base is None:
            return
        fstf = FASTInputFile(self._fstf_file)
        wt_refs = [row[3].replace('"', "") for row in fstf["WindTurbines"]]
        for wt_ref in wt_refs:
            wt_path = os.path.join(self._farm_base, wt_ref)
            if not os.path.exists(wt_path):
                continue
            wt = FASTInputFile(wt_path)
            # ElastoDyn
            ed_rel = wt["EDFile"].replace('"', "")
            self._inject_outlist_channels(
                os.path.join(self._farm_base, ed_rel), ED_OUTS)
            # ServoDyn
            try:
                srv_rel = wt["ServoFile"].replace('"', "")
                self._inject_outlist_channels(
                    os.path.join(self._farm_base, srv_rel), SRV_OUTS)
            except Exception:
                pass
            # InflowWind（路径可能相对 FarmInputs）
            try:
                ifw_rel = wt["InflowFile"].replace('"', "")
                ifw_path = os.path.join(self._farm_base, ifw_rel)
                self._inject_outlist_channels(ifw_path, IFW_OUTS)
            except Exception:
                pass

    @staticmethod
    def _inject_outlist_channels(module_path: str, channels) -> None:
        """委托至 _outlist 模块。"""
        _inject_outlist_channels(module_path, channels)

    def _fix_inflow_setup(self) -> None:
        if self._farm_base is None:
            return
        ip = os.path.join(self._farm_base, "InflowWind.dat")
        if not os.path.exists(ip):
            return
        wdir = self._current_wind.direction if self._current_wind else self.config.wind.direction

        # 风场旋转法下 PropagationDir 必须恒为 0（风沿 +X）。
        prop_dir = 0.0
        inflow = FASTInputFile(ip)
        inflow["PropagationDir"] = prop_dir
        inflow.write(ip)
        with open(ip, "rb") as f:
            raw = f.read()
        raw = raw.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
        text = raw.decode("utf-8", errors="replace")
        import re
        text = re.sub(
            r"^(\s*)[0-9.+-]+\s+RotorApexOffsetPos",
            r"\1 0.0, 0.0, 0.0   RotorApexOffsetPos",
            text, flags=re.MULTILINE,
        )
        with open(ip, "wb") as f:
            f.write(text.encode("utf-8"))
        speed = self._current_wind.speed if self._current_wind else self.config.wind.speed
        write_inflow_info(ip, float(speed))

    def _is_mod_ambwind3(self) -> bool:
        """读取 .fstf 判断是否 Mod_AmbWind=3。"""
        try:
            if self._fstf_file and os.path.exists(self._fstf_file):
                f = FASTInputFile(self._fstf_file)
                return int(f["Mod_AmbWind"]) == 3
        except Exception:
            pass
        return False

    def _write_wind_config(self, wind: WindConfig) -> None:
        if self._farm_base is None:
            return
        ip = os.path.join(self._farm_base, "InflowWind.dat")
        if not os.path.exists(ip):
            return
        inflow = FASTInputFile(ip)
        inflow["WindType"] = int(wind.wind_type)
        inflow["HWindSpeed"] = wind.speed
        inflow["RefHt"] = wind.reference_height
        inflow["PLExp"] = wind.shear_exponent
        # 风场旋转法：PropagationDir 恒为 0
        prop_dir = 0.0
        inflow["PropagationDir"] = prop_dir

        if wind.wind_type == WindType.TURBSIM_BTS and wind.wind_file:
            _farminputs = os.environ.get(
                "WFCRL_FARMINPUTS_DIR",
                r"D:\HR_Project\wfcrl-env-HRL\FarmInputs",
            )
            inflow["FileName_BTS"] = f'"{os.path.join(_farminputs, wind.wind_file)}"'
        elif wind.wind_type == WindType.UNIFORM and wind.wind_file:
            inflow["Filename_Uni"] = f'"{wind.wind_file}"'
        elif wind.wind_type in (WindType.BLADED_BIN, WindType.BLADED_NATIVE) and wind.wind_file:
            inflow["FileNameRoot"] = f'"{wind.wind_file}"'
        elif wind.wind_type == WindType.HAWC and wind.wind_file:
            inflow["FileName_u"] = f'"{wind.wind_file}"'
            inflow["URef"] = wind.speed
            inflow["RefHt_Hawc"] = wind.reference_height

        inflow.write(ip)
        with open(ip, "rb") as f:
            raw = f.read()
        raw = raw.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
        import re
        text = raw.decode("utf-8", errors="replace")
        text = re.sub(
            r"^(\s*)[0-9.+-]+\s+RotorApexOffsetPos",
            r"\1 0.0, 0.0, 0.0   RotorApexOffsetPos",
            text, flags=re.MULTILINE,
        )
        with open(ip, "wb") as f:
            f.write(text.encode("utf-8"))

    def _set_controls_in_files(self, controls: ControlInput) -> None:
        if self._fstf_file is None or self._farm_base is None:
            return
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
            ed = FASTInputFile(ed_path)
            if controls.yaw is not None and i < len(controls.yaw):
                wdir = (self._current_wind.direction
                        if self._current_wind else self.config.wind.direction)
                ed["NacYaw"] = nacyaw_from_misalignment(wdir, float(controls.yaw[i]))
            if controls.pitch is not None and i < len(controls.pitch):
                pv = float(controls.pitch[i])
                ed["BlPitch(1)"] = pv
                ed["BlPitch(2)"] = pv
                ed["BlPitch(3)"] = pv
            ed.write(ed_path)
            with open(ed_path, "rb") as f:
                raw = f.read()
            raw = raw.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
            with open(ed_path, "wb") as f:
                f.write(raw)

    def _run_subprocess(self) -> None:
        if self._fstf_file is None or self._farm_base is None:
            raise RuntimeError("No .fstf file")
        proc = _sp.Popen(
            [self._fastfarm_exe, self._fstf_file],
            cwd=self._farm_base,
            stdout=_sp.PIPE,
            stderr=_sp.STDOUT,
            text=True,
            bufsize=1,
        )
        for _ in proc.stdout:
            pass
        proc.wait()

    def _parse_step_output(self) -> SimulationOutput:
        if self._fstf_file is None or self._farm_base is None:
            raise RuntimeError("No simulation output to parse")
        prefix = os.path.splitext(os.path.basename(self._fstf_file))[0]
        parsed = _parse_all_outb(self._farm_base, prefix, self.n_turbines)

        local_time = parsed.get("time")
        if local_time is None or np.asarray(local_time).size == 0:
            time_vec = np.array([self._cumulative_time + self.config.dt])
        else:
            # Each segmented FAST.Farm process starts its clock at zero.  Expose
            # one monotonically increasing experiment clock at the public API.
            time_vec = np.asarray(local_time, dtype=float) + self._cumulative_time
        power_raw = parsed.get("power")
        if power_raw is not None and power_raw.size > 0:
            power_mw = np.atleast_2d(power_raw)
        else:
            power_mw = np.zeros((len(time_vec), self.n_turbines))

        wind_x = parsed.get("wind_x")
        wind_y = parsed.get("wind_y")
        wind_z = parsed.get("wind_z")
        wind_speed = None
        wind_direction = None
        if wind_x is not None:
            if wind_y is not None:
                ws = np.sqrt(wind_x**2 + wind_y**2 + (wind_z**2 if wind_z is not None else 0))
                wd = np.degrees(np.arctan2(wind_y, wind_x))
                wd = (270.0 - wd) % 360
                wind_speed, wind_direction = ws, wd
            else:
                wind_speed = np.abs(wind_x)

        yaw_deg = parsed.get("yaw")
        if yaw_deg is not None:
            _wdir = (self._current_wind.direction
                     if self._current_wind else self.config.wind.direction)
            _arr = np.asarray(yaw_deg, dtype=float)
            yaw_deg = np.array([misalignment_from_nacyaw(_wdir, float(v)) for v in _arr.ravel()]).reshape(_arr.shape)
        pitch_deg = parsed.get("pitch")
        torque_nm = parsed.get("generator_torque")
        rotor_speed_rpm = parsed.get("rotor_speed")

        b1 = parsed.get("blade_load_1")
        b2 = parsed.get("blade_load_2")
        b3 = parsed.get("blade_load_3")
        blade_loads = None
        if b1 is not None and b2 is not None and b3 is not None:
            blade_loads = np.stack([b1, b2, b3], axis=-1)

        return SimulationOutput(
            time=time_vec,
            power_mw=power_mw,
            wind_speed=wind_speed,
            wind_direction=wind_direction,
            yaw_deg=yaw_deg,
            pitch_deg=pitch_deg,
            torque_nm=torque_nm,
            rotor_speed_rpm=rotor_speed_rpm,
            generator_torque_nm=torque_nm,
            blade_loads=blade_loads,
            metadata={"step": self._step_idx, "dt": self.config.dt},
        )
