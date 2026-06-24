"""
统一仿真器接口
==============
提供统一的 SimulatorInterface 抽象基类和两个实现：
- FastFarmInterface : 基于 subprocess 的 FAST.Farm 在线接口
- FlorisInterface    : 基于 FLORIS Python API 的在线接口

用法
----
# 通用模式
from wfcrl.config import WindConfig, ControlInput
from wfcrl.simul_config import SimulationConfig
from wfcrl.interface import FastFarmInterface, FlorisInterface

config = SimulationConfig(
    case_name="demo", num_turbines=3,
    xcoords=[0,504,1008], ycoords=[0,0,0],
    dt=3.0, max_iter=10,
    wind=WindConfig(speed=10, direction=270),
)

# FAST.Farm
ff = FastFarmInterface(config.create_fastfarm())
ff.setup()
ff.reset(config.wind)
for step in range(config.max_iter):
    controls = ControlInput.scalar(3, yaw_deg=0, pitch_deg=0)
    output = ff.step(controls)
    print(f"Step {step}: farm_power={output.farm_power_mw[-1]:.2f} MW")
ff.close()

# 批量运行（语法糖）
controls_list = [ControlInput.scalar(3, yaw_deg=i) for i in range(10)]
output = ff.run(controls_list)
output.to_csv("results.csv")
"""

from __future__ import annotations

import copy
import os
import re
import subprocess as _sp
import time
import warnings
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from floris import FlorisModel
from openfast_toolbox.io.fast_input_file import FASTInputFile
from openfast_toolbox.io import FASTOutputFile
from scipy.optimize import fsolve

from wfcrl.config import ControlInput, SimulationOutput, WindConfig, WindType
from wfcrl.simul_config import FastFarmConfig, FlorisConfig, SimulationConfig
from wfcrl.simul_utils import (
    create_ff_case,
    create_floris_case,
    write_inflow_info,
)


# =========================================================================
# SimulatorInterface — 统一接口抽象基类
# =========================================================================

class SimulatorInterface(ABC):
    """
    所有仿真器接口的抽象基类。

    子类必须实现：
    - setup(): 生成仿真输入文件
    - reset(wind): 根据风况初始化/重置仿真器
    - step(controls): 执行一步仿真，返回 SimulationOutput
    - close(): 清理资源
    """

    n_turbines: int
    config: SimulationConfig

    @abstractmethod
    def setup(self) -> None:
        """生成仿真输入文件，准备仿真环境。"""

    @abstractmethod
    def reset(self, wind: WindConfig) -> None:
        """
        根据风况初始化/重置仿真器。

        在 setup() 之后调用，或每次改变风况时调用。
        """

    @abstractmethod
    def step(self, controls: ControlInput) -> SimulationOutput:
        """
        执行一步仿真。

        Parameters
        ----------
        controls : ControlInput
            当前步的控制输入（偏航/变桨/转矩）。

        Returns
        -------
        SimulationOutput
            当前步的输出。
        """

    def run(self, controls_list: Sequence[ControlInput]) -> SimulationOutput:
        """
        批量运行多步仿真（step() 的循环语法糖）。

        Parameters
        ----------
        controls_list : Sequence[ControlInput]
            每一步的控制输入序列。

        Returns
        -------
        SimulationOutput
            汇总所有步的输出。
        """
        outputs: List[SimulationOutput] = []
        for controls in controls_list:
            outputs.append(self.step(controls))
        return self._merge_outputs(outputs)

    @abstractmethod
    def close(self) -> None:
        """清理资源。"""

    @staticmethod
    def _merge_outputs(outputs: List[SimulationOutput]) -> SimulationOutput:
        """合并多个单步输出为一个批量输出。"""
        if not outputs:
            raise ValueError("Empty outputs list")

        def _concat(attr: str) -> Optional[np.ndarray]:
            vals = [getattr(o, attr) for o in outputs if getattr(o, attr) is not None]
            return np.concatenate(vals, axis=0) if vals else None

        return SimulationOutput(
            time=_concat("time"),
            power_mw=_concat("power_mw"),
            wind_speed=_concat("wind_speed"),
            wind_direction=_concat("wind_direction"),
            yaw_deg=_concat("yaw_deg"),
            pitch_deg=_concat("pitch_deg"),
            torque_nm=_concat("torque_nm"),
            rotor_speed_rpm=_concat("rotor_speed_rpm"),
            generator_torque_nm=_concat("generator_torque_nm"),
            thrust_n=_concat("thrust_n"),
            blade_loads=_concat("blade_loads"),
            metadata=outputs[0].metadata if outputs else {},
        )


# =========================================================================
# 工具：.outb 解析
# =========================================================================

# 默认 OutList 通道 — 映射到 SimulationOutput 字段
DEFAULT_OUTLIST_CHANNELS = [
    "GenPwr",       # 发电功率 → power_mw
    "GenTq",        # 发电机转矩 → generator_torque_nm
    "RotSpeed",     # 风轮转速 → rotor_speed_rpm
    "RootMIP1",     # 叶根面内弯矩 1
    "RootMOoP1",    # 叶根面外弯矩 1
    "RootMzb1",     # 叶根扭矩
    "YawPzn",       # 偏航位置 → yaw_deg
    "BldPitch1",    # 叶片变桨 → pitch_deg
    "HSShftP",      # 高速轴功率
    "Wind1VelX",    # 风速 X 分量 → wind_speed
    "Wind1VelY",    # 风速 Y 分量
    "Wind1VelZ",    # 风速 Z 分量
]

# OutList 通道名 → 内部解析键
CHANNEL_TO_OUTPUT = {
    "GenPwr": "power",
    "GenTq": "generator_torque",
    "RotSpeed": "rotor_speed",
    "RootMIP1": "blade_load_1",
    "RootMOoP1": "blade_load_2",
    "RootMzb1": "blade_load_3",
    "YawPzn": "yaw",
    "BldPitch1": "pitch",
    "HSShftP": "hss_power",
    "Wind1VelX": "wind_x",
    "Wind1VelY": "wind_y",
    "Wind1VelZ": "wind_z",
}


def _parse_outb_file(
    outb_path: str,
    channel_map: Optional[Dict[str, str]] = None,
) -> Dict[str, np.ndarray]:
    """
    解析 FAST.Farm .outb 文件，提取指定通道。

    Returns
    -------
    Dict[str, np.ndarray]
        {输出键: 值数组}，每个数组 shape 为 (n_time_steps,).
    """
    if channel_map is None:
        channel_map = CHANNEL_TO_OUTPUT

    result: Dict[str, np.ndarray] = {}
    df = FASTOutputFile(outb_path).toDataFrame()
    time_vec = df.iloc[:, 0].values
    result["time"] = time_vec

    for col in df.columns:
        base = col.split("[")[0].strip("_ ").strip()
        for ch_name, out_key in channel_map.items():
            if base == ch_name or base.startswith(ch_name):
                val = df[col].values.astype(np.float64)
                if "[" in col:
                    unit = col.split("[")[1].split("]")[0].upper()
                    if ch_name == "GenPwr" and unit in ("KW", "KILOWATT"):
                        val = val / 1e3
                    elif ch_name == "GenPwr" and unit == "W":
                        val = val / 1e6
                result[out_key] = val
                break

    return result


def _parse_all_outb(
    farm_base: str,
    prefix: str,
    n_turbines: int,
    channel_map: Optional[Dict[str, str]] = None,
) -> Dict[str, np.ndarray]:
    """
    解析所有风机的 .outb 文件。

    Returns
    -------
    Dict[str, np.ndarray]
        每个键的数组 shape 为 (n_time_steps, n_turbines)。
    """
    if channel_map is None:
        channel_map = CHANNEL_TO_OUTPUT

    per_turbine: Dict[str, List[np.ndarray]] = {}
    time_vec = None

    for i in range(1, n_turbines + 1):
        ob = os.path.join(farm_base, f"{prefix}.T{i}.outb")
        o_path = os.path.join(farm_base, f"{prefix}.T{i}.out")
        path = ob if os.path.exists(ob) else (o_path if os.path.exists(o_path) else None)
        if path is None:
            continue

        parsed = _parse_outb_file(path, channel_map)
        if time_vec is None:
            time_vec = parsed.get("time")

        for key, val in parsed.items():
            if key == "time":
                continue
            if key not in per_turbine:
                per_turbine[key] = []
            if time_vec is not None and len(val) != len(time_vec):
                if len(val) < len(time_vec):
                    val = np.pad(val, (0, len(time_vec) - len(val)), constant_values=np.nan)
                else:
                    val = val[:len(time_vec)]
            per_turbine[key].append(val)

    result: Dict[str, np.ndarray] = {
        "time": time_vec if time_vec is not None else np.array([])
    }
    for key, vals in per_turbine.items():
        max_len = max(len(v) for v in vals)
        aligned = []
        for v in vals:
            if len(v) < max_len:
                aligned.append(np.pad(v, (0, max_len - len(v)), constant_values=np.nan))
            else:
                aligned.append(v[:max_len])
        result[key] = np.column_stack(aligned) if aligned else np.array([])

    return result


# =========================================================================
# FastFarmInterface — 基于 subprocess 的在线接口
# =========================================================================

_FF_PROJECT_ROOT = Path(__file__).resolve().parent.parent


class FastFarmInterface(SimulatorInterface):
    """
    FAST.Farm 在线仿真接口（基于 subprocess，兼容 v5.0.0+）。

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

    def __init__(self, config: FastFarmConfig):
        self.config = config
        self.n_turbines = config.num_turbines
        self._fastfarm_exe = config.fastfarm_exe or os.environ.get(
            "FAST_FARM_EXE",
            str(_FF_PROJECT_ROOT / "wfcrl/simulators/fastfarm/bin/FAST.Farm_x64_OMP.exe"),
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

        if self._fstf_file is None:
            self.setup()

        self._write_wind_config(wind)

    def step(self, controls: ControlInput) -> SimulationOutput:
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

        from wfcrl.simul_utils import create_ff_case as _create_ff

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

    # ========== 内部方法 ==========

    def _add_outlist(self) -> None:
        """在所有风机 .fst 文件末尾无条件追加 OutList 通道列表。"""
        if self._fstf_file is None or self._farm_base is None:
            return
        fstf = FASTInputFile(self._fstf_file)
        wt_refs = [row[3].replace('"', "") for row in fstf["WindTurbines"]]
        for wt_ref in wt_refs:
            wt_path = os.path.join(self._farm_base, wt_ref)
            if not os.path.exists(wt_path):
                continue
            with open(wt_path, "rb") as f:
                raw = f.read()
            text = raw.decode("ascii", errors="replace")
            lines = text.rstrip().split("\n")

            # 移除旧 END of input file（避免重复）
            lines = [l for l in lines if not l.strip().upper().startswith("END")]

            # 检查是否已有我们的通道
            has_our_channels = any(
                line.strip().startswith('"GenPwr"')
                for line in lines
            )
            if has_our_channels:
                continue

            # 在末尾追加 OutList 通道（不重复 OUTPUT 头部参数）
            for ch in DEFAULT_OUTLIST_CHANNELS:
                lines.append(f'"{ch}"    {ch}')
            lines.append("")
            lines.append("END of input file")

            out = "\n".join(lines).encode("ascii", errors="replace")
            out = out.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
            with open(wt_path, "wb") as f:
                f.write(out)

    def _fix_inflow_setup(self) -> None:
        if self._farm_base is None:
            return
        ip = os.path.join(self._farm_base, "InflowWind.dat")
        if not os.path.exists(ip):
            return
        wdir = self._current_wind.direction if self._current_wind else self.config.wind.direction
        prop_dir = (wdir + 90) % 360
        inflow = FASTInputFile(ip)
        inflow["PropagationDir"] = prop_dir
        inflow.write(ip)
        with open(ip, "rb") as f:
            raw = f.read()
        raw = raw.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
        text = raw.decode("ascii")
        text = re.sub(
            r"^(\s*)[0-9.+-]+\s+RotorApexOffsetPos",
            r"\1 0.0, 0.0, 0.0   RotorApexOffsetPos",
            text, flags=re.MULTILINE,
        )
        with open(ip, "wb") as f:
            f.write(text.encode("ascii"))
        speed = self._current_wind.speed if self._current_wind else self.config.wind.speed
        write_inflow_info(ip, float(speed))

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
        prop_dir = (wind.direction + 90) % 360
        inflow["PropagationDir"] = prop_dir

        if wind.wind_type == WindType.TURBSIM_BTS and wind.wind_file:
            inflow["FileName_BTS"] = f'"{wind.wind_file}"'
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
        text = raw.decode("ascii")
        text = re.sub(
            r"^(\s*)[0-9.+-]+\s+RotorApexOffsetPos",
            r"\1 0.0, 0.0, 0.0   RotorApexOffsetPos",
            text, flags=re.MULTILINE,
        )
        with open(ip, "wb") as f:
            f.write(text.encode("ascii"))

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
                ed["NacYaw"] = float(controls.yaw[i])
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

        time_vec = parsed.get("time", np.array([self._cumulative_time]))
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


# =========================================================================
# FlorisInterface — 基于 FLORIS Python API 的在线接口
# =========================================================================

class FlorisInterface(SimulatorInterface):
    """
    FLORIS 在线仿真接口。

    直接使用 FLORIS Python API。

    注意
    ----
    - FLORIS 仅支持 yaw 控制（不支持 pitch/torque）
    - FLORIS 仅原生支持稳态风 (WindType=1)，其他 WindType 自动降级
    - 输出中 thrust_n 可用；blade_loads 为代理值

    用法
    ----
    config = FlorisConfig(case_name="demo", num_turbines=3, ...)
    fl = FlorisInterface(config)
    fl.setup()
    fl.reset(wind)
    output = fl.step(ControlInput.scalar(3, yaw_deg=10))
    """

    def __init__(self, config: FlorisConfig):
        self.config = config
        self.n_turbines = config.num_turbines
        self._current_wind: Optional[WindConfig] = None
        self._step_idx = 0
        self._cumulative_time = 0.0
        self._fi: Optional[FlorisModel] = None
        self._simul_file: Optional[str] = None
        self._wind_generator = None

    # ========== SimulatorInterface 实现 ==========

    def setup(self) -> None:
        output_dir = self.config.output_dir or "."
        os.makedirs(output_dir, exist_ok=True)
        self._simul_file = create_floris_case(
            self.config.to_legacy_dict(), output_dir=output_dir,
        )
        self._fi = FlorisModel(self._simul_file)
        # 保存基准 config dict，供 curtailment 重建 FlorisModel
        import yaml as _yaml
        with open(self._simul_file, 'r') as _f:
            self._base_config_dict = _yaml.safe_load(_f)
        if 'turbine_library_path' not in self._base_config_dict.get('farm', {}):
            # FLORIS can resolve built-in turbines without explicit path
            pass
        self._base_power_thrust_table = copy.deepcopy(
            self._fi.core.farm.turbine_definitions[0]['power_thrust_table']
        )
        self._last_ratios = None

    def reset(self, wind: WindConfig) -> None:
        self._current_wind = wind
        self._step_idx = 0
        self._cumulative_time = 0.0
        if self._fi is None:
            self.setup()
        if not wind.is_floris_supported:
            warnings.warn(
                f"FLORIS does not support WindType={wind.wind_type.name}. "
                f"Falling back to steady wind (speed={wind.speed}, dir={wind.direction})."
            )
        self._wind_generator = self._make_wind_generator(wind)
        ws, wd = next(self._wind_generator)
        wd = wd % 360
        self._fi.set(wind_speeds=[ws], wind_directions=[wd])
        self._fi.run()

    # ========== Curtailment helpers (FLORIS induction factor control) ==========

    @staticmethod
    def _get_a_from_ct(ct: float) -> float:
        """Ct -> axial induction factor a."""
        ct = float(np.clip(ct, 0.0, 0.999))
        return 0.5 * (1.0 - np.sqrt(1.0 - ct))

    @staticmethod
    def _solve_new_a(a_old: float, ratio: float) -> float:
        """给定旧 a 和限功率比 ratio，求解新 a (上限 1/3)。"""
        cp_old = 4.0 * a_old * (1.0 - a_old) ** 2
        if cp_old <= 0.0:
            return 0.0
        cp_new = float(ratio) * cp_old
        def _func(a):
            return 4.0 * a * (1.0 - a) ** 2 - cp_new
        try:
            a_new = fsolve(_func, x0=a_old * ratio)[0]
            return float(np.clip(a_new, 0.0, 1.0 / 3.0))
        except Exception:
            return float(np.clip(a_old * ratio, 0.0, 1.0 / 3.0))

    @classmethod
    def _apply_curtailment(cls, base_table: dict, ratio: float) -> dict:
        """对 power_thrust_table 施加限功率比，返回新表。"""
        new_table = copy.deepcopy(base_table)
        powers = np.array(new_table['power'], dtype=np.float64)
        cts = np.array(new_table['thrust_coefficient'], dtype=np.float64)
        new_powers = powers * ratio
        new_cts = np.zeros_like(cts)
        for i, ct in enumerate(cts):
            if ct > 0.0:
                a_old = cls._get_a_from_ct(float(ct))
                a_new = cls._solve_new_a(a_old, ratio)
                new_cts[i] = 4.0 * a_new * (1.0 - a_new)
        new_table['power'] = new_powers.tolist()
        new_table['thrust_coefficient'] = new_cts.tolist()
        return new_table

    def _apply_turbine_curtailment(self, ratios: np.ndarray) -> None:
        """写 curtailed turbine yaml → turbine_library_path → 重建 FlorisModel。"""
        if self._last_ratios is not None and np.allclose(self._last_ratios, ratios, atol=0.001):
            return

        base_table = self._base_power_thrust_table
        if base_table is None:
            td0 = self._fi.core.farm.turbine_definitions[0]
            self._base_power_thrust_table = copy.deepcopy(td0['power_thrust_table'])
            base_table = self._base_power_thrust_table

        td0_full = copy.deepcopy(self._fi.core.farm.turbine_definitions[0])
        base_turb_name = td0_full.get('turbine_type', 'nrel_5MW')
        new_cfg = copy.deepcopy(self._base_config_dict)

        # 在 output_dir 下建立临时 turbine 库
        lib_dir = os.path.join(self.config.output_dir or '.', '_turbine_lib')
        os.makedirs(lib_dir, exist_ok=True)
        import yaml as _yaml

        new_types = []
        for i in range(self.n_turbines):
            r = float(np.clip(ratios[i], 0.01, 1.0))
            tname = f"{base_turb_name}_cr{i}"
            td_i = copy.deepcopy(td0_full)
            td_i['power_thrust_table'] = self._apply_curtailment(base_table, r)
            td_i['turbine_type'] = tname
            # 移除不可序列化的 Python 对象字段（如 pathlib.Path）
            for _bad_key in list(td_i.keys()):
                _v = td_i[_bad_key]
                if not isinstance(_v, (str, int, float, bool, list, dict, type(None))):
                    del td_i[_bad_key]
            # 写 turbine yaml
            tbl_path = os.path.join(lib_dir, f"{tname}.yaml")
            with open(tbl_path, 'w') as _tf:
                _yaml.dump(td_i, _tf)
            new_types.append(tname)

        new_cfg['farm']['turbine_type'] = new_types
        new_cfg['farm']['turbine_library_path'] = lib_dir

        self._fi = FlorisModel(new_cfg)
        self._last_ratios = np.asarray(ratios, dtype=np.float64).copy()

    # ========== step() — 3-mode dispatch (FLORIS) ==========

    def step(self, controls: ControlInput) -> SimulationOutput:
        if self._fi is None:
            raise RuntimeError("Call setup() and reset() before step()")

        ws, wd = next(self._wind_generator)
        wd = wd % 360

        # Determine control mode
        mode = int(controls.mode[0]) if (
            controls.mode is not None and len(controls.mode) > 0
        ) else 0

        # --- Induction factor (curtailment), modes 1 & 2 ---
        if mode in (1, 2):
            ratio = controls.power if controls.power is not None else np.ones(self.n_turbines)
            ratio = np.asarray(ratio, dtype=np.float64).ravel()[:self.n_turbines]
            ratio = np.clip(ratio, 0.01, 1.0)
            self._apply_turbine_curtailment(ratio)
        elif mode == 0 and getattr(self, '_last_ratios', None) is not None:
            # 恢复全功率 turbine 定义
            self._apply_turbine_curtailment(np.ones(self.n_turbines))

        # --- Yaw, modes 0 & 2 ---
        yaw = controls.yaw.reshape(1, -1).astype(np.float64) if controls.yaw is not None else np.zeros((1, self.n_turbines))
        self._fi.set(wind_speeds=[ws], wind_directions=[wd], yaw_angles=yaw)
        self._fi.run()

        # --- Collect output ---
        power_mw = self._fi.get_turbine_powers().flatten().reshape(1, -1) / 1e6
        ws_arr, wd_arr = self._local_wind_measurements()
        yaw_arr = self._fi.core.farm.yaw_angles.squeeze().reshape(1, -1)

        thrust = None
        try:
            t = self._fi.get_turbine_thrusts()
            if t is not None:
                thrust = t.flatten().reshape(1, -1)
        except Exception:
            pass

        tl, vu, vv, vw = self._local_load_proxies()
        blade_loads = np.stack([tl, vu, vv, vw], axis=-1).reshape(1, self.n_turbines, -1)

        output = SimulationOutput(
            time=np.array([self._cumulative_time]),
            power_mw=power_mw,
            wind_speed=ws_arr.reshape(1, -1),
            wind_direction=wd_arr.reshape(1, -1),
            yaw_deg=yaw_arr,
            thrust_n=thrust,
            blade_loads=blade_loads,
            metadata={"step": self._step_idx, "dt": self.config.dt, "simulator": "FLORIS", "mode": mode},
        )
        self._step_idx += 1
        self._cumulative_time += self.config.dt
        return output

    def close(self) -> None:
        self._step_idx = 0
        self._cumulative_time = 0.0
        self._fi = None
        self._base_power_thrust_table = None
        self._last_ratios = None
        self._base_power_thrust_table = None

    # ========== 内部方法 ==========

    def _make_wind_generator(self, wind: WindConfig):
        if wind.wind_time_series is not None:
            ts = wind.wind_time_series

            def gen():
                start = np.random.randint(0, ts.shape[0])
                rolled = np.r_[ts[start:], ts[:start]]
                for row in rolled:
                    yield float(row[0]), float(row[1])
            return gen()
        elif wind.segments:
            segs = wind.segments

            def gen():
                for seg in segs:
                    n_steps = max(1, int(seg.duration / self.config.dt))
                    for _ in range(n_steps):
                        yield seg.speed, seg.direction
            return gen()
        else:
            ws, wd = wind.speed, wind.direction

            def gen():
                while True:
                    yield ws, wd
            return gen()

    def _local_wind_measurements(self) -> Tuple[np.ndarray, np.ndarray]:
        u = self._fi.core.flow_field.u
        v = self._fi.core.flow_field.v
        velocities = np.cbrt(np.mean(u**3, axis=(2, 3))).squeeze()
        directions = self._fi.wind_directions[0] - np.degrees(
            np.arctan2(np.mean(v, axis=(2, 3)), np.mean(u, axis=(2, 3)))
        )
        return velocities, directions.squeeze() % 360

    def _local_load_proxies(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        turbulences = self._fi.core.flow_field.turbulence_intensity_field.squeeze()
        u = self._fi.core.flow_field.u
        v = self._fi.core.flow_field.v
        w = self._fi.core.flow_field.w
        var_u = np.std(u, axis=(2, 3)).squeeze()
        var_v = np.std(v, axis=(2, 3)).squeeze()
        var_w = np.std(w, axis=(2, 3)).squeeze()
        return turbulences, var_u, var_v, var_w


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

    def __init__(self, config: FastFarmConfig):
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

        from wfcrl.simul_utils import create_ff_case as _create_ff, create_dll
        self._fstf_file = _create_ff(full_config, output_dir=self.config.output_dir)
        self._farm_base = os.path.dirname(self._fstf_file)

        self._add_outlist()
        self._fix_inflow_setup()
        self._fix_initial_yaw()

        # 部署 DISCON bridge DLL + DISCON.IN
        create_dll(self._fstf_file)

        # controls.txt 放在 FarmInputs 同级（DISCON 从 cwd 读取）
        self._controls_file = os.path.join(self._farm_base, "controls.txt")

        print(f"ContinuousFastFarmInterface ready: {self.n_turbines} turbines")

    def _fix_initial_yaw(self) -> None:
        """根据入流风向设置各风机 ED 文件的初始 NacYaw，
        避免 t=0 时 WakeDynamics 检测到 90° 偏航误差而中止。"""
        if self._fstf_file is None or self._farm_base is None:
            return
        wdir = self._current_wind.direction if self._current_wind else self.config.wind.direction
        # OpenFAST 坐标: 0°=东(+X), 90°=北(+Y)
        initial_yaw = (270 - wdir) % 360

        fstf = FASTInputFile(self._fstf_file)
        wt_refs = [row[3].replace('"', "") for row in fstf["WindTurbines"]]
        for wt_ref in wt_refs:
            wt_path = os.path.join(self._farm_base, wt_ref)
            if not os.path.exists(wt_path):
                continue
            wt = FASTInputFile(wt_path)
            ed_rel = wt["EDFile"].replace('"', "")
            ed_path = os.path.join(self._farm_base, ed_rel)
            if not os.path.exists(ed_path):
                continue
            ed = FASTInputFile(ed_path)
            ed["NacYaw"] = initial_yaw
            ed.write(ed_path)
            # 修复换行符
            with open(ed_path, "rb") as f:
                raw = f.read()
            raw = raw.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
            with open(ed_path, "wb") as f:
                f.write(raw)

    def _write_initial_controls(self) -> None:
        """在 FAST.Farm 启动前写入初始 controls.txt（5-mode protocol）。
        初始用 mode=0 零偏航增量，让 ROSCO 自行对风。"""
        if self._controls_file is None:
            return
        lines = ["step=0"]
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

    def stop(self) -> SimulationOutput:
        """发停止信号，等进程结束，解析 .outb 获取完整输出。"""
        if self._process is None:
            raise RuntimeError("No running process")

        # 写 END 标记
        if self._controls_file:
            with open(self._controls_file, 'w') as f:
                f.write('step=-1\nEND\n')

        self._process.wait()
        self._process = None
        if hasattr(self, '_proc_log') and self._proc_log is not None:
            self._proc_log.close()
            self._proc_log = None

        return self._parse_step_output()

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
        for t in range(self.n_turbines):
            m = int(controls.mode[t]) if t < len(controls.mode) else 0
            y = controls.yaw[t] if t < len(controls.yaw) else 0.0
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
                                    if v:  # "step=0" 格式
                                        try:
                                            vals[k] = float(v)
                                        except ValueError:
                                            pass
                                        last_label = None
                                    else:  # "step=" 后跟独立 token
                                        last_label = k
                                elif last_label is not None:
                                    # "step=" + "0" 格式
                                    try:
                                        vals[last_label] = float(part)
                                    except ValueError:
                                        pass
                                    last_label = None
                except (OSError, IOError):
                    continue
                if 'step' not in vals or int(vals['step']) != expected_step:
                    continue
                # 等待仿真时间推进到目标时间（允许 dt/2 容差）
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
            time.sleep(0.1)

        if data is None:
            return SimulationOutput(
                time=np.array([self._cumulative_time]),
                power_mw=np.zeros((1, n)),
                metadata={"step": self._step_idx, "warning": "timeout waiting for DISCON"},
            )

        # 初始化数组（缺失的风机保持 0）
        power_mw = np.zeros(n)
        wind_speed = np.zeros(n)
        yaw_deg = np.zeros(n)
        pitch_deg = np.zeros(n)
        torque_nm = np.zeros(n)
        rotor_speed = np.zeros(n)
        blade_loads = np.zeros((n, 3))

        # 填充所有风机数据：已有的用各自测量值，缺失的保持 0
        for t_id, vals in data.items():
            i = t_id - 1
            power_mw[i] = vals.get('genpwr', 0.0) / 1000.0       # Fortran writes kW → MW
            wind_speed[i] = vals.get('wind_x', 0.0)
            yaw_deg[i] = vals.get('nacyaw', 0.0)                # Fortran writes rad*57.3 = degrees
            pitch_deg[i] = vals.get('blpitch', 0.0)              # Fortran writes rad*57.3 = degrees
            torque_nm[i] = vals.get('gentq', 0.0)                # avrSWap(23) = GenTqMeas (Nm)
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
