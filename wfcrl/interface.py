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
# 风向 ↔ 机舱朝向 的统一换算（关键：与 PropagationDir 约定一致）
# =========================================================================
# InflowWind 用 PropagationDir = (风向 + 90) % 360 旋转来流（稳态/湍流一致）。
# 为使转子**对准来流**，机舱绝对朝向 NacYaw 必须等于 (风向 + 90)，而**不是**
# 旧代码里的 (270 - 风向) —— 后者在风向偏移时会让转子相对来流多偏 2×偏移量，
# 在 ±20° 工况下产生 ~40° 基准失准，触发 chi≈110°+ 的偏斜与
# "Rotor diameter must be greater than zero" 中止。
# NacYaw 必须落在 ElastoDyn 要求的 (-180,180]。

def _set_fast_scalar(path: str, key: str, value: str) -> None:
    """在 OpenFAST 输入文件中就地修改一个标量参数（按注释列的参数名匹配）。

    OpenFAST 行格式为：`   <value>   <KEY>   - description`。本函数按第二列的
    KEY 精确匹配该行，替换第一列的值，保留其余内容与 CRLF。仅改**首个**匹配行。
    用原文本编辑而非 FASTInputFile.write()，以免丢失文件里已追加的 OutList 通道。
    """
    if not os.path.exists(path):
        return
    with open(path, "rb") as f:
        raw = f.read()
    text = raw.decode("ascii", errors="replace").replace("\r\n", "\n")
    lines = text.split("\n")
    for i, line in enumerate(lines):
        # 参数行：值 + 空白 + KEY + 空白/行尾（KEY 作为独立 token 出现在第二列）
        toks = line.split()
        if len(toks) >= 2 and toks[1] == key:
            # 用新值替换第一 token，尽量保留原有列宽风格
            # 重建：<value><原分隔><KEY> ... 其余保留
            # 简单稳妥：把该行第一 token 换成 value
            idx0 = line.find(toks[0])
            new_line = line[:idx0] + value + line[idx0 + len(toks[0]):]
            lines[i] = new_line
            break
    out = "\n".join(lines).encode("ascii", errors="replace")
    out = out.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
    with open(path, "wb") as f:
        f.write(out)


class FastFarmAborted(RuntimeError):
    """FAST.Farm 进程在仿真过程中意外退出/崩溃/挂起时抛出。

    上层（回放驱动）捕获它后可立即：(1) 停止本工况，(2) 保存已推进出的部分结果，
    (3) 快速切换到下一个工况，而不必逐步空等超时。
    """
    pass


def _wrap180(x: float) -> float:
    """归一化角度到 (-180, 180]。"""
    return ((float(x) + 180.0) % 360.0) - 180.0


def nacyaw_face_wind(wind_direction_deg: float) -> float:
    """转子对准来流时的绝对机舱朝向（度，∈(-180,180]）。

    与 InflowWind PropagationDir=(wd+90) 约定一致：NacYaw_base = wrap180(wd+90)。
    """
    return 0.0


def nacyaw_from_misalignment(wind_direction_deg: float, yaw_misalign_deg: float) -> float:
    """由“相对来流的偏航失准角”换算成绝对 NacYaw（度，∈(-180,180]）。

    FLORIS/LUT 的 yaw 是相对来流的失准角；FAST.Farm/ED 需要绝对机舱朝向。
    """
    return _wrap180(float(yaw_misalign_deg))


def misalignment_from_nacyaw(wind_direction_deg: float, nacyaw_abs_deg: float) -> float:
    """由绝对 NacYaw 反算“相对来流的偏航失准角”（度，∈(-180,180]）。

    用于把 FAST.Farm 回读的绝对机舱角换算回与 FLORIS 一致的失准角口径。
    """
    return _wrap180(float(nacyaw_abs_deg))


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

    # 各通道应写入的模块 OutList（ElastoDyn / ServoDyn / InflowWind）。
    # 关键：ElastoDyn 通道（YawPzn/BldPitch1/RotSpeed/RootM*）必须写进 ED 的 OutList，
    # 写在 .fst 顶层会被忽略 —— 这正是之前 CSV 里没有测量 yaw/pitch 的原因。
    _ED_OUTS = ["YawPzn", "BldPitch1", "RotSpeed", "RootMIP1", "RootMOoP1", "RootMzb1"]
    _SRV_OUTS = ["GenPwr", "GenTq"]
    _IFW_OUTS = ["Wind1VelX", "Wind1VelY", "Wind1VelZ"]

    @staticmethod
    def _inject_outlist_channels(module_path: str, channels) -> None:
        """把 channels 注入某模块输入文件的 OutList 段（若尚未存在）。

        在包含 'OutList' 的行之后、'END' 段之前插入 "\"Ch\"  Ch" 行。
        已存在的通道不重复添加。保持 CRLF。
        """
        if not os.path.exists(module_path):
            return
        with open(module_path, "rb") as f:
            raw = f.read()
        text = raw.decode("ascii", errors="replace")
        text = text.replace("\r\n", "\n")
        lines = text.split("\n")

        # 找 OutList 段头：该行的**参数名列**（第一个 token，或紧跟数值后的 token）
        # 是 'OutList'。OpenFAST 里这行形如：
        #   "              OutList      - The next line(s) ... OutListParameters.xlsx ..."
        # 特征：包含独立 token 'OutList'，且**不是**以引号开头的通道行，
        # 也**不是** END 行。用 token 精确匹配避免误命中 END/说明文字。
        outlist_idx = None
        for i, l in enumerate(lines):
            s = l.strip()
            if s.startswith('"') or s.upper().startswith("END"):
                continue
            toks = s.split()
            if toks and toks[0] == "OutList":
                outlist_idx = i
                break
        if outlist_idx is None:
            return  # 该模块没有 OutList 段，跳过

        # 找该段的 END（第一个以 END 开头的行，在 outlist_idx 之后）
        end_idx = None
        for i in range(outlist_idx + 1, len(lines)):
            if lines[i].strip().upper().startswith("END"):
                end_idx = i
                break
        if end_idx is None:
            end_idx = len(lines)

        # 现有通道名集合（该段内已引用的）
        existing = set()
        for i in range(outlist_idx + 1, end_idx):
            s = lines[i].strip()
            if s.startswith('"'):
                nm = s.split('"')
                if len(nm) >= 2:
                    existing.add(nm[1].strip())

        # 待插入的新通道
        to_add = [c for c in channels if c not in existing]
        if not to_add:
            return
        insert_lines = [f'"{c}"    {c}' for c in to_add]
        lines = lines[:end_idx] + insert_lines + lines[end_idx:]

        out = "\n".join(lines).encode("ascii", errors="replace")
        out = out.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
        with open(module_path, "wb") as f:
            f.write(out)

    def _add_outlist(self) -> None:
        """把测量通道注入各风机的**模块** OutList（ElastoDyn/ServoDyn/InflowWind）。

        这样 .outb 才会包含**执行结果**：实测机舱偏航(YawPzn)、变桨(BldPitch1)、
        转速、功率、转矩、叶根载荷、风速。之前把通道写在 .fst 顶层会被忽略，
        导致 CSV 缺少测量 yaw/pitch。
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
                os.path.join(self._farm_base, ed_rel), self._ED_OUTS)
            # ServoDyn
            try:
                srv_rel = wt["ServoFile"].replace('"', "")
                self._inject_outlist_channels(
                    os.path.join(self._farm_base, srv_rel), self._SRV_OUTS)
            except Exception:
                pass
            # InflowWind（路径可能相对 FarmInputs）
            try:
                ifw_rel = wt["InflowFile"].replace('"', "")
                ifw_path = os.path.join(self._farm_base, ifw_rel)
                self._inject_outlist_channels(ifw_path, self._IFW_OUTS)
            except Exception:
                pass

    def _fix_inflow_setup(self) -> None:
        if self._farm_base is None:
            return
        ip = os.path.join(self._farm_base, "InflowWind.dat")
        if not os.path.exists(ip):
            return
        wdir = self._current_wind.direction if self._current_wind else self.config.wind.direction

        # 风场旋转法下 PropagationDir 必须恒为 0（风沿 +X）。
        # layout_for_case() 已把风机列旋转 -offset 使入流沿 +X；
        # 若同时设 PropagationDir=(wd+90) 会和布局旋转叠加，导致 Grid4D 在
        # 错误坐标系中采样，运行中报 "Outside the grid bounds".
        # wd=0 时 (270+90)%360=0 偶然正确，故头对风不报错，偏差工况则崩溃。
        prop_dir = 0.0
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
        # 风场旋转法：PropagationDir 恒为 0，风向已由 layout_for_case 几何旋转体现。
        prop_dir = 0.0
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
                # controls.yaw 是“相对来流的偏航失准角”(FLORIS 约定)；ED 的 NacYaw
                # 是绝对机舱朝向。换算见 nacyaw_from_misalignment（与 PropagationDir 一致）。
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
        # .outb 的 YawPzn 是绝对机舱朝向(度)；换算回相对来流失准角，与 FLORIS 口径一致。
        # 风场旋转法下 PropagationDir=0，失准角 = 绝对 NacYaw（见 misalignment_from_nacyaw）。
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
            nacyaw_abs = nacyaw_from_misalignment(wdir, mis)  # 绝对机舱朝向 ∈(-180,180]
            # 用**原文本替换**改写 NacYaw / YawDOF，避免 FASTInputFile.write() 丢掉
            # 之前注入 ED OutList 的测量通道（YawPzn/BldPitch1 等）。
            _set_fast_scalar(ed_path, "NacYaw", f"{nacyaw_abs:.4f}")
            _set_fast_scalar(ed_path, "YawDOF", "False")
        self._fixed_yaw_applied = arr.copy()

    def _fix_initial_yaw(self) -> None:
        """兼容旧接口：默认所有风机偏航对准来流（失准角 0）、锁定偏航 DOF。
        真正的 per-turbine 偏航由 set_fixed_yaw() 在 start() 前设置。"""
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

        # 解析 .outb（可能是完整或部分）。解析失败时按需返回空输出而非抛出。
        try:
            return self._parse_step_output()
        except Exception as e:  # noqa
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
        # controls.yaw 是“相对来流的偏航失准角”(FLORIS 约定)；DISCON_bridge.f90
        # 的 cmd_yaw 被当作**绝对**机舱朝向目标，故换算与 ED 一致（与 PropagationDir 一致）。
        wdir = (self._current_wind.direction
                if self._current_wind else self.config.wind.direction)
        for t in range(self.n_turbines):
            m = int(controls.mode[t]) if t < len(controls.mode) else 0
            y_misalign = controls.yaw[t] if t < len(controls.yaw) else 0.0
            y = nacyaw_from_misalignment(wdir, float(y_misalign))   # 绝对 NacYaw 目标 ∈(-180,180]
            p = controls.pitch[t] if t < len(controls.pitch) else 0.0
            pw = controls.power[t] if controls.power is not None and t < len(controls.power) else 0.0
            mp = controls.min_pitch[t] if controls.min_pitch is not None and t < len(controls.min_pitch) else 0.0
            lines.append(f"T{t+1} mode={m} yaw={y:.3f} pitch={p:.3f} power={pw:.3f} minpitch={mp:.3f}")
        lines.append("END")

        with open(self._controls_file, 'w') as f:
            f.write('\n'.join(lines) + '\n')

    def _read_measurements_with_poll(self, timeout: float = 45.0) -> SimulationOutput:
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
            # 关键（快速失败）：检查 FAST.Farm 进程是否已退出/崩溃。
            # 否则一旦 FAST.Farm abort，本步会空等满 timeout，后续每一步都再等一次
            # → 整个工况卡死很久。检测到进程退出立即抛异常，让上层保存已有结果并跳到下一工况。
            if self._process is not None and self._process.poll() is not None:
                rc = self._process.returncode
                raise FastFarmAborted(
                    f"FAST.Farm process exited (returncode={rc}) at step={self._step_idx} "
                    f"while waiting for measurements (likely internal abort; see "
                    f"fastfarm_continuous.log)."
                )
            time.sleep(0.1)

        if data is None:
            # 超时但进程仍在：也当作失败快速抛出，避免逐步累积等待。
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

        # 报告的 nacyaw 是 OpenFAST 绝对机舱朝向(度)；换算回相对来流失准角，
        # 与 FLORIS 口径一致。换算与下发对称（与 PropagationDir 一致）。
        _wdir = (self._current_wind.direction
                 if self._current_wind else self.config.wind.direction)

        def _to_misalign(nac_abs_deg: float) -> float:
            return misalignment_from_nacyaw(_wdir, nac_abs_deg)

        # 填充所有风机数据：已有的用各自测量值，缺失的保持 0
        for t_id, vals in data.items():
            i = t_id - 1
            power_mw[i] = vals.get('genpwr', 0.0) / 1000.0       # Fortran writes kW → MW
            wind_speed[i] = vals.get('wind_x', 0.0)
            yaw_deg[i] = _to_misalign(vals.get('nacyaw', 0.0))   # 绝对→相对来流失准角
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
