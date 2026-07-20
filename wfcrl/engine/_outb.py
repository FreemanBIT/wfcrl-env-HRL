"""
FAST.Farm .outb / .out 文件解析
================================
内部模块，负责解析 FAST.Farm 输出的二进制 (.outb) 和文本 (.out) 文件。

从每个风机的独立输出文件中提取通道数据，按 turbine 维度堆叠。

用法
----
# (内部使用，不直接导出)
from wfcrl.engine._outb import _parse_outb_file, _parse_all_outb

parsed = _parse_all_outb(
    farm_base="/path/to/FarmInputs",
    prefix="demo_0000",
    n_turbines=3,
)
# → {"time": ndarray(T,), "power": ndarray(T, 3), "yaw": ndarray(T, 3), ...}
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional

import numpy as np
from openfast_toolbox.io import FASTOutputFile


# =========================================================================
# 默认 OutList 通道 — 映射到 SimulationOutput 字段
# =========================================================================

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


# =========================================================================
# _parse_outb_file — 单文件解析
# =========================================================================

def _parse_outb_file(
    outb_path: str,
    channel_map: Optional[Dict[str, str]] = None,
) -> Dict[str, np.ndarray]:
    """
    解析 FAST.Farm .outb 或 .out 文件，提取指定通道。

    Parameters
    ----------
    outb_path : str
        .outb 或 .out 文件的完整路径。
    channel_map : dict, optional
        通道名 → 输出键的映射。默认使用 CHANNEL_TO_OUTPUT。

    Returns
    -------
    Dict[str, np.ndarray]
        {输出键: 值数组}，每个数组 shape 为 (n_time_steps,)。
        始终包含 "time" 键。
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


# =========================================================================
# _parse_all_outb — 多风机解析并堆叠
# =========================================================================

def _parse_all_outb(
    farm_base: str,
    prefix: str,
    n_turbines: int,
    channel_map: Optional[Dict[str, str]] = None,
) -> Dict[str, np.ndarray]:
    """
    解析所有风机的 .outb 文件，按风机维度堆叠。

    遍历 T1, T2, ..., T<n_turbines> 的输出文件，将各风机数据
    按列合并（每个通道的数组 shape 从 (T,) → (T, n_turbines)）。

    Parameters
    ----------
    farm_base : str
        FarmInputs 目录路径（包含 .T*.outb 文件）。
    prefix : str
        输出文件前缀（不含 .T1.outb 扩展名）。
    n_turbines : int
        风机数量。
    channel_map : dict, optional
        通道名 → 输出键的映射。默认使用 CHANNEL_TO_OUTPUT。

    Returns
    -------
    Dict[str, np.ndarray]
        每个键的数组 shape 为 (n_time_steps, n_turbines)。
        始终包含 "time" 键（第一台风机的 time 向量）。

    注意
    ----
    若某一风机的输出文件缺失，其对应列保持 NaN。
    若某风机的时间长度不一致，通过截断或补零对齐。
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
