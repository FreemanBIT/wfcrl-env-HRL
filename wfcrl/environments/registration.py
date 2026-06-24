"""
环境注册与工厂函数
==================
提供 `make(env_id)` 创建 Gym/PettingZoo 风场控制环境。
"""

from __future__ import annotations

import math
import re
from itertools import product
from typing import Union

from wfcrl.config import WindConfig
from wfcrl.environments.data_cases import (
    DefaultControl,
    FarmRowFastfarm,
    FarmRowFloris,
    named_cases_dictionary,
)
from wfcrl.interface import FastFarmInterface, FlorisInterface
from wfcrl.multiagent_env import MAWindFarmEnv
from wfcrl.simple_env import WindFarmEnv
from wfcrl.wrappers import AECLogWrapper, LogWrapper

env_pattern = r"(Dec_)*(\w+\d*_)(\w+)"
layout_pattern = r"Turb(\d+)_Row(\d+)"

registered_simulators = ["Fastfarm", "Floris"]
registered_layouts = list(named_cases_dictionary.keys())
registered_layouts.extend([f"Turb{n}_Row1_" for n in range(1, 13)])
control_types = ["", "Dec_"]
registered_envs = [
    "".join(env_descs)
    for env_descs in product(control_types, registered_layouts, registered_simulators)
]


def get_default_control(controls):
    dc = DefaultControl()
    cd = {}
    if "yaw" in controls:
        cd["yaw"] = dc.yaw
    if "pitch" in controls:
        cd["pitch"] = dc.pitch
    if "torque" in controls:
        cd["torque"] = dc.torque
    if "power" in controls:
        cd["power"] = dc.power
    return cd


def get_case(name: str, simulator: str):
    si = registered_simulators.index(simulator)
    if name in named_cases_dictionary:
        return named_cases_dictionary[name][si]

    match = re.match(layout_pattern, name)
    n_turbs = int(match.group(1))
    n_rows = int(match.group(2))
    assert n_rows == 1

    cls = FarmRowFastfarm if si == 0 else FarmRowFloris
    return cls(
        num_turbines=n_turbs,
        xcoords=cls.get_xcoords(n_turbs),
        ycoords=cls.get_ycoords(n_turbs),
        dt=cls.dt,
        t_init=cls.t_init,
        buffer_window=cls.buffer_window,
        set_wind_direction=cls.set_wind_direction,
        set_wind_speed=cls.set_wind_speed,
    )


def validate_case(env_id, case):
    try:
        assert len(case.xcoords) == len(case.ycoords), "x/y coords mismatch"
    except Exception as e:
        raise ValueError(f"Invalid case {env_id}: {e}")


def make(env_id: str, controls: Union[dict, list] = ["yaw"], log=True, **env_kwargs):
    """
    创建风场控制 RL 环境。

    Parameters
    ----------
    env_id : str
        环境 ID，如 "DafengH1_Floris", "Dec_Ablaincourt_Fastfarm"。
    controls : Union[dict, list]
        控制配置。
    log : bool
        是否包装日志 wrapper。
    **env_kwargs
        传递给 env 构造函数的额外参数。
    """
    if env_id not in registered_envs:
        raise ValueError(f"Unknown env: {env_id}. Use list_envs() to see available.")

    match = re.match(env_pattern, env_id)
    decentralized = match.group(1)
    name = match.group(2)
    simulator = match.group(3)

    case = get_case(name, simulator)
    validate_case(env_id, case)

    env_class = MAWindFarmEnv if decentralized == "Dec_" else WindFarmEnv

    if not isinstance(controls, dict):
        controls = get_default_control(controls)

    # ---- 从 FarmCase 构建仿真配置并实例化接口 ----
    wind = WindConfig(
        speed=case.simul_params.get("speed", 8.0),
        direction=case.simul_params.get("direction", 270.0),
    )

    if "wind_time_series" in env_kwargs:
        case.wind_time_series = env_kwargs.pop("wind_time_series")
        wind.wind_time_series = case.wind_time_series
    if "path_to_simulator" in env_kwargs:
        case.path_to_simulator = env_kwargs.pop("path_to_simulator")
    if "t_init" in env_kwargs:
        case.t_init = env_kwargs.pop("t_init")

    # 实例化仿真器接口
    if simulator == "Fastfarm":
        from wfcrl.simul_config import FastFarmConfig as FFC
        config = FFC(
            case_name=name.rstrip("_"),
            num_turbines=case.num_turbines,
            xcoords=case.xcoords,
            ycoords=case.ycoords,
            dt=case.dt,
            max_iter=case.max_iter,
            wind=wind,
            t_init=case.t_init,
        )
        if hasattr(case, 'path_to_simulator') and case.path_to_simulator:
            config.fastfarm_exe = case.path_to_simulator
        interface = FastFarmInterface(config)
    else:
        from wfcrl.simul_config import FlorisConfig as FC
        config = FC(
            case_name=name.rstrip("_"),
            num_turbines=case.num_turbines,
            xcoords=case.xcoords,
            ycoords=case.ycoords,
            dt=case.dt,
            max_iter=case.max_iter,
            wind=wind,
        )
        if hasattr(case, 'turbine_type') and case.turbine_type:
            config.turbine_type = case.turbine_type
        if hasattr(case, 'turbine_library_path') and case.turbine_library_path:
            config.turbine_library_path = case.turbine_library_path
        interface = FlorisInterface(config)

    interface.setup()
    interface.reset(wind)

    env = env_class(
        interface=interface,
        farm_case=case,
        controls=controls,
        start_iter=math.ceil(case.t_init / case.dt),
        **env_kwargs,
    )

    if log:
        wrapper_class = AECLogWrapper if decentralized == "Dec_" else LogWrapper
        env = wrapper_class(env)

    return env


def list_envs():
    return registered_envs
