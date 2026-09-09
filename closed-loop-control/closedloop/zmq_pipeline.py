"""zmq_pipeline.py — 新架构（ZeroMQ）场控仿真流水线共享构建器（Phase 7+）。

examples 脚本统一使用本模块：由 closedloop.case_config.FarmCase 生成 wfcrl
FastFarmConfig -> FastFarmZmqEngine（setup/start）-> ZmqFarmBridge -> ZmqPlant。
"""
from __future__ import annotations

import os
import time
from typing import Optional

from wfcrl.config import WindConfig
from wfcrl.config.layout import LayoutRegistry
from wfcrl.config.simulator import FastFarmConfig
from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine

from .case_config import FarmCase
from .bridge import ZmqFarmBridge
from .runner import ZmqPlant


def resolve_layout(case_name: str):
    reg = LayoutRegistry.from_builtin()
    if case_name in reg:
        return reg.get(case_name)
    import re
    m = re.match(r'Turb(\d+)_Row(\d+)$', case_name)
    if m:
        n = int(m.group(1))
        return reg.get_row_layout(n)
    raise ValueError('unknown case %r; available: %s' % (case_name, reg.list_names()))


def build_farm_case(case_name: str, wind_speed: float, wind_direction: float,
                    **overrides) -> FarmCase:
    layout = resolve_layout(case_name)
    return FarmCase.from_wfcrl_layout(layout, wind_speed=wind_speed,
                                      wind_direction=wind_direction, **overrides)


def fastfarm_config_from_case(case: FarmCase, dt: float, max_iter: int,
                              output_dir: str,
                              wind_time_series: Optional[str] = None) -> FastFarmConfig:
    from wfcrl.config.types import WindType
    if wind_time_series:
        wind = WindConfig(speed=case.wind_speed, direction=case.wind_direction,
                          wind_type=WindType.TURBSIM_BTS, wind_file=wind_time_series)
    else:
        wind = WindConfig(speed=case.wind_speed, direction=case.wind_direction)
    return FastFarmConfig(
        case_name=case.name, num_turbines=case.n_turbines,
        xcoords=list(case.layout_x), ycoords=list(case.layout_y),
        dt=dt, max_iter=max_iter, wind=wind, output_dir=output_dir,
    )


def launch_farm(case: FarmCase, dt: float, max_iter: int, output_dir: str,
                cmd_port: int = 5556, state_port: int = 5557, env_extra=None,
                wind_time_series: Optional[str] = None):
    """生成 case、启动 FAST.Farm（FCR+ZeroMQ 集成 DLL），返回 (engine, plant)。"""
    os.makedirs(output_dir, exist_ok=True)
    config = fastfarm_config_from_case(case, dt, max_iter, output_dir,
                                       wind_time_series=wind_time_series)
    engine = FastFarmZmqEngine(config, cmd_port=cmd_port, state_port=state_port)
    engine.setup()
    engine.start(env_extra=env_extra)
    bridge = ZmqFarmBridge(engine, case.n_turbines,
                           wind_direction=case.wind_direction)
    plant = ZmqPlant(bridge, step_timeout=180.0)
    return engine, bridge, plant


def wait_first_state(engine, timeout_s: float = 60.0) -> dict:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        st = engine.get_state(timeout_s=5.0)
        if st is not None and st.get('fast_step', 0) > 0:
            return st
    raise TimeoutError('FAST.Farm 未在 %gs 内发出状态帧' % timeout_s)


__all__ = ['build_farm_case', 'fastfarm_config_from_case', 'launch_farm',
           'wait_first_state', 'resolve_layout']