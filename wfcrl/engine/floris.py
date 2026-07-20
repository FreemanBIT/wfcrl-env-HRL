"""
FLORIS 在线仿真接口
==================
提供 FlorisInterface — 基于 FLORIS Python API 的在线仿真接口。

注意
----
- FLORIS 仅支持 yaw 控制（不支持 pitch/torque）
- FLORIS 仅原生支持稳态风 (WindType=1)，其他 WindType 自动降级
- 输出中 thrust_n 可用；blade_loads 为代理值（湍流强度 + 速度分量方差）

用法
----
from wfcrl.engine import FlorisInterface
from wfcrl.config import ControlInput

config = FlorisConfig(case_name="demo", num_turbines=3, ...)
fl = FlorisInterface(config)
fl.setup()
fl.reset(wind)
output = fl.step(ControlInput.scalar(3, yaw_deg=10))
"""

from __future__ import annotations

import copy
import os
import warnings
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml
from floris import FlorisModel
from scipy.optimize import fsolve

from wfcrl.config.control import ControlInput
from wfcrl.config.output import SimulationOutput
from wfcrl.config.types import WindConfig
from wfcrl.engine._floris_case import (
    create_floris_case,
)
from wfcrl.engine.base import (
    SimulatorInterface,
)
from wfcrl.engine.angle_utils import (
    nacyaw_from_misalignment,
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

    def __init__(self, config):
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
        with open(self._simul_file, 'r') as _f:
            self._base_config_dict = yaml.safe_load(_f)
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

        new_types = []
        for i in range(self.n_turbines):
            r = float(np.clip(ratios[i], 0.01, 1.0))
            tname = f"{base_turb_name}_cr{i}"
            td_i = copy.deepcopy(td0_full)
            td_i['power_thrust_table'] = self._apply_curtailment(base_table, r)
            td_i['turbine_type'] = tname
            # 移除不可序列化的 Python 对象字段
            for _bad_key in list(td_i.keys()):
                _v = td_i[_bad_key]
                if not isinstance(_v, (str, int, float, bool, list, dict, type(None))):
                    del td_i[_bad_key]
            # 写 turbine yaml
            tbl_path = os.path.join(lib_dir, f"{tname}.yaml")
            with open(tbl_path, 'w') as _tf:
                yaml.dump(td_i, _tf)
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
