"""
WindFarmMDP — 风场马尔可夫决策过程
===================================
实现风场控制的底层 MDP：
- 状态：自由流风速/风向 + per-turbine yaw/pitch/torque
- 动作：离散或连续的偏航/变桨/转矩增量

controls 示例:
    {"yaw": (-20, 20, 2), "pitch": (-10, 10, 1)}
"""

from __future__ import annotations

import copy
from collections import OrderedDict
from typing import Dict, Iterable, Optional
from warnings import warn

import numpy as np
from gymnasium import spaces

from wfcrl.config import ControlInput, SimulationOutput, WindConfig
from wfcrl.environments import FarmCase
from wfcrl.interface import SimulatorInterface


def clip_to_dict_space(element: dict, space: spaces.Dict) -> dict:
    for name, value in element.items():
        element[name] = np.clip(value, space[name].low, space[name].high)
    return element


class WindFarmMDP:
    """风场 MDP。"""

    CONTROL_SET = ["yaw", "pitch", "mode", "power", "min_pitch"]
    POSSIBLE_STATE_ATTRIBUTES = [
        "freewind_measurements", "wind_speed", "wind_direction",
        "yaw", "pitch", "mode", "power", "min_pitch",
    ]
    DEFAULT_BOUNDS = {
        "wind_speed": [3, 28], "wind_direction": [0, 360],
        "yaw": [-40, 40], "pitch": [0, 45],
        "mode": [0, 4], "power": [0, 10], "min_pitch": [0, 45],
    }
    ACTUATORS_RATE = {"yaw": 0.3, "pitch": 8}

    def __init__(
        self,
        interface: SimulatorInterface,
        farm_case: FarmCase,
        controls: dict,
        continuous_control: bool = True,
        start_iter: int = 0,
        horizon: int = int(1e6),
    ):
        farm_case.max_iter = horizon
        self.interface = interface
        self.num_turbines = farm_case.num_turbines
        self.continuous_control = continuous_control
        self.horizon = horizon
        self.start_iter = start_iter
        self.farm_case = farm_case
        self._current_wind: Optional[WindConfig] = None
        self._step_count = 0
        self._last_powers = np.zeros(self.num_turbines, dtype=np.float32)

        self._check_controls(controls)
        self.controls = controls
        self.num_controls = len(controls)

        self.measures = [
            obs for obs in self.POSSIBLE_STATE_ATTRIBUTES if obs not in controls
        ]
        self.state_attributes = list(self.controls.keys()) + self.measures

        # 动作空间
        if self.continuous_control:
            self.action_space = spaces.Dict({
                name: spaces.Box(-bs[2], bs[2], shape=(self.num_turbines,), dtype=np.float32)
                for name, bs in self.controls.items()
            })
        else:
            self.action_space = spaces.Dict({
                name: spaces.MultiDiscrete([3] * self.num_turbines) for name in self.controls
            })

        # 状态空间
        state_dict = OrderedDict()
        barr = np.ones(self.num_turbines, dtype=np.float32)
        lws, hws = self.DEFAULT_BOUNDS["wind_speed"]
        lwd, hwd = self.DEFAULT_BOUNDS["wind_direction"]
        for attr in self.state_attributes:
            if attr == "freewind_measurements":
                lo = np.array([lws, lwd], dtype=np.float32)
                hi = np.array([hws, hwd], dtype=np.float32)
            elif attr in controls:
                lo = barr * controls[attr][0]
                hi = barr * controls[attr][1]
            else:
                lo = barr * self.DEFAULT_BOUNDS[attr][0]
                hi = barr * self.DEFAULT_BOUNDS[attr][1]
            state_dict[attr] = spaces.Box(lo, hi, shape=lo.shape, dtype=np.float32)
        self.state_space = spaces.Dict(state_dict)
        self.start_state = None

        self._actuation_accumulator = {
            c: np.zeros(self.num_turbines, dtype=np.float32) for c in controls
        }

    # ========== 核心 API ==========

    def reset(self, seed: int = None, options: dict = None) -> dict:
        rng = np.random.default_rng(seed)
        ws: float = 8.0
        wd: float = 270.0

        if options and "wind_speed" in options:
            ws = float(options["wind_speed"])
        elif not (self.farm_case.set_wind_speed or bool(getattr(self.farm_case, 'wind_time_series', None))):
            ws = float(np.clip(8 * rng.weibull(8), *self.DEFAULT_BOUNDS["wind_speed"]))

        if options and "wind_direction" in options:
            wd = float(options["wind_direction"])
        elif not (self.farm_case.set_wind_direction or bool(getattr(self.farm_case, 'wind_time_series', None))):
            wd = float(rng.normal(270, 20) % 360)

        self._current_wind = WindConfig(speed=ws, direction=wd)
        self._step_count = 0
        self.interface.reset(self._current_wind)

        zero = ControlInput.zero(self.num_turbines)
        for _ in range(self.start_iter + 1):
            self.interface.step(zero)

        state = self._build_initial_state(zero)
        self.start_state = clip_to_dict_space(state, self.state_space)
        self._actuation_accumulator = {c: np.zeros(self.num_turbines, dtype=np.float32) for c in self.controls}
        return self.start_state

    def step_interface(self, state: Dict) -> tuple:
        ctrl = ControlInput(
            mode=np.asarray(state.get("mode", np.zeros(self.num_turbines, dtype=np.int32)), dtype=np.int32),
            yaw=np.asarray(state.get("yaw", np.zeros(self.num_turbines)), dtype=np.float64),
            pitch=np.asarray(state.get("pitch", np.zeros(self.num_turbines)), dtype=np.float64),
            power=np.asarray(state.get("power", np.zeros(self.num_turbines)), dtype=np.float64)
            if "power" in self.controls else None,
            min_pitch=np.asarray(state.get("min_pitch", np.zeros(self.num_turbines)), dtype=np.float64)
            if "min_pitch" in self.controls else None,
        )
        output = self.interface.step(ctrl)
        next_state, powers, loads, done = self._output_to_state(output)
        # 控制量（非测量量）需从输入状态传递，保证状态一致性
        if "power" in self.controls and "power" in next_state:
            next_state["power"] = state.get("power",
                np.zeros(self.num_turbines, dtype=np.float32))
        if "min_pitch" in self.controls and "min_pitch" in next_state:
            next_state["min_pitch"] = state.get("min_pitch",
                np.zeros(self.num_turbines, dtype=np.float32))
        if "mode" in self.controls and "mode" in next_state:
            next_state["mode"] = state.get("mode",
                np.zeros(self.num_turbines, dtype=np.int32))
        return next_state, powers, loads, done

    def take_action(self, state: Dict, joint_action: Dict) -> tuple:
        ns = self.get_controlled_state_transition(state, joint_action)
        return self.step_interface(ns)

    def get_controlled_state_transition(self, state: Dict, joint_action: Dict) -> Dict:
        if not isinstance(joint_action, dict):
            raise TypeError("Joint action must be dict")
        state = clip_to_dict_space(self._cast_dict(state), self.state_space)
        ns = copy.deepcopy(state)
        for ctrl, cmd in joint_action.items():
            assert ctrl in self.controls
            cmd = np.array(cmd, dtype=np.float32)
            if self.continuous_control:
                cmd = np.clip(cmd, self.action_space[ctrl].low, self.action_space[ctrl].high)
            else:
                cmd = (cmd - 1) * self.controls[ctrl][-1]
            ns[ctrl] = np.clip(state[ctrl] + cmd, self.state_space[ctrl].low, self.state_space[ctrl].high)
            if ctrl in self._actuation_accumulator:
                self._actuation_accumulator[ctrl] += np.abs(cmd)
        return ns

    # ========== 辅助 ==========

    def get_state_powers(self) -> np.ndarray:
        return self._last_powers

    def get_accumulated_actions(self, agent=None) -> dict:
        return self._actuation_accumulator.copy()

    def _build_initial_state(self, ctrl: ControlInput) -> OrderedDict:
        w = self._current_wind or WindConfig()
        st = OrderedDict()
        for attr in self.state_attributes:
            if attr == "freewind_measurements":
                st[attr] = np.array([w.speed, w.direction], dtype=np.float32)
            elif attr == "wind_speed":
                st[attr] = np.full(self.num_turbines, w.speed, dtype=np.float32)
            elif attr == "wind_direction":
                st[attr] = np.full(self.num_turbines, w.direction, dtype=np.float32)
            elif attr == "yaw":
                st[attr] = ctrl.yaw.astype(np.float32)
            elif attr == "pitch":
                st[attr] = ctrl.pitch.astype(np.float32)
            elif attr == "torque":
                st[attr] = (ctrl.torque if ctrl.torque is not None
                            else np.zeros(self.num_turbines, dtype=np.float32))
            elif attr == "power":
                st[attr] = (ctrl.power if ctrl.power is not None
                            else np.zeros(self.num_turbines, dtype=np.float32))
            elif attr == "min_pitch":
                st[attr] = (ctrl.min_pitch if ctrl.min_pitch is not None
                            else np.zeros(self.num_turbines, dtype=np.float32))
            elif attr == "mode":
                st[attr] = ctrl.mode.astype(np.int32)
        return st

    def _output_to_state(self, out: SimulationOutput) -> tuple:
        pw = out.power_mw
        powers = pw[-1, :] if pw.ndim == 2 else pw
        self._last_powers = powers.astype(np.float32)

        loads = None
        if out.blade_loads is not None:
            bl = out.blade_loads
            loads = bl[-1, :, :] if bl.ndim == 3 else bl

        def _last2d(arr, default):
            if arr is None:
                return np.full(self.num_turbines, default)
            return arr[-1, :] if arr.ndim == 2 else np.atleast_1d(arr)

        ws = _last2d(out.wind_speed, 8.0)
        wd = _last2d(out.wind_direction, 270.0)
        yaw = _last2d(out.yaw_deg, 0.0)
        pitch = _last2d(out.pitch_deg, 0.0)
        torque = _last2d(out.torque_nm, 0.0)

        st = OrderedDict()
        for attr in self.state_attributes:
            if attr == "freewind_measurements":
                st[attr] = np.array([np.mean(ws), np.mean(wd)], dtype=np.float32)
            elif attr == "wind_speed":
                st[attr] = ws.astype(np.float32)
            elif attr == "wind_direction":
                st[attr] = wd.astype(np.float32)
            elif attr == "yaw":
                st[attr] = yaw.astype(np.float32)
            elif attr == "pitch":
                st[attr] = pitch.astype(np.float32)
            elif attr == "torque":
                st[attr] = torque.astype(np.float32)
            elif attr == "power":
                st[attr] = np.zeros(self.num_turbines, dtype=np.float32)
            elif attr == "min_pitch":
                st[attr] = np.zeros(self.num_turbines, dtype=np.float32)
            elif attr == "mode":
                st[attr] = np.zeros(self.num_turbines, dtype=np.int32)

        done = self._step_count >= self.horizon
        self._step_count += 1
        return st, powers.astype(np.float32), loads, done

    def _cast_dict(self, state: Dict) -> OrderedDict:
        sc = OrderedDict()
        for attr, val in state.items():
            sc[attr] = val.astype(np.float32)
        return sc

    def _check_controls(self, cd: Dict) -> None:
        for name, bs in cd.items():
            if name not in self.CONTROL_SET:
                raise ValueError(f"Cannot control '{name}'. Allowed: {self.CONTROL_SET}")
            if not (isinstance(bs, Iterable) and 2 <= len(bs) <= 3):
                raise TypeError(f"Bounds for '{name}': [lower, upper] or [lower, upper, step]")
            if bs[0] >= bs[1]:
                raise ValueError(f"Bounds for '{name}': lower < upper")
            if len(bs) == 2:
                cd[name] = bs + (1,)
                warn(f"No step size for '{name}'. Defaulting to 1.")
            if not self.continuous_control and len(bs) == 3 and bs[2] <= 0:
                raise ValueError(f"Step size for '{name}' must be > 0")
