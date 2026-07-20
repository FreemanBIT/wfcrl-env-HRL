"""Compatibility shim: provides FarmCase aliases for code that hasn't migrated yet.

Deprecated: new code should use FarmLayout + SimulationConfig directly.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Union, Callable


@dataclass
class FarmCase:
    """Compatibility stub for old FarmCase. Use FarmLayout + SimulationConfig instead."""
    num_turbines: int
    xcoords: Union[List, Callable]
    ycoords: Union[List, Callable]
    dt: int = 3
    buffer_window: int = 300
    t_init: int = 300
    max_iter: int = 100
    set_wind_speed: bool = False
    set_wind_direction: bool = False
    wind_time_series: str = None

    def __repr__(self):
        return f"FarmCase({self.num_turbines}T, dt={self.dt}s, max_iter={self.max_iter})"


class FastFarmCase(FarmCase):
    simulator: str = "FastFarm"
    path_to_simulator: str = None

    @property
    def avg_window(self):
        return int(self.buffer_window / self.dt)


class FlorisCase(FarmCase):
    simulator: str = "Floris"
    turbine_type: list = None
    turbine_library_path: str = None


@dataclass
class DefaultControl:
    yaw = (-40, 40, 5)
    pitch = (0, 45, 1)
    torque = (-2e4, 2e4, 1e3)
    power = (0, 10, 0.5)


class FarmRowFastfarm(FastFarmCase):
    dt = 3
    buffer_window = 1
    t_init = 100
    set_wind_direction = True
    set_wind_speed = False

    @classmethod
    def get_xcoords(cls, num_turbines):
        return [i * 4 * 126.0 for i in range(num_turbines)]

    @classmethod
    def get_ycoords(cls, num_turbines):
        return [0.0 for _ in range(num_turbines)]


class FarmRowFloris(FlorisCase):
    dt = 60
    buffer_window = 1
    t_init = 0
    set_wind_direction = False
    set_wind_speed = False

    @classmethod
    def get_xcoords(cls, num_turbines):
        return [i * 4 * 126.0 for i in range(num_turbines)]

    @classmethod
    def get_ycoords(cls, num_turbines):
        return [0.0 for _ in range(num_turbines)]
