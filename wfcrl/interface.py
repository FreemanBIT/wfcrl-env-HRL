import os
import platform
import re
import time
import warnings
from abc import ABC
from pathlib import Path
from typing import Dict, Iterable, List, Union

import numpy as np
import pandas as pd
from floris import FlorisModel
from mpi4py import MPI

from wfcrl.environments import FarmCase
from wfcrl.simul_utils import (
    create_dll,
    create_ff_case,
    create_floris_case,
    get_inflow_file_path,
    read_inflow_info,
    read_simul_info,
    reset_simul_file,
    write_inflow_info,
)
from openfast_toolbox.io.fast_input_file import FASTInputFile
from openfast_toolbox.io import FASTOutputFile


class BaseInterface(ABC):
    def __init__(self):
        self.num_turbines = None

    @property
    def wind_speed(self):
        pass

    @property
    def wind_dir(self):
        pass

    def set_yaw_angles(self, yaws: List):
        pass

    def get_yaw_angles(self) -> List:
        pass

    def avg_powers(self) -> List:
        pass

    def init(self):
        pass

    def next_wind(self):
        pass


class PowerBuffer:
    def __init__(self, num_turbines: int, size: int = 50_000, agg: str = np.mean):
        self._agg_fn = agg
        self._size = size
        self.pos = -1
        self._buffer = np.zeros((self._size, num_turbines))

    def add(self, measure: np.array):
        if self.pos < self._size - 1:
            self.pos += 1
        else:
            self._buffer = np.roll(self._buffer, -1, axis=0)

        self._buffer[self.pos, :] = measure

    def get_last(self):
        return self._buffer[self.pos, :]

    def get_all(self, window: int = 1):
        start = self.pos - window if self.pos > window else 0
        return self._buffer[start : self.pos + 1, :]

    def get_agg(self, window: int = 1):
        start = self.pos - window if self.pos > window else 0
        return self._agg_fn(self._buffer[start : self.pos + 1, :], 0)

    def empty(self):
        self._buffer[:] = 0.0
        self.pos = -1


class MPI_Interface(BaseInterface):
    CONTROL_SET = ["yaw", "pitch", "torque"]
    YAW_TAG = 1
    PITCH_TAG = 2
    TORQUE_TAG = 3
    COM_TAG = 0
    MEASURES_TAG = 4

    def __init__(
        self,
        measure_map: dict,
        num_turbines: int,
        buffer_size: int = 50_000,
        log_file: str = None,
        comm: MPI.Comm = MPI.COMM_WORLD,
        target_process_rank: int = None,
        max_iter: int = 500,
        default_avg_window: int = 1,
    ):
        super().__init__()

        # Check communication channels
        self._comm = comm
        if target_process_rank is None:
            rank = self._comm.Get_rank()
            target_process_rank = 1 - rank
        self._target_process_rank = target_process_rank
        self._buffer_size = buffer_size
        self._default_avg_window = default_avg_window
        self._num_measures = None
        self.current_measures = None
        self.max_iter = max_iter

        # Validate measure map and store names of measure in array
        self._validate_measure_map(measure_map)
        self.num_turbines = num_turbines
        self._power_buffers = PowerBuffer(self.num_turbines, size=self._buffer_size)
        self._wind_buffers = PowerBuffer(2, size=self._buffer_size)
        self._current_yaw_command = np.zeros(num_turbines + 1, dtype=np.double)
        self._current_pitch_command = np.zeros(num_turbines + 1, dtype=np.double)
        self._current_torque_command = np.zeros(num_turbines + 1, dtype=np.double)
        self._num_iter = 0
        self._comm_connected = True

        self._logging = False
        if log_file is not None:
            self._log_file = log_file
            self._logging = True

    @property
    def wind_speed(self):
        return self.avg_wind()[0]

    @property
    def wind_dir(self):
        return self.avg_wind()[1]

    def _validate_measure_map(self, measure_map):
        inv_measure_map = {}
        for name, indice in measure_map.items():
            if isinstance(indice, int):
                inv_measure_map[indice] = name
            elif isinstance(indice, Iterable):
                for j, indice_i in enumerate(indice):
                    inv_measure_map[indice_i] = f"{name}_{j}"

        assert min(inv_measure_map.keys()) == 0
        assert max(inv_measure_map.keys()) == len(inv_measure_map) - 1
        measure_names = list(inv_measure_map.values())
        self.measure_map = measure_map
        self.measure_names = measure_names

    def update_command(
        self,
        yaw: np.ndarray = None,
        pitch: np.ndarray = None,
        torque: np.ndarray = None,
    ):
        assert self.current_measures is not None, "Call `init` before `update_command`"
        if not self._comm_connected:
            return True
        if yaw is not None:
            self._current_yaw_command[1:] = np.radians(yaw.astype(np.double))
            self._current_yaw_command[0] = 1.0
        if pitch is not None:
            self._current_pitch_command[1:] = np.radians(pitch.astype(np.double))
            self._current_pitch_command[0] = 1.0
        if torque is not None:
            self._current_torque_command[1:] = torque.astype(np.double)
            self._current_torque_command[0] = 1.0

        self._comm.Send(
            buf=self._current_yaw_command,
            dest=self._target_process_rank,
            tag=self.YAW_TAG,
        )
        self._comm.Send(
            buf=self._current_pitch_command,
            dest=self._target_process_rank,
            tag=self.PITCH_TAG,
        )
        self._comm.Send(
            buf=self._current_torque_command,
            dest=self._target_process_rank,
            tag=self.TORQUE_TAG,
        )
        power, wind = self._wait_for_sim_output()
        self._power_buffers.add(power)
        self._wind_buffers.add(wind)

        self._num_iter += 1
        if self._num_iter == self.max_iter:
            self._finalize_mpi_comm()

        if self._logging:
            with open(self._log_file, "a") as fp:
                fp.write(
                    f"Sent command YAW {self.get_yaw_command()} - "
                    f" PITCH {self.get_pitch_command()}"
                    f" TORQUE {self.get_torque_command()}\n"
                    f"***********Received Power: {power} - "
                    f"Filtered Power: (window {self._default_avg_window}):"
                    f"{self.avg_powers()} - "
                    f" Wind : {self.avg_wind()}\n"
                )

        return self._num_iter == self.max_iter

    def set_comm(self, comm):
        self._comm = comm

    def init(self):
        self._num_iter = 0
        self._comm_connected = True
        self._current_yaw_command = np.zeros(self.num_turbines + 1, dtype=np.double)
        self._current_pitch_command = np.zeros(self.num_turbines + 1, dtype=np.double)
        self._current_torque_command = np.zeros(self.num_turbines + 1, dtype=np.double)
        self._power_buffers.empty()
        self._wind_buffers.empty()

        num_measures = np.array([0], dtype=int)
        self._comm.Recv(
            num_measures, source=self._target_process_rank, tag=self.COM_TAG
        )
        self._comm.Send(
            buf=np.array([self.max_iter], dtype=np.double),
            dest=self._target_process_rank,
            tag=self.COM_TAG,
        )
        self._num_measures = num_measures[0]
        print(
            f"Interface: will receive {self._num_measures} measures at every iteration"
        )
        self.current_measures = (
            np.zeros((self.num_turbines, self._num_measures)) * np.nan
        )

    def _finalize_mpi_comm(self):
        # Mark as disconnected and release the MPI comm reference.
        # This prevents mpi4py from attempting cleanup on exit.
        self._comm_connected = False
        self._comm = None

    def get_yaw_command(self):
        if 1 - self._current_yaw_command[0]:
            return None
        return np.degrees(self._current_yaw_command).copy()[1:]

    def get_pitch_command(self):
        if 1 - self._current_pitch_command[0]:
            return None
        return np.degrees(self._current_pitch_command).copy()[1:]

    def get_torque_command(self):
        if 1 - self._current_torque_command[0]:
            return None
        return self._current_torque_command.copy()[1:]

    def avg_farm_power(self, window: int = None):
        powers = self.avg_powers(window).squeeze()
        return powers.sum()

    def avg_powers(self, window: int = None) -> List:
        if window is None:
            window = self._default_avg_window
        return np.atleast_1d(self._power_buffers.get_agg(window).squeeze())

    def avg_wind(self, window: int = None) -> List:
        if window is None:
            window = self._default_avg_window
        return self._wind_buffers.get_agg(window).squeeze()

    def last_powers(self, window: int = 0) -> np.ndarray:
        return np.atleast_1d(self._power_buffers.get_all(window).squeeze())

    def last_wind(self, window: int = 0) -> np.ndarray:
        return self._wind_buffers.get_all(window).squeeze()

    def get_measure(self, measure: str) -> np.ndarray:
        if measure == "freewind_measurements":
            return np.atleast_1d(self.last_wind().squeeze())
        return np.atleast_1d(
            self.current_measures[:, self.measure_map[measure]].squeeze()
        )

    def get_all_measures(self) -> Dict:
        df = pd.DataFrame(self.current_measures, columns=self.measure_names)
        # convert angles to degrees
        df[["yaw", "pitch"]] = np.degrees(df[["yaw", "pitch"]])
        return df

    def _wait_for_sim_output(self):
        size_buffer = self.num_turbines * self._num_measures
        measures = np.zeros(size_buffer, dtype=np.double)
        self._comm.Recv(
            measures, source=self._target_process_rank, tag=self.MEASURES_TAG
        )
        self._comm.Barrier()
        measures = measures.reshape((self.num_turbines, self._num_measures))
        # print(f"Received measures matrix from simulator: {measures}")

        # format wind directions - keep wind direction positive
        directions = measures[:, self.measure_map["wind_direction"]].flatten()
        directions = np.degrees(directions) - 90
        directions[directions < 0] = directions[directions < 0] + 360
        measures[:, self.measure_map["wind_direction"]] = directions

        # retrieve wind speeds and power outputs
        speeds = measures[:, self.measure_map["wind_speed"]].flatten()
        powers = measures[:, self.measure_map["power"]].flatten()

        # get upstream point = point of maximum speed
        upstream_point = np.argmax(speeds)
        wspeed = speeds[upstream_point]
        wdir = directions[upstream_point]
        # wdir = np.degrees(directions[upstream_point])
        # wdir = wdir - 90
        # Keep wind direction positive
        # if wdir < 0:
        #     wdir = wdir + 360
        self.current_measures = measures
        return powers.astype(np.float32), np.array([wspeed, wdir], dtype=np.float32)


# Compute project root once at module level
_FF_SCRIPT_DIR = Path(__file__).resolve().parent
_FF_PROJECT_ROOT = _FF_SCRIPT_DIR.parent

class FastFarmInterface(MPI_Interface):
    system_type = platform.system().lower()
    if system_type == "windows":
        default_exe_path = str(_FF_PROJECT_ROOT / "wfcrl/simulators/fastfarm/bin/FAST.Farm_x64_OMP.exe")
    else:
        default_exe_path = "FAST.Farm"
    # `wind_measurements` is not read from the simulator
    # but computed by the interface
    measure_map = {
        "wind_speed": 0,
        "power": 1,
        "wind_direction": 2,
        "yaw": 3,
        "pitch": 4,
        "torque": 5,
        "load": [6, 7, 8, 9, 10, 11],
        "freewind_measurements": None,
    }

    def __init__(
        self,
        num_turbines: int,
        fstf_file: bool,
        buffer_size: int = 50_000,
        log_file: str = None,
        max_iter: int = int(1e4),
        fast_farm_executable: str = default_exe_path,
        default_avg_window: int = 1,
    ):
        self._path_to_fastfarm_exe = fast_farm_executable
        self._simul_file = fstf_file
        self._inflow_file = get_inflow_file_path(fstf_file)
        self._num_resets = 0

        super().__init__(
            default_avg_window=default_avg_window,
            buffer_size=buffer_size,
            num_turbines=num_turbines,
            log_file=log_file,
            measure_map=self.measure_map,
            comm=None,
            target_process_rank=0,
            max_iter=max_iter,
        )

    @classmethod
    def from_file(
        cls,
        fstf_file,
        fast_farm_executable: str = default_exe_path,
        buffer_size: int = 50_000,
        log_file: str = None,
        default_avg_window: int = 1,
    ):
        print(f"Simulation will be started from fstf file {fstf_file}")
        num_turbines, max_iter = read_simul_info(fstf_file)
        print(f"Creating new DLLs for simulation {fstf_file}")
        create_dll(fstf_file)

        return cls(
            num_turbines=num_turbines,
            fstf_file=fstf_file,
            default_avg_window=default_avg_window,
            buffer_size=buffer_size,
            log_file=log_file,
            max_iter=max_iter,
            fast_farm_executable=fast_farm_executable,
        )

    @classmethod
    def from_case(
        cls,
        case: FarmCase,
        fast_farm_executable: str = default_exe_path,
        buffer_size: int = 50_000,
        log_file: str = None,
        output_dir: str = None,
    ):
        if output_dir is None:
            name = f"{case.simulator}__{case.t_init + case.max_iter * case.dt}s"
            name += f"__{case.num_turbines}T_{time.time()}"
            output_dir = str(_FF_PROJECT_ROOT / f"__simul__/fastfarm/{name}/")

        fstf_file = create_ff_case(
            case.dict(),
            output_dir=output_dir,
        )

        return cls(
            num_turbines=case.num_turbines,
            fstf_file=fstf_file,
            default_avg_window=case.avg_window,
            buffer_size=buffer_size,
            log_file=log_file,
            max_iter=case.max_iter,
            fast_farm_executable=fast_farm_executable,
        )

    def init(self, wind_speed: float = None, wind_direction: float = None):
        # Clean up previous FAST.Farm before spawning a new one
        if hasattr(self, '_comm') and self._comm is not None:
            self._finalize_mpi_comm()
        # wind speed and direction ignored for now
        if wind_direction is not None:
            warnings.warn(
                f"Wind direction = {wind_direction} requested, but FastFarmInterface"
                "cannot set wind direction in the simulator. Request will be ignored."
            )

        if wind_speed is not None:
            simul_wind_speed = read_inflow_info(self._inflow_file)
            if simul_wind_speed != wind_speed:
                write_inflow_info(self._inflow_file, float(wind_speed))

        simul_file = reset_simul_file(self._simul_file, self._num_resets)
        print("Spawning process", self._path_to_fastfarm_exe, simul_file)
        spawn_comm = MPI.COMM_SELF.Spawn(
            self._path_to_fastfarm_exe, args=[simul_file], maxprocs=1
        )
        self.set_comm(spawn_comm)
        self._num_resets += 1
        super().init()


class FlorisInterface(BaseInterface):
    CONTROL_SET = ["yaw"]
    # `wind_measurements` handled separately

    DEFAULT_MEASURE_MAP = {
        "yaw": 0,
        "wind_speed": 1,
        "wind_direction": 2,
        "load": [3, 4, 5, 6],
        "freewind_measurements": None,
    }

    YAW_TAG = 1
    PITCH_TAG = 2
    TORQUE_TAG = 3
    COM_TAG = 0
    MEASURES_TAG = 4

    def __init__(
        self,
        num_turbines: int,
        simul_file: str,
        max_iter: int = int(1e4),
        log_file: str = None,
        wind_speed: float = None,
        wind_direction: float = None,
        wind_time_series: Union[str, np.ndarray] = None,
    ):
        """
        time_series: np.numpy array or path to csv file
                    first column: speed, second column: direction
        """
        super().__init__()

        self.num_turbines = num_turbines
        self.fi = FlorisModel(simul_file)
        self.measure_map = self.DEFAULT_MEASURE_MAP
        self._num_measures = sum(
            [
                len(indices) if isinstance(indices, list) else 1
                for indices in self.measure_map.values()
            ]
        )
        self._num_measures -= 1
        self.dt = 60
        self.max_iter = max_iter
        self._logging = False

        # Handle wind as speed / direction time series
        self.wind_time_series = wind_time_series
        self.wind_generator = self._make_wind_generator(
            wind_speed, wind_direction, wind_time_series
        )
        wind_speed, wind_direction = next(self.wind_generator)
        self.init(wind_speed, wind_direction)
        if log_file is not None:
            self._log_file = log_file
            self._logging = True

    def _make_wind_generator(
        self, wind_speed=None, wind_direction=None, time_series=None
    ):
        if time_series is None:

            def wind_generator():
                while True:
                    yield wind_speed, wind_direction

        else:
            if isinstance(time_series, str):
                time_series = pd.read_csv(time_series).values
            assert isinstance(time_series, np.ndarray)
            # start at a random point in the time series
            start = np.random.randint(0, time_series.shape[0])
            time_series = np.r_[time_series[start:], time_series[:start]]

            def wind_generator():
                for ts in time_series:
                    yield ts

        return wind_generator()

    @classmethod
    def from_case(
        cls,
        case: FarmCase,
        log_file: str = None,
        output_dir: str = None,
    ):
        if output_dir is None:
            name = f"{case.simulator}__{case.t_init + case.max_iter * case.dt}s"
            name += f"__{case.num_turbines}T_{time.time()}"
            output_dir = f"__simul__/floris/{name}/"

        simul_file = create_floris_case(case.dict(), output_dir=output_dir)
        return cls(
            num_turbines=case.num_turbines,
            simul_file=simul_file,
            max_iter=case.max_iter,
            log_file=log_file,
            wind_speed=float(case.simul_params["speed"]),
            wind_direction=float(case.simul_params["direction"]),
            wind_time_series=case.simul_params["wind_time_series"],
        )

    @property
    def wind_speed(self):
        return self.fi.wind_speeds[0]

    @property
    def wind_dir(self):
        return self.fi.wind_directions[0]

    def update_command(
        self,
        yaw: np.ndarray = None,
    ):
        if yaw is not None:
            self._current_yaw_command = yaw.astype(np.double).reshape(1, -1)

        wind_speed, wind_direction = next(self.wind_generator)
        wind_direction = wind_direction % 360

        # Combine wind and yaw setting into a single set() call (v4 API)
        self.fi.set(
            wind_speeds=[wind_speed],
            wind_directions=[wind_direction],
            yaw_angles=self._current_yaw_command,
        )
        self.fi.run()

        self.current_measures[
            :, self.measure_map["yaw"]
        ] = self.fi.core.farm.yaw_angles.squeeze()
        wind_measures_indices = [
            self.measure_map["wind_speed"],
            self.measure_map["wind_direction"],
        ]
        self.current_measures[:, wind_measures_indices] = np.array(
            self.local_wind_measurements()
        ).T
        self.current_measures[:, self.measure_map["load"]] = (
            np.array(self.local_load_proxies()).T * 1e7
        )
        self._num_iter += 1
        if self._logging:
            with open(self._log_file, "a") as fp:
                fp.write(
                    f"Sent command YAW {self.get_yaw_command()} - "
                    f"***********Received Power: {self.avg_powers()}"
                    f" Wind : {self.avg_wind()}\n"
                )
        return self._num_iter == self.max_iter

    def init(self, wind_speed: float = None, wind_direction: float = None):
        if self.wind_time_series and wind_speed is not None:
            warnings.warn(
                f"Wind speed = {wind_speed} requested, but wind_time_series"
                "mode is activated. Request will be ignored."
            )
            wind_speed = None
        if self.wind_time_series and wind_direction is not None:
            warnings.warn(
                f"Wind direction = {wind_direction} requested, but wind_time_series"
                "mode is activated. Request will be ignored."
            )
            wind_direction = None
        self.wind_generator = self._make_wind_generator(
            wind_speed, wind_direction, self.wind_time_series
        )
        # Initialize flow field with first wind condition (v4 API: set instead of reinitialize)
        ws, wd = next(self.wind_generator)
        wd = wd % 360
        self.fi.set(wind_speeds=[ws], wind_directions=[wd])

        self._num_iter = 0
        self._current_yaw_command = np.zeros((1, self.num_turbines))
        self.current_measures = (
            np.zeros((self.num_turbines, self._num_measures)) * np.nan
        )

    def get_yaw_command(self):
        return self._current_yaw_command.copy().flatten()

    def avg_farm_power(self):
        powers = self.avg_powers()
        return powers.sum()

    def avg_powers(self) -> List:
        return self.fi.get_turbine_powers().flatten()

    def avg_wind(self) -> List:
        """returns average free-stream wind"""
        return np.array([self.wind_speed, self.wind_dir]).squeeze()

    def local_load_proxies(self) -> np.ndarray:
        """we use local turbulences and wind speed variations on the rotors
        as expressed by standard deviation as proxies for loads"""
        turbulences = self.fi.core.flow_field.turbulence_intensity_field.squeeze()
        # wind speed variations along the 3 axes - standard deviation on rotor speed
        # FLORIS v4: u shape is (n_findex, n_turbines, grid_y, grid_z)
        var_u = np.std(self.fi.core.flow_field.u, axis=(2, 3)).squeeze()
        var_v = np.std(self.fi.core.flow_field.v, axis=(2, 3)).squeeze()
        var_w = np.std(self.fi.core.flow_field.w, axis=(2, 3)).squeeze()
        return turbulences, var_u, var_v, var_w

    def local_wind_measurements(self) -> np.ndarray:
        # u (np.array): x-component of velocity.
        # v (np.array): y-component of velocity.
        # w (np.array): z-component of velocity.
        # FLORIS v4: u shape is (n_findex, n_turbines, grid_y, grid_z)
        velocities = np.cbrt(np.mean(self.fi.core.flow_field.u**3, axis=(2, 3)))
        directions = self.wind_dir - np.degrees(
            np.arctan2(self.fi.core.flow_field.v, self.fi.core.flow_field.u)
        )
        directions = np.mean(directions, axis=(2, 3))
        return velocities.squeeze(), directions.squeeze()

    def get_measure(self, measure: str) -> np.ndarray:
        if measure not in self.measure_map:
            return None
        if measure == "freewind_measurements":
            return self.avg_wind()
        return self.current_measures[:, self.measure_map[measure]]

    def get_parameters(self):
        pass


# =====================================================================
# Standalone FAST.Farm Interface (no MPI, subprocess-based)
# =====================================================================
class FastFarmStandaloneInterface:
    """
    Standalone interface for FAST.Farm farm-level control.
    Uses subprocess (not MPI) — compatible with FAST.Farm v5.0.0+.

    Architecture:
        Controller -> set_yaw_pitch() -> run FAST.Farm -> parse .outb -> measurements
    """

    def __init__(self, config_dict, output_dir, farm_base=None):
        self.config = config_dict
        self.output_dir = output_dir
        self.n_turbines = config_dict.get('num_turbines', 0)
        self.fstf_file = None
        self.farm_base = farm_base

    def setup(self, add_outlist=True):
        """Generate FAST.Farm input files and configure environment."""
        farm_base_dir = os.path.join(self.output_dir, "FarmInputs")
        os.makedirs(farm_base_dir, exist_ok=True)
        self.fstf_file = create_ff_case(self.config, output_dir=self.output_dir)
        self.farm_base = os.path.dirname(self.fstf_file) if self.farm_base is None else self.farm_base
        if add_outlist:
            self._add_outlist()
        self._fix_inflow()

    def _add_outlist(self):
        """Add OutList channels to generated turbine .fst files."""
        fstf = FASTInputFile(self.fstf_file)
        wt_refs = [row[3].replace('"', "") for row in fstf["WindTurbines"]]
        channels = ["GenPwr","GenTq","RotSpeed","RootMIP1","RootMOoP1","RootMzb1","YawPzn","BldPitch1","HSShftP"]
        for wt_ref in wt_refs:
            wt_path = os.path.join(self.farm_base, wt_ref)
            with open(wt_path, 'rb') as f: raw = f.read()
            text = raw.decode('ascii', errors='replace')
            if 'OutList' in text: continue
            lines = text.split('\n'); nl = []; inserted = False
            for line in lines:
                nl.append(line)
                if not inserted and line.strip().startswith('"G0"'):
                    nl.append('')
                    for ch in channels: nl.append(f'"{ch}"    {ch}')
                    nl.append('END of input file'); inserted = True
            if inserted:
                ob = '\n'.join(nl).encode('ascii', errors='replace')
                ob = ob.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
                with open(wt_path, 'wb') as f: f.write(ob)

    def _fix_inflow(self):
        """Fix InflowWind.dat wind direction and RotorApexOffsetPos."""
        inflow_path = os.path.join(self.farm_base, "InflowWind.dat")
        if not os.path.exists(inflow_path): return
        wdir = self.config.get('direction', 270)
        prop_dir = (wdir + 90) % 360
        inflow = FASTInputFile(inflow_path)
        inflow["PropagationDir"] = prop_dir; inflow.write(inflow_path)
        with open(inflow_path, 'rb') as f: raw = f.read()
        raw = raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
        text = raw.decode('ascii')
        text = re.sub(r'^(\s*)[0-9.+-]+\s+RotorApexOffsetPos',
                      r'\1 0.0, 0.0, 0.0   RotorApexOffsetPos',
                      text, flags=re.MULTILINE)
        with open(inflow_path, 'wb') as f: f.write(text.encode('ascii'))
        write_inflow_info(inflow_path, float(self.config.get('speed', 10)))

    def set_yaw_pitch(self, yaw_deg, pitch_deg):
        """Set yaw/pitch via ElastoDyn NacYaw/BlPitch (ROSCO-style)."""
        fstf = FASTInputFile(self.fstf_file)
        wt_refs = [row[3].replace('"', "") for row in fstf["WindTurbines"]]
        for wt_ref in wt_refs:
            wt_path = os.path.join(self.farm_base, wt_ref)
            wt = FASTInputFile(wt_path)
            ed_path = os.path.join(self.farm_base, wt["EDFile"].replace('"', ""))
            ed = FASTInputFile(ed_path)
            ed["NacYaw"] = float(yaw_deg)
            ed["BlPitch(1)"] = float(pitch_deg)
            ed["BlPitch(2)"] = float(pitch_deg)
            ed["BlPitch(3)"] = float(pitch_deg)
            ed.write(ed_path)
            with open(ed_path, 'rb') as f: raw = f.read()
            raw = raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
            with open(ed_path, 'wb') as f: f.write(raw)

    def run(self):
        """Run FAST.Farm as standalone subprocess. Returns dict with 'time', 'power_mw'."""
        import subprocess as _sp
        ff_exe = FastFarmInterface.default_exe_path
        if not os.path.exists(ff_exe): raise FileNotFoundError(f"FAST.Farm not found: {ff_exe}")
        proc = _sp.Popen([ff_exe, self.fstf_file], cwd=self.farm_base,
                         stdout=_sp.PIPE, stderr=_sp.STDOUT, text=True, bufsize=1)
        for _ in proc.stdout: pass
        proc.wait()
        prefix = os.path.splitext(os.path.basename(self.fstf_file))[0]
        all_power = []; time_vec = None
        for i in range(1, self.n_turbines + 1):
            ob = os.path.join(self.farm_base, f"{prefix}.T{i}.outb")
            o = os.path.join(self.farm_base, f"{prefix}.T{i}.out")
            if os.path.exists(ob): df = FASTOutputFile(ob).toDataFrame()
            elif os.path.exists(o): df = FASTOutputFile(o).toDataFrame()
            else: continue
            if time_vec is None: time_vec = df.iloc[:, 0].values
            for col in df.columns:
                base = col.split('[')[0].strip('_ ')
                if base == 'GenPwr':
                    unit = col.split('[')[1].split(']')[0] if '[' in col else 'W'
                    p = df[col].values
                    p_mw = p / 1e3 if unit.upper() in ('KW','KILOWATT','KWATT') else p / 1e6
                    all_power.append(p_mw); break
        return {'time': time_vec, 'power_mw': np.column_stack(all_power) if all_power else None}


class FastFarmOnlineInterface(FastFarmStandaloneInterface):
    """
    Online optimization interface for FAST.Farm.
    Runs one DT_low step at a time — controller decides next action based on
    current step's measurements. Supports closed-loop / online optimization.

    Architecture:
        interface = FastFarmOnlineInterface(config, out_dir)
        interface.setup()
        for step in range(n_steps):
            meas = interface.step(yaw_deg, pitch_deg)
            # meas['power_mw'] -> per-turbine power (MW)
            # meas['farm_power_mw'] -> total farm power (MW)
            yaw_next, pitch_next = controller.optimize(meas)
    """

    def __init__(self, config_dict, output_dir, farm_base=None):
        super().__init__(config_dict, output_dir, farm_base)
        self._step_idx = 0
        self._cumulative_time = 0.0
        self.step_history = []

    def setup(self, add_outlist=True):
        super().setup(add_outlist)
        self._template_config = self.config.copy()

    def step(self, yaw_deg, pitch_deg):
        """Run one FAST.Farm time step with given yaw/pitch. Returns measurements."""
        seg_dir = os.path.join(self.output_dir, f"step_{self._step_idx:04d}")
        os.makedirs(seg_dir, exist_ok=True)

        config = self._template_config.copy()
        config["max_iter"] = 1

        from wfcrl.simul_utils import create_ff_case
        self.fstf_file = create_ff_case(config, output_dir=seg_dir)
        self.farm_base = os.path.dirname(self.fstf_file)
        self._add_outlist()
        self._fix_inflow()
        self.set_yaw_pitch(float(yaw_deg), float(pitch_deg))

        import subprocess as _sp
        ff_exe = FastFarmInterface.default_exe_path
        proc = _sp.Popen([ff_exe, self.fstf_file], cwd=self.farm_base,
                         stdout=_sp.PIPE, stderr=_sp.STDOUT, text=True, bufsize=1)
        for _ in proc.stdout: pass
        proc.wait()

        prefix = os.path.splitext(os.path.basename(self.fstf_file))[0]
        power_mw = np.zeros(self.n_turbines)
        for i in range(1, self.n_turbines + 1):
            ob = os.path.join(self.farm_base, f"{prefix}.T{i}.outb")
            o = os.path.join(self.farm_base, f"{prefix}.T{i}.out")
            if os.path.exists(ob): df = FASTOutputFile(ob).toDataFrame()
            elif os.path.exists(o): df = FASTOutputFile(o).toDataFrame()
            else: continue
            for col in df.columns:
                base = col.split('[')[0].strip('_ ')
                if base == 'GenPwr':
                    unit = col.split('[')[1].split(']')[0] if '[' in col else 'W'
                    p = df[col].values[-1]
                    p_mw = p / 1e3 if unit.upper() in ('KW','KILOWATT','KWATT') else p / 1e6
                    power_mw[i-1] = p_mw; break

        meas = {
            'step': self._step_idx, 'time': self._cumulative_time,
            'dt': self.config.get('dt', 3.0), 'power_mw': power_mw,
            'farm_power_mw': power_mw.sum(),
            'yaw_cmd': yaw_deg, 'pitch_cmd': pitch_deg,
        }
        self.step_history.append(meas)
        self._step_idx += 1
        self._cumulative_time += self.config.get('dt', 3.0)
        return meas


# =====================================================================
# HyCon-Style Controller Base Class
# =====================================================================
class FarmControllerBase:
    """HyCon-style farm-level controller base class. Subclasses implement compute_controls()."""
    def __init__(self, n_turbines):
        self.n_turbines = n_turbines
        self.control_history = []

    def compute_controls(self, measurement_dict):
        raise NotImplementedError

    def step(self, measurement_dict):
        controls = self.compute_controls(measurement_dict)
        self.control_history.append({
            'time': measurement_dict.get('time', 0),
            'yaw_cmd': controls.get('yaw'),
            'pitch_cmd': controls.get('pitch'),
        })
        return controls

    def sample_parameters(self):
        pass

    def update_wind(self, wind_speed: float = None, wind_direction: float = None):
        wind_direction = wind_direction % 360
        if wind_speed != self.wind_speed or wind_direction != self.wind_dir:
            self.fi.set(
                wind_speeds=[wind_speed] if wind_speed is not None else None,
                wind_directions=[wind_direction]
                if wind_direction is not None
                else None,
            )
