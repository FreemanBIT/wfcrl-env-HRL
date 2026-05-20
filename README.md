# WFCRL: Interfacing and Benchmark Reinforcement Learning for Wind Farm Control

## Environments

List all environments with:

```
from wfcrl import environments as envs
envs.list_envs()
```

All wind farms environments are implemented with both the `Gymnasium` and `PettingZoo` API, and can be run on both the `Floris` and the `FAST.Farm` wind farm simulators.

The root name of the environment is associated with a specific layout, of arrangement of turbines in the field. It is combined with a prefix and a suffix:
- A `Dec_` prefix is added before environment names to indicate an Agent Environment Cycle implementation supported by `PettingZoo`.
- A `Floris` or `FAST.Farm` suffix is added after the name of the environment to indicate the name of the background simulator.


| Root Name          | **\# Agents** | **Description**     |
|----------------------------------|--------------------|--------------------------------------------------------------------------------------|
| DafengH1                         | 24                 | Layout of the DafengH1 wind farm (24 x Goldwind 8.5MW turbines, China)               |
| Ablaincourt                      | 7                  | Inspired by layout of the Ablaincourt farm    (Duc et al, 2019)            |
| Turb16_TCRWP                    | 16                 | Layout of the [Total Control Reference Wind Power Plant](https://farmconners.readthedocs.io/en/latest/provided_data_sets.html) (TC RWP) (the first 16 turbines)   |
| Turb6_Row2                      | 6                  | Custom case  - 2 rows of 3 turbines                                  |
| Turb16_Row5                     | 16                 | Layout of the first 32 turbines in the the CL-Windcon project [as implemented in WFSim](https://github.com/TUDelft-DataDrivenControl/WFSim/blob/master/layoutDefinitions/layoutSet_clwindcon_80turb.m)           |
| Turb32_Row5                     | 32                 | Layout of the farm used in the                            |
| TurbX_Row1 for X in [1, 12] | X                  | Procedurally generated single row layout with X turbines, |
| Ormonde                          | 31                 | Layout of the Ormonde Offshore Wind Farm                                             |
| WMR                              | 36                 | Layout of the Westermost Rough Offshore Wind Farm                                    |
| HornsRev1                        | 76                 | Layout of the Horns Rev 1 Offshore Wind Farm                                         |
| HornsRev2                        | 92                 | Layout of the Horns Rev 2 Offshore Wind Farm                                         |

A visual overview of some layouts:

| Turb7_Row1      | Ormonde | HornsRev2     |
|----------------------------------|--------------------|--------------------------------------------------------------------------------------|
|<img src="docs/layouts/layoutTurb7_Row1.svg" >   | <img src="docs/layouts/layoutOrmonde.svg" >   | <img src="docs/layouts/layoutHornsRev2.svg" >   |


## Example

Creating a wind farm environment of the Ablaincourt layout with the Floris background on Gymnasium:

```
from wfcrl import environments as envs
env = envs.make("Ablaincourt_Floris")
```

Examples of test cases using the PettingZoo environment are given in the `examples` folder:

| Script | Description |
|--------|-------------|
| `python examples/example_floris.py` | Simulate `Ablaincourt` layout on FLORIS |
| `mpiexec -n 1 python examples/run_dafeng_fastfarm.py` | Simulate `DafengH1` layout on FAST.Farm v5.0.0 (24 turbines) |

More detailed examples can be found in the `demo.ipynb` notebook. See below under *Running Example Notebooks*.

## Installation

In the virtual environment of your choice:

```
pip install -e .
```

### FAST.Farm Simulator (Windows)

WFCRL supports **FAST.Farm v5.0.0** (upgraded from v3.5.1).

1. **Download FAST.Farm v5.0.0** from the [OpenFAST v5.0.0 release page](https://github.com/OpenFAST/openfast/releases/tag/v5.0.0):
   - `FAST.Farm_x64_OMP.exe` (or `FAST.Farm_x64.exe` for non-OpenMP)
   - Place the executable in `wfcrl/simulators/fastfarm/bin/FAST.Farm_x64_OMP.exe`

2. **Download DISCON v5.0.0 DLL** from the [same release page](https://github.com/OpenFAST/openfast/releases/tag/v5.0.0):
   - `DISCON.dll` — NREL 5MW reference controller, pre-compiled for OpenFAST v5.0.0
   - Place it in `wfcrl/simulators/fastfarm/servo_dll/DISCON_WT1.dll`

3. **Install MS-MPI**:
   Download **BOTH** Windows MPI setup (.exe) and MPI SDK (.msi) from [Microsoft MPI](https://www.microsoft.com/en-us/download/details.aspx?id=100593)

   Verify your installation by running `set MSMPI` in a command prompt:

   ```
   MSMPI_BENCHMARKS=C:\Program Files\Microsoft MPI\Benchmarks\
   MSMPI_BIN=C:\Program Files\Microsoft MPI\Bin\
   MSMPI_INC=C:\Program Files (x86)\Microsoft SDKs\MPI\Include\
   MSMPI_LIB32=C:\Program Files (x86)\Microsoft SDKs\MPI\Lib\x86\
   MSMPI_LIB64=C:\Program Files (x86)\Microsoft SDKs\MPI\Lib\x64\
   ```

4. **Test the setup**:

   ```
   mpiexec -n 1 python examples/run_dafeng_fastfarm.py
   ```

## More details on interfacing with FAST.Farm

A simple tutorial to start a simulation with the FAST.Farm interface is available in the notebook `interface.ipynb` notebook. To properly launch the notebook, see the intructions below in *Running Examples Notebook*.

**Creating an interface from a WFCRL case:**

```
from wfcrl.environments import data_cases as cases
from wfcrl.interface import FastFarmInterface

config = cases.fastfarm_6t
interface = FastFarmInterface(config)
```

On Windows, by default, your FAST.Farm executable is assumed to be located in `wfcrl/simulators/fastfarm/bin/FAST.Farm_x64_OMP.exe`. If not, you can also pass it to the interface:

```
interface = FastFarmInterface(config, fast_farm_executable=path_to_exe)
```

**Creating an interface from existing configuration files:**
Alternatively, if you already have your simulation files ready, you can just point towards the `.fstf` file:
```
ff_interface = FastFarmInterface(fstf_file=path_to_fstf)
```

At every iteration, the FAST.Farm interface retrieves 12 measures per turbine:
- 2 wind measurements: wind velocity and direction at the entrance of the farm
- The current output power of the turbine
- The yaw of the turbine
- The pitch of the turbine
- The torque of the turbine
- 6 measures of blade loads

A detailed example can be found in the `interface.ipynb` notebook. To run this notebook, follow the instructions under *Running Example Notebooks*.


# Running Example Notebooks

On Windows, to run the `interface.ipynb` and `demo.ipynb` examples, you will first need to install the WFCRL kernel:

- Install `jupyter notebook` and `seaborn`:

```
pip install notebook seaborn
```

- Install the jupyter kernel

```
from wfcrl import jupyter_utils
jupyter_utils.create_ipykernel()
```
