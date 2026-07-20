# WFCRL 架构重构方案：RL 环境与仿真器解耦

> 状态：**待审核** | 作者：Hermes Agent | 日期：2026-07-20

## 0. 设计目标

1. **仿真器层完全独立**：`wfcrl/simulator/` 不依赖 gymnasium / pettingzoo / 任何 RL 库
2. **MPC 等非 RL 算法可直接使用仿真器**：`setup() → reset(wind) → start() → apply()/read_measurements() → stop()`
3. **RL 环境作为薄封装层**：`wfcrl/envs/` 仅做状态/奖励/动作空间转换
4. **MDP 保留为桥接层**：`wfcrl/mdp/` 负责 RL action ↔ ControlInput ↔ SimulatorState 转换
5. **约束管理下沉到仿真器层**：偏航/变桨速率限制等物理约束是仿真器能力，作为可选项

## 1. 目标架构

```
wfcrl/
│
├── config/                          ← [NEW] 纯数据类型，无逻辑依赖
│   ├── __init__.py                  # 统一导出
│   ├── types.py                     # WindType, WindSegment, WindConfig
│   ├── control.py                   # ControlInput (5-Mode Protocol)
│   ├── output.py                    # SimulationOutput
│   ├── simulator.py                 # SimulationConfig, FastFarmConfig, FlorisConfig
│   └── layout.py                    # FarmLayout dataclass + LayoutRegistry
│
├── simulator/                       ← [NEW] 零 RL 依赖的纯仿真器层
│   ├── __init__.py                  # 统一导出
│   ├── base.py                      # SimulatorInterface ABC
│   ├── state.py                     # SimulatorState (测量值快照), ApplyResult
│   ├── constraints.py               # ActuatorConstraints (可选的物理约束)
│   ├── angle_utils.py               # _wrap180, nacyaw_from_misalignment, misalignment_from_nacyaw
│   ├── _outb.py                     # _parse_outb_file, _parse_all_outb (内部)
│   ├── _outlist.py                  # _inject_outlist_channels (内部)
│   ├── _ff_case.py                  # create_ff_case, create_dll (内部, 从 simul_utils 迁移)
│   ├── _floris_case.py              # create_floris_case (内部)
│   ├── fastfarm_step.py             # FastFarmInterface — 每步重启 subprocess
│   ├── fastfarm_continuous.py       # ContinuousFastFarmInterface — 一次启动，流场连续
│   └── floris.py                    # FlorisInterface — FLORIS Python API
│
├── mdp/                             ← [KEEP] RL 桥接层
│   ├── __init__.py
│   └── mdp.py                       # WindFarmMDP (状态/动作空间 + 转换)
│
├── envs/                            ← [REFACTOR] 仅依赖 simulator + mdp
│   ├── __init__.py
│   ├── base.py                      # BaseWindFarmEnv (公共逻辑提取)
│   ├── centralized.py               # WindFarmEnv (Gymnasium)
│   ├── multiagent.py                # MAWindFarmEnv (PettingZoo)
│   ├── wrappers.py                  # LogWrapper, AECLogWrapper
│   └── rewards.py                   # RewardShaper, DoNothingReward, etc.
│
├── registration.py                  ← [SIMPLIFY] 环境注册（仅依赖 config + envs）
├── __init__.py                      # 顶层导出
│
├── simulators/                      ← [UNCHANGED] 二进制 + 模板
│   ├── fastfarm/
│   │   ├── bin/                     # FAST.Farm_x64_OMP.exe
│   │   ├── servo_dll/               # DISCON_WT1.dll
│   │   ├── inputs/template/         # 模板文件
│   │   └── src/                     # DISCON.F90
│   └── floris/
│       └── inputs/                  # FLORIS 模板 + turbine_library
│
└── [REMOVED]:
    ├── interface.py     → 拆分为 simulator/*.py
    ├── config.py        → 拆分为 config/types.py + config/control.py + config/output.py
    ├── simul_config.py  → config/simulator.py
    ├── simul_utils.py   → 拆分为 simulator/_ff_case.py + simulator/_floris_case.py
    ├── simple_env.py    → envs/centralized.py
    ├── multiagent_env.py → envs/multiagent.py
    ├── mdp.py           → mdp/mdp.py
    ├── wrappers.py      → envs/wrappers.py
    ├── rewards.py       → envs/rewards.py
    └── environments/    → 布局数据移至 config/layout.py，其余移至 registration.py
```

## 2. 关键接口设计

### 2.1 SimulatorInterface ABC（纯仿真器接口）

```python
# wfcrl/simulator/base.py

class SimulatorInterface(ABC):
    """纯仿真器接口 — 零 RL 依赖。"""

    n_turbines: int
    config: SimulationConfig

    @abstractmethod
    def setup(self) -> None:
        """生成仿真输入文件。"""
        ...

    @abstractmethod
    def reset(self, wind: WindConfig) -> None:
        """根据风况重置仿真器。"""
        ...

    @abstractmethod
    def step(self, controls: ControlInput) -> SimulationOutput:
        """执行一步仿真（批量模式）。"""
        ...

    def run(self, controls_list: Sequence[ControlInput]) -> SimulationOutput:
        """批量运行多步。"""
        ...

    @abstractmethod
    def close(self) -> None:
        """清理资源。"""
        ...
```

### 2.2 ContinuousFastFarmInterface（新增 apply/read_measurements 两步式）

```python
# wfcrl/simulator/fastfarm_continuous.py

class ContinuousFastFarmInterface(FastFarmInterface):
    """
    流场连续型 FAST.Farm 接口。

    生命周期：
        setup() → reset(wind) → start() → [apply() / read_measurements()] × N → stop() → close()

    三步控制模式（供 MPC 等）：
        state = sim.read_measurements()    # 读取当前测量
        controls = mpc.solve(state)         # 外部算法计算控制
        sim.apply(controls)                 # 下发控制，推进仿真

    便捷一步式（供简单脚本）：
        state = sim.wait_step(controls)     # apply + read_measurements
    """

    def setup(self) -> None: ...
    def reset(self, wind: WindConfig) -> None: ...
    def start(self) -> None: ...

    def apply(self, controls: ControlInput) -> None:
        """
        写 controls.txt，立即返回。DISCON 在下一个 DT_low 读取并应用。
        约束检查（若启用）：写入前对 controls 做 ActuatorConstraints.check()。
        """
        ...

    def read_measurements(self, timeout: float = 45.0) -> SimulatorState:
        """
        轮询 measurements_T*.txt，等到当前 step 的测量值可用为止。

        Returns:
            SimulatorState: 当前步的所有风机测量值快照

        Raises:
            FastFarmAborted: 进程退出或超时
        """
        ...

    def wait_step(self, controls: ControlInput, timeout: float = 45.0) -> SimulatorState:
        """
        便捷方法：apply(controls) + read_measurements(timeout)。

        适用于 RL / 简单脚本 — 下发控制并等待反馈。
        """
        self.apply(controls)
        return self.read_measurements(timeout=timeout)

    def stop(self, *, allow_partial: bool = True) -> SimulationOutput:
        """停止仿真，解析 .outb，返回完整时序。"""
        ...

    def close(self) -> None: ...
```

### 2.3 SimulatorState（测量值快照）

```python
# wfcrl/simulator/state.py

@dataclass
class SimulatorState:
    """一次 read_measurements() 的快照，仿真器层的数据结构。"""

    step_idx: int
    sim_time: float                                   # 当前仿真时间 (s)
    power_mw: np.ndarray                              # (n_turbines,)
    wind_speed: Optional[np.ndarray] = None           # (n_turbines,) 或 None
    yaw_deg: Optional[np.ndarray] = None              # (n_turbines,) 相对来流失准角
    pitch_deg: Optional[np.ndarray] = None            # (n_turbines,)
    torque_nm: Optional[np.ndarray] = None            # (n_turbines,)
    rotor_speed_rpm: Optional[np.ndarray] = None      # (n_turbines,)
    blade_loads: Optional[np.ndarray] = None          # (n_turbines, 3)
    n_turbines_measured: int = 0                      # 实际读到的风机数
    metadata: dict = field(default_factory=dict)      # {"warning": "...", ...}
```

### 2.4 ActuatorConstraints（物理约束，仿真器层可选项）

```python
# wfcrl/simulator/constraints.py

@dataclass
class ActuatorConstraints:
    """
    执行器物理约束 — 仿真器层的可选功能。

    若 FastFarmConfig.actuator_constraints 为 None，不施加任何约束。
    若设置，apply() 会在写 controls.txt 前自动裁剪。
    """

    yaw_rate_max: float = 0.3         # 偏航最大速率 (deg/s)
    pitch_rate_max: float = 8.0       # 变桨最大速率 (deg/s)
    max_actuation_fraction: float = 0.1  # 单控制维度最大执行时间占比

    def check(
        self,
        prev_controls: ControlInput,
        new_controls: ControlInput,
        dt: float,
        accumulated: Dict[str, np.ndarray],
        num_steps: int,
    ) -> ControlInput:
        """
        对 new_controls 施加约束，返回裁剪后的 ControlInput。

        内部追踪各维度累计执行量，当 actuating_frac >= max_actuation_fraction
        时将该维度清零。
        """
        ...

    def reset(self) -> None:
        """重置所有累计执行量。"""
        ...
```

### 2.5 Config 改造 — FastFarmConfig 新增字段

```python
# wfcrl/config/simulator.py

@dataclass
class FastFarmConfig(SimulationConfig):
    fastfarm_exe: Optional[str] = ...
    template_dir: Optional[str] = None
    dt_low: Optional[float] = None
    fstf_overrides: dict = field(default_factory=dict)
    wind_time_series_file: Optional[str] = None
    use_mod_ambwind3: bool = False
    actuator_constraints: Optional[ActuatorConstraints] = None  # ← NEW
    warmup_steps: int = 0  # ← NEW (替代 start_iter 在 env 层的硬编码)
```

### 2.6 Layout 外部化

```python
# wfcrl/config/layout.py

@dataclass
class FarmLayout:
    """纯风场布局数据 — 不依赖任何仿真器或 RL 配置。"""
    name: str
    num_turbines: int
    xcoords: List[float]
    ycoords: List[float]
    turbine_type: str = "nrel_5MW"
    metadata: dict = field(default_factory=dict)  # {"source": "HornsRev1", ...}

class LayoutRegistry:
    """风场布局注册表 — 从 YAML 文件或代码注册加载。"""

    @classmethod
    def from_yaml_directory(cls, path: str) -> "LayoutRegistry": ...

    @classmethod
    def from_builtin(cls) -> "LayoutRegistry":
        """加载内置布局（从 YAML 文件，而非硬编码 Python）。"""
        ...

    def get(self, name: str) -> FarmLayout: ...

    def list_names(self) -> List[str]: ...
```

布局数据文件格式（`FarmInputs/layouts/hornsrev1.yaml`）：
```yaml
name: HornsRev1
num_turbines: 80
turbine_type: nrel_5MW
source: "Horns Rev 1 offshore wind farm"
xcoords: [978.664, 1046.949, ...]
ycoords: [5447.127, 4888.844, ...]
```

### 2.7 RL Environment（精简后）

```python
# wfcrl/envs/centralized.py

class WindFarmEnv(gym.Env):
    """
    集中式风场控制 RL 环境 (Gymnasium API)。

    不再直接依赖 SimulatorInterface — 通过 MDP 间接使用。
    """

    def __init__(
        self,
        mdp: WindFarmMDP,
        reward_shaper: RewardShaper = DoNothingReward(),
        load_coef: float = 0.1,
    ):
        self.mdp = mdp
        self.reward_shaper = reward_shaper
        self.load_coef = load_coef
        self.action_space = mdp.action_space
        self.observation_space = mdp.state_space
        ...

    def reset(self, seed=None, options=None) -> dict:
        state = self.mdp.reset(seed, options)
        self.reward_shaper.reset()
        return state

    def step(self, action: dict):
        state, powers, loads, done = self.mdp.take_action(self._state, action)
        reward = self._compute_reward(powers, loads)
        self._state = state
        return state, reward, done, False, {"power": powers}
```

### 2.8 MDP（保留，下游依赖改为 SimulatorState）

```python
# wfcrl/mdp/mdp.py

class WindFarmMDP:
    """
    RL-MDP 桥接层。

    职责：
    1. RL 动作空间 / 状态空间定义
    2. RL action → ControlInput 转换
    3. SimulatorState → RL observation 转换
    4. 动作累积追踪（配合 ActuatorConstraints）
    """

    def __init__(
        self,
        simulator: SimulatorInterface,       # 接口依赖，非具体实现
        layout: FarmLayout,                  # 从 config 层注入
        controls: dict,                      # {"yaw": (-40,40,5), ...}
        continuous_control: bool = True,
        horizon: int = int(1e6),
        wind: WindConfig = None,             # 初始风况
        warmup_steps: int = 0,               # 替代 start_iter
    ):
        ...

    def reset(self, seed=None, options=None) -> dict:
        """重置仿真器 + 返回初始 RL 观测。"""
        ...

    def take_action(self, state: dict, action: dict) -> tuple:
        """RL action → ControlInput → simulator.wait_step() → RL observation。"""
        ...
```

## 3. 使用示例

### 3.1 非 RL 使用（MPC）

```python
from wfcrl.config import FastFarmConfig, WindConfig, ControlInput, FarmLayout
from wfcrl.config.layout import LayoutRegistry
from wfcrl.simulator import ContinuousFastFarmInterface

# 加载布局
layouts = LayoutRegistry.from_builtin()
layout = layouts.get("DafengH1")

# 配置
config = FastFarmConfig(
    case_name="mpc_demo",
    **layout.to_config_args(),  # num_turbines, xcoords, ycoords
    dt=3.0, max_iter=100,
    wind=WindConfig(speed=10, direction=270),
)

# 使用仿真器（纯仿真，无 RL 依赖）
sim = ContinuousFastFarmInterface(config)
sim.setup()
sim.reset(config.wind)
sim.start()

# MPC 循环
for step in range(config.max_iter):
    state = sim.read_measurements()           # 获取当前测量
    controls = my_mpc_solver.solve(state)     # 优化求解
    sim.apply(controls)                       # 下发控制

final_output = sim.stop()
sim.close()
final_output.to_csv("mpc_results.csv")
```

### 3.2 RL 使用

```python
from wfcrl.envs import WindFarmEnv
from wfcrl.mdp import WindFarmMDP
from wfcrl.config import FastFarmConfig, WindConfig
from wfcrl.config.layout import LayoutRegistry
from wfcrl.simulator import ContinuousFastFarmInterface

# 仿真器
layouts = LayoutRegistry.from_builtin()
config = FastFarmConfig(...)
sim = ContinuousFastFarmInterface(config)
sim.setup()
sim.reset(config.wind)
sim.start()

# MDP 桥接
mdp = WindFarmMDP(
    simulator=sim,
    layout=layouts.get("DafengH1"),
    controls={"yaw": (-20, 20, 5)},
    warmup_steps=33,   # t_init / dt
)

# RL 环境
env = WindFarmEnv(mdp=mdp)

obs = env.reset()
for _ in range(500):
    action = policy(obs)
    obs, reward, done, truncated, info = env.step(action)
```

## 4. 输出变量配置系统（NEW）

### 4.1 设计动机

当前代码的 `DEFAULT_OUTLIST_CHANNELS` 和 `CHANNEL_TO_OUTPUT` 硬编码在 `interface.py` 中，用户无法选择输出哪些变量。对于不同的研究场景（功率优化 vs 载荷分析 vs 尾流研究），需要不同的输出通道组合。本方案将：

1. 将 FAST.Farm/OpenFAST 的所有可用输出通道整理为参考 YAML 文件
2. 将 FLORIS 的所有可用输出变量整理为参考 YAML 文件
3. 在仿真配置中允许用户选择输出通道（按名称 / 按预设组）
4. 程序自动修改仿真输入文件（.fstf / .fst / ED/ServoDyn 的 OutList）

### 4.2 参考文件

```text
FarmInputs/output_channels/
├── fastfarm_channels.yaml    # FAST.Farm/OpenFAST 全部输出通道参考
└── floris_channels.yaml      # FLORIS 全部输出变量参考
```

#### fastfarm_channels.yaml 格式

```yaml
# FAST.Farm / OpenFAST Output Channels Reference
# 来源：OpenFAST v5.0.0 OutListParameters.xlsx
# 用途：程序根据此文件判断每个通道属于哪个模块、如何解析

channels:
  # ---- ServoDyn 通道 ----
  - name: GenPwr
    module: ServoDyn
    description: "Generator power"
    units: kW
    output_key: power        # 映射到 SimulationOutput.power_mw（自动 /1000）
    group: performance
    preset: [minimal, standard]

  - name: GenTq
    module: ServoDyn
    description: "Generator torque"
    units: kN·m
    output_key: generator_torque
    group: torque
    preset: [standard]

  # ---- ElastoDyn 通道 ----
  - name: YawPzn
    module: ElastoDyn
    description: "Yaw position (absolute nacelle heading)"
    units: deg
    output_key: yaw
    group: orientation
    preset: [minimal, standard]

  - name: BldPitch1
    module: ElastoDyn
    description: "Blade 1 pitch angle"
    units: deg
    output_key: pitch
    group: pitch
    preset: [minimal, standard]

  - name: RotSpeed
    module: ElastoDyn
    description: "Rotor speed"
    units: rpm
    output_key: rotor_speed
    group: rotor
    preset: [standard]

  - name: RootMIP1
    module: ElastoDyn
    description: "Blade 1 root in-plane moment"
    units: kN·m
    output_key: blade_load_mip
    group: loads
    preset: [standard, loads]

  - name: RootMOoP1
    module: ElastoDyn
    description: "Blade 1 root out-of-plane moment"
    units: kN·m
    output_key: blade_load_moop
    group: loads
    preset: [standard, loads]

  - name: RootMzb1
    module: ElastoDyn
    description: "Blade 1 root torsional moment"
    units: kN·m
    output_key: blade_load_mzb
    group: loads
    preset: [standard, loads]

  # ---- InflowWind 通道 ----
  - name: Wind1VelX
    module: InflowWind
    description: "Wind velocity X at point 1"
    units: m/s
    output_key: wind_x
    group: wind
    preset: [standard]

  - name: Wind1VelY
    module: InflowWind
    description: "Wind velocity Y at point 1"
    units: m/s
    output_key: wind_y
    group: wind
    preset: [standard]

  - name: Wind1VelZ
    module: InflowWind
    description: "Wind velocity Z at point 1"
    units: m/s
    output_key: wind_z
    group: wind
    preset: [standard]

  # ---- AeroDyn 通道（可选高级通道） ----
  - name: B1N3Flx1
    module: AeroDyn
    description: "Blade 1 node 3 flapwise moment"
    group: aero_loads
    preset: [loads]

  # ... (约 50-80 个常用通道，覆盖所有相关模块)

# 预设组：快速选择常用输出组合
presets:
  minimal:
    description: "最小输出 — 仅功率 + 偏航 + 变桨"
    channels: [GenPwr, YawPzn, BldPitch1]

  standard:
    description: "标准输出 — 功率/载荷/风速/偏航/变桨/转速"
    channels: [GenPwr, GenTq, RotSpeed, YawPzn, BldPitch1,
               RootMIP1, RootMOoP1, RootMzb1,
               Wind1VelX, Wind1VelY, Wind1VelZ]

  loads:
    description: "载荷分析输出 — 标准 + 叶根全部弯矩"
    channels: [GenPwr, GenTq, RotSpeed, YawPzn, BldPitch1,
               RootMIP1, RootMIP2, RootMIP3,
               RootMOoP1, RootMOoP2, RootMOoP3,
               RootMzb1, RootMzb2, RootMzb3,
               Wind1VelX, Wind1VelY, Wind1VelZ]

  full:
    description: "全部通道"
    channels: "@all"
```

#### floris_channels.yaml 格式

```yaml
# FLORIS Output Variables Reference
# FLORIS 输出是结构化的（API 方法调用），不是通道注入。

variables:
  - name: turbine_powers
    description: "Per-turbine electrical power"
    units: W
    output_key: power           # /1e6 → SimulationOutput.power_mw
    group: performance
    api_method: get_turbine_powers
    default: true

  - name: turbine_thrusts
    description: "Per-turbine thrust force"
    units: N
    output_key: thrust
    group: loads
    api_method: get_turbine_thrusts
    default: false

  - name: turbine_yaw_angles
    description: "Per-turbine yaw angle"
    units: deg
    output_key: yaw
    group: control
    api_method: attribute (core.farm.yaw_angles)
    default: true

  - name: local_wind_speed
    description: "Local wind speed at each turbine"
    units: m/s
    output_key: wind_speed
    group: wind
    api_method: custom (_local_wind_measurements)
    default: true

  - name: local_wind_direction
    description: "Local wind direction at each turbine"
    units: deg
    output_key: wind_direction
    group: wind
    api_method: custom (_local_wind_measurements)
    default: true

  - name: turbulence_intensity
    description: "Turbulence intensity at each turbine"
    output_key: turbulence
    group: wind
    api_method: custom (_local_load_proxies)
    default: false

  - name: velocity_std
    description: "Velocity standard deviation (u, v, w) at each turbine"
    output_key: velocity_std
    group: loads
    api_method: custom (_local_load_proxies)
    default: false

presets:
  minimal:
    channels: [turbine_powers, turbine_yaw_angles]
  standard:
    channels: [turbine_powers, turbine_thrusts, turbine_yaw_angles,
               local_wind_speed, local_wind_direction]
  loads:
    channels: [turbine_powers, turbine_thrusts, turbine_yaw_angles,
               local_wind_speed, local_wind_direction,
               turbulence_intensity, velocity_std]
```

### 4.3 配置接口

```python
# wfcrl/config/simulator.py

@dataclass
class FastFarmConfig(SimulationConfig):
    ...
    # 输出通道配置（三种方式，优先级从高到低）
    output_channels: Optional[List[str]] = None    # 精确指定通道名列表
    output_preset: Optional[str] = "standard"      # 使用预设组: minimal/standard/loads/full
    output_channel_ref: Optional[str] = None       # 参考文件路径，默认使用内置

@dataclass
class FlorisConfig(SimulationConfig):
    ...
    output_variables: Optional[List[str]] = None
    output_preset: Optional[str] = "standard"
    output_variable_ref: Optional[str] = None
```

### 4.4 内部实现

#### ChannelRegistry（通道注册表）

```python
# wfcrl/simulator/_channel_registry.py

@dataclass
class ChannelSpec:
    """单个输出通道/变量的元数据。"""
    name: str
    module: str           # FAST.Farm: 模块名 (ElastoDyn/...); FLORIS: ""
    description: str
    output_key: str       # 映射到 SimulationOutput 的字段
    group: str = ""
    api_method: str = ""  # FLORIS: API 方法名

class ChannelRegistry:
    """从 YAML 参考文件加载输出通道/变量定义。"""

    def __init__(self, yaml_path: str): ...

    def resolve(self, channels: List[str] = None, preset: str = None) -> List[ChannelSpec]:
        """
        解析用户指定的通道列表。

        优先级: channels > preset > default(=standard)
        返回: 按模块分组的 ChannelSpec 列表
        """
        ...

    def group_by_module(self, specs: List[ChannelSpec]) -> Dict[str, List[ChannelSpec]]:
        """按模块分组（仅 FAST.Farm 有意义）。"""
        ...

    @property
    def default_preset(self) -> str:
        return "standard"
```

#### _outlist.py 改造

```python
# wfcrl/simulator/_outlist.py

def inject_outlist_channels(
    farm_base: str,
    fstf_file: str,
    channel_registry: ChannelRegistry,
    selected_specs: List[ChannelSpec],
) -> None:
    """
    根据用户选择的通道，自动注入到各风机对应模块的 OutList 中。

    逻辑：
    1. 从 channel_registry 获取每个通道所属模块
    2. 按模块分组
    3. 遍历每台风机，向 ED/ServoDyn/InflowWind/AeroDyn 注入对应通道
    """
    by_module = channel_registry.group_by_module(selected_specs)
    # ED 通道 → ElastoDyn .dat OutList
    # SrvD 通道 → ServoDyn .dat OutList
    # IfW 通道 → InflowWind .dat OutList
    # AD 通道 → AeroDyn .dat OutList
    ...
```

#### _outb.py 改造

```python
# wfcrl/simulator/_outb.py

def parse_outb_with_registry(
    outb_path: str,
    channel_registry: ChannelRegistry,
    selected_specs: List[ChannelSpec],
) -> Dict[str, np.ndarray]:
    """
    根据 channel_registry 中的 output_key + units 信息，
    解析 .outb 并正确转换单位。
    """
    ...
```

---

## 5. induction_vs_yaw_study 重构方案

### 5.1 现状

```text
induction_vs_yaw_study/      # ~3,900 行 Python
├── __init__.py
├── constants.py
├── analysis/                # 分析与可视化
│   ├── metrics.py           # 功率/载荷指标计算
│   └── plots.py             # 对比图绘制
├── cases/                   # 测试用例定义
│   └── three_nrel5mw.py
├── conversion/              # 功率设定值转换工具
│   └── power_setpoint_tools.py
├── lut/                     # Lookup Table (离线预计算)
│   ├── schema.py
│   └── interpolate.py
├── optimize/                # FLORIS 优化 (偏航/降额)
│   ├── _floris_model.py
│   ├── floris_yaw_opt.py
│   └── floris_derating_opt.py
├── replay/                  # 在 FAST.Farm 上复现优化结果
│   ├── fastfarm_replay.py
│   ├── floris_replay.py
│   └── schedule.py
├── steady/                  # 稳态变体
│   ├── cases_steady.py
│   ├── replay_steady.py
│   ├── run_compare_steady.py
│   └── run_replay_steady.py
├── run/                     # 编排脚本
│   ├── stage0_probe.py      # Step 0: 探测贪婪基准
│   ├── build_luts.py        # Step 1: 构建 LUT
│   ├── run_compare.py       # Step 2: 运行对比
│   └── run_replay.py        # Step 3: 在 FAST.Farm 复现
├── turbsim_inp/             # TurbSim 输入文件生成
│   └── generate_turbsim_inputs.py
└── tests/                   # 现有测试（保留）
    ├── test_cases.py
    ├── test_conversion.py
    └── test_lut.py
```

### 5.2 重构策略

**核心原则**：induction_vs_yaw_study 作为独立的应用案例包，**不合并进 wfcrl 核心库**。重构仅涉及：
1. 更新导入路径到新的 `wfcrl.config` / `wfcrl.simulator`
2. 使用 `ContinuousFastFarmInterface.apply()/read_measurements()` 替代旧的 `wait_step()` 模式
3. 使用 `SimulatorState` 替代直接操作 `SimulationOutput`
4. 使用新的 `FarmLayout` + `LayoutRegistry` 替代旧的 `FarmCase`

**不做**：
- 不改变其目录结构
- 不改变其数据分析逻辑 (analysis/)
- 不改变其优化算法 (optimize/)
- 不改变其 LUT 数据结构 (lut/)

### 5.3 具体变更

#### 5.3.1 导入路径更新

```python
# 旧的导入 (全部更新)
from wfcrl.config import WindConfig, ControlInput, SimulationOutput, WindType
from wfcrl.simul_config import FastFarmConfig, FlorisConfig
from wfcrl.interface import ContinuousFastFarmInterface, FastFarmInterface, FlorisInterface
from wfcrl.environments import FarmCase

# 新的导入
from wfcrl.config import WindConfig, ControlInput, SimulationOutput, WindType
from wfcrl.config.simulator import FastFarmConfig, FlorisConfig
from wfcrl.config.layout import FarmLayout, LayoutRegistry
from wfcrl.simulator import (
    ContinuousFastFarmInterface,
    FastFarmInterface,
    FlorisInterface,
    SimulatorState,
)
```

#### 5.3.2 replay/fastfarm_replay.py — 关键变更

```python
# 旧的模式 (wait_step 一步式)
def replay_schedule(sim: ContinuousFastFarmInterface, schedule: Schedule, ...):
    for step, controls in enumerate(schedule.to_controls()):
        output = sim.wait_step(controls)      # 一步完成
        records.append(extract(output))

# 新的模式 (apply + read_measurements 两步式，支持中间计算)
def replay_schedule(sim: ContinuousFastFarmInterface, schedule: Schedule, ...):
    for step, controls in enumerate(schedule.to_controls()):
        sim.apply(controls)                    # 下发控制，立即返回
        state = sim.read_measurements()        # 等待 + 读取测量
        records.append(state_to_record(state))
```

#### 5.3.3 optimize/ — FLORIS 优化脚本

```python
# 旧的模式
from wfcrl.interface import FlorisInterface
from wfcrl.simul_config import FlorisConfig

config = FlorisConfig(...)
floris = FlorisInterface(config)
floris.setup()
floris.reset(wind)
output = floris.step(controls)

# 新的模式 (接口不变，仅导入路径变化)
from wfcrl.simulator.floris import FlorisInterface
from wfcrl.config.simulator import FlorisConfig

config = FlorisConfig(...)
floris = FlorisInterface(config)
floris.setup()
floris.reset(wind)
output = floris.step(controls)
```

#### 5.3.4 run/ 编排脚本 — 使用新 API

```python
# run/run_replay.py — 重构后
from wfcrl.simulator import ContinuousFastFarmInterface
from wfcrl.config.simulator import FastFarmConfig
from wfcrl.config.layout import LayoutRegistry

layouts = LayoutRegistry.from_builtin()
layout = layouts.get("HornsRev1")

config = FastFarmConfig(
    case_name="yaw_replay",
    num_turbines=layout.num_turbines,
    xcoords=layout.xcoords,
    ycoords=layout.ycoords,
    dt=3.0,
    max_iter=100,
    wind=WindConfig(speed=8, direction=0),
    output_preset="standard",     # 使用标准输出通道
    actuator_constraints=ActuatorConstraints(),  # 启用偏航速率检查
)

sim = ContinuousFastFarmInterface(config)
sim.setup()
sim.reset(config.wind)
sim.start()

replay_schedule(sim, optimized_schedule)

final_output = sim.stop()
sim.close()
```

### 5.4 重构文件清单

| 文件 | 变更类型 | 说明 |
|------|---------|------|
| `induction_vs_yaw_study/__init__.py` | 修改 | 更新导入 |
| `induction_vs_yaw_study/replay/fastfarm_replay.py` | **重写** | apply()/read_measurements() 两步式 |
| `induction_vs_yaw_study/replay/floris_replay.py` | 修改 | 导入路径更新 |
| `induction_vs_yaw_study/replay/schedule.py` | 修改 | 导入路径更新 |
| `induction_vs_yaw_study/optimize/_floris_model.py` | 修改 | 导入路径更新 |
| `induction_vs_yaw_study/optimize/floris_yaw_opt.py` | 修改 | 导入路径更新 |
| `induction_vs_yaw_study/optimize/floris_derating_opt.py` | 修改 | 导入路径更新 |
| `induction_vs_yaw_study/lut/interpolate.py` | 无变更 | LUT 数据结构不变 |
| `induction_vs_yaw_study/lut/schema.py` | 无变更 | LUT 数据结构不变 |
| `induction_vs_yaw_study/conversion/power_setpoint_tools.py` | 修改 | 导入路径更新 |
| `induction_vs_yaw_study/analysis/metrics.py` | 修改 | SimulatorState → 指标计算 |
| `induction_vs_yaw_study/analysis/plots.py` | 修改 | 导入路径更新 |
| `induction_vs_yaw_study/cases/three_nrel5mw.py` | 修改 | FarmLayout 替代 FarmCase |
| `induction_vs_yaw_study/run/stage0_probe.py` | 修改 | 新 API |
| `induction_vs_yaw_study/run/build_luts.py` | 修改 | 新导入 |
| `induction_vs_yaw_study/run/run_compare.py` | 修改 | 新 API |
| `induction_vs_yaw_study/run/run_replay.py` | **重写** | 新 API + output_channels |
| `induction_vs_yaw_study/steady/*.py` | 修改 | 导入路径统一更新 |
| `induction_vs_yaw_study/tests/*.py` | 修改 | 导入路径更新 |
| `induction_vs_yaw_study/constants.py` | 修改 | 使用 FarmLayout 替代 FarmCase |

## 6. 文件变更清单

### 6.1 新建文件

| 文件 | 来源 | 行数估算 |
|------|------|---------|
| `wfcrl/config/__init__.py` | NEW | ~30 |
| `wfcrl/config/types.py` | config.py (WindConfig parts) | ~250 |
| `wfcrl/config/control.py` | config.py (ControlInput) | ~120 |
| `wfcrl/config/output.py` | config.py (SimulationOutput) | ~130 |
| `wfcrl/config/simulator.py` | simul_config.py | ~280 |
| `wfcrl/config/layout.py` | data_cases.py (layout data only) | ~150 |
| `wfcrl/simulator/__init__.py` | NEW | ~40 |
| `wfcrl/simulator/base.py` | interface.py (SimulatorInterface ABC) | ~80 |
| `wfcrl/simulator/state.py` | NEW | ~60 |
| `wfcrl/simulator/constraints.py` | simple_env + multiagent_env (actuator logic) | ~100 |
| `wfcrl/simulator/angle_utils.py` | interface.py (angle functions) | ~60 |
| `wfcrl/simulator/_channel_registry.py` | **NEW** — 输出通道注册表 | ~120 |
| `wfcrl/simulator/_outb.py` | interface.py (outb parser, 改造) | ~150 |
| `wfcrl/simulator/_outlist.py` | interface.py (outlist injection, 改造) | ~120 |
| `wfcrl/simulator/_ff_case.py` | simul_utils.py (FAST.Farm part) | ~450 |
| `wfcrl/simulator/_floris_case.py` | simul_utils.py (FLORIS part) | ~60 |
| `wfcrl/simulator/fastfarm_step.py` | interface.py (FastFarmInterface) | ~250 |
| `wfcrl/simulator/fastfarm_continuous.py` | interface.py (ContinuousFastFarmInterface) | ~450 |
| `wfcrl/simulator/floris.py` | interface.py (FlorisInterface) | ~280 |
| `wfcrl/mdp/__init__.py` | NEW | ~5 |
| `wfcrl/mdp/mdp.py` | mdp.py (原位置迁移) | ~250 |
| `wfcrl/envs/__init__.py` | NEW | ~20 |
| `wfcrl/envs/base.py` | NEW (公共逻辑提取) | ~80 |
| `wfcrl/envs/centralized.py` | simple_env.py (精简) | ~80 |
| `wfcrl/envs/multiagent.py` | multiagent_env.py (精简) | ~200 |
| `wfcrl/envs/wrappers.py` | wrappers.py | ~90 |
| `wfcrl/envs/rewards.py` | rewards.py | ~50 |
| `FarmInputs/layouts/*.yaml` | data_cases.py 坐标数据 | 11 个 YAML 文件 |
| `FarmInputs/output_channels/fastfarm_channels.yaml` | **NEW** — 全部通道参考 | ~200 |
| `FarmInputs/output_channels/floris_channels.yaml` | **NEW** — 全部变量参考 | ~80 |

### 6.2 修改文件

| 文件 | 变更 |
|------|------|
| `wfcrl/__init__.py` | 更新导入路径 |
| `wfcrl/registration.py` | 使用 LayoutRegistry + 新导入路径 |
| `pyproject.toml` | 更新 package 包含路径 |
| `README.md` | 更新架构说明 + 导入示例 |
| `examples/example_FASTFarm.py` | 更新导入路径 |
| `examples/example_floris.py` | 更新导入路径 |

### 6.3 删除文件

| 文件 | 原因 |
|------|------|
| `wfcrl/config.py` | 拆分为 config/{types,control,output}.py |
| `wfcrl/simul_config.py` | → config/simulator.py |
| `wfcrl/simul_utils.py` | 拆分为 simulator/{_ff_case,_floris_case}.py |
| `wfcrl/interface.py` | 拆分为 simulator/{base,fastfarm_step,fastfarm_continuous,floris}.py + 内部工具 |
| `wfcrl/simple_env.py` | → envs/centralized.py |
| `wfcrl/multiagent_env.py` | → envs/multiagent.py |
| `wfcrl/mdp.py` | → mdp/mdp.py |
| `wfcrl/wrappers.py` | → envs/wrappers.py |
| `wfcrl/rewards.py` | → envs/rewards.py |
| `wfcrl/environments/__init__.py` | FarmCase 等不再需要 |
| `wfcrl/environments/data_cases.py` | 布局数据 → FarmInputs/layouts/*.yaml |
| `wfcrl/jupyter_utils.py` | 不再需要（MPI kernel 是遗留物） |

## 7. 向后兼容

提供了一个兼容层 `wfcrl/compat.py`（临时，1-2 个版本后删除）：

```python
# wfcrl/compat.py — 临时向后兼容
from wfcrl.config.control import ControlInput
from wfcrl.config.output import SimulationOutput
from wfcrl.config.types import WindConfig, WindType, WindSegment
from wfcrl.config.simulator import SimulationConfig, FastFarmConfig, FlorisConfig
from wfcrl.simulator import (
    SimulatorInterface,
    FastFarmInterface,
    ContinuousFastFarmInterface,
    FlorisInterface,
)

__all__ = [...]  # 所有旧名称
```

旧代码 `from wfcrl.config import ControlInput` 仍可用（通过重导出），但建议逐步迁移到新路径。

## 8. 执行顺序

```
Phase 1: 新建包结构（无破坏性）
  1.1 创建 wfcrl/config/{__init__,types,control,output,simulator,layout}.py
  1.2 创建 wfcrl/simulator/{__init__,base,state,constraints,angle_utils}.py
  1.3 创建 wfcrl/simulator/{_outb,_outlist,_ff_case,_floris_case}.py
  1.4 创建 wfcrl/simulator/{fastfarm_step,fastfarm_continuous,floris}.py
  1.5 创建 wfcrl/mdp/__init__.py
  1.6 创建 wfcrl/envs/{__init__,base,centralized,multiagent,wrappers,rewards}.py
  1.7 创建 FarmInputs/layouts/*.yaml（从 data_cases.py 提取坐标）
  1.8 创建 FarmInputs/output_channels/fastfarm_channels.yaml
  1.9 创建 FarmInputs/output_channels/floris_channels.yaml
  1.10 创建 wfcrl/compat.py（向后兼容层）

Phase 2: 迁移代码（内容搬运，保持功能不变）
  2.1 从 config.py 提取内容到 config/{types,control,output}.py
  2.2 从 simul_config.py 迁移到 config/simulator.py
  2.3 从 interface.py 拆分到 simulator/ 各文件
  2.4 从 simul_utils.py 拆分到 simulator/{_ff_case,_floris_case}.py
  2.5 从 mdp.py 迁移到 mdp/mdp.py（依赖改为 SimulatorState）
  2.6 从 simple_env.py + multiagent_env.py 迁移到 envs/
  2.7 迁移 wrappers.py, rewards.py 到 envs/

Phase 3: 切换导入（更新所有引用）
  3.1 更新 wfcrl/__init__.py
  3.2 更新 registration.py
  3.3 更新 examples/
  3.4 更新 induction_vs_yaw_study/ （导入路径 + API 切换）
  3.5 更新 docs/ 和 README.md

Phase 4: 验证
  4.1 运行现有测试（induction_vs_yaw_study/tests/）
  4.2 运行示例脚本 example_FASTFarm.py
  4.3 运行示例脚本 example_floris.py
  4.4 验收：新路径可以独立使用仿真器（不 import gymnasium）
  4.5 验收：induction_vs_yaw_study 脚本可正常运行

Phase 5: 清理
  5.1 删除旧文件列表（见 6.3）
  5.2 更新 pyproject.toml
```

## 9. 风险与缓解

| 风险 | 缓解 |
|------|------|
| 导入路径变更导致下游代码断裂 | Phase 1 先创建 compat.py，Phase 3 再切换 |
| FAST.Farm 模板目录相对路径变化 | `_ff_case.py` 内部 `__file__` 路径需更新（从 `wfcrl/simulator/` 而非 `wfcrl/` 计算） |
| `induction_vs_yaw_study/` 依赖旧导入 | 该目录使用 `from wfcrl.interface import ...`，需在 Phase 3 统一更新 |
| 本次重构涉及 ~15 个新建文件 + ~15 个删除文件 | Claude Code 逐 Phase 执行，每 Phase 结束后人工审核 diff |

---

> **此方案待审核。审核通过后交由 Claude Code 按 Phase 顺序执行。**
