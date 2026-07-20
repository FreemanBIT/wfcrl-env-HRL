# WFCRL — Wind Farm Control RL & Simulation Interface

风电场控制仿真与强化学习框架，底层驱动 FAST.Farm（高保真 CFD/BEM）与 FLORIS（工程尾流模型）。

---

## 架构

```
wfcrl/
├── config/                    纯数据类型（无 RL 依赖）
│   ├── types.py               WindType, WindConfig, WindSegment
│   ├── control.py             ControlInput（5-Mode 场控协议）
│   ├── output.py              SimulationOutput（统一输出结构）
│   ├── simulator.py           SimulationConfig, FastFarmConfig, FlorisConfig
│   └── layout.py              FarmLayout + LayoutRegistry（11 个内置风场）
│
├── engine/                    仿真引擎层（零 RL 依赖，MPC/优化可直接使用）
│   ├── base.py                SimulatorInterface ABC
│   ├── state.py               SimulatorState（单步测量快照）
│   ├── constraints.py         ActuatorConstraints（执行器物理约束）
│   ├── angle_utils.py         风向/机舱朝向换算
│   ├── fastfarm_step.py       FastFarmInterface（每步重启 subprocess）
│   ├── fastfarm_continuous.py ContinuousFastFarmInterface（流场连续，DISCON bridge）
│   ├── floris.py              FlorisInterface（FLORIS Python API）
│   ├── _ff_case.py            FAST.Farm 输入文件生成
│   ├── _floris_case.py        FLORIS 输入文件生成
│   └── _outb.py / _outlist.py .outb 解析 / OutList 注入
│
├── mdp/                       RL 桥接层
│   └── mdp.py                 WindFarmMDP（状态/动作空间 + 转换）
│
├── envs/                      RL 环境（Gymnasium + PettingZoo）
│   ├── base.py                BaseWindFarmEnv
│   ├── centralized.py         WindFarmEnv（单智能体）
│   ├── multiagent.py          MAWindFarmEnv（多智能体）
│   ├── wrappers.py            LogWrapper, AECLogWrapper
│   └── rewards.py             RewardShaper
│
├── compat.py                  向后兼容层（旧 import 路径自动重导出）
├── compat_data.py             FarmCase 等遗产类兼容桩
└── registration.py            环境注册工厂

FarmInputs/
├── layouts/                   11 个风场布局 YAML
│   ├── dafeng_h1.yaml        (24 × Goldwind 8.5MW)
│   ├── horns_rev1.yaml       (80 × NREL 5MW)
│   ├── horns_rev2.yaml       (92 × NREL 5MW)
│   ├── ormonde.yaml          (31 × NREL 5MW)
│   ├── wmr.yaml              (36 × NREL 5MW)
│   ├── ablaincourt.yaml       (7 × NREL 5MW)
│   ├── tcrwp.yaml            (32 × NREL 5MW)
│   └── turb3_row1 ~ turb32_row5 (程序化生成)
│
└── output_channels/           输出通道/变量参考
    ├── fastfarm_channels.yaml (OpenFAST OutList 通道定义)
    └── floris_channels.yaml   (FLORIS 输出变量定义)
```

---

## 快速开始

### 纯仿真使用（MPC / 优化 — 零 RL 依赖）

```python
from wfcrl.config import FastFarmConfig, WindConfig
from wfcrl.engine import ContinuousFastFarmInterface
from wfcrl.config.layout import LayoutRegistry

layouts = LayoutRegistry.from_builtin()
layout = layouts.get("dafeng_h1")

config = FastFarmConfig(
    case_name="demo",
    num_turbines=layout.num_turbines,
    xcoords=layout.xcoords,
    ycoords=layout.ycoords,
    dt=3.0, max_iter=100,
    wind=WindConfig(speed=10, direction=270),
    output_preset="standard",
)

sim = ContinuousFastFarmInterface(config)
sim.setup()
sim.reset(config.wind)
sim.start()

for step in range(config.max_iter):
    controls = my_controller.compute(sim.read_measurements())
    sim.apply(controls)

final = sim.stop()
sim.close()
final.to_csv("results.csv")
```

### RL 使用

```python
from wfcrl.envs.centralized import WindFarmEnv
from wfcrl.mdp.mdp import WindFarmMDP
from wfcrl.config import FastFarmConfig, WindConfig
from wfcrl.engine import ContinuousFastFarmInterface

sim = ContinuousFastFarmInterface(config)
sim.setup(); sim.reset(config.wind); sim.start()

mdp = WindFarmMDP(simulator=sim, layout=layout,
                  controls={"yaw": (-20, 20, 5)}, warmup_steps=33)
env = WindFarmEnv(mdp=mdp)

obs = env.reset()
for _ in range(500):
    action = policy(obs)
    obs, reward, done, _, info = env.step(action)
```

### 兼容旧代码

```python
# 旧 import 路径仍然有效
from wfcrl import WindConfig, ControlInput, FastFarmInterface
```

---

## 仿真器接口

### 三种接口对比

| 接口 | 模式 | 流场 | 适用场景 |
|------|------|------|---------|
| `FastFarmInterface` | 每步重启 subprocess | 不连续 | 离线批量对比 |
| `ContinuousFastFarmInterface` | 一次启动，DISCON bridge | 物理连续 | 闭环控制 / MPC / RL |
| `FlorisInterface` | Python API 调用 | 稳态 | 快速迭代 / 优化 / LUT |

### ContinuousFastFarmInterface — 两步式 API

```python
sim.apply(controls)            # 写 controls.txt，立即返回
state = sim.read_measurements() # 轮询等待 DISCON 测量值
```

也提供便捷一步式 `sim.wait_step(controls)`。

### 5-Mode Farm Control Protocol

| Mode | 控制量 | 说明 |
|------|--------|------|
| 0 | yaw | 偏航绝对值 |
| 1 | power + min_pitch | 功率目标 + 最小变桨约束 |
| 2 | pitch | 变桨绝对值 |
| 3 | pitch + yaw | 变桨 + 偏航 |
| 4 | power + min_pitch + yaw | 全组合 |

```python
controls = ControlInput.mode0_yaw(n_turbines=24, yaw_deg=15.0)
controls = ControlInput.mode1_power(n_turbines=24, power_mw=4.0, min_pitch_deg=3.0)
```

---

## 输出配置

通过 `output_preset` 或 `output_channels` 选择输出变量：

```python
config = FastFarmConfig(
    ...,
    output_preset="standard",   # minimal / standard / loads / full
    # 或精确指定：
    # output_channels=["GenPwr", "YawPzn", "BldPitch1", "RootMIP1"],
)
```

通道定义文件：`FarmInputs/output_channels/fastfarm_channels.yaml`，可按需扩展。

### SimulationOutput

所有接口返回统一的 `SimulationOutput`：

| 属性 | 形状 | 说明 |
|------|------|------|
| `time` | (n_steps,) | 时间向量 |
| `power_mw` | (n_steps, n_turbines) | 每台风机功率 |
| `farm_power_mw` | (n_steps,) | 风场总功率 |
| `yaw_deg` | (n_steps, n_turbines) | 偏航失准角 |
| `pitch_deg` | (n_steps, n_turbines) | 变桨角 |
| `wind_speed` | (n_steps, n_turbines) | 风速 |
| `rotor_speed_rpm` | (n_steps, n_turbines) | 风轮转速 |
| `blade_loads` | (n_steps, n_turbines, 3) | 叶根弯矩 |

```python
output.to_csv("results.csv")
df = output.to_dataframe()
```

---

## 风场布局

| 布局 | 风机数 | 类型 |
|------|--------|------|
| dafeng_h1 | 24 | Goldwind 8.5MW |
| horns_rev1 | 80 | NREL 5MW |
| horns_rev2 | 92 | NREL 5MW |
| ormonde | 31 | NREL 5MW |
| wmr | 36 | NREL 5MW |
| tcrwp | 32 | NREL 5MW |
| ablaincourt | 7 | NREL 5MW |
| turb3_row1 ~ turb32_row5 | 3~32 | NREL 5MW（程序化） |

```python
from wfcrl.config.layout import LayoutRegistry
layouts = LayoutRegistry.from_builtin()
layout = layouts.get("horns_rev1")
```

---

## 安装

```bash
pip install -e ".[rl]"        # 含 RL 依赖 (gymnasium, pettingzoo)
pip install -e "."            # 仅仿真引擎 (MPC / 优化)
```

依赖：`numpy`, `pandas`, `PyYAML`, `matplotlib`, `FLORIS>=4.5`, `openfast_toolbox`

FAST.Farm 使用需额外：
1. 下载 `FAST.Farm_x64_OMP.exe` (OpenFAST v5.0.0) → 放入 `wfcrl/simulators/fastfarm/bin/`
2. 编译 DISCON bridge：`gfortran -shared -static -o DISCON_WT1.dll DISCON.F90`

---

## 相关文档

- `docs/REFACTOR_PLAN.md` — 架构重构方案
- `docs/INTERFACE.md` — 仿真接口详细文档
- `docs/BENCHMARK.md` — 基准测试说明
- `DEVELOPMENT_LOG.md` — 开发记录
