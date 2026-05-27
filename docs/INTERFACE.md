# WFCRL: 统一仿真接口文档

## 概述

WFCRL 提供统一的仿真器接口，支持 FAST.Farm (v5.0.0+) 和 FLORIS (v4.5+) 两种风场仿真器。
所有接口遵循 `SimulatorInterface` 协议，通过 subprocess（FAST.Farm）或 Python API（FLORIS）驱动仿真。

## 安装

```bash
pip install -r requirements.txt
```

可选 RL 依赖：
```bash
pip install -e ".[rl]"
```

## 快速开始

```python
from wfcrl.config import WindConfig, ControlInput
from wfcrl.interface import FastFarmInterface
from wfcrl.simul_config import FastFarmConfig

# 1. 配置
config = FastFarmConfig(
    case_name="demo",
    num_turbines=3,
    xcoords=[0, 504, 1008],
    ycoords=[0, 0, 0],
    dt=3.0,
    max_iter=10,
    wind=WindConfig(speed=10, direction=270),
)

# 2. 创建接口
ff = FastFarmInterface(config)
ff.setup()
ff.reset(config.wind)

# 3. 运行
controls = ControlInput.scalar(3, yaw_deg=5, pitch_deg=0)
output = ff.step(controls)
print(f"Farm power: {output.farm_power_mw[-1]:.2f} MW")

ff.close()
```

## 风况配置 (WindConfig)

`WindConfig` 支持 FAST.Farm InflowWind.dat 的全部 WindType (1-7)：

```python
from wfcrl.config import WindConfig, WindType

# 稳态风 (WindType=1)
w1 = WindConfig(wind_type=WindType.STEADY, speed=10, direction=270)

# TurbSim 二进制全流场 (WindType=3)
w2 = WindConfig(wind_type=WindType.TURBSIM_BTS, wind_file="90m_08mps.bts")

# 多段风况
from wfcrl.config import WindSegment
w3 = WindConfig(
    wind_type=WindType.STEADY,
    segments=[
        WindSegment(duration=60, speed=8, direction=270),
        WindSegment(duration=60, speed=12, direction=300),
    ],
)
```

**FAST.Farm 与 FLORIS 的区别**：
- FLORIS 仅原生支持 WindType=1 (稳态)
- 其他 WindType 在 FLORIS 中自动降级为稳态近似，并发出 warning

## 控制输入 (ControlInput)

支持 per-turbine 控制：

```python
controls = ControlInput(
    yaw=np.array([5.0, -3.0, 0.0]),     # 每台风机独立偏航 (度)
    pitch=np.array([0.0, 0.0, 2.0]),     # 每台风机独立变桨 (度)
    torque=None,                           # 转矩控制（可选）
)

# 所有风机相同
controls = ControlInput.scalar(3, yaw_deg=5.0, pitch_deg=0.0)

# 全零
controls = ControlInput.zero(3)
```

**注意**：FLORIS 仅支持 yaw 控制，pitch/torque 被忽略并发出 warning。

## 仿真输出 (SimulationOutput)

统一的输出数据结构：

| 字段 | 类型 | 说明 |
|---|---|---|
| `time` | (n_steps,) | 时间向量 (s) |
| `power_mw` | (n_steps, n_turbines) | 每台风机功率 (MW) |
| `wind_speed` | (n_steps, n_turbines) | 风速 (m/s) |
| `wind_direction` | (n_steps, n_turbines) | 风向 (度) |
| `yaw_deg` | (n_steps, n_turbines) | 偏航角 (度) |
| `pitch_deg` | (n_steps, n_turbines) | 变桨角 (度) |
| `torque_nm` | (n_steps, n_turbines) | 转矩 (Nm) |
| `rotor_speed_rpm` | (n_steps, n_turbines) | 风轮转速 (RPM) |
| `blade_loads` | (n_steps, n_turbines, 3+) | 叶片载荷 (FAST.Farm) |
| `thrust_n` | (n_steps, n_turbines) | 推力 (N) (FLORIS) |
| `farm_power_mw` | (n_steps,) | 风场总功率 (MW) |

方法：
- `output.to_dataframe()` → `pd.DataFrame`
- `output.to_csv(path)` → 保存 CSV

## 在线闭环控制

```python
ff = FastFarmInterface(config)
ff.setup()
ff.reset(wind)

for step in range(N_STEPS):
    # 根据上一步输出计算控制
    controls = controller.compute(output)

    # 执行一步仿真
    output = ff.step(controls)
    print(f"Step {step}: {output.farm_power_mw[-1]:.2f} MW")

ff.close()
```

## 批量运行

```python
controls_list = [ControlInput.scalar(3, yaw_deg=i) for i in range(10)]
output = ff.run(controls_list)
output.to_csv("results.csv")
```

## 预设案例

```python
from wfcrl.environments.data_cases import named_cases_dictionary

# 查看可用案例
print(list(named_cases_dictionary.keys()))
# ['Turb_TCRWP_', 'Turb3_Row1_', 'Turb6_Row2_', 'DafengH1_', 'Ablaincourt_', ...]

# 获取案例
base = named_cases_dictionary["DafengH1_"][0]  # [0]=layout info
print(f"{base.num_turbines} turbines, dt={base.dt}s")
```

## RL 环境

```python
from wfcrl import environments as envs

env = envs.make("DafengH1_Floris", max_num_steps=100)
env.reset()
# ... PettingZoo / Gymnasium API ...
```

## 配置

FAST.Farm 可执行文件路径通过以下方式设置（优先级从高到低）：
1. 环境变量 `FAST_FARM_EXE`
2. 构造函数参数 `FastFarmConfig(fastfarm_exe=...)`
3. 默认路径 `wfcrl/simulators/fastfarm/bin/FAST.Farm_x64_OMP.exe`
