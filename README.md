# WFCRL: Interfacing and Benchmark Reinforcement Learning for Wind Farm Control

---

## 环境总览

列出所有可用环境：

```python
from wfcrl import environments as envs
envs.list_envs()
```

所有风场环境同时支持 `Gymnasium` 和 `PettingZoo` API，可在 **FLORIS** 和 **FAST.Farm** 两种风场仿真器上运行。

环境命名规则：
- `Dec_` 前缀 → PettingZoo Agent Environment Cycle 实现
- `{布局名称}_{仿真器}` 后缀 → 如 `DafengH1_Floris`、`DafengH1_Fastfarm`

| 布局名称 | 风机数 | 说明 |
|---------|--------|------|
| DafengH1 | 24 | 江苏大丰 H1 风场 (24 × Goldwind 8.5MW) |
| Ablaincourt | 7 | 基于 Ablaincourt 风场布局 (Duc et al, 2019) |
| Turb16_TCRWP | 16 | TC RWP 参考风场（前 16 台） |
| Turb6_Row2 | 6 | 自定义 2 行 × 3 列布局 |
| Turb16_Row5 | 16 | CL-Windcon 项目布局 |
| Turb32_Row5 | 32 | CL-Windcon 项目布局 |
| TurbX_Row1 (X=1..12) | X | 程序生成单行布局 |
| Ormonde | 31 | Ormonde 海上风场 |
| WMR | 36 | Westermost Rough 海上风场 |
| HornsRev1 | 76 | Horns Rev 1 海上风场 |
| HornsRev2 | 92 | Horns Rev 2 海上风场 |
| TCRWP | 32 | Total Control Reference Wind Power Plant |

部分布局可视化：

| Turb7_Row1 | Ormonde | HornsRev2 | DafengH1 |
|-----------|---------|-----------|----------|
| <img src="docs/layouts/layoutTurb7_Row1.svg"> | <img src="docs/layouts/layoutOrmonde.svg"> | <img src="docs/layouts/layoutHornsRev2.svg"> | <img src="docs/layouts/DafengH1.png"> |

---

## 示例脚本

所有示例脚本位于 `examples/` 目录：

| 脚本 | 说明 |
|------|------|
| `python examples/example_FASTFarm.py` | **FAST.Farm 连续闭环控制** — 使用 `ContinuousFastFarmInterface` 一次启动 FAST.Farm，通过 DISCON bridge DLL 逐时间步交换控制/测量，流场持续演化（物理正确）。支持 CLI：`--case`/`--steps`/`--wind_speed`/`--wind_direction`。输出全场 + 前 6 台单机功率时序图。 |
| `python examples/example_floris.py` | **FLORIS 在线控制** — 使用 `FlorisInterface` 通过 FLORIS Python API 逐时间步计算尾流和功率。与 `example_FASTFarm.py` 相同的 CLI 参数和控制逻辑（启发式偏航扫描）。仅支持偏航控制。输出功率时序图。 |
| `python examples/demo.ipynb` | Jupyter Notebook 演示 — RL 环境创建、控制循环、奖励计算与可视化 |
| `python examples/interface.ipynb` | Jupyter Notebook 教程 — 统一仿真接口的详细使用说明 |

运行示例：

```bash
# FAST.Farm 连续控制（需 FAST.Farm v5.0.0 + DISCON_WT1.dll）
python examples/example_FASTFarm.py --case DafengH1 --steps 3 --wind_speed 10

# FLORIS 在线控制（纯 Python，秒出结果）
python examples/example_floris.py --case DafengH1 --steps 5 --wind_speed 10
```

---

## 架构总览

In the virtual environment of your choice:

```
```
wfcrl/
├── config.py              # WindConfig, ControlInput, SimulationOutput
├── interface.py           # SimulatorInterface + 三种实现
│   ├── FastFarmInterface             # subprocess 每步重启
│   ├── ContinuousFastFarmInterface   # 一次启动，流场连续（推荐）
│   └── FlorisInterface               # FLORIS Python API
├── simul_config.py        # SimulationConfig / FastFarmConfig / FlorisConfig
├── simul_utils.py         # create_ff_case, create_dll, create_floris_case
├── environments/
│   ├── data_cases.py      # 预定义风场布局
│   ├── registration.py    # Gymnasium / PettingZoo 注册
│   └── multiagent_env.py  # PettingZoo 多智能体环境
├── simulators/
│   ├── fastfarm/
│   │   ├── bin/            # FAST.Farm_x64_OMP.exe
│   │   ├── servo_dll/      # DISCON_WT1.dll (WFCRL Bridge)
│   │   ├── inputs/         # FAST.Farm 模板输入文件
│   │   └── src/            # DISCON_bridge.f90 Fortran 源码
│   └── floris/
│       └── inputs/         # FLORIS 模板 / 风机库
├── rewards.py             # 奖励函数
├── mdp.py                 # MDP 定义
├── wrappers.py            # Gymnasium wrappers
└── jupyter_utils.py       # Jupyter kernel 工具
```

WFCRL supports **FAST.Farm v5.0.0**.

1. **下载 FAST.Farm** — 从 [OpenFAST v5.0.0 Release](https://github.com/OpenFAST/openfast/releases/tag/v5.0.0) 获取 `FAST.Farm_x64_OMP.exe`，放入 `wfcrl/simulators/fastfarm/bin/`

2. **编译 DISCON Bridge DLL** — 使用 TDM-GCC 编译 WFCRL 自定义桥接控制器：
   ```bash
   cd wfcrl/simulators/fastfarm/src
   gfortran -shared -static -o ..\servo_dll\DISCON_WT1.dll DISCON_bridge.f90
   ```

3. **验证安装**：
   ```bash
   python examples/example_FASTFarm.py --case DafengH1 --steps 2
   ```

---

## 接口使用

### 1. FAST.Farm 连续接口（推荐 — 流场持续演化）

```python
from wfcrl.config import WindConfig, ControlInput
from wfcrl.interface import ContinuousFastFarmInterface
from wfcrl.simul_config import FastFarmConfig

config = FastFarmConfig(
    case_name="DafengH1", num_turbines=24,
    xcoords=[...], ycoords=[...],
    dt=3.0, max_iter=10,
    wind=WindConfig(speed=10, direction=270),
)

ff = ContinuousFastFarmInterface(config)
ff.setup()              # 生成输入文件 + 部署 DISCON bridge
ff.reset(wind)
ff.start()              # 后台启动 FAST.Farm（异步）

for step in range(10):
    controls = ControlInput.scalar(24, yaw_deg=5, pitch_deg=0)
    output = ff.wait_step(controls)   # 阻塞直到 DISCON 返回测量
    print(f"Farm power: {output.farm_power_mw[-1]:.2f} MW")

final_output = ff.stop()   # 等待 FAST.Farm 完成 → 解析 .outb
ff.close()
final_output.to_csv("results.csv")
```

### 2. FAST.Farm 每步重启接口

```python
from wfcrl.interface import FastFarmInterface

ff = FastFarmInterface(config)
ff.setup()
ff.reset(wind)

controls = ControlInput.scalar(24, yaw_deg=5, pitch_deg=0)
output = ff.run([controls] * 10)
ff.close()
```

### 3. FLORIS 接口

```python
from wfcrl.interface import FlorisInterface
from wfcrl.simul_config import FlorisConfig

config = FlorisConfig(
    case_name="DafengH1", num_turbines=24,
    xcoords=[...], ycoords=[...],
    dt=60.0, max_iter=10,
    wind=WindConfig(speed=10, direction=270),
)

fl = FlorisInterface(config)
fl.setup()
fl.reset(wind)

for step in range(10):
    output = fl.step(ControlInput.scalar(24, yaw_deg=5))
    print(f"Farm power: {output.farm_power_mw[-1]:.2f} MW")
fl.close()
```

### 4. 统一输出结构 (SimulationOutput)

所有接口返回 `SimulationOutput` 对象：

| 属性 | 形状 | 说明 |
|------|------|------|
| `time` | (n_steps,) | 时间向量 (s) |
| `power_mw` | (n_steps, n_turbs) | 每台风机功率 (MW) |
| `farm_power_mw` | (n_steps,) | 风场总功率 (MW) |
| `yaw_deg` | (n_steps, n_turbs) | 偏航角 (度) |
| `pitch_deg` | (n_steps, n_turbs) | 变桨角 (度) |
| `wind_speed` | (n_steps, n_turbs) | 风速 (m/s) |
| `torque_nm` | (n_steps, n_turbs) | 发电机转矩 (Nm) |
| `rotor_speed_rpm` | (n_steps, n_turbs) | 风轮转速 (RPM) |
| `blade_loads` | (n_steps, n_turbs, 3) | 叶根弯矩 |

```python
output.to_csv("results.csv")
df = output.to_dataframe()
```

---

## 运行 Jupyter Notebook

```bash
pip install notebook seaborn
python -c "from wfcrl import jupyter_utils; jupyter_utils.create_ipykernel()"
jupyter notebook examples/demo.ipynb
```

---

## 相关文档

- `docs/INTERFACE.md` — 统一仿真接口详细文档
- `docs/BENCHMARK.md` — 基准测试说明
- `DEVELOPMENT_LOG.md` — 开发记录
- `wfcrl/simulators/fastfarm/src/DISCON_bridge.f90` — DISCON Fortran 桥接源码
