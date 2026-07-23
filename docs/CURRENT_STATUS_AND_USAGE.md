# WFCRL 动态仿真实验平台：当前更新与使用说明

> 文档版本：v1  
> 整理日期：2026-07-22  
> 项目定位：面向风电场尾流控制算法研究的统一动态仿真与实验平台

## 1. 项目目标

本项目基于原始 `wfcrl-env-HRL` 代码继续开发，目标是形成一套适合研究人员使用的风电场尾流控制实验平台，主要解决以下问题：

- 不同仿真器接口、控制模式和功率单位不统一；
- 控制算法与 FLORIS、FAST.Farm 等具体后端耦合；
- FAST.Farm 难以进行可靠的逐步动态闭环控制；
- 偏航、降额和轴向诱导控制缺少统一物理语义；
- 实验配置、输出、复现和对比分析流程分散；
- 控制器原始指令与执行器实际指令难以区分；
- 缺少快速、确定性的测试后端。

当前平台已经具备以下基本流程：

```text
YAML 实验配置
    ↓
控制器插件
    ↓
物理控制意图 ControlIntent
    ↓
执行器约束
    ↓
后端控制适配
    ↓
Mock / FLORIS / FAST.Farm
    ↓
统一 SimulationOutput
    ↓
CSV、JSON、配置快照和图表
    ↓
多实验对比分析
```

## 2. 当前已经完成的功能

### 2.1 统一仿真器接口

Mock、FLORIS 和 FAST.Farm 遵循统一调用流程：

```python
simulator.setup()
simulator.reset(wind)
output = simulator.step(control)
simulator.close()
```

统一输出 `SimulationOutput` 可以包含：

- 仿真时间；
- 各风机功率；
- 风场总功率；
- 局部风速和风向；
- 偏航角和变桨角；
- 转矩和转速；
- 推力及叶片载荷（后端支持时）；
- 控制指令和实验元数据。

各仿真后端通过 `SimulatorCapabilities` 声明：

- 仿真精度类别；
- 时间模型；
- 单步同步机制；
- 支持的控制模式；
- 支持的测量通道；
- 是否支持流场输出；
- 是否满足严格单步协议。

详细说明见：[SIMULATOR_CONTRACT_V1.md](SIMULATOR_CONTRACT_V1.md)。

### 2.2 确定性 Mock 仿真器

新增 `DeterministicMockSimulator`，用于：

- 控制算法接口调试；
- 闭环流程验证；
- YAML 实验配置验证；
- 执行器约束验证；
- 自动化单元测试；
- 在运行 FLORIS 或 FAST.Farm 前快速发现软件错误。

Mock 后端不是尾流物理模型，不能用于发表物理结论，但运行速度快、结果可重复。

### 2.3 FAST.Farm 严格动态握手

实现了 Python 与 FAST.Farm/DISCON 之间的全风机逐步握手协议：

```text
Python 写入第 k 步控制
→ 全部风机读取该步控制
→ FAST.Farm 推进到目标时刻
→ 全部风机写回该步测量
→ Python 收集测量并返回
```

已经完成的真实仿真验证包括：

- 2 台风机、3 个控制步；
- 2 台风机、100 个控制步；
- 6 台风机、10 个控制步；
- 加入控制器延迟后仍保持严格仿真时间同步。

详细说明见：[FASTFARM_STRICT_HANDSHAKE_V1.md](FASTFARM_STRICT_HANDSHAKE_V1.md)。

### 2.4 统一控制器插件接口

控制器不再直接构造含义随后端变化的 `ControlInput`，而是输出后端无关的 `ControlIntent`。

支持的物理控制通道包括：

| 字段 | 物理含义 | 单位或范围 |
|---|---|---|
| `yaw_misalignment_deg` | 相对局部来流的偏航失准角 | deg |
| `pitch_deg` | 绝对集体变桨角 | deg |
| `power_setpoint_mw` | 绝对功率目标 | MW |
| `derating_ratio` | 相对未降额参考功率的比例 | `(0, 1]` |
| `axial_induction` | 轴向诱导因子 | `[0, 1/3]` |
| `min_pitch_deg` | 最小变桨约束 | deg |

`power_setpoint_mw`、`derating_ratio` 和 `axial_induction` 三种降额表达互斥，避免单位和物理含义混用。

平台自动将物理控制意图映射到不同后端：

| 控制意图 | FLORIS | FAST.Farm / Mock |
|---|---|---|
| 偏航 | mode 0 | mode 0 |
| 降额比例 | mode 1；与偏航组合为 mode 2 | 转为 MW，使用 mode 1/4 |
| 绝对功率 MW | 除以参考功率转成比例 | 直接使用 MW，mode 1/4 |
| 轴向诱导因子 | 先转成功率比例 | 转成功率比例后再转 MW |
| 绝对变桨 | 不支持时明确报错 | mode 2；与偏航组合为 mode 3 |

详细说明见：[CONTROLLER_PLUGIN_V1.md](CONTROLLER_PLUGIN_V1.md)。

### 2.5 动态可用功率估计

支持三种功率参考估计器：

- `FixedPowerReference`：固定额定功率或固定基线；
- `PowerCurveReference`：按照每台风机局部风速插值功率曲线；
- `ExponentialSmoothingReference`：对其他估计器结果进行指数低通滤波。

功率参考用于：

- 将降额比例转换为 FAST.Farm 所需的 MW；
- 将绝对功率目标转换为 FLORIS 所需的比例；
- 将轴向诱导因子转换为后端可以执行的功率控制量。

平台不会默认使用已经降额的上一时刻实测功率作为参考，以避免参考功率不断降低形成复合衰减。

### 2.6 统一执行器约束

新增后端无关的 `ControlIntentConstraints`，支持：

- 偏航角上下限；
- 偏航速率限制；
- 偏航死区；
- 变桨角上下限；
- 变桨速率和死区；
- 绝对功率上下限；
- 功率变化率；
- 降额比例上下限和变化率；
- 轴向诱导因子上下限和变化率；
- 最小变桨角范围裁剪。

闭环运行结果会区分：

- `result.intents`：控制器原始请求；
- `result.constrained_intents`：执行器约束后的物理意图；
- `result.controls`：最终送入仿真后端的指令；
- `result.constraint_results`：裁剪通道、风机掩码和裁剪原因。

### 2.7 YAML 配置驱动实验

可以通过一个 YAML 文件定义：

- 仿真后端；
- 仿真时间步长；
- 风机数量、类型和布局；
- 风速、风向和湍流强度；
- 控制器；
- 可用功率估计器；
- 执行器约束；
- 仿真步数；
- 输出目录。

每次实验自动生成：

```text
experiment-directory/
├── resolved_config.yaml
├── timeseries.csv
├── controls.csv
├── metadata.json
└── overview.png
```

详细说明见：[EXPERIMENT_CONFIG_V1.md](EXPERIMENT_CONFIG_V1.md)。

### 2.8 多实验对比分析

对比工具可以计算：

- 总发电量 MWh；
- 时间加权平均风场功率；
- 相对基线的能量增益；
- 相对基线的平均功率增益；
- 偏航总活动量；
- 变桨总活动量；
- 指令降额能量；
- 约束触发步数；
- 仿真墙钟耗时。

自动生成：

```text
comparison-directory/
├── comparison.csv
├── comparison.json
├── metrics.png
└── farm_power.png
```

详细说明见：[EXPERIMENT_COMPARISON_V1.md](EXPERIMENT_COMPARISON_V1.md)。

## 3. 安装与环境准备

### 3.1 进入项目目录

```powershell
cd "E:\风电场尾流控制研究\仿真平台\面向风电场控制算法研究的动态仿真实验平台codex"
```

### 3.2 以开发模式安装

```powershell
python -m pip install -e .
```

开发模式安装后，修改 Python 源代码通常不需要重复安装。但修改 `pyproject.toml` 中的命令行入口后，建议重新执行该命令。

### 3.3 避免加载电脑上的旧版 WFCRL

如果 Python 错误加载了其他目录中的旧版项目，可以在当前 PowerShell 会话执行：

```powershell
$env:PYTHONPATH = (Get-Location).Path
```

检查实际加载路径：

```powershell
python -c "import wfcrl; print(wfcrl.__file__)"
```

输出路径应位于当前项目目录。

## 4. 最快上手流程

### 4.1 运行 Mock 偏航实验

```powershell
python -m wfcrl.experiments.cli `
  examples/experiments/mock_fixed_yaw.yaml
```

安装项目后，也可以执行：

```powershell
wfcrl-experiment examples/experiments/mock_fixed_yaw.yaml
```

命令结束后会打印本次实验目录。

示例配置见：[mock_fixed_yaw.yaml](../examples/experiments/mock_fixed_yaw.yaml)。

### 4.2 运行动态降额实验

```powershell
python -m wfcrl.experiments.cli `
  examples/experiments/mock_derating.yaml
```

该示例同时演示：

- 偏航与降额组合控制；
- 风速—功率曲线；
- 功率参考指数滤波；
- 偏航速率约束；
- 降额比例变化率约束。

示例配置见：[mock_derating.yaml](../examples/experiments/mock_derating.yaml)。

### 4.3 查看实验结果

实验目录中的主要文件用途如下：

| 文件 | 内容 |
|---|---|
| `overview.png` | 风场总功率和各风机偏航指令曲线 |
| `timeseries.csv` | 功率、风速、风向、偏航和载荷等测量时间序列 |
| `controls.csv` | 原始意图、约束后意图、后端指令和功率参考 |
| `metadata.json` | 仿真器、规模、耗时、平均功率和约束事件统计 |
| `resolved_config.yaml` | 本次实验实际采用的完整配置快照 |

## 5. YAML 实验配置示例

### 5.1 偏航控制

```yaml
name: yaw-test

simulator:
  backend: mock
  dt: 2.0

farm:
  turbine_type: nrel_5MW
  xcoords: [0.0, 630.0, 1260.0]
  ycoords: [0.0, 0.0, 0.0]

wind:
  speed: 8.0
  direction: 270.0
  turbulence_intensity: 0.08

controller:
  type: fixed_yaw
  yaw_misalignment_deg: [10.0, 0.0, 0.0]

constraints:
  yaw_bounds_deg: [-25.0, 25.0]
  yaw_rate_deg_s: 0.3
  yaw_deadband_deg: 0.5

steps: 100

output:
  root: ../../__simul__/experiments
  plot: true
```

### 5.2 降额控制

```yaml
name: derating-test

simulator:
  backend: mock
  dt: 2.0

farm:
  turbine_type: nrel_5MW
  xcoords: [0.0, 630.0, 1260.0]
  ycoords: [0.0, 0.0, 0.0]

wind:
  speed: 8.0
  direction: 270.0

controller:
  type: fixed_derating
  derating_ratio: [0.8, 1.0, 1.0]
  yaw_misalignment_deg: [5.0, 0.0, 0.0]

power_reference:
  type: power_curve
  wind_speed_ms: [0.0, 3.0, 5.0, 8.0, 11.4, 25.0, 30.0]
  power_mw: [0.0, 0.0, 0.35, 1.75, 5.0, 5.0, 0.0]
  availability_factor: [1.0, 0.94, 0.88]
  smoothing_alpha: 0.2

constraints:
  yaw_rate_deg_s: 0.3
  derating_ratio_rate_s: 0.02

steps: 100

output:
  root: ../../__simul__/experiments
  plot: true
```

## 6. 编写自己的控制器

所有控制器继承 `WindFarmController`：

```python
import numpy as np

from wfcrl.controllers import ControlIntent, WindFarmController


class MyWakeController(WindFarmController):
    def reset(self, context):
        super().reset(context)
        self.step_index = 0

    def compute(self, observation):
        yaw = np.zeros(self.context.n_turbines)

        if observation is not None:
            if observation.farm_power_mw < 5.0:
                yaw[0] = 10.0

        self.step_index += 1
        return ControlIntent(yaw_misalignment_deg=yaw)
```

首个控制步的 `observation` 为 `None`。后续步骤可以读取：

```python
observation.time_s
observation.power_mw
observation.farm_power_mw
observation.wind_speed
observation.wind_direction_deg
observation.yaw_misalignment_deg
observation.pitch_deg
```

通过统一运行器执行：

```python
from wfcrl.controllers import ControllerRunner

runner = ControllerRunner(
    simulator,
    MyWakeController(),
    intent_constraints=constraints,
)

result = runner.run(
    n_steps=100,
    wind=config.wind,
    setup=True,
    power_reference_estimator=reference,
)
```

目前 YAML 工厂支持三个内置控制器：

- `greedy`；
- `fixed_yaw`；
- `fixed_derating`。

自定义 Python 控制器已经可以通过 `ControllerRunner` 使用，但尚未加入 YAML 控制器插件注册表。

## 7. 切换到 FLORIS

在 YAML 中修改：

```yaml
simulator:
  backend: floris
  dt: 30.0
```

其他控制器配置通常可以保持不变，由适配器自动转换。

注意事项：

- FLORIS 是稳态序列，不模拟真实尾流传播延迟；
- FLORIS 不支持绝对变桨控制；
- FLORIS 原生降额控制使用比例；
- 当前降额实现会调整功率/推力曲线，适合算法和接口验证；
- 大规模优化前仍需要继续改善 FLORIS 降额运行效率。

## 8. 切换到 FAST.Farm

在 YAML 中配置：

```yaml
simulator:
  backend: fastfarm
  dt: 2.0
  options:
    enable_strict_handshake: true
    handshake_timeout: 120.0
```

运行前应确认：

- FAST.Farm 可执行文件存在；
- `DISCON_WT1.dll` 已使用当前协议源码编译；
- FAST.Farm 模板和风机文件完整；
- 实验输出目录具有写权限。

建议先进行短实验：

```yaml
steps: 3
```

确认握手、测量和控制指令均正常后，再增加到 100 步或更长时间。

## 9. 对比多个实验

假设已有两个实验目录：

```text
__simul__/experiments/greedy-...
__simul__/experiments/controlled-...
```

执行：

```powershell
python -m wfcrl.analysis.cli `
  "__simul__/experiments/greedy-..." `
  "__simul__/experiments/controlled-..." `
  --baseline greedy `
  --output "__simul__/comparisons/greedy-vs-controlled"
```

安装命令行入口后也可以执行：

```powershell
wfcrl-compare `
  "__simul__/experiments/greedy-..." `
  "__simul__/experiments/controlled-..." `
  --baseline greedy `
  --output "__simul__/comparisons/greedy-vs-controlled"
```

`--baseline` 使用 YAML 中的实验 `name`，不是实验目录名。

默认要求实验具有相同的：

- 风机数量和布局；
- 风速和风向；
- 仿真时间步长；
- 仿真步数。

如果仅进行探索性分析，可以使用 `--allow-incompatible`，但不同风况或持续时间下的相对增益通常没有严格物理可比性。

## 10. 测试与验证

运行完整测试：

```powershell
$env:PYTHONPATH = (Get-Location).Path
python -m pytest -q
```

当前回归结果：

```text
93 passed, 1 skipped
```

跳过项是需要显式启用的真实 FAST.Farm 集成测试，并非普通功能测试失败。

真实 FAST.Farm 测试需要满足外部可执行文件、DLL 和模板等运行条件。

## 11. 主要代码目录

```text
wfcrl/
├── config/          # 仿真、风况、控制输入和输出数据结构
├── controllers/     # 控制器契约、适配器、约束和功率参考
├── engine/          # Mock、FLORIS、FAST.Farm 后端
├── experiments/     # YAML 配置、组件工厂、实验执行和归档
├── analysis/        # 多实验指标与对比报告
└── simulators/      # FAST.Farm 模板、控制器源码和 DLL
```

重点入口：

- `wfcrl.controllers.ControllerRunner`：Python 闭环实验入口；
- `wfcrl.experiments.run_experiment`：配置驱动实验入口；
- `wfcrl.analysis.write_comparison_report`：实验对比入口；
- `python -m wfcrl.experiments.cli`：实验命令行；
- `python -m wfcrl.analysis.cli`：对比命令行。

## 12. 当前限制

当前版本仍有以下限制：

1. YAML 中只支持三个内置参考控制器，自定义控制器插件注册尚未实现；
2. FLORIS 是稳态序列，不能代替 FAST.Farm 的动态尾流传播；
3. FLORIS 降额时需要调整风机功率/推力表，批量优化性能仍可提高；
4. 功率曲线估计器需要研究者提供与风机型号匹配的功率曲线；
5. `availability_factor` 是标定输入，不是在线尾流状态估计器；
6. 指令降额能量是控制命令层指标，不等同于实际尾流耦合发电损失；
7. 当前还没有批量参数扫描、并行实验和断点续跑；
8. 当前还没有交互式桌面或 Web 可视化界面；
9. 当前工作区修改尚未整理成正式 Git 提交。

## 13. 推荐的实际使用顺序

1. 运行 `mock_fixed_yaw.yaml`，确认环境和实验输出正常；
2. 运行 `mock_derating.yaml`，理解功率参考和约束记录；
3. 复制示例 YAML，建立自己的风场布局和风况；
4. 使用 Mock 调通自己的控制器接口；
5. 切换到 FLORIS 做稳态尾流控制快速研究；
6. 使用短时 FAST.Farm 严格握手实验验证动态闭环；
7. 增加 FAST.Farm 步数，开展正式动态控制实验；
8. 使用 `wfcrl-compare` 与贪婪基线进行统一对比；
9. 保存 YAML、CSV、JSON、图表和代码版本，保证研究可复现。

## 14. 建议的下一步开发

下一阶段建议依次实施：

1. 整理 Git 变更并创建阶段性版本；
2. 建立自定义控制器插件注册机制；
3. 实现批量参数扫描与实验矩阵；
4. 支持并行执行、失败重试和断点续跑；
5. 增加随机种子和湍流工况重复实验；
6. 建立交互式实验管理和可视化界面；
7. 加入更完整的载荷、疲劳和控制代价指标。

