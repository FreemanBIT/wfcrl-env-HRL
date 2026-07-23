# 统一实验配置与结果记录 v1

实验文件集中描述仿真后端、风场布局、风况、控制器、功率参考、执行器约束和结果目录。平台仅允许内置组件名称，不会从 YAML 动态执行 Python 代码。

## 运行

安装项目后：

```powershell
wfcrl-experiment examples/experiments/mock_fixed_yaw.yaml
```

也可以直接使用模块入口：

```powershell
python -m wfcrl.experiments.cli examples/experiments/mock_fixed_yaw.yaml
```

命令完成后会打印本次实验目录。

## YAML 结构

```yaml
name: example
simulator:
  backend: mock       # mock | floris | fastfarm
  dt: 2.0
  options: {}         # 对应后端配置的可选覆盖

farm:
  turbine_type: nrel_5MW
  xcoords: [0.0, 630.0]
  ycoords: [0.0, 0.0]

wind:
  speed: 8.0
  direction: 270.0

controller:
  type: fixed_yaw     # greedy | fixed_yaw | fixed_derating
  yaw_misalignment_deg: [10.0, 0.0]

constraints:
  yaw_bounds_deg: [-25.0, 25.0]
  yaw_rate_deg_s: 0.3
  yaw_deadband_deg: 0.5

steps: 100
output:
  root: ../../__simul__/experiments
  plot: true
```

降额实验可以增加 `power_reference`。支持 `fixed` 和 `power_curve`，功率曲线还可以使用 `smoothing_alpha` 包装指数滤波。

## 实验目录

每次运行产生：

- `resolved_config.yaml`：本次实际采用的完整配置；
- `timeseries.csv`：仿真测量时间序列；
- `controls.csv`：原始意图、约束后意图、后端 mode/指令和功率参考；
- `metadata.json`：后端、运行时间、规模、平均功率和约束事件统计；
- `overview.png`：风场总功率和各风机偏航指令的时间序列图。

指定 `output.directory` 时目录必须尚不存在，以防覆盖历史实验；使用 `output.root` 时平台自动添加实验名称和时间戳。

## FAST.Farm

FAST.Farm 默认使用连续接口。严格握手实验应配置：

```yaml
simulator:
  backend: fastfarm
  dt: 2.0
  options:
    enable_strict_handshake: true
    handshake_timeout: 120.0
```

运行前应确认 FAST.Farm 可执行文件和配套 DISCON DLL 已安装。
