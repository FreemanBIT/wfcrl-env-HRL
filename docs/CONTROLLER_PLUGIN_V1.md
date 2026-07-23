# 控制器插件接口 v1

该接口让偏航、降额和轴向诱导控制器只表达物理意图，不再直接处理 FLORIS 与 FAST.Farm 不同的 mode 编号和功率单位。

## 核心数据流

```text
ControllerObservation → WindFarmController.compute()
                      → ControlIntent（物理语义）
                      → ControlIntentAdapter（后端语义）
                      → ControlInput → 仿真器
```

`ControlIntent` 将三种降额表达明确分开：

- `power_setpoint_mw`：绝对目标功率，单位 MW；
- `derating_ratio`：相对未降额参考功率的比例，范围 `(0, 1]`；
- `axial_induction`：轴向诱导因子，范围 `[0, 1/3]`。

三者互斥。轴向诱导因子通过一维致动盘关系转换为功率比例：`ratio = 4a(1-a)^2 / (16/27)`。

## 编写控制器

```python
import numpy as np
from wfcrl.controllers import WindFarmController, ControlIntent

class MyController(WindFarmController):
    def compute(self, observation):
        yaw = np.zeros(self.context.n_turbines)
        if observation is not None and observation.farm_power_mw < 5.0:
            yaw[0] = 10.0
        return ControlIntent(yaw_misalignment_deg=yaw)
```

首步 `observation` 为 `None`，之后包含上一控制步的测量结果。`reset(context)` 的默认实现提供风机数、控制步长和仿真器能力声明。

## 统一运行入口

```python
from wfcrl.controllers import ControllerRunner

result = ControllerRunner(simulator, MyController()).run(
    100, wind=config.wind, setup=True
)
result.output.to_csv("closed_loop.csv")
```

跨 MW 与比例语义转换时，必须显式提供每台风机的未降额参考功率：

```python
result = runner.run(100, power_reference_mw=[5.0, 5.0, 5.0])
```

接口不会默认使用上一时刻实测功率，因为连续降额会造成参考值逐步降低和指令复合衰减。实验应使用同风况贪婪基线、功率曲线或在线可用功率估计器。

## 动态可用功率估计

平台提供三种可组合估计器：

- `FixedPowerReference`：固定额定功率或已标定基线；
- `PowerCurveReference`：按照每台风机局部风速插值功率曲线；
- `ExponentialSmoothingReference`：包装任意估计器进行低通滤波，降低湍流导致的指令抖动。

```python
from wfcrl.controllers import (
    ControllerRunner,
    ExponentialSmoothingReference,
    PowerCurveReference,
)

curve = PowerCurveReference(
    wind_speed_ms=[0, 3, 5, 8, 11.4, 25, 30],
    power_mw=[0, 0, 0.35, 1.75, 5.0, 5.0, 0],
    availability_factor=[1.0, 0.92, 0.85],
)
reference = ExponentialSmoothingReference(curve, alpha=0.2)

result = ControllerRunner(simulator, controller).run(
    100,
    wind=config.wind,
    setup=True,
    power_reference_estimator=reference,
)
```

首步使用 `WindConfig.speed`，后续步骤使用上一控制步测得的每台风机局部风速。`availability_factor` 可表达离线标定的尾流效率或可用率，但不会把已经降额的实测功率反馈为下一步参考值。每一步实际使用的参考值保存在 `result.power_references_mw` 中，便于实验复现和审计。

## 后端映射

| 控制意图 | FLORIS | FAST.Farm / Mock |
|---|---|---|
| 偏航 | mode 0，deg | mode 0，deg |
| 降额比例 | mode 1；与偏航组合为 mode 2 | 乘参考功率转 MW；mode 1/4 |
| 绝对功率 MW | 除以参考功率转比例；mode 1/2 | mode 1；与偏航组合为 mode 4 |
| 轴向诱导因子 | 转比例；mode 1/2 | 转比例后乘参考功率；mode 1/4 |
| 绝对变桨 | 不支持并明确报错 | mode 2；与偏航组合为 mode 3 |

适配失败会抛出 `ControlAdaptationError`，不会静默猜测单位或采用隐式参考功率。

## 执行器约束层

`ControlIntentConstraints` 在后端适配之前对物理控制意图施加约束，因此相同的偏航、变桨和降额约束可以跨 FLORIS 与 FAST.Farm 使用。

```python
from wfcrl.controllers import (
    ControlConstraintConfig,
    ControlIntentConstraints,
    ControllerRunner,
)

constraints = ControlIntentConstraints(ControlConstraintConfig(
    yaw_bounds_deg=(-25.0, 25.0),
    yaw_rate_deg_s=0.3,
    yaw_deadband_deg=0.5,
    pitch_bounds_deg=(0.0, 30.0),
    pitch_rate_deg_s=8.0,
    power_rate_mw_s=0.2,
    derating_ratio_rate_s=0.02,
))

runner = ControllerRunner(simulator, controller, constraints)
result = runner.run(100, wind=config.wind, setup=True)
```

支持的约束包括：

- 偏航角上下限、偏航速率和死区；
- 绝对变桨角上下限、变桨速率和死区；
- 绝对功率上下限和 MW/s 变化率；
- 降额比例上下限和每秒变化率；
- 轴向诱导因子上下限和每秒变化率；
- 最小变桨约束的物理范围裁剪。

约束状态会在每次 `run()` 开始时复位。原始控制器输出保存在 `result.intents`，实际送入适配器的意图保存在 `result.constrained_intents`，逐步裁剪掩码与原因保存在 `result.constraint_results`。这样可以区分算法行为和执行器实际行为。
