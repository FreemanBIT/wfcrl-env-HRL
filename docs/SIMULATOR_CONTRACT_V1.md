# 统一同步仿真接口 v1

## 目标

该契约把控制算法与具体仿真器解耦。控制器只依赖 `ControlInput`、
`SimulationOutput` 和 `SimulatorInterface.step()`，不需要导入 FLORIS、
FAST.Farm 或强化学习框架。

v1 的核心验收条件是：同一个偏航控制器不修改代码即可运行在确定性 Mock
和 FLORIS 上，并获得一致的数组形状、单位、时间戳和契约元数据。

## `step()` 语义

对声明 `strict_step=True` 的后端：

1. 一次 `step(control)` 恰好推进一个控制周期 `dt`；
2. 返回一个采样点，`time.shape == (1,)`；
3. 第一次调用的时间为 `dt`，之后依次为 `2*dt, 3*dt, ...`；
4. `power_mw` 及所有风机级二维通道的形状为 `(1, n_turbines)`；
5. 输入形状、有限值、控制模式和输出形状在公共入口自动校验；
6. `reset()` 后时间重新从零开始，并可重复得到确定性结果（若后端本身确定）。

`SimulationOutput.metadata` 会补充：

- `contract_version`：当前为 `1.0`；
- `simulator_id`：后端标识；
- `time_model`：时间/尾流演化模型；
- `step_synchronization`：同步机制；
- `strict_step`：是否满足严格单周期语义。

## 单位与坐标约定

| 字段 | 语义 | 单位/形状 |
|---|---|---|
| `ControlInput.yaw` | 相对局部来流的偏航失准角，不是绝对机舱航向 | deg，`(n_turbines,)` |
| `ControlInput.pitch` | 集体变桨绝对角 | deg，`(n_turbines,)` |
| `ControlInput.power` | 由控制模式与能力声明确定；FAST.Farm 为 MW，FLORIS 旧模式为比例 | `(n_turbines,)` |
| `SimulationOutput.time` | 控制周期结束时刻 | s，`(n_samples,)` |
| `SimulationOutput.power_mw` | 单机有功功率 | MW，`(n_samples, n_turbines)` |
| `SimulationOutput.yaw_deg` | 实际偏航失准角 | deg，`(n_samples, n_turbines)` |

后端负责把统一的偏航失准角转换成求解器自己的绝对机舱坐标。

## 后端能力边界

| 后端 | 时间模型 | 同步方式 | `strict_step` | v1 定位 |
|---|---|---:|---:|---|
| `DeterministicMockSimulator` | 确定性测试模型 | 严格 | 是 | 控制器集成与快速回归，不代表尾流物理 |
| `FlorisInterface` | 连续稳态解序列 | 严格 | 是 | 快速工程尾流计算，不包含尾流输运延迟 |
| `FastFarmInterface` | 每步重启流场 | 子进程阻塞 | 否 | 可阻塞等待，但步间尾流物理不连续 |
| `ContinuousFastFarmInterface`（默认） | 连续动态流场 | 文本文件轮询 | 否 | 向后兼容模式，控制器必须快于控制周期 |
| `ContinuousFastFarmInterface`（协议 v2） | 连续动态流场 | 全风机文件握手 | 是 | 显式启用；已通过 2/6 风机及 100 步压力测试 |

能力可通过 `simulator.capabilities` 查询，包括支持的控制模式、控制通道、
测量通道、时间模型和同步机制。调用不支持的模式会在进入求解器前失败。

特别注意：当前 FLORIS 保留项目原有的三模式约定；其中 mode 1/2 的
`ControlInput.power` 表示降额比例，而不是 MW。能力声明明确暴露了这一差异，
避免控制器误把 FAST.Farm 的 mode 2（变桨）直接用于 FLORIS。当前 FLORIS
整场只能使用一个共同模式，混合的逐风机模式会在公共入口被拒绝。

## FAST.Farm 严格同步的后续工作

连续 FAST.Farm 的默认模式仍使用 `controls.txt` 与 `measurements_T*.txt` 轮询。
协议 v2 已增加全风机边界阻塞、`APPLIED/READY` 状态、严格连续步号和完整记录标志，
可通过 `enable_strict_handshake=True` 显式启用。

协议实现、模拟测试和官方 FAST.Farm v5.0.0 OpenMP 真实测试均已完成，包括 2 台
风机 100 步、6 台风机屏障，以及慢控制器延迟注入。详见
[FASTFARM_STRICT_HANDSHAKE_V1.md](FASTFARM_STRICT_HANDSHAKE_V1.md)。

## 最小使用示例

```python
from wfcrl.config import ControlInput
from wfcrl.engine import DeterministicMockSimulator

sim = DeterministicMockSimulator(config)
sim.setup()
sim.reset(config.wind)

command = ControlInput.mode0_yaw(sim.n_turbines, yaw_deg=10.0)
output = sim.step(command)
print(output.time, output.farm_power_mw, output.metadata)

sim.close()
```
