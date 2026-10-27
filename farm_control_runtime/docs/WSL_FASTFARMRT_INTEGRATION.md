# 基于 fastfarmRT 的 WSL 交付评估与接入指南

> **前提更新（2026-10-22）**：下一阶段开发人员的 WSL 环境**具备 fastfarmRT**，
> 可在 CPU / GPU 上部署运行 FAST.Farm 各模块（OF / WD / AWAE）。
> **本文替代** `WSL_READINESS.md` §0 中"WSL 不能跑 FAST.Farm 闭环"的结论。
> 配套：`WSL_DEPLOY_AND_TEST_GUIDE.md`（部署与测试步骤）、`RT_DATA_AND_TIME_POLICY.md`（策略）

---

## 0. 结论：可以进行 WSL 测试，且可从"模块级"升级到"fastfarmRT 闭环级"

| 测试层级 | 之前评估 | **基于 fastfarmRT 的重新评估** |
|---|---|---|
| 场控模块单元/RT 测试 | ✅ | ✅ 不变（Windows 99/99，Linux 92/92） |
| 调度器时序（Mock） | ✅ | ✅ 不变 |
| **FAST.Farm 闭环（6T 案例）** | ❌ 不可 | ✅ **可做**：由 fastfarmRT 提供 OF/WD/AWAE，场控模块挂在调度时序上 |
| 硬件在环 / RT 指标 | ❌ | ❌ 仍属采购后 M4 |

**判断**：交付内容的主体（C11 核心 + RT 接入层 + Python 协议镜像 + 策略文档）**可直接用于 fastfarmRT 集成**；
但要让 fastfarmRT **真正闭环**，还需解决第 3 节的 1 项代码缺口（**已在本版补齐**）与 4 项平台约定。

---

## 1. fastfarmRT 与本模块的分工

| 角色 | fastfarmRT | 场控交互模块（本交付） |
|---|---|---|
| 物理模型（OF/WD/AWAE，CPU/GPU） | ✅ 负责 | ❌ 不承担 |
| 时间推进 / 调度 | ✅ 唯一时间源 | 只读 `RTimeCtx`，不自推时间 |
| 场级控制接口（命令/状态/桥接/降级） | — | ✅ 负责 |
| 与 RCP 的通信 | **负责**（按已确认约定） | 只提供接口数据（无 socket） |
| 单机控制律（ROSCO） | 由平台决定（模块或库） | 只产出 S08 setpoint |

**插入位置**：fastfarmRT 主循环中，在 **AWAE 更新之后**（低速/主控阶段）调用场控模块，
每 `DT_SCADA`（默认 1 s）做一次对外交换；若平台按 10 ms 调度，则 `fcr_rt_step` 每 10 ms 调用、
SCADA 交换由模块内部计数器在 1 s 边界触发（与调度周期解耦，二者皆可）。

---

## 2. 接口映射表（fastfarmRT hook → 本模块 API）

| fastfarmRT 侧数据/时点 | 本模块 API | 信号组 | 建议频率 |
|---|---|---|---|
| 调度步（n_low/t_low/dt_low/n_fast_low） | `fcr_rt_step(h, &FcrRtTimeCtx, &status)` | — | 10 ms 或每个 DT_Low |
| **OF 子步输出**（转速/电功率/桨距/机舱方位/推力/轴矩/塔基弯矩） | **`fcr_rt_set_openfast_state(h, &FcrOpenFASTRawState)`** | **S02** | 每 OF 子步或 10 ms |
| 单机控制输出（状态字/转矩命令/桨距命令/已采纳 seq） | `fcr_rt_set_state(h, tid, &FcrRoscoRawState)` | S03 | 10 ms |
| 额外载荷（Ct/Cp/载荷测点，可缺省） | `fcr_rt_set_extra(h, &FcrTurbineExtraState)` | S02 扩展 | 10 ms |
| **AWAE 流场**（上游/盘面风速等，按测点） | `fcr_rt_set_flow(h, &FcrFlowState)` | S05 | 1 s（ZOH） |
| AWAE 静态测点目录 | `fcr_rt_set_awae_catalog(h, &FcrAWAECatalogFrame)` | S04 | 启动/变更时 |
| 风估计（来源/质量） | `fcr_rt_set_wind_estimate(h, &FcrTurbineWindEstimate)` | S06 | 1 s |
| RCP 命令（调度器搬运） | `fcr_rt_set_command(h, &FarmCommandFrame)` | S01 | 1 s |
| 状态帧（→ RCP） | `fcr_rt_get_output(h, &FarmStateFrame)` | S10 | 1 s |
| AWAE 二跳帧 | `fcr_rt_get_awae(h, &obs, &cat)` | S12/S13 | 1 s |
| 单机控制设定（→ ROSCO/控制器） | `fcr_rt_get_setpoint(h, tid, &FcrRoscoExternalSetpoint)` | S08 | 每控制步 |

**伪代码（fastfarmRT 主循环内）**

```c
/* 每个调度步 */
FcrRtTimeCtx t = { .n_low = N, .t_low = N*DT_LOW, .dt_low = DT_LOW,
                   .n_fast_low = sub, .dt_high = DT_HIGH, .n_high_low = N_HIGH };
FcrRtStatusInfo st;

for (int i = 0; i < n_turbines; ++i)
    fcr_rt_set_openfast_state(h, &of_state[i]);       /* S02：OF 物理量测 */

/* 每 DT_SCADA（或每个 10 ms，由平台决定） */
if (scada_boundary) {
    fcr_rt_set_flow(h, &awae_flow);                   /* S05 */
    fcr_rt_set_command(h, &rcp_cmd);                  /* S01（调度器从 RCP 取回） */
}
for (int i = 0; i < n_turbines; ++i)
    fcr_rt_set_state(h, i + 1, &rosco_state[i]);       /* S03 */

FcrRtStatus s = fcr_rt_step(h, &t, &st);              /* 10 ms 入口 */

/* 每 DT_SCADA：把状态送回 RCP；每控制步：取 setpoint 喂控制器 */
if (scada_boundary) fcr_rt_get_output(h, &state_frame);      /* S10 */
fcr_rt_get_setpoint(h, 1, &sp);                              /* S08 */
```

> **调用顺序不重要**：模块对 S02/S03 采用**字段级归属**（OF 拥有物理量测，ROSCO 拥有命令/状态/seq），
> 任意顺序调用都不会互相清零（已由单测 `rt_set_openfast_state` 固化）。

---

## 3. 本版已补齐的缺口 / 仍需平台确认的项

### 3.1 已补齐（本次交付新增）

| # | 缺口 | 处理 | 验证 |
|---|---|---|---|
| C1 | **S02（OpenFAST 原始状态）无输入通路**——只定义了 `FcrOpenFASTRawState` 类型，未接入运行时 | 新增 `fcr_rt_set_openfast_state()` + `FcrStateStore.of[]` 槽 + 字段级合并；RT ABI **v1→v2**（仅新增函数，向后兼容） | `rt_set_openfast_state` 单测：仅喂 S02 即可产出 S10（含推力统计）；补 S03 后物理量不被清零 |
| C2 | S02 的推力/轴矩/塔基弯矩未进入 1 s 统计 | `set_openfast_state` 同步喂 extra 统计 | 同上（S10 中 `rotor_aero_thrust_n` 非零） |
| C3 | 统计样本取原始输入（可能被 0 覆盖） | `set_state` / `set_openfast_state` 改为喂**合并后**的值 | 单测覆盖 |

### 3.2 仍需平台确认 / 补齐

| # | 事项 | 影响 | 建议 |
|---|---|---|---|
| P1 | **AWAE 测点数**：本模块容量 `FCR_MAX_OBSERVATION_POINTS = 64` | 若 GPU AWAE 输出点数 > 64，会被截断 | 平台告知点数：≤64 直接用；>64 则在适配层**降采样/选点**（推荐），或提 ABI v3 扩容（需同步 `check_abi`/CSV） |
| P2 | **调用线程**：模块约定单线程调用（RT 路径无锁） | fastfarmRT 若 OF/WD 多线程并行，需固定一个线程调用场控 | 建议在 **AWAE 更新后的主控/低速线程**统一调用 `set_*`/`step`/`get_*` |
| P3 | **时间基**：`RTimeCtx` 各字段由 fastfarmRT 填充 | 字段语义偏差会导致 TTL/SCADA 边界错位 | 确认 `DT_Low`（1 s？3 s？）、`n_fast_low` 语义；SCADA 边界由模块内部计数器独立于 DT_Low |
| P4 | **单机控制形态**：fastfarmRT 中的 ROSCO 是模块还是库 | 决定 S08 由谁消费 | 若是 C/C++ 接口，直接 `fcr_rt_get_setpoint()`；若是 Fortran，用 `rosco/FarmControlCBindings.f90` 绑定 |
| P5 | **ROSCO 侧 Linux 构建**：本交付的 `_compile.py` 仅 Windows | 若需复用我们的 ROSCO 集成（`FCR_Step`/`FCR_ApplyReferences` 钩子） | 需补 Linux 构建脚本（gfortran `-shared -fPIC`）；若用 fastfarmRT 自带控制器管线则可省 |
| P6 | **S02 采样频率** | OF 子步与 10 ms 不对齐时，S10 的 1 s 均值含义变化 | 建议每 10 ms 喂一次（OF 子步内取最新值，ZOH） |

---

## 4. WSL 测试里程碑（基于 fastfarmRT）

| 里程碑 | 内容 | 依赖 | 预期证据 |
|---|---|---|---|
| **M1 模块** | `bash tools/build_linux.sh` → 92/92；`rt_minimal_example` 冒烟 | 无 | ctest 输出、示例输出 |
| **M2 接口** | ABI/门禁/审计三件套；S02/S03/S05/S08 映射联调（可用 Mock 数据） | P1/P3 | `check_abi.py` PASS、映射表核对 |
| **M3 调度/闭环** | 用 fastfarmRT 跑 6T 案例：验证 10 ms 步进、1 s SCADA、命令生效、TTL→D3、状态帧上送 | P1~P6 | 与 Windows 基线（T-21 4379 kW）对比、时序日志 |
| **M4 RT** | 采购后：RT 框架、Jitter/Deadline/WorstCase | 采购 | 实测报告 |

**M3 建议的第一轮验证**（最小闭环，不依赖 RCP）：
1. fastfarmRT 跑 6T（270°、11 m/s，与 Windows 基线同工况）；
2. 场控模块**不下发任何命令**（纯 baseline），验证 `get_output` 的 1 s 帧与功率量级；
3. 与 Windows 基线对比整场平均功率（预期偏差 < 1%，差异来源为 GPU/CPU 数值差异与采样对齐）；
4. 再注入偏航/诱导命令，复现 Windows 侧 T-22/T-23 的结论（诱导 T4 −15.4%、偏航 NacYaw 真执行）。

---

## 5. 交付物在 fastfarmRT 下的取用清单

| 需要 | 路径 |
|---|---|
| 头文件 | `farm_control_runtime/include/`（`fcr_rt.h` 为入口；RT ABI **v2**） |
| 静态库 | `build-linux/libfarm_control_runtime.a`（`bash tools/build_linux.sh` 生成） |
| 最小示例 | `farm_control_runtime/examples/rt_minimal_example.c` |
| RT 测试 | `farm_control_runtime/tests/unit/test_rt_adapter.c`（含 1000×10 ms Mock Scheduler） |
| 策略/时间/降级 | `03_RT_DATA_AND_TIME_POLICY.md` |
| Fortran 绑定（若用 ROSCO） | `farm_control_runtime/rosco/FarmControlCBindings.f90`、`FarmControlInterface.f90` |
| 信号字典 | `docs/farm_control/inputs/FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv` |
| **不需要** | `wfcrl/.../DISCON_WT1.dll`（Windows PE）、`transport/modbus`、`transport/zmq` |

---

## 6. 风险小结

| 风险 | 影响 | 缓解 |
|---|---|---|
| AWAE 点数 > 64 | 流场信息截断，二跳帧不完整 | P1：适配层选点/降采样，或扩容 ABI |
| 多线程调用 | 数据竞争（模块无锁） | P2：固定单线程调用点 |
| DT_Low ≠ 1 s | SCADA 边界/TTL 语义偏差 | 已实现独立计数器；需 P3 确认 |
| ROSCO 形态不同 | S08 无人消费或需新绑定 | P4：C 接口直接取用；Fortran 用现成绑定 |
| GPU/CPU 数值差异 | 与 Windows 基线功率偏差 | 首轮 baseline 对比，建立新基线 |

---

*版本：2026-10-22　对应交付包 `farm_control_runtime_delivery_20261022.zip`（RT ABI v2）*
