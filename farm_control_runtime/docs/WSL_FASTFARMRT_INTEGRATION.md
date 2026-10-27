# 基于 fastfarmRT 的 WSL 交付评估与接入指南

> **前提更新（2026-10-22）**：下一阶段开发人员的 WSL 环境**具备 fastfarmRT**，
> 可在 CPU / GPU 上部署运行 FAST.Farm 各模块（OF / WD / AWAE）。
> **本文替代** `WSL_READINESS.md` §0 中"WSL 不能跑 FAST.Farm 闭环"的结论。
> 配套：`WSL_DEPLOY_AND_TEST_GUIDE.md`（部署与测试步骤）、`RT_DATA_AND_TIME_POLICY.md`（策略）

---

## 0. 结论：可以进行 WSL 测试，且可从"模块级"升级到"fastfarmRT 闭环级"

| 测试层级 | 之前评估 | **基于 fastfarmRT 的重新评估** |
|---|---|---|
| 场控模块单元/RT 测试 | ✅ | ✅ 不变（Windows 100/100，Linux 93/93） |
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
| C4 | **调度器与 Fortran 侧是两个独立 runtime**（S08 不同源） | 新增单例注入：`fcr_rt_init` 调 `fcr_rosco_api_adopt_runtime`；RT 模式下 `fcr_rosco_step_high` no-op；`fcr_rt_destroy` 先 detach | `rt_shared_runtime_with_rosco_side` |
| C5 | AWAE 容量 64 点不足 | 容量扩到 **1000 点**（`FCR_MAX_OBSERVATION_POINTS_V2`、`FCR_CAPACITY_VERSION=2`），同步帧尺寸/Modbus 区/Python 镜像/ABI 校验 | `check_abi.py` PASS、93/93 单测、12/12 协议 |
| C6 | ROSCO 无 Linux `.a` 构建方式 | 新增 `rosco/src/`（9 个源文件）+ `tools/build_rosco_linux.sh` + `rosco/README_ROSCO_SRC.md` | 需在 WSL 首次实测 |

### 3.2 平台方已确认（2026-10-22）与本版处理

| # | 事项 | 平台方答复 | 本版处理 |
|---|---|---|---|
| P1 | AWAE 测点数 | **扩容到 1000** | ✅ `FCR_MAX_OBSERVATION_POINTS_V2 = 1000`，`FCR_MAX_OBSERVATION_POINTS` 指向 V2，`FCR_CAPACITY_VERSION = 2`；帧尺寸：Catalog **40024 B**、Observation **48024 B**、`FcrFlowState` **57568 B**；Modbus 寄存器区随容量扩展（`MB_TOTAL_REGS` ≈ 44.3k，仍 < 65536） |
| P2 | 多线程 | **先不考虑**（单线程） | 保持"单线程调用、RT 路径无锁"的约定；`FCR_LOCK` 只用于 Windows DISCON 路径与 Fortran 侧入口 |
| P3 | DT_Low | **1 s** | 与内部实现一致；SCADA 边界仍由计数器驱动（`steps_per_scada`），`DT_Low` 变化时无需改代码 |
| P4 | ROSCO 形态 | **与 OpenFAST 同一 CPU 核，以 `.a` 被调用** | ✅ 新增**运行时共享**：`fcr_rt_init()` 自动 `adopt` 全局单例，调度器与 Fortran 侧读写**同一份** setpoint；RT 模式下 `fcr_rosco_step_high` 变 no-op（避免双重步进）；`fcr_rt_destroy` 先 detach |
| P5 | ROSCO/bridge 源码 | **需要 Fortran 源码以在 WSL 构建 `.a`** | ✅ 新增 `rosco/src/`（9 个 ROSCO 源文件）+ `rosco/README_ROSCO_SRC.md` + `tools/build_rosco_linux.sh` → `build-linux/libfcr_rosco.a`（与 C 运行时一起链接） |
| P6 | S02 采样频率 | **10 ms** | 与 `RTimeCtx.dt_high` 一致；建议每控制步调用一次 `fcr_rt_set_openfast_state`（ZOH 到下一步） |

> **P4 的接口一致性是本轮最关键的结构性修正**：此前 `fcr_rt_*`（调度器句柄）与
> `fcr_rosco_*`（Fortran 侧全局实例）是**两个独立 runtime**；若原样交给 fastfarmRT，
> 调度器算出的 S08 与 ROSCO 读到的将不是同一份数据。现已统一为单例共享，
> 并有单测 `rt_shared_runtime_with_rosco_side` 固化（覆盖 adopt / 同源 setpoint / no-op step / detach）。

### 3.3 容量升级的影响面（P1 实施记录）

| 对象 | 旧 (V1, 64 点) | 新 (V2, 1000 点) | 受影响处 |
|---|---|---|---|
| `FcrAWAECatalogFrame` (S13) | 2584 B | **40024 B** | ZMQ/Modbus 传输、Python 镜像 `farm_protocol.py`、`check_abi.py`、`test_abi.c`、协议测试 |
| `FcrAWAEObservationFrame` (S12) | 3096 B | **48024 B** | 同上 |
| `FcrFlowState`（内部，1 s ZOH） | 5152 B | **57568 B** | StateStore 槽、bridge 输入、AWAE 构建 |
| Modbus 寄存器区 | 0x4000（16384） | **`MB_TOTAL_REGS` ≈ 44304** | `modbus_transport.c` 分配、`test_modbus.c` 边界断言 |
| `FCR_CAPACITY_VERSION` | 1 | **2** | 收方据 `protocol_version` + 容量版本选择布局 |

> 说明：`FCR_MAX_TURBINES` **保持 64**（本项只涉及 AWAE 观察点容量）；
> `FarmStateFrame`（S10）仍为 25664 B，未受影响。

---

## 4. WSL 测试里程碑（基于 fastfarmRT）

| 里程碑 | 内容 | 依赖 | 预期证据 |
|---|---|---|---|
| **M1 模块** | `bash tools/build_linux.sh` → 93/93；`rt_minimal_example` 冒烟 | 无 | ctest 输出、示例输出 |
| **M2 接口** | ABI/门禁/审计三件套；S02/S03/S05/S08 映射联调（可用 Mock 数据）；ROSCO `.a` 构建 | 无（P1~P6 已确认） | `check_abi.py` PASS、`libfcr_rosco.a` 生成、映射表核对 |
| **M3 调度/闭环** | 用 fastfarmRT 跑 6T 案例：验证 10 ms 步进、1 s SCADA、命令生效、TTL→D3、状态帧上送、S08 被 ROSCO 采纳 | 平台侧联调 | 与 Windows 基线（T-21 4379 kW）对比、时序日志、`FcrRoscoAck` 回读 |
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
| 头文件 | `farm_control_runtime/include/`（`fcr_rt.h` 入口；RT ABI **v2**，AWAE 容量 **V2/1000 点**） |
| C 静态库 | `build-linux/libfarm_control_runtime.a`（`bash tools/build_linux.sh`） |
| **Fortran 静态库（P5）** | `build-linux/libfcr_rosco.a`（`bash tools/build_rosco_linux.sh`；源码在 `rosco/src/`） |
| ROSCO 源码与说明 | `farm_control_runtime/rosco/src/`、`rosco/README_ROSCO_SRC.md` |
| 最小示例 | `farm_control_runtime/examples/rt_minimal_example.c` |
| RT 测试 | `farm_control_runtime/tests/unit/test_rt_adapter.c`（含 1000×10 ms Mock Scheduler、共享 runtime 用例） |
| 策略/时间/降级 | `04_RT_DATA_AND_TIME_POLICY.md` |
| Fortran 绑定 | `farm_control_runtime/rosco/FarmControlCBindings.f90`、`FarmControlInterface.f90` |
| 信号字典 | `docs/farm_control/inputs/FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv` |
| **不需要** | `wfcrl/.../DISCON_WT1.dll`（Windows PE）、`transport/modbus`、`transport/zmq` |

**链接顺序（两库一起给链接器）**

```bash
gfortran your_main.o build-linux/libfcr_rosco.a build-linux/libfarm_control_runtime.a -lpthread -lm
```

---

## 6. 风险小结（P1~P6 确认后）

| 风险 | 影响 | 状态 / 缓解 |
|---|---|---|
| AWAE 点数不足 | 流场信息截断 | ✅ **已解决**：容量扩到 1000 点（ABI V2） |
| 多线程调用 | 数据竞争（模块无锁） | ✅ 平台确认单线程；约定写入 `fcr_rt.h` |
| DT_Low 语义偏差 | SCADA 边界/TTL 错位 | ✅ 平台确认 **1 s**；边界由计数器独立驱动 |
| S08 与 ROSCO 不同源 | 命令不生效 | ✅ **已解决**：单例共享 + `fcr_rosco_step_high` no-op（`rt_shared_runtime_with_rosco_side`） |
| ROSCO 无法在 Linux 构建 | 闭环无法成立 | ✅ 已提供源码 + `build_rosco_linux.sh`；**首次构建需在 WSL 实测** |
| GPU/CPU 数值差异 | 与 Windows 基线功率偏差 | 首轮 baseline 对比，建立 WSL 新基线 |
| GPU/CPU 数值差异 | 与 Windows 基线功率偏差 | 首轮 baseline 对比，建立新基线 |

---

*版本：2026-10-22　对应交付包 `farm_control_runtime_delivery_20261022.zip`（RT ABI v2）*
