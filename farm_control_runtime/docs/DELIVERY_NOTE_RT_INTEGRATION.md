# farm_control_runtime 交付说明（ICD v1.5 / Windows 离线验证完成版）

> 交付对象：LinuxRT 实时机开发人员（后续阶段 R-01~R-07）
> 仓库：`D:\HR_Project\wfcrl-env-HRL`　分支：`dev/farm-control-runtime`
> 基线提交：`f920ceb`（含信号审计 `audit_signals.py` 与 S1101 保留接口）
> 编制：2026-10-20

---

## 1. 交付内容

| 类别 | 内容 | 位置 |
|---|---|---|
| **C 运行时源码** | runtime 核心（command_store/state_store/state_aggregator/rosco_api/induction_supervisor/yaw_action_manager/watchdog/bridge_runtime_adapter）+ bridge core + AWAE 模块 + offline provider + ZMQ/Modbus 传输 | `farm_control_runtime/{include,src,bridge,awae,offline,transport}/` |
| **Fortran 绑定** | `FarmControlCBindings.f90`（BIND(C) 结构镜像）+ `FarmControlInterface.f90`（语义层） | `farm_control_runtime/rosco/` |
| **公共头文件（对外 ABI）** | `fcr_protocol.h` / `fcr_state_types.h` / `fcr_awae_types.h` / `fcr_rosco_api.h` / `fcr_transport.h` | `farm_control_runtime/include/` |
| **Python 协议镜像（RCP 侧）** | `farm_protocol.py`（ctypes 逐字节镜像）+ `farm_zmq_client.py`（ZMQ 客户端） | `wfcrl/transport/` |
| **构建脚本** | `_compile.py`（DISCON_WT1.dll 含 runtime+ROSCO）、`tools/_build_test.bat`（单测）、`tools/build_windows.ps1` | `wfcrl/simulators/fastfarm/src/`、`farm_control_runtime/tools/` |
| **测试** | C 单测（80/80）、Python 协议测试（12/12）、集成测试（26/26）、G-1/G-2/G-3 门禁、信号审计 | `farm_control_runtime/{tests,tools}/` |
| **产物** | `DISCON_WT1.dll`（2359.4 KB，含 S1101 保留接口） | `wfcrl/simulators/fastfarm/servo_dll/` |
| **文档** | ICD 进度/任务计划/交接/信号清单 CSV | `farm_control_runtime/docs/`、`docs/farm_control/inputs/` |

---

## 2. 架构总览

```
【下行：命令】RCP ──ZeroMQ/SHM──> CommandStore ──> ROSCO Bridge ──> ROSCO Controller
                 (FarmCommandFrame)   (状态机/TTL)   (Mode0/1/2/3)   (CntrPar 注入)
                    │ S0701-0710              │ S0801-0807
                    ▼                         ▼
                 FcrBridgeInput          FcrRoscoExternalSetpoint
                    └────── fcr_bridge_core_step ──────┘

【上行：状态】OpenFAST/ROSCO ──> StateStore ──> StateAggregator ──> FarmStateFrame ──ZMQ──> RCP
              (S02/S03 RawState)   (10ms 快状态)     (1s 统计聚合)    (S10, 25664B)   (Python)

【AWAE 二跳】AWAE Provider ──S04/S05──> ObservationManager ──S12/S13──ZMQ──> RCP
             （离线由 offline_fastfarm_provider 合成注入，带 SIMULATED 质量位）
```

**三层角色**：
- **RCP（场级调度器 / RL Agent）**：Python 侧 `wfcrl/`，决定每台机偏航目标/诱导因子/控制模式
- **farm_control_runtime（本项目核心）**：命令分发、状态聚合、通道状态机、安全回退、协议转换
- **ROSCO / OpenFAST**：单机控制器 + 气弹仿真（Bladed-DLL 接口）

---

## 3. 构建（Windows 离线验证环境）

```bat
cd /d D:\HR_Project\wfcrl-env-HRL

REM ---- 1) 单元测试（gcc，无 FAST.Farm）----
farm_control_runtime\tools\_build_test.bat && farm_control_runtime\test_runner.exe   REM 80/80

REM ---- 2) ABI 交叉校验（Python↔C 82 字段偏移）----
.venv\Scripts\python.exe farm_control_runtime\tools\check_abi.py

REM ---- 3) ICD 门禁 G-1/G-2/G-3（基于 SingleHop CSV）----
.venv\Scripts\python.exe farm_control_runtime\tools\check_icd_gates.py

REM ---- 4) 信号列表一致性审计（88 信号 ↔ C 结构）----
.venv\Scripts\python.exe farm_control_runtime\tools\audit_signals.py

REM ---- 5) Python 协议测试（含跨语言往返）----
.venv\Scripts\python.exe farm_control_runtime\tests\integration\test_icd_v15_abi_python.py   REM 12/12

REM ---- 6) 集成测试（FAST.Farm 闭环，逐文件避免 libzmq teardown abort）----
.venv\Scripts\python.exe -m pytest farm_control_runtime\tests\integration\test_offline_provider_states.py -q
... （其余 5 个文件逐个跑）

REM ---- 7) 构建 DISCON_WT1.dll（含 runtime + ROSCO + Fortran）----
.venv\Scripts\python.exe wfcrl\simulators\fastfarm\src\_compile.py
```

---

## 4. ABI 冻结（RT 集成红线）

所有公共结构体尺寸/偏移由 `tests/unit/test_abi.c` + `tools/check_abi.py` 双重锁定，
**任何改动必须同步三处**：C 头文件 + test_abi.c + check_abi.py（+ Python 镜像 farm_protocol.py）。

### 4.1 关键尺寸（64 位 TDM-GCC-64 10.3.0 实测）

| 结构 | 尺寸 | 说明 |
|---|---|---|
| `FcrTurbineCommand` | 88 | S01 每机命令（yaw/induction 双通道独立 seq） |
| `FarmCommandFrame` | 5664 | 命令帧（32 头 + 88×64） |
| `FcrOpenFASTRawState` | 104 | S02 |
| `FcrRoscoRawState` | 232 | S03 + 扩展 |
| `FcrSignalStats` | 48 | 1s 统计容器 |
| `FcrTurbineExtraState` | 840 | S0208-0210 + 载荷 |
| `FcrFlowState` | 5152 | AWAE 流场（64 点） |
| `FcrBridgeInput` | 72 | S07 |
| `FcrBridgeOutput` | 56 | S08 |
| `FcrBridgeAck` | 32 | S09 |
| `FcrRoscoExternalSetpoint` | **112** | S08+S1101（`ct_tracking_error` 保留接口，追加在末尾） |
| `FcrRoscoAck` | 32 | S09 |
| `FcrFarmTurbineState` | 400 | S10 |
| `FarmStateFrame` | 25664 | 状态帧（64 头 + 400×64） |
| `FcrAWAECatalogEntry/Frame` | 40 / 2584 | S04/S13 |
| `FcrAWAEObservation/Frame` | 48 / 3096 | S05/S12 |
| `FcrTurbineWindEstimate` | 32 | S06 |

### 4.2 容量版本化（W-09）

```c
// fcr_protocol.h
#define FCR_CAPACITY_VERSION          1u
#define FCR_MAX_TURBINES_V1           64u
#define FCR_MAX_OBSERVATION_POINTS_V1 64u
#define FCR_MAX_TURBINES              FCR_MAX_TURBINES_V1
#define FCR_MAX_OBSERVATION_POINTS    FCR_MAX_OBSERVATION_POINTS_V1
```
- 线协议容量冻结（64 机 / 64 观察点），帧尺寸不变；
- 所有内部消费者（bridge、legacy AWAE）统一引用这些常量；
- **RT 侧若要扩容，必须新增 `*_V2` 常量 + 提升 `FCR_CAPACITY_VERSION`，帧的
  `protocol_version` 字段决定接收方按哪套布局解析**（版本化 ABI 机制）。

### 4.3 ZMQ 帧类型区分（W-08）

ZMQ 消息 = 一整帧，帧类型由**冻结尺寸**区分（state 25664 / AWAE obs 3096 / AWAE catalog 2584）。
RT 侧若改走 Modbus，AWAE 帧按 `modbus_register_map.h` 布局（0x3000 区，32 寄存器/点）。

---

## 5. 信号列表一致性（审计结论）

`tools/audit_signals.py` 对照 `FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv`（88 信号 / 13 组）逐一映射：

- **88/88 全部一致**：每个信号都有对应 C 结构字段、类型匹配、方向/承载正确；
- **S11（Future_CT_Feedback）**：`ct_tracking_error` 已作为**保留接口**加入
  `FcrRoscoExternalSetpoint`（本期恒 0，RT 阶段实现闭环）；
- **2 条 NOTE**（无实际影响）：`S0201/S1003 turbine_id` CSV 标 uint32、C 用 int32_t
  （ABI 足迹相同 4B，值恒 ≥1，仅符号解释差异）；
- **G-1/G-2/G-3 门禁 PASS**：Sender/Receiver 唯一、v1.4 保留、S12/S13 不冲突、
  Upstream_Signal_ID 跨两跳可追。

---

## 6. 验证状态（全部实测）

| 验证项 | 结果 |
|---|---|
| C 单元测试 | **80/80 PASS** |
| Python 协议测试 | **12/12 PASS**（含 T-05 跨语言往返） |
| ABI 交叉校验 | **PASS**（C 尺寸 + 82 Python↔C 字段偏移） |
| 集成测试（FAST.Farm 闭环） | **26/26 PASS**（libzmq teardown abort 为已知环境问题） |
| G-1/G-2/G-3 门禁 | **PASS** |
| 信号审计 | **PASS（88/88）** |
| T-21 基线回归（300s） | **PASS**（4366 kW vs 基线 4366 kW，0.00%） |
| T-22 诱导通道（300s） | PASS（T1 a=0.20 → -15.14%） |
| T-23 偏航同 run 复核 | PASS（命令执行正确；旧 -27.1% 为 FAST.Farm wake 失稳污染） |
| T-24 多机隔离（300s） | PASS（逐机 seq 隔离，三台回读） |
| T-25 Bridge 模式 | PASS（离线 ctypes，Mode0/1/2 + Ct + yaw + safety） |
| T-27 Modbus 吞吐 | PASS（30/100 台解析模型） |

---

## 7. RT 集成要点（LinuxRT 阶段）

### 7.1 必须遵守的红线（ICD v1.5 + 本项目约定）

1. **ROSCO 10 ms 控制路径内禁止**：互斥锁、socket、malloc、等待 GPU/Modbus/AWAE。
   （`fcr_rosco_read_setpoint` / `fcr_rosco_publish_state` / `fcr_rosco_publish_ack` 是
   runtime ↔ ROSCO 的唯一 10ms 接口，共享内存读写。）
2. **角度一律用 OpenFAST 内部约定**（CCW+，0 = +X 方向）；`yaw_target_heading_rad` 是
   绝对目标（S0108/S0802），不是增量。RCP 侧 CW+ 动作语义由 `FarmZmqClient.send_yaw_delta`
   内部换算，RT 侧不得重复换算。
3. **时间基准不可混用**：`command_basis_sim_time_s`(S0102) ≠ `bridge_step_sim_time_s`(S0701)；
   AWAE 源仿真时间（S0502）不得被墙钟替换（G-4）。
4. **AWAE / 离线不可得量**：保留接口字段，由外部注入（离线 Provider 的
   `FCR_CFG_AWAE_SYNTH_POINTS`），**禁止因无数据源而删除字段**。
5. **容量版本化**：扩容必须走 `*_V2` 常量 + `FCR_CAPACITY_VERSION`，不得硬改冻结尺寸。

### 7.2 RT 阶段待办（R-01~R-07，需 RT 环境）

| 项 | 内容 | 建议 |
|---|---|---|
| R-01 | 实例隔离 | 每机组独立 runtime 实例（当前 Windows 是每 WT 进程独立 DLL） |
| R-02 | 快照原子性 | `rt_snapshot` 已提供基础；RT 需确认 double-buffer 无撕裂 |
| R-03 | 10ms 无锁 | 确认 10ms 路径零锁（当前已满足，RT 需复查编译/调度） |
| R-04 | 容量版本化（Linux 侧） | 使用 `FCR_CAPACITY_VERSION` 机制对齐 |
| R-05 | Modbus Client | `transport/modbus/` 已实现 Server 侧；RT 需 Client（读命令区/写状态区） |
| R-06 | 大结构拷贝 | `FarmStateFrame`(25664B)/`FarmCommandFrame`(5664B) 的传递方式 |
| R-07 | 供应商调度核 / ROSCO 主源码 | 与 ROSCO v2.10.6 供应商源码对齐 |

### 7.3 集成顺序建议

1. **先接 ABI**：把 `farm_control_runtime/include/*.h` 放入 RT 工程，跑 `test_abi.c`
   尺寸断言（或移植 `check_abi.py` 的 C 探针），确认结构逐字节一致；
2. **再接 10ms 路径**：实现 `fcr_rosco_read_setpoint` / `publish_state` / `publish_ack`
   的共享内存读写（对齐 `FcrRoscoExternalSetpoint`/`FcrRoscoRawState`/`FcrRoscoAck`）；
3. **再接低速路径**：`fcr_runtime_step_low` 聚合 + 状态帧发布（S10）；
4. **再接命令路径**：CommandStore 状态机（TTL/seq/control_enable/requested_effective_time）；
5. **最后接 AWAE / Modbus / 二跳**（S04/S05/S12/S13/S06）。

### 7.4 已知限制与缓解（Windows 离线验证中发现）

- **FAST.Farm t≈237s 失稳（T-28）**：根因 = FAST.Farm wake 平面互相超越被移除
  （`Reduce f_c to prevent planes from overtaking`，f_c=0.105651 过高），基线无命令也出现
  （162 条警告）。**非 runtime 缺陷**；缓解 = 研究侧调低 `.fstf` 的 `f_c`（T-29）。
- **libzmq teardown abort（Windows）**：`Assertion failed: Socket operation on non-socket`
  在集成测试 teardown 偶发 abort 进程——环境问题，逐文件跑即可；RT 侧建议用共享内存
  或自研传输替代 libzmq（R-05 Modbus Client 方向）。

---

### 7.5 WSL 环境测试（采购前可做）

模块级与调度器级测试**可在 WSL 完成**（不需要 FAST.Farm）：

```bash
sudo apt-get install -y build-essential cmake
cd farm_control_runtime
bash tools/build_linux.sh          # cmake 构建 + ctest
# 预期：91/91 通过（Windows 为 98/98，差 7 个 Windows-only Modbus 用例）
```

- Linux 自动排除的 Windows-only 模块：`modbus_transport.c`（Winsock）、
  `shm_command_source.c`（Windows SHM，替换为 no-op stub）、`zmq_transport.cpp`（LoadLibrary）；
- RT 路径本身平台无关：`fcr_rt_adapter.c` 用 `CLOCK_MONOTONIC`，`rosco_api.c` 用 pthread；
- **不在 WSL 范围**：`DISCON_WT1.dll`（Windows PE）与 FAST.Farm 闭环（属 M4）。

详细审计与验证清单见 **`docs/WSL_READINESS.md`**。

---

## 8. 关键文档索引

| 文档 | 内容 |
|---|---|
| `farm_control_runtime/docs/ICD_V1_5_PROGRESS.md` | 迁移进度、验证证据、审查修复状态 |
| `farm_control_runtime/docs/ICD_V1_5_TASK_PLAN.md` | 任务执行计划（T-01~T-29 定义） |
| `farm_control_runtime/docs/ICD_V1_5_HANDOFF.md` | 开发交接（架构/改名表/红线） |
| `farm_control_runtime/docs/ICD_V1_5_IMPLEMENTATION_PLAN.md` | 接口变更清单 |
| `farm_control_runtime/transport/zmq/ZMQ_PROTOCOL.md` | ZMQ 帧协议（含 AWAE 二跳） |
| `farm_control_runtime/transport/modbus/modbus_register_map.h` | Modbus 寄存器布局 |
| `farm_control_runtime/docs/FUNCTIONAL_TEST_REPORT.md` | 功能测试报告（同 run 方法） |
| `docs/farm_control/inputs/FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv` | ICD v1.5 信号权威清单 |
| `farm_control_runtime/docs/M0_ROSCO_VERSION_LOCK.md` | ROSCO/FAST.Farm 版本锁定 |

---

*本说明随 `f920ceb` 之后的信号审计与 S1101 保留接口交付。RT 集成如有疑问，以
`ICD_V1_5_PROGRESS.md` 与 `check_abi.py`/`audit_signals.py`/`check_icd_gates.py`
三个自动化检查为准。*
