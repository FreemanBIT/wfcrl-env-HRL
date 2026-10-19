# FAST.Farm 场控接口 ICD v1.5 改造 —— 开发交接文档

> 用途：本文件用于**交接给新的 agent 会话**继续开发。包含项目全貌、当前进展、未完成内容、未解决问题、关键约定与下一步建议。
> 仓库：`D:\HR_Project\wfcrl-env-HRL`　分支：`dev/farm-control-runtime`
> 基线提交：`d1d6977`（工作区有未提交改动）
> 文档编制：2026-10-19

---

# 第一部分　项目全貌

## 1.1 这是什么项目

**wfcrl-env-HRL**：基于 **FAST.Farm**（NREL 多风机尾流仿真）的**风电场协同控制强化学习环境**。

本分支 `dev/farm-control-runtime` 的核心工作，是构建一个**场级控制运行时（farm_control_runtime）**——介于"上层调度器/RL 智能体（RCP）"与"底层单机控制器 ROSCO / OpenFAST"之间的**实时中间层**。

### 三层角色

| 角色 | 说明 | 所在位置 |
|------|------|----------|
| **RCP（场级调度器 / RL Agent）** | 决定每台机的偏航目标、诱导因子、控制模式 | Python 侧 `wfcrl/` |
| **farm_control_runtime（本项目核心）** | 命令分发、状态聚合、通道状态机、安全回退、协议转换 | `farm_control_runtime/`（C / Fortran） |
| **ROSCO / OpenFAST** | 单机控制器 + 气弹仿真；通过 Bladed-DLL 接口（`avrSWAP`）交互 | `farm_control_runtime/rosco/` + 仿真器 |

### 数据流（两条链路）

```
【下行：命令】RCP ──ZeroMQ/SHM──> CommandStore ──> ROSCO Bridge ──> ROSCO Controller
                 (FarmCommandFrame)   (状态机/TTL)   (Mode0/1/2/3)   (CntrPar 注入)

【上行：状态】OpenFAST/ROSCO ──> StateStore ──> StateAggregator ──> FarmStateFrame ──ZMQ──> RCP
                              (10ms 快状态)     (1s 统计聚合)                        (Python)
```

## 1.2 为什么有这次改造（ICD v1.5）

用户提供了最新的接口规范：

- `FASTFarm_ICD_v1_5_Definitions.md`（设计说明）
- `FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv`（70 条信号的完整定义）

**ICD v1.5 的核心变化**：把场控接口定义为 **13 个 Interface_Group、70 条信号**，每条信号严格规定
`Sender → Receiver` 单跳对（SingleHop），并明确数据来源、单位、更新率、无效值语义、上游信号追溯。

**改造目标**：让工程实现与 ICD v1.5 逐条对齐（字段名、语义、单位、边界、时间基准）。

## 1.3 13 个接口组（70 信号）

| 组 | 名称 | 信号范围 | 方向 | 当前状态 |
|----|------|----------|------|----------|
| 01 | RCP_Command | S0101–S0112 | RCP → Scheduler | ✅ 结构已完成 |
| 02 | OpenFAST_RawState | S0201–S0210 | OF → Provider | ✅ 结构已完成 |
| 03 | ROSCO_RawState | S0301–S0303 | ROSCO → Provider | ✅ 结构已完成 |
| 04 | AWAE_Catalog | S0401–S0403 | AWAE → Scheduler | ⚠️ 结构需重建 |
| 05 | AWAE_Observation | S0501–S0507 | AWAE → Scheduler | ⚠️ 结构需重建 |
| 06 | TurbineWindEstimate_to_Bridge | S0601–S0604 | Scheduler → Bridge | ⚠️ 结构已定义，未接线 |
| 07 | Scheduler_to_Bridge | S0701–S0710 | Scheduler → Bridge | ⚠️ 结构已定义，未接线 |
| 08 | Bridge_to_ROSCO | S0801–S0807 | Bridge → ROSCO | ✅ 已完成 |
| 09 | Bridge_Ack_to_Scheduler | S0901–S0904 | Bridge → Scheduler | ⚠️ 部分完成 |
| 10 | Scheduler_to_RCP_Turbine | S1001–S1017 | Scheduler → RCP | ✅ 已完成 |
| 11 | Future_CT_Feedback | S1101 | Bridge → Scheduler | ⛔ 仅保留接口（本期不实现） |
| 12 | Scheduler_to_RCP_AWAE_Observation | S1201–S1207 | Scheduler → RCP | ⛔ 未实现 |
| 13 | Scheduler_to_RCP_AWAE_Catalog | S1301–S1303 | Scheduler → RCP | ⛔ 未实现 |

## 1.4 关键 ICD v1.5 改名（**最容易踩的坑**）

| 旧名（v1.4 及以前） | 新名（v1.5） | 说明 |
|---------------------|--------------|------|
| `yaw_delta_rad` | **删除** | 只保留绝对 `yaw_target_heading_rad` |
| `source_time_s`（命令侧） | `command_basis_sim_time_s` | 语义：RCP 计算所依据的状态时刻 |
| `power_ratio` | `r_pitch` | S0803 |
| `speed_ref_ratio` | `r_speed` | S0804 |
| `torque_limit_ratio` | `r_torque` | S0805 |
| `min_pitch_rad` | `min_pitch_ref_rad` | S0806 |
| `frame_seq`（命令帧） | `command_frame_seq` | S0101 |
| `frame_seq`（状态帧） | `state_frame_seq` | S1001 |
| `sim_time_s`（状态帧） | `state_sample_sim_time_s` | S1002 |
| `commit_seq` | **删除** | — |
| `yaw_effective_low_step` / `induction_effective_low_step` | `requested_effective_sim_time_s` | S0112，仿真时间轴而非步号 |
| `yaw_enable` | `external_yaw_enable` | S0801 |
| `FcrRoscoFastState` | `FcrRoscoRawState` | C 结构类型名 |
| `controller_status_flags`（ROSCO 侧） | `rosco_controller_status_flags` | S0301 |
| `gen_torque_cmd_nm` | `generator_torque_command_nm` | S0302 |
| `pitch_cmd_rad` | `pitch_command_rad` | S0303 |
| `rotor_thrust_n` | `rotor_aero_thrust_n` | S0208 / S1010 |
| `nacelle_heading_rad`（FarmTurbineState） | `nacelle_heading_actual_rad` | — |

## 1.5 新增字段

| 信号 | 字段 | 语义 |
|------|------|------|
| S0104 | `control_enable` | 0 = 外部场控禁用，回退 ROSCO 原控制 |
| S0111 | `control_mode` | 0=基线, 1=PRC, 2=Ct前馈, 3=闭环预留（**必须报 UNSUPPORTED**） |
| S0807 | `collective_pitch_offset_rad` | 集体桨距修正量（本期保留不用） |
| S0903 | `bridge_status_flags` | Bridge 状态字 |
| S0904 | `ct_ref` | 目标推力系数 |
| S0601–S0604 | `estimated_wind_mps` / `estimate_sim_time_s` / `wind_source` / `wind_quality_flags` | 风速估计 |
| S1012 | `ct_hat` | 等效推力系数估计 |
| S1015 | `turbine_status_flags` | 机组综合状态字 |

---

# 第二部分　代码结构

## 2.1 目录布局

```
farm_control_runtime/
├── include/                     # 公共头文件（对外 ABI）
│   ├── fcr_protocol.h           #   协议版本宏、命令标志位
│   ├── fcr_state_types.h        #   所有信号结构体（ICD 核心映射）
│   ├── fcr_awae_types.h         #   【新增】AWAE S04/S05/S06 结构
│   ├── fcr_rosco_api.h          #   S08/S09 setpoint + ack
│   └── fcr_types.h
├── src/                         # 核心实现
│   ├── fcr_internal.h           #   内部结构（CommandStore/StateStore/Runtime）
│   ├── command_store.c          #   S01 命令状态机（TTL/seq/通道独立）
│   ├── state_store.c            #   各源最新快照槽
│   ├── state_aggregator.c       #   S10 状态聚合（1s 统计窗口）
│   ├── runtime.c                #   装配与高低速步进
│   ├── rosco_api.c              #   DLL 内单例 + C ABI 入口
│   ├── induction_supervisor.c   #   legacy 诱导映射（非 bridge 路径）
│   ├── yaw_action_manager.c     #   偏航动作管理
│   ├── signal_statistics.c      #   1s 统计（mean/rms/min/max）
│   ├── watchdog.c               #   健康检查
│   └── bridge_runtime_adapter.c #   Bridge core ↔ runtime 适配
├── bridge/
│   ├── fcr_bridge_core.c/.h     #   S07/S08 实时核心（Mode0/1/2/3）
│   ├── yaw_adapter.c            #   legacy delta → 绝对目标适配
│   └── rt_snapshot.c/.h         #   RT 侧快照
├── awae/                        # ⚠️ 仍基于 ICD v1.1，需重建
│   ├── awae_point_frame.h
│   ├── observation_manager.c/.h
│   └── wind_mapper.c/.h
├── offline/
│   ├── harness/shm_command_source.c   # 共享内存命令源
│   └── provider/offline_fastfarm_provider.c  # 离线状态提供者
├── transport/
│   ├── zmq/zmq_transport.cpp    # ZeroMQ 传输（C++，单独 DLL）
│   └── modbus/modbus_transport.c # Modbus 传输（⚠️ 未对齐 v1.5）
├── rosco/                       # Fortran 侧
│   ├── FarmControlCBindings.f90 #   BIND(C) 结构镜像
│   └── FarmControlInterface.f90 #   语义层（ApplyReferences / PublishState）
├── tests/
│   ├── unit/                    # 17 个 C 单测文件
│   └── integration/             # 6 个 Python 集成测试
├── config/                      # modbus_map.yaml / induction_mapping.yaml
├── docs/                        # 文档（含本次计划与报告）
└── tools/
    ├── _build_test.bat          # 单元测试构建脚本
    └── check_abi.py             # Python↔C ABI 一致性检查
```

## 2.2 Python 侧

```
wfcrl/
├── transport/farm_protocol.py        # ctypes 结构镜像（必须与 C 逐字节一致）
└── simulators/fastfarm/src/_compile.py  # 构建 DISCON_WT1.dll 的脚本
```

## 2.3 关键不变量（**红线**）

1. **Python ctypes 与 C 结构必须逐字节一致**——尺寸/偏移不一致会导致 ZMQ 帧全部被拒（历史上发生过，见 WIN-05）。
2. **ROSCO 10ms 控制路径内禁止**：互斥锁、socket、malloc、等待 GPU/Modbus/AWAE。
3. **角度一律用 OpenFAST 内部约定**（CCW+，0 = +X 方向）。
4. **时间基准不可混用**：`command_basis_sim_time_s`(S0102) ≠ `bridge_step_sim_time_s`(S0701)；AWAE 源仿真时间不得被墙钟替换。
5. **AWAE / 离线不可得量**：保留接口字段，由外部注入，**禁止因无数据源而删除字段**。

---

# 第三部分　当前进展

## 3.1 构建状态

| 目标 | 状态 | 命令 |
|------|------|------|
| **DISCON_WT1.dll（含 ROSCO + runtime）** | ✅ **编译成功** | `python wfcrl\simulators\fastfarm\src\_compile.py` |
| **单元测试 test_runner.exe** | ❌ **编译失败（108 错误）** | `farm_control_runtime\tools\_build_test.bat` |

DLL 产物：`wfcrl/simulators/fastfarm/servo_dll/DISCON_WT1.dll`（2,404,198 字节，2026-10-19 09:11）

## 3.2 已完成（本次会话）

### 已改完并能编译的文件

| 文件 | 改动 |
|------|------|
| `include/fcr_protocol.h` | v1/v2 版本宏；`FCR_CMD_FLAG_CONTROL_DISABLED` |
| `include/fcr_awae_types.h` | **新建**：S04/S05/S06 结构 + wind_source 枚举 |
| `include/fcr_state_types.h` | 命令帧/状态帧全部 v1.5 字段；`FcrInductionState` 补充 |
| `include/fcr_rosco_api.h` | Setpoint 用 r_pitch/r_speed/r_torque/min_pitch_ref_rad；新增 `FcrRoscoAck` |
| `src/fcr_internal.h` | 命令状态新字段；`state_frame_seq` |
| `src/command_store.c` | 绝对偏航 + `control_enable` 回退 + `requested_effective_sim_time_s` + **修复 seq_applied 未回填的 bug** |
| `src/induction_supervisor.c` | 输出 r_pitch/r_speed/r_torque/min_pitch_ref_rad |
| `src/state_store.c` | `FcrRoscoRawState` |
| `src/state_aggregator.c` | v1.5 字段 + S10 新字段填充 |
| `src/runtime.c` | 类型/字段改名 + 新字段 |
| `src/rosco_api.c` | 类型改名 + **新增 `fcr_rosco_publish_ack` / `fcr_rosco_get_ack`** |
| `src/bridge_runtime_adapter.c/.h` | 类型/字段改名 |
| `bridge/fcr_bridge_core.c/.h` | S08 全量改名 + S0807 |
| `rosco/FarmControlCBindings.f90` | BIND(C) 镜像同步；删除多余尾字段（ABI 修正） |
| `rosco/FarmControlInterface.f90` | PublishState / ApplyReferences 全部新名 |
| `offline/harness/shm_command_source.c` | `command_frame_seq` |
| `offline/provider/offline_fastfarm_provider.c` | `rotor_aero_thrust_n` |
| `transport/zmq/zmq_transport.cpp` | `command_basis_sim_time_s` |
| `wfcrl/transport/farm_protocol.py` | **assert 修正为 88 / 5664**（原 104/6696 是错的，会 import 抛错） |
| `tests/unit/test_command_store.c` | **完全重写**为 v1.5 语义（绝对偏航 + control_enable + 8 个用例） |

### 本次会话修复的两个真实 bug

1. **`yaw_seq_applied` / `ind_seq_applied` 从未被赋值**（S0901/S0902 回读恒为 0）
   → 已在 `command_store.c` 的 SCHEDULED→ACTIVE 转换处回填。
2. **`FcrFastStateC`（Fortran）多一个 C 侧不存在的尾字段 `controller_status_flags`**（ABI 不一致）
   → 已删除。

## 3.3 ABI 尺寸（**必须锁定**）

```
FcrTurbineCommand  sizeof = 88
  yaw_target_heading_rad            @0    (double)
  yaw_seq                           @8    (uint64)
  yaw_valid                         @16   (uint32)
  induction_ref                     @24   (double)   ← 20-23 填充
  induction_seq                     @32   (uint64)
  induction_valid                   @40   (uint32)
  control_enable                    @44   (uint32)
  control_mode                      @48   (uint16)
  requested_effective_sim_time_s    @56   (double)   ← 50-55 填充
  command_basis_sim_time_s          @64   (double)
  yaw_ttl_s                         @72   (double)
  induction_ttl_s                   @80   (double)

FarmCommandFrame   sizeof = 5664
  protocol_version                  @0    (uint32)
  command_frame_seq                 @8    (uint64)
  n_turbines                        @16   (uint32)
  receive_time_s                    @24   (double)
  turbines[64]                      @32   (88 × 64 = 5632)
```

---

# 第四部分　未完成内容

## 4.1 阻塞项：单元测试编译失败（108 错误）

**构建命令**：
```
cmd /c "cd /d D:\HR_Project\wfcrl-env-HRL\farm_control_runtime && tools\_build_test.bat"
```

### 错误分布（按文件）

| 文件 | 错误数 | 需要的修改 |
|------|--------|------------|
| `tests/unit/test_abi.c` | 2 | `FcrRoscoFastState`→`FcrRoscoRawState`；**尺寸断言 88/5664 + 新偏移** |
| `tests/unit/test_rosco_api.c` | 17 | 类型改名；`frame_seq`→`command_frame_seq`；`yaw_delta_rad`→绝对目标；`source_time_s`→`command_basis_sim_time_s`；`yaw_enable`→`external_yaw_enable`；**补 `control_enable=1`** |
| `tests/unit/test_yaw_action_manager.c` | 15 | `frame_seq`→`command_frame_seq`；`yaw_delta_rad`→绝对目标（**语义改动**） |
| `tests/unit/test_state_aggregator.c` | 15 | 类型改名；`frame_seq`→`state_frame_seq`；`sim_time_s`→`state_sample_sim_time_s`；`rotor_thrust_n`→`rotor_aero_thrust_n`；`nacelle_heading_rad`→`nacelle_heading_actual_rad` |
| `tests/unit/test_induction_supervisor.c` | 17 | `FcrInductionState` 字段：`power_ratio`→`r_pitch` 等 |
| `tests/unit/test_bridge_adapter.c` | 16 | 类型改名；`FcrTurbineCommandState.source_time_s`→`command_basis_sim_time_s`；`yaw_enable`→`external_yaw_enable` |
| `tests/unit/test_bridge_core.c` | 9 | `FcrRoscoExternalSetpoint` 字段改名 |
| `tests/unit/test_modbus.c` | 5 | `frame_seq`/`yaw_delta_rad`/`sim_time_s`/`rotor_thrust_n`/`nacelle_heading_rad` |
| `tests/unit/test_offline_provider.c` | 2 | `rotor_thrust_n`→`rotor_aero_thrust_n` |
| **`src/yaw_action_manager.c`** | 1 | **源码**：`tc->yaw_target_heading`→`yaw_target_heading_rad` |
| **`transport/modbus/modbus_transport.c`** | 9 | **源码**：`yaw_delta_rad`/`*_effective_low_step`/`commit_seq`/`source_time_s`/`frame_seq`/`sim_time_s`/`rotor_thrust_n`/`nacelle_heading_rad` |

> **重要陷阱**：`frame_seq` 的改名**不能全局替换**——
> `FcrAwaePointFrame.frame_seq`（`awae/awae_point_frame.h`）**仍然叫 frame_seq**，是合法字段。
> 只有 `FarmCommandFrame.frame_seq`→`command_frame_seq` 和 `FarmStateFrame.frame_seq`→`state_frame_seq` 需要改。

### 修复建议顺序

1. 先修 2 个**源码**文件（`yaw_action_manager.c`、`modbus_transport.c`）——它们是库的一部分。
2. 再修 `test_abi.c`（含尺寸断言，是 ABI 守门人）。
3. 其余测试文件按上表机械改名，注意 `control_enable=1` 的语义补充。

## 4.2 未完成的接口实现

### （A）AWAE 接口（S04 / S05 / S12 / S13）—— **重点**

**现状**：`farm_control_runtime/awae/` 下三个文件仍基于 **ICD v1.1**：
- `awae_point_frame.h`：`frame_seq` / `source_time_s` / `FCR_AWAE_MAX_POINTS=1024`
- `observation_manager.c/.h`：点位注册 + 环形缓冲
- `wind_mapper.c/.h`：观察点 → 盘面等效风（四类语义：ambient_upstream / disk_background / disk_disturbed）

**需要做的**：
1. 以 `include/fcr_awae_types.h` 为准重建 S04/S05 结构：
   - S0401 `point_id` / S0402 `position_xyz` / S0403 `point_catalog_version`
   - S0501 `observation_frame_seq` / S0502 `observation_sample_sim_time_s` / S0503 `observation_point_count` / S0504 `point_id` / S0505 `wind_uvw` / S0506 `observation_quality_flags` / S0507 `TI`
2. 实现 S12/S13 的**第二跳发布链路**（Scheduler → RCP）。
3. `wind_mapper` 输出接入 S06 `FcrTurbineWindEstimate`。

**用户明确指示**：
> "有一些变量是在 windows 离线仿真环境下 fastfarm 无法提供的，比如 awae 的一些测量值等。这些变量保留接口，测试的时候提供外部输入即可。"

→ **保留全部字段与端口契约**；离线测试由 Provider 注入合成/外部数据
（可参考 `offline_fastfarm_provider.c` 里已有的 `fcr_offline_provider_generate_synthetic_flow` 思路）。
**不得**因 FAST.Farm 无法产生而删除字段。

### （B）S06 TurbineWindEstimate（S0601–S0604）

- 结构 `FcrTurbineWindEstimate` 已在 `fcr_awae_types.h` 定义
- **未接入** `FcrBridgeMeas`
- 离线注入路径：ROSCO `WE_Vw` 优先，否则外部注入

### （C）S07 Scheduler_to_Bridge（S0701–S0710）

- `FcrBridgeInput` 结构已定义
- **未在 runtime 中真正构造**——目前 `bridge_runtime_adapter.c` 仍是把 `FcrTurbineCommandState` 直接转成 `FcrBridgeCmd`
- 需改为以 S07 十字段快照作为 Bridge 的唯一输入

### （D）S09 Bridge_Ack（S0901–S0904）

- `FcrRoscoAck` 结构 + `fcr_rosco_publish_ack`/`fcr_rosco_get_ack` 已实现
- **未接线**：`FcrRoscoRawState.yaw_seq_applied` / `induction_seq_applied` 已在 `FarmControlInterface.f90` 填充，
  但 Bridge 侧 `bridge_status_flags`(S0903) 与 `ct_ref`(S0904) 尚未经 ack 通道回传调度器

### （E）Modbus 传输对齐（`transport/modbus/`）

- `modbus_transport.c` + `config/modbus_map.yaml` 仍用 v1.4 字段
- ICD v1.5 明确"不定义 Modbus 地址"，但字段映射需跟随改名

### （F）RT 快照（`bridge/rt_snapshot.c/.h`）

- 仍引用旧 `source_time_s` 语义，需按 S07/S10 对齐

### （G）S11 Future_CT_Feedback（S1101）

- `ct_tracking_error` —— **仅保留接口**，本期不实现闭环（ICD 明确）

## 4.3 未完成的测试

### 接口契约测试（T-05 … T-20，**尚未编写**）

| 编号 | 测试内容 | 断言要点 |
|------|----------|----------|
| T-05 | 命令帧字段往返 | Python→C→回读，S0101–S0112 全字段一致 |
| T-06 | `control_enable=0` 回退 | 通道清零 + `FCR_CMD_FLAG_CONTROL_DISABLED` + ROSCO 回原控制 |
| T-07 | `control_mode` 枚举 | 0/1/2 正常；**3 必须报 UNSUPPORTED**（WIN-15 回归） |
| T-08 | `requested_effective_sim_time_s` 排程 | 早于时刻不生效，到点生效 |
| T-09 | 绝对偏航独立序号回读 | `yaw_seq_applied` 独立于 induction |
| T-10 | 诱导独立序号回读 | `induction_seq_applied` 独立于 yaw |
| T-11 | TTL 到期回退 | 到期后 `yaw_valid=0` 且目标清除 |
| T-12 | 时间戳不混用 | `command_basis_sim_time_s` vs `bridge_step_sim_time_s` 独立保存 |
| T-13 | AWAE Catalog 注入 | 外部注入后 `point_catalog_version` 保持 |
| T-14 | AWAE Observation 注入 | 源仿真时间不被墙钟替换；quality_flags 透传 |
| T-15 | AWAE 二跳发布 | S1201–S1207 / S1301–S1303 |
| T-16 | TurbineWindEstimate | `wind_source` 正确标注 |
| T-17 | Scheduler_to_Bridge 快照 | S0701–S0710 十字段齐全 |
| T-18 | Bridge_Ack | seq 回读 + `bridge_status_flags` |
| T-19 | S10 状态帧字段 | `ct_hat`/`estimated_wind_mps`/`status_flags` |
| T-20 | `Upstream_Signal_ID` 追溯 | S0105→S0705→S0803 同业务值可追 |

### 系统级仿真回归（**ICD v1.5 后需全部重跑**）

| 编号 | 测试 | 脚本 | 判据 |
|------|------|------|------|
| T-21 | 300s 基线回归 | `induction_vs_yaw_study/run/regress_300s.py` | 重建基线（旧基线 4366 kW 是 v1.4 的） |
| T-22 | 诱导通道 300s | `run/_test_induction_only.py` | T1 a=0.20 → **-15.8%** ±2% |
| T-23 | 偏航通道 300s | `run/_test_yaw_only.py` | T1 +10° → **同 run 惩罚 -2.8%~-8.1%** |
| T-24 | 多机隔离 300s | `run/verify_multiturbine_isolation.py` | T1/T2/T3 独立生效、零串扰 |
| T-25 | Bridge 模式 300s | `run/verify_bridge_modes.py` | Bridge vs Legacy 偏差 <0.1% |
| T-26 | 三台全机回读 | §5.1 要求 | **T1/T2/T3 三台均通过**（非任一台） |
| T-27 | Modbus 通道 | `run/bench_modbus_throughput.py` | 新字段映射正确 |

---

# 第五部分　未解决的问题

## 5.1 【严重】T1 在多机场景 t≈237s 失速

**现象**：multi-turbine 场景中，T1 功率在 t≈237s 突降到 0 kW。

**证据**：FAST.Farm 日志显示
```
The BEM solution is being turned off due to low TSR. (TSR = 1.9963)
```

**已定位的机理**：
- 诱导降载（`r_torque` 0.908×、`r_speed` 0.953×）使控制器在阵风下进入深度失速
- BEM 求解器因低叶尖速比（TSR ≈ 2.0）关闭 → 气动力失效 → 功率归零

**当前处理**：已在功能测试报告中记录为**已知限制**，t>237s 的数据从多机平均中排除。

**未做**：根因彻底定位与缓解方案（候选：阵风检测、抗失速保护、变化率限制、失速边界约束）。

## 5.2 偏航效果的统计方法论（已澄清，勿重蹈覆辙）

**问题**：曾报告"T1 偏航 10° 后功率反而提升 3.2%"，被用户指出不正常。

**根因**：FAST.Farm 在 TI=10% 下是**混沌系统**，不同 run 的风场实现不同，
**跨 run 平均值比较无效**。

**正确方法**：**同一次仿真内、逐时间步比较**（same-run per-step comparison）。

**正确结论**（已重新出报告）：
- 诱导 T1 平均 **-270 kW（-15.8%）**，与理论相符
- 偏航 T1 平均 **-91 kW（-5.2%）**，与 cos³(10°) ≈ -5% 相符

> ⚠️ 任何后续仿真验证**必须**沿用 same-run 方法，否则结论不可信。

## 5.3 Hindsight 记忆服务不可用

`hindsight_*` 工具全部返回：
```
connect ECONNREFUSED 127.0.0.1:9077
```
本仓库**尚无知识页面**。若需要项目记忆，需先启动 Hindsight 服务。

## 5.4 临时调试脚本未清理

以下文件是调试遗留，应在收尾时删除或归档：
```
induction_vs_yaw_study/run/_analyze_multi.py
induction_vs_yaw_study/run/_baseline_diag.py
induction_vs_yaw_study/run/_check_bts.py
induction_vs_yaw_study/run/_full_diag.py
induction_vs_yaw_study/run/_yaw_diag.py
```

## 5.5 文档待更新

以下文档仍描述旧接口，需按 v1.5 更新：
- `farm_control_runtime/docs/ROSCO_INTEGRATION.md`（引用 power_ratio / speed_ref_ratio / min_pitch_rad）
- `farm_control_runtime/docs/ZMQ_PROTOCOL.md`（引用 frame_seq / commit_seq）
- `farm_control_runtime/docs/OFFLINE_PROVIDER.md`（引用 rotor_thrust_n）
- `farm_control_runtime/docs/M0_ROSCO_VERSION_LOCK.md`（ABI 尺寸）

---

# 第六部分　环境与命令

## 6.1 环境

| 项 | 值 |
|----|-----|
| 仓库 | `D:\HR_Project\wfcrl-env-HRL` |
| 分支 | `dev/farm-control-runtime` |
| Python | `.venv\Scripts\python.exe` |
| 工具链 | TDM-GCC-64 10.3.0（gcc / g++ / gfortran，`D:\TDM-GCC-64\bin`） |
| 工作目录注意 | DSH 会话工作目录是 `E:\software\deepseek-dsh\workspace\fastfarm simulation`，**与仓库无关**，命令需 `cd /d` 到 D 盘 |

## 6.2 关键命令

```bat
REM ---- 构建 DLL（含 runtime + ROSCO + Fortran）----
cmd /c "cd /d D:\HR_Project\wfcrl-env-HRL && python wfcrl\simulators\fastfarm\src\_compile.py 2>&1"

REM ---- 构建并运行单元测试 ----
cmd /c "cd /d D:\HR_Project\wfcrl-env-HRL\farm_control_runtime && tools\_build_test.bat 2>&1"
cmd /c "cd /d D:\HR_Project\wfcrl-env-HRL\farm_control_runtime && test_runner.exe 2>&1"

REM ---- 300s 多机隔离验证（默认 300s）----
cmd /c "cd /d D:\HR_Project\wfcrl-env-HRL && .venv\Scripts\python.exe -m induction_vs_yaw_study.run.verify_multiturbine_isolation 2>&1"
```

> PowerShell 里 `&&` 无效，需用 `cmd /c "..."` 包裹。

## 6.3 仿真场景参数（当前基线）

| 参数 | 值 |
|------|-----|
| 风速 | 8 m/s |
| 湍流强度 | TI = 10% |
| 随机种子 | seed = 1 |
| 机组间距 | 6D |
| 仿真时长 | 300 s |
| 控制步长 | DT = 3 s |
| 入流文件 | `coarse/inflow_U08_TI10_seed01.bts` |
| 机舱初始偏航 | NacYaw = 0（正对风） |
| 机组数 | 3（T1/T2/T3） |

**基线参考**（v1.4，单 run 平均，方差大 ±437 kW）：
全场 4366 kW；T1≈1633、T2≈1450、T3≈1283 kW

**隔离测试结果**（v1.4，300s PASS）：
- T1 诱导 a=0.20 → 1495 kW，ind_seq=1
- T2 偏航 +10° → 1330 kW，yaw_seq=1，heading -0.174 rad
- T3 无命令 → 1437 kW，seqs 0

## 6.4 Bridge 环境变量

| 变量 | 含义 |
|------|------|
| `FCR_BRIDGE_ENABLE` | 1 = 用 bridge core 生成诱导 setpoint（否则走 legacy InductionSupervisor） |
| `FCR_BRIDGE_INDUCTION_MODE` | 0/1/2/3 |
| `FCR_BRIDGE_ALLOC_W_PITCH` / `_W_SPEED` / `_W_TORQUE` | 控制分配权重（千分比，和=1000） |

## 6.5 诱导映射默认配置（`FcrInductionMapConfig`）

```
a0 = 1/3
a_min = 0.05
a_max = 1/3            (WIN-12)
ct_coef = 1.0
min_pitch_gain = 0.55
min_pitch_max = 0.30
power_cap = 1.0
```

## 6.6 命令下发模式（Python 测试脚本）

```python
cmds = {
    1: Cmd(turbine_id=1, yaw_delta_deg=..., induction_a=...),
    2: Cmd(turbine_id=2, ...),
}
```
`induction_vs_yaw_study/replay/fastfarm_zmq_replay.py` 的 `_send_cmds` 会**为每台机发独立帧**。

---

# 第七部分　下一步建议（给新 agent）

## 推荐执行顺序

### 阶段 A：恢复单元测试编译（**最高优先级**）
1. 修 `src/yaw_action_manager.c`（1 处）与 `transport/modbus/modbus_transport.c`（9 处）
2. 修 `tests/unit/test_abi.c`——**锁定 88/5664 及新偏移**
3. 按第四部分 4.1 的表修其余 8 个测试文件
4. 目标：`_build_test.bat` 输出 `BUILD_OK`，`test_runner.exe` 全绿

> **注意**：`frame_seq` 不能全局替换（`FcrAwaePointFrame` 仍用该名）。
> 测试用例普遍需要补 `cmd->control_enable = 1`，否则通道会被 `control_enable=0` 分支清空。

### 阶段 B：补齐接口实现
5. `bridge/rt_snapshot.c/.h` 对齐 S07/S10
6. `transport/modbus/` + `config/modbus_map.yaml` 对齐
7. S06 风速估计接入 `FcrBridgeMeas`
8. S07 `FcrBridgeInput` 真正作为 Bridge 唯一输入
9. S09 ack 回传链路打通

### 阶段 C：AWAE 接口落地
10. 重建 `awae/awae_point_frame.h` → S04/S05（`fcr_awae_types.h` 为准）
11. `observation_manager` 支持 frame_seq / sample_sim_time_s / quality_flags / catalog_version
12. `wind_mapper` 输出接 S06
13. S12/S13 二跳发布
14. **外部注入 API**（离线测试用）：`fcr_offline_provider_inject_awae_catalog/observation`

### 阶段 D：测试补全
15. 新增 T-05 … T-20 接口契约测试
16. 更新 `tools/check_abi.py`

### 阶段 E：系统级仿真回归
17. **先重建 300s 基线**（v1.4 的 4366 kW 不再适用）
18. T-21 … T-27 全部重跑
19. **必须用 same-run 逐时间步比较法**（见 5.2）

### 阶段 F：遗留与收尾
20. T1 失速根因分析（见 5.1）
21. 清理 5 个临时调试脚本
22. 更新 4 份文档（见 5.5）
23. 提交（当前工作区有 22 个修改文件 + 3 个新增文件未提交）

## 验收门禁（ICD v1.5 §5.3）

| # | 门禁 |
|---|------|
| G-1 | 每条 CSV 的 Sender/Receiver 唯一；每组固定一对 |
| G-2 | v1.4 全部 Signal_ID 保留；S07(8)/S12(7)/S13(3) 不冲突 |
| G-3 | `Upstream_Signal_ID` 可跨两跳追溯 |
| G-4 | `command_basis_sim_time_s` 与 `bridge_step_sim_time_s` 不混用；AWAE 源时间不被墙钟替换 |
| G-5 | 各机绝对偏航/诱导独立执行、独立回读、TTL 回退 |
| G-6 | 不产生 Modbus 地址约束 |

---

# 附录　本次会话踩过的坑（避免重复）

| # | 坑 | 教训 |
|---|-----|------|
| 1 | Python assert 写 104/6696，实际是 88/5664 | **改结构后立即用脚本算偏移并验证 import** |
| 2 | Fortran `FcrFastStateC` 多一个 C 侧不存在的尾字段 | BIND(C) 结构必须逐字段与 C 对齐 |
| 3 | `yaw_seq_applied` 从未赋值 | 状态机转换点要检查回读字段是否回填 |
| 4 | `edit` 工具对不存在的老字符串会报错 | 多模式替换应"读全文 → JS 转换 → 写回"，而非逐个 edit |
| 5 | 测试用例未设 `control_enable` 导致通道被清空 | 新增门控字段后必须同步所有测试夹具 |
| 6 | 跨 run 比较 FAST.Farm 结果 | TI=10% 是混沌系统，**只能 same-run 比较** |
| 7 | PowerShell 里用 `&&` | 必须 `cmd /c "..."` 包裹 |
| 8 | `python -c "..."` 在 pwsh 下引号被吞 | 写临时 .py 文件再执行 |
| 9 | 在 DLL 里 `fprintf(stderr)` 调试日志捕获不到 | 改写到文件，且**收尾前删除调试代码** |
| 10 | `frame_seq` 全局替换会误伤 `FcrAwaePointFrame` | 改名务必先确认字段归属 |
