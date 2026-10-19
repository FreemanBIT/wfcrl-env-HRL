# FAST.Farm 场控接口 ICD v1.5 改造与测试补全 —— 任务执行计划

> 依据：`FASTFarm_ICD_v1_5_Definitions.md` + `FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv`
> 仓库：`D:\HR_Project\wfcrl-env-HRL`  分支：`dev/farm-control-runtime`
> 编制时间：2026-10-19  基线提交：`d1d6977`

---

## 0. 当前状态快照

### 0.1 编译状态：**FAIL**

当前工作区处于 ICD v1.5 改造中途，DLL **无法编译**：

| 错误位置 | 数量 | 原因 |
|----------|------|------|
| `src/runtime.c` | 40 | 仍用 `FcrRoscoFastState`、`rotor_thrust_n`、`FarmStateFrame.sim_time_s`、`FcrFarmTurbineState.nacelle_heading_rad` 等旧名 |
| `src/bridge_runtime_adapter.h` | 2 | 形参类型 `FcrRoscoFastState` 已重命名为 `FcrRoscoRawState` |
| 级联 | — | `rosco_api.c` / `state_aggregator.c` / 单测待修 |

### 0.2 已完成（头文件层 + 部分实现层）

| 文件 | 状态 |
|------|------|
| `include/fcr_protocol.h` | ✅ v1/v2 版本宏；新增 `FCR_CMD_FLAG_CONTROL_DISABLED` |
| `include/fcr_awae_types.h` | ✅ **新建**：S04/S05/S06 结构 + wind_source 枚举 |
| `include/fcr_state_types.h` | ✅ 命令帧 S0101-S0112、S0201-S0210、S0301-S0303、S07、S08、S09、S10 结构 |
| `include/fcr_rosco_api.h` | ✅ Setpoint 用 r_pitch/r_speed/r_torque/min_pitch_ref_rad；新增 `FcrRoscoAck` |
| `src/fcr_internal.h` | ✅ 通道状态机字段改名 |
| `rosco/FarmControlCBindings.f90` | ✅ BIND(C) 镜像同步 |
| `rosco/FarmControlInterface.f90` | ✅ `FCR_ApplyReferences` 用新名；新增 `FCR_GetRPitch/RSpeed/RTorque` |
| `src/command_store.c` | ✅ 绝对偏航 + `control_enable` 回退 + `requested_effective_sim_time_s` |
| `src/induction_supervisor.c` | ✅ r_pitch/r_speed/r_torque/min_pitch_ref_rad |
| `src/state_store.c` | ✅ `FcrRoscoRawState` |
| `transport/zmq/zmq_transport.cpp` | ✅ 帧校验用 `command_basis_sim_time_s` |
| `wfcrl/transport/farm_protocol.py` | ⚠️ 字段已改，**但 assert 尺寸写错**（见 §1.1） |

### 0.3 未完成（本次计划主体）

- `src/runtime.c`、`src/rosco_api.c`、`src/state_aggregator.c`、`src/bridge_runtime_adapter.{c,h}`
- `bridge/fcr_bridge_core.{c,h}`、`bridge/yaw_adapter.c`
- `offline/provider/offline_fastfarm_provider.c`
- `transport/modbus/*`、`src/rt_snapshot.{c,h}`
- `awae/observation_manager.{c,h}`、`awae/wind_mapper.{c,h}`（仍基于 ICD v1.1）
- `tests/unit/*.c`（17 个测试文件）、`tests/integration/*.py`
- `tests/unit/check_abi.py`、`config/modbus_map.yaml`、`config/induction_mapping.yaml`

---

## 1. 接口变更清单（按 Interface_Group）

### 1.1 立即修复的缺陷（阻塞项）

| # | 缺陷 | 修正 |
|---|------|------|
| **D-1** | `farm_protocol.py` assert 尺寸 = **104 / 6696** | 改为 **88 / 5664**（实测新布局） |
| **D-2** | `FcrTurbineCommand` 新布局 | 88 B：`yaw_target_heading_rad`@0, `yaw_seq`@8, `yaw_valid`@16, `induction_ref`@24, `induction_seq`@32, `induction_valid`@40, `control_enable`@44, `control_mode`@48, `requested_effective_sim_time_s`@56, `command_basis_sim_time_s`@64, `yaw_ttl_s`@72, `induction_ttl_s`@80 |
| **D-3** | `FarmCommandFrame` 新布局 | 5664 B：头 32 B（`protocol_version`@0, `command_frame_seq`@8, `n_turbines`@16, `receive_time_s`@24）+ 88×64 |
| **D-4** | `test_abi.c` 仍锁 104/6696、offset 56 | 重算并锁定新尺寸/偏移 |

### 1.2 GG-01 RCP_Command（S0101–S0112）

| 变更 | 说明 |
|------|------|
| 删除 `yaw_delta_rad` | ICD v1.5 仅绝对目标 `yaw_target_heading_rad`(S0108) |
| 删除 `yaw_absolute_valid` | 与 `yaw_valid`(S0110) 合并 |
| 删除 `yaw/induction_effective_low_step` | 由 `requested_effective_sim_time_s`(S0112) 取代（仿真时间轴，非步号） |
| `source_time_s` → `command_basis_sim_time_s`(S0102) | 语义：RCP 计算所依据的状态仿真时刻 |
| 新增 `control_enable`(S0104) | 0 → 逐步回退 ROSCO 原控制 |
| 新增 `control_mode`(S0111) | 0=基线 1=PRC 2=Ct前馈 3=闭环预留（**不得报告成功**） |
| `frame_seq` → `command_frame_seq`(S0101)；删除 `commit_seq` | — |

**影响**：`command_store.c`（已改）、`farm_protocol.py`、`zmq_transport.cpp`、`modbus_transport.c`、`shm_command_source.c`、`test_command_store.c`、`test_modbus.c`

### 1.3 GG-02/03 OpenFAST/ROSCO RawState

| 变更 | 说明 |
|------|------|
| `FcrRoscoFastState` → `FcrRoscoRawState` | 24 处引用 |
| 新增 S0301 `rosco_controller_status_flags` | 由原 `controller_status_flags` 改名 |
| 新增 S0302 `generator_torque_command_nm` | 由 `gen_torque_cmd_nm` 改名 |
| 新增 S0303 `pitch_command_rad[3]` | 由 `pitch_cmd_rad[3]` 改名 |
| S0208 `rotor_aero_thrust_n` / S0209 `shaft_torque_nm` / S0210 `tower_base_fa_moment_nm` | 迁入 `FcrTurbineExtraState` |
| S0206 `blade_pitch_actual_rad[3]` | 迁入 `FcrOpenFASTRawState` |

**影响**：`runtime.c`、`state_aggregator.c`、`rosco_api.c`、`bridge_runtime_adapter.c`、`offline_fastfarm_provider.c`、`rt_snapshot.c`

### 1.4 GG-04/05/12/13 AWAE（**Windows 离线不可提供 → 保留接口 + 外部注入**）

| 组 | 信号 | 现状 | 计划 |
|----|------|------|------|
| GG-04 AWAE_Catalog | S0401 point_id / S0402 position_xyz / S0403 catalog_version | `awae_point_frame.h` 有前两项，**缺 version** | 以 `fcr_awae_types.h` 为准重建；**测试注入** |
| GG-05 AWAE_Observation | S0501 frame_seq / S0502 sample_sim_time_s / S0503 point_count / S0504 point_id / S0505 wind_uvw / S0506 quality_flags / S0507 TI | 缺 frame_seq 命名、quality_flags、float32 精度 | 重建结构 + 注入通道 |
| GG-12 Scheduler→RCP AWAE_Obs | S1201–S1207 | **未实现** | 发布链路（第二跳） |
| GG-13 Scheduler→RCP AWAE_Catalog | S1301–S1303 | **未实现** | 发布链路（第二跳） |

**关键约束（用户指示）**：FAST.Farm 离线无法产生 AWAE 量测。
→ **保留全部字段与端口契约**；离线测试由 **Provider 注入合成/外部数据**（沿用 WIN-13 的
`fcr_offline_provider_generate_synthetic_flow` 思路），**不得**因缺数据而删字段。

### 1.5 GG-06 TurbineWindEstimate_to_Bridge（S0601–S0604）

| 信号 | 计划 |
|------|------|
| S0601 estimated_wind_mps / S0602 estimate_sim_time_s / S0603 wind_source / S0604 wind_quality_flags | 新增 `FcrTurbineWindEstimate`（已在 `fcr_awae_types.h`）；接入 `FcrBridgeMeas`；离线由 Provider 注入（ROSCO `WE_Vw` 优先，否则外部） |

### 1.6 GG-07 Scheduler_to_Bridge（S0701–S0710）

| 信号 | 计划 |
|------|------|
| S0701 bridge_step_sim_time_s / S0702 valid_flags / S0703 turbine_id / S0704 control_enable / S0705 induction_ref / S0706 induction_seq / S0707 yaw_target_heading_rad / S0708 yaw_seq / S0709 control_mode / S0710 command_basis_sim_time_s | `FcrBridgeInput` 已定义；**需在 runtime 中真正构造并作为 Bridge 唯一输入**（替代目前 `FcrBridgeCmd` 直填） |

**注意**：ICD 明确 `command_basis_sim_time_s`(S0102) ≠ `bridge_step_sim_time_s`(S0701)，
不得混用（验收门禁 §5.3-4）。

### 1.7 GG-08 Bridge_to_ROSCO（S0801–S0807）

| 变更 | 说明 |
|------|------|
| 新增 `external_yaw_enable`(S0801) | 与 `yaw_enable` 统一命名 |
| `power_ratio` → `r_pitch`(S0803) | 49 处引用 |
| `speed_ref_ratio` → `r_speed`(S0804) | 21 处 |
| `torque_limit_ratio` → `r_torque`(S0805) | 21 处 |
| `min_pitch_rad` → `min_pitch_ref_rad`(S0806) | 20 处 |
| 新增 `collective_pitch_offset_rad`(S0807) | ROSCO v2.10.6 暂未使用，保留 |

**影响**：`fcr_bridge_core.{c,h}`、`bridge_runtime_adapter.c`、`induction_mapping.yaml`、`test_bridge_core.c`、`test_induction_supervisor.c`

### 1.8 GG-09 Bridge_Ack_to_Scheduler（S0901–S0904）

新增 `FcrRoscoAck`（已定义）：`induction_seq_applied` / `yaw_seq_applied` /
`bridge_status_flags`(S0903) / `ct_ref`(S0904)；经 `fcr_rosco_publish_ack` 上报。

### 1.9 GG-10 Scheduler_to_RCP_Turbine（S1001–S1017）

| 变更 | 说明 |
|------|------|
| `frame_seq` → `state_frame_seq`(S1001) | — |
| `sim_time_s` → `state_sample_sim_time_s`(S1002) | — |
| 新增 S1011 `estimated_wind_mps` | 来自 GG-06 |
| 新增 S1012 `ct_hat` | 由推力+风估计计算 |
| 新增 S1015 `turbine_status_flags` | ROSCO + Bridge 综合 |
| 新增 S1016 `wind_source` / S1017 `wind_quality_flags` | 质量追溯 |
| `gen_power_w.mean/instant` | 映射 S1004/S1005 |
| `rotor_thrust_n` → `rotor_aero_thrust_n`(S1010) | 32 处引用 |

### 1.10 GG-11 Future_CT_Feedback（S1101）

`ct_tracking_error` —— **仅保留接口**，本期不实现闭环。

---

## 2. 未完成的测试清单

### 2.1 阻塞类（编译/ABI）

| # | 测试 | 现状 | 目标 |
|---|------|------|------|
| T-01 | `fcr_unit_tests` 全量 | **无法编译** | 全部通过 |
| T-02 | `test_abi.c` 尺寸/偏移锁定 | 锁旧值 104/6696 | 锁 88/5664 及新偏移 |
| T-03 | `check_abi.py`（Python↔C 一致性） | 引用旧名 | 通过 |
| T-04 | `farm_protocol.py` import | assert 会抛错 | import 成功 |

### 2.2 接口契约类（ICD v1.5 新增，尚未编写）

| # | 测试 | 覆盖信号 | 断言要点 |
|---|------|----------|----------|
| T-05 | 命令帧字段往返 | S0101–S0112 | 每个字段 Python→C→回读一致 |
| T-06 | `control_enable=0` 回退 | S0104 | 通道清零、`FCR_CMD_FLAG_CONTROL_DISABLED` 置位、ROSCO 回原控制 |
| T-07 | `control_mode` 枚举 | S0111 | 0/1/2 正常；**3 必须报 UNSUPPORTED**（WIN-15 回归） |
| T-08 | `requested_effective_sim_time_s` 排程 | S0112 | 早于时刻不生效，到点生效 |
| T-09 | 绝对偏航独立序号回读 | S0108/S0109/S0110 | `yaw_seq_applied` 独立于 induction |
| T-10 | 诱导独立序号回读 | S0105/S0106/S0107 | `induction_seq_applied` 独立于 yaw |
| T-11 | TTL 到期回退 | S0108–S0110 | 到期后 `yaw_valid=0` 且目标清除 |
| T-12 | 时间戳不混用 | S0102 vs S0701 | 两值独立保存，不得互相覆盖 |
| T-13 | AWAE Catalog 注入 | S0401–S0403 | 外部注入 → `point_catalog_version` 保持 |
| T-14 | AWAE Observation 注入 | S0501–S0507 | 源仿真时间不被墙钟替换；quality_flags 透传 |
| T-15 | AWAE 二跳发布 | S1201–S1207 / S1301–S1303 | 第一跳≠第二跳接收端 |
| T-16 | TurbineWindEstimate | S0601–S0604 | `wind_source` 正确标注来源 |
| T-17 | Scheduler_to_Bridge 快照 | S0701–S0710 | 10 字段齐全且与 CommandStore 一致 |
| T-18 | Bridge_Ack | S0901–S0904 | seq 回读 + `bridge_status_flags` |
| T-19 | S10 状态帧字段 | S1001–S1017 | `ct_hat`/`estimated_wind_mps`/status_flags 有效 |
| T-20 | `Upstream_Signal_ID` 追溯 | 跨两跳 | S0105→S0705→S0803 同业务值可追 |

### 2.3 系统级仿真类（**待 18:17:32 后执行**）

| # | 测试 | 命令 | 判据 |
|---|------|------|------|
| T-21 | 300s 基线回归 | `regress_300s.py` | 与 4366 kW 基线一致（ICD v1.5 后需重建基线） |
| T-22 | 诱导通道 300s | `_test_induction_only.py` | T1 a=0.20 → -15.8% ±2% |
| T-23 | 偏航通道 300s | `_test_yaw_only.py` | T1 +10° → 同 run 惩罚 -2.8%~-8.1% |
| T-24 | 多机隔离 300s | `verify_multiturbine_isolation.py` | T1/T2/T3 独立生效，零串扰 |
| T-25 | Bridge 模式 300s | `verify_bridge_modes.py` | Bridge vs Legacy 偏差 <0.1% |
| T-26 | 全机 3 台回读 | §5.1 要求 | **T1/T2/T3 三台均通过**，非任一台 |
| T-27 | Modbus 通道 | `bench_modbus_throughput.py` | 新字段映射正确 |

### 2.4 遗留问题（上一轮报告）

| # | 问题 | 状态 |
|---|------|------|
| T-28 | **T1 在 multi 场景 t≈237s 失速**（TSR→2.0，BEM 关闭，P→0） | 已记录为已知限制，**未定位根因** |
| T-29 | 失速根因分析与缓解（阵风检测/抗失速/变化率限制） | 未开始 |

---

## 3. 分阶段执行计划

### Phase A —— 恢复编译（阻塞解除）
1. `runtime.c`：`FcrRoscoFastState`→`FcrRoscoRawState`；`rotor_thrust_n`→`rotor_aero_thrust_n`；`FarmStateFrame.sim_time_s`→`state_sample_sim_time_s`；`nacelle_heading_rad`→`nacelle_heading_actual_rad`；`frame_seq`→`state_frame_seq`
2. `bridge_runtime_adapter.{c,h}`：类型改名
3. `rosco_api.c`：setpoint 新字段 + `fcr_rosco_publish_ack`
4. `state_aggregator.c`：新字段填充（S1004–S1017）
5. `farm_protocol.py`：**修正 assert 为 88/5664**
6. 编译通过 → 进入 Phase B

### Phase B —— 桥接与离线层对齐
7. `fcr_bridge_core.{c,h}`：`power_ratio/speed_ref_ratio/torque_limit_ratio/min_pitch_rad` → `r_pitch/r_speed/r_torque/min_pitch_ref_rad`；新增 `collective_pitch_offset_rad`
8. `yaw_adapter.c`：保留 legacy delta 适配，但正式通道走绝对目标
9. `offline_fastfarm_provider.c`：S0203–S0210 字段对齐；**AWAE 合成注入**
10. `rt_snapshot.{c,h}`、`transport/modbus/*`、`config/modbus_map.yaml` 对齐

### Phase C —— AWAE 接口落地（保留接口 + 外部注入）
11. `awae/awae_point_frame.h` 或新头：对齐 S0401–S0407
12. `observation_manager.{c,h}`：frame_seq/sample_sim_time_s/quality_flags/catalog_version
13. `wind_mapper.{c,h}`：输出接入 S06 `FcrTurbineWindEstimate`
14. 二跳发布链路 S12/S13（离线由 Provider 实现）
15. 外部注入 API（测试用）：`fcr_offline_provider_inject_awae_catalog/observation`

### Phase D —— 单元测试与 ABI
16. 更新 17 个单测文件中的旧名
17. `test_abi.c` 锁 88/5664 + 新偏移
18. 新增 T-05…T-20 用例
19. `ctest` 全绿

### Phase E —— 系统级仿真（**18:17:32 之后**）
20. T-21 重建 300s 基线
21. T-22…T-25 通道/隔离/Bridge 回归
22. T-26 三台全机回读
23. T-27 Modbus 通道

### Phase F —— 遗留问题与收尾
24. T-28/T-29 失速根因分析
25. 清理临时脚本（`_analyze_multi.py`、`_baseline_diag.py`、`_check_bts.py`、`_full_diag.py`、`_yaw_diag.py`）
26. 文档更新（`ICD_V1_5_IMPLEMENTATION_PLAN.md` → 本计划；`ROSCO_INTEGRATION.md`、`ZMQ_PROTOCOL.md`）
27. 提交

---

## 4. 验收门禁（ICD v1.5 §5.3）

| # | 门禁 | 验证方式 |
|---|------|----------|
| G-1 | 每条 CSV 的 Sender/Receiver 唯一；每组固定一对 | 静态检查脚本 |
| G-2 | v1.4 全部 Signal_ID 保留；S07(8)/S12(7)/S13(3) 不冲突 | CSV diff |
| G-3 | `Upstream_Signal_ID` 可跨两跳追溯 | T-20 |
| G-4 | `command_basis_sim_time_s` 与 `bridge_step_sim_time_s` 不混用；AWAE 源时间不被墙钟替换 | T-12 / T-14 |
| G-5 | 各机绝对偏航/诱导独立执行、独立回读、TTL 回退 | T-09/T-10/T-11/T-26 |
| G-6 | 不产生 Modbus 地址约束 | 文档确认 |

---

## 5. 约束与排期

- **仿真执行窗口**：按用户指示，重型 FAST.Farm 300s 仿真（Phase E）在 **18:17:32 之后**启动；
  在此之前仅做编译、单元测试（轻量）与代码改造。
- **每完成一次修改跑一次 300s 仿真**：Phase E 起对每次实质修改执行 T-21 回归。
- **离线不可提供量**：AWAE（S04/S05/S12/S13）与风估计（S06）在 Windows 离线环境由
  **外部注入**，接口字段一律保留，禁止因无数据源而删除。
- **协议版本**：`FCR_PROTOCOL_VERSION = 2`，v1 兼容宏保留。

---

## 6. 关键风险

| 风险 | 影响 | 缓解 |
|------|------|------|
| 结构体尺寸变更导致 ABI 断裂 | ZMQ 帧校验全拒（WIN-05 曾发生） | T-02/T-03/T-04 先绿再跑仿真 |
| `FcrRoscoRawState` 改名遗漏 | 编译失败/静默字段错位 | 全仓 grep + ABI 断言 |
| AWAE 注入数据与真实量纲不一致 | 测试误判 | 注入接口做量纲校验 |
| T1 失速未定位 | 多机场景数据不可信 | T-28 单独调查 |
| Modbus 映射滞后 | RT 侧字段缺失 | Phase B 对齐或显式标注延期 |
