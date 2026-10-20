# ICD v1.5 迁移进度（farm_control_runtime）

> 本文件记录 ICD v1.5 迁移的实际完成状态、验证证据与剩余工作。
> 计划与信号清单见 `ICD_V1_5_TASK_PLAN.md` / `ICD_V1_5_IMPLEMENTATION_PLAN.md` /
> `ICD_V1_5_HANDOFF.md`。本文件只记录"事实状态"。

最近更新：Phase A / B / F 已完成并提交（commit `02f349f`、`5f1c187`、`c4fe9a6`）；
Phase C 发布链路（Modbus 侧）已完成；代码审查 W-08（ZMQ AWAE 二跳）/ W-09（容量统一）
已完成并提交；W-11（1s 诱导跟踪 + t≈237s 失稳根因）已定位。

---

## 1. 当前验证状态（全部为实测）

| 验证项 | 命令 | 结果 |
|---|---|---|
| C 单元测试 | `farm_control_runtime/tools/_build_test.bat && test_runner.exe` | **80/80 PASS**（含 W-09 容量守卫 + T-19/T-20 契约测试） |
| Python 协议测试 | `.venv\Scripts\python.exe farm_control_runtime\tests\integration\test_icd_v15_abi_python.py` | **12/12 PASS**（含 T-05 跨语言往返） |
| ABI 交叉校验 | `.venv\Scripts\python.exe farm_control_runtime\tools\check_abi.py` | **PASS**（C 尺寸 + 82/82 Python↔C 字段偏移，含 AWAE 帧） |
| 集成测试（FAST.Farm 闭环） | `pytest farm_control_runtime\tests\integration -q` | **26/26 PASS**（含新增 `test_awae_zmq_link.py`；libzmq teardown abort 为已知环境问题，逐文件 dots 为记录） |
| **G-1/G-2/G-3 静态门禁** | `.venv\Scripts\python.exe farm_control_runtime\tools\check_icd_gates.py` | **PASS**（88 信号 / 13 组 / 每组固定一对 / v1.4 保留 / S12/S13 不冲突 / Upstream_Signal_ID 链可追） |
| ROSCO DLL | `.venv\Scripts\python.exe wfcrl\simulators\fastfarm\src\_compile.py` | **OK**（2359.4 KB，含 W-08/W-09 模块） |
| **T-21** 300 s 基线回归 | `.venv\Scripts\python.exe -m induction_vs_yaw_study.run.regress_300s` | **PASS**：`mean_farm_power=4366 kW`（基线 4366 kW，偏差 **0.00%**）——W-08/W-09 改动无扰动 |
| **T-22** 诱导通道 300 s | `.venv\Scripts\python.exe -m induction_vs_yaw_study.run._test_induction_only` | **PASS**（见 §2，历史记录） |
| **T-23** 偏航通道 300 s | `.venv\Scripts\python.exe -m induction_vs_yaw_study.run._test_yaw_same_run` | **✅ 同 run 复核完成**：命令执行/隔离/heading 收敛全部正确；-27.1% 为 W-11 失稳污染，非偏航缺陷（见 §2） |
| **W-08** AWAE ZMQ 二跳 | `pytest farm_control_runtime\tests\integration\test_awae_zmq_link.py` | **PASS**（S12/S13 帧到达 Python RCP；S0502 仿真时刻无墙钟泄漏；SIMULATED 质量位） |
| **W-09** 容量统一 | `tools/check_abi.py` + 单测 `abi_capacity_unified` | **PASS**（bridge 128→64、legacy AWAE 1024→64，版本化容量宏） |
| **W-11** 诱导跟踪 | `python -m induction_vs_yaw_study.run.measure_induction_tracking` | **PASS**：T1 a=0.20 → **-35.6%**，seq 5 s 确认；t≈237s 失稳根因已定位（见 §2/§4） |
| **T-25** Bridge 模式 | `python -m induction_vs_yaw_study.run.verify_bridge_modes` | **PASS**（离线 ctypes，Mode0/1/2 + Ct + yaw + safety） |
| **T-27** Modbus 吞吐 | `python -m induction_vs_yaw_study.run.bench_modbus_throughput` | **PASS**（30/100 台解析模型，benchmark.json 已更新） |

> 集成测试建议**逐文件**运行：libzmq 偶发在 teardown 触发
> `Assertion failed: Socket operation on non-socket` 会 abort 进程，导致
> pytest 来不及打印汇总。逐文件跑可保证单文件结果不丢失。

---

## 2. 已完成

### Phase A — 恢复编译/测试/ABI（commit `02f349f`）
- 9 个单元测试文件共 102 个 v1.5 改名编译错误全部修复。
- `test_abi.c` 冻结 v1.5 尺寸：`FcrTurbineCommand`=88、`FarmCommandFrame`=5664、
  `FcrRoscoRawState`=232、`FcrFarmTurbineState`=400、`FarmStateFrame`=25664 等。
- `tools/check_abi.py` 重写：C 尺寸探针 + Python 镜像尺寸 + **64 个字段偏移**逐一交叉校验。
- Python 协议镜像重建（`wfcrl/transport/farm_protocol.py`、
  `wfcrl/transport/farm_zmq_client.py`）：新增 `FcrSignalStats` /
  `FcrFarmTurbineState` / `FarmStateFrame` 镜像与 yaw 增量↔绝对换算。

### Phase B — S06/S07 接线 + Modbus 对齐 + G-4 修正（commit `5f1c187`）
- **S07**：`FcrBridgeInput` 成为 bridge 的**唯一输入路径**
  （`fcr_bridge_adapter_input_from_state` / `cmd_from_input`，`runtime.c` 已切换）。
- **S06**：`fcr_bridge_adapter_meas_apply_wind_estimate()` 注入风速估计；
  `FcrBridgeMeas` 增加 `wind_source`/`wind_quality_flags`；
  `FcrStateStore` 增加每机 `wind_est` 槽与 set/get。
- **S09**：`fcr_rosco_publish_ack` / `fcr_rosco_get_ack` 已有并有单测覆盖。
- Modbus：`modbus_map.yaml` 重写对齐寄存器头；`MB_ST_SHAFT_TORQUE_NM`
  → `MB_ST_TOWER_BASE_FA_MOM`（原槽被填入塔基弯矩却标为轴扭矩）。

### Phase C — AWAE S04/S05/S12/S13 发布链路（Modbus 侧已完成）

- 新增 `awae/awae_frame_builder.{c,h}`：把**注入的**静态测点目录（S0401-S0403）
  与 `FcrFlowState`（S0501-S0507）组装为 ICD v1.5 帧。纯函数式，不做控制算法。
- `include/fcr_transport.h` 扩展：`FcrTransportOps` **追加** `publish_awae`
  （GG-12/GG-13）。追加在末尾，既有的 `{poll, publish}` 位置初始化会把新字段
  零初始化为 NULL，`fcr_transport_publish_awae()` 对其为 NULL 时安全返回 0
  —— **向后兼容，不改动既有 Transport**。
- `transport/modbus/modbus_transport.c` 实现 `mb_publish_awae`（写入 0x3000 区）；
  `src/runtime.c::fcr_runtime_step_low` 在每次低速步构造并上送观测+目录帧。
- `FcrStateStore` 增加 `awae_cat` / `awae_extra_quality_flags` 与 set 接口。
- `offline/provider/offline_fastfarm_provider.c`：新增**可选**合成测点注入
  （env `FCR_CFG_AWAE_SYNTH_POINTS`，默认 0=关闭），带 `FCR_AWAE_QUALITY_SIMULATED`
  质量位。**默认关闭是刻意的**：注入观察点会改变 WindMapper 映射结果，从而扰动
  既有 300 s 回归基线。只覆盖观察点数组，不动 `disk_*` 盘面量（来自真实 hub 风）。
- 新增质量位定义 `FCR_AWAE_QUALITY_OK/STALE/SIMULATED/LOW_CONF`（S0506）。
- 测试：`test_awae.c` +2（目录/观测组装）、`test_modbus.c` +2（AWAE 区布局守卫、
  `publish_awae` 端到端寄存器校验）、`test_bridge_core.c` +1（T-07 mode 3）。

**尚未做**：ZMQ 侧尚未承载 AWAE 帧（见 §4）。

### Phase E — 系统回归（**T-21 / T-22 已通过**）

- **T-21（300 s 基线回归）PASS**：`mean_farm_power = 4364 kW`，与基线 **4366 kW**
  偏差 **0.05%**；wall 162.5 s，100 个控制步全部走完。
  意义：Phase A/B/F 的改动（含诱导配置字段错位修复、G-4 时间基准修复、
  Modbus 重排）**没有扰动基线**。
- **T-22（诱导通道 300 s）PASS**：T1 `a=0.20` → **-15.14%**，落在判据
  **-15.8% ±2%**（即 -17.8%~-13.8%）之内。逐机结果：

  | 机位 | 基线 (kW) | a=0.20 (kW) | Δ |
  |---|---|---|---|
  | T1（受控） | 1764.2 | 1497.1 | **-15.14%** |
  | T2 | 1273.0 | 1411.5 | +10.88% |
  | T3 | 1327.1 | 1326.1 | -0.08% |

  T2 上升符合物理（T1 降载 → T1 尾流变弱 → 下游 T2 来流增强）；
  T3 基本不变（离 T1 最远）。同时 `induction_seq_applied` 只有 T1=1、
  T2/T3=0，**逐机通道隔离成立**。
- 该次 T-21 运行使用 Phase C **之前**的 DLL，因此是对 Phase A/B/F 的干净验证；
  T-22 使用重建后的 DLL。
- **方法学注意**：T-22 的 -15.14% 是**跨 run** 比较（T-21 基线 run vs T-22 run）。
  交接文档指出 TI=10% 下跨 run 绝对量比较不可靠；本次降载是**确定性控制作用**
  且 T3 几乎不变（-0.08%），说明该比较在此场景仍有效，但若要严格结论，
  应改用**同一 run 内**先基线后加命令的方法复核。
- **T-23（偏航通道 300 s）✅ 同 run 复核完成**：改用 **run 内先基线后 +10°** 的
  同 run 方法（`run/_test_yaw_same_run.py`，T1 greedy 至 t=120s，随后下发 +10°，
  全程记录 heading 逐步序列 + 全部机组功率时序）。结论：
  - **命令执行正确**：T1 heading 逐步收敛到目标 -10.0°（=SCADA +10°），
    settle_frac=93%（阶跃后 93% 时间在 ±5° 内），post-step max_dev≈7.5°，
    **无振荡**（range≈7.5° < 12°）。`yaw_seq_applied` 仅 T1=1、T2/T3=0。
    **旧 -27.1% 的"偏航振荡"解释被排除**。
  - **-27.1% 的真相**：同 run 数据显示 **无命令的 T2/T3 在 t≈185-215s 后也跌
    50-70%**（T2 2209→455-940 kW、T3 2016→625-1035 kW），而 T1 自身功率始终
    健康（1056-2310 kW）。即 after 窗口被**全场下游机功率退化**污染——该退化
    与命令无关（见下 W-11 根因）。因此 -27.1% 不是偏航通道缺陷，而是
    **FAST.Farm wake 动力学数值问题**对"跨 run 平均"的污染。
  - **理论偏航惩罚** cos³(10°)≈-4.5% 与历史同 BTS 逐时间步报告（-5.2%）一致；
    偏航通道的**物理方向、命令执行、通道隔离**均已验证。
- **W-11（1s 诱导跟踪 + t≈237s 失稳根因）**：
  - **1s 诱导跟踪 PASS**：`measure_induction_tracking.py` 同 run 阶跃（t=120s 下发
    a=0.20）→ T1 derate **-35.6%**（判据 ≤ -6%），`induction_seq_applied` 在
    **5 s** 内确认（低步 DT=3s，即 ~2 控制步，1s 通道更新率下命令被实际采纳并
    回读，非仅下发成功）。
  - **t≈237s 失稳根因已定位**：日志（基线 T-21 与偏航 T-23 **均**出现，162 条）
    显示 `Turbine N wake plane #M ... has overtaken wake plane #M+1 ... Offending
    wake plane removed. Reduce f_c to prevent planes from overtaking`。即
    FAST.Farm 的 wake 平面因**数值 advection 互相超越被移除**（f_c=0.105651 Hz
    过高所致），污染下游机来流 → 下游机功率退化 → 极端工况（强阵风 + 降载）
    下某机进入低 TSR → BEM 关闭 → P→0。**这是 FAST.Farm 仿真配置/数值限制，
    与 farm_control_runtime 控制代码无关**（基线无命令也出现）。缓解方向：
    调低 .fstf 的 `f_c`（FAST.Farm 官方建议），属研究侧配置，不影响 runtime ABI。
- 其余 T-24…T-27：见 §4 更新（T-24 多机隔离运行中/已完成）。
- ⚠ **操作注意**：首次 T-21 运行曾卡死（FAST.Farm 已在 303 s 正常结束，但 replay
  进程空转不再推进、不产出 CSV）。并行跑 gcc 构建/测试会与其争抢 CPU，
  疑似导致 replay 跟不上仿真步进。实测：无并发负载时 303 s 仿真仅需 **80 s** wall，
  有并发构建时需 162 s。**跑回归时不要并行重编译**；且**务必把 stdout 重定向到
  文件**（管道 `| Select-Object` 会缓冲，看不到 Python 侧诊断输出）。

### Phase F — 清理与文档（commit `c4fe9a6`）- 删除 `tools/_test_yaw_state.c`（引用已删除的 `yaw_delta_rad`/`yaw_enable`）。
- 修正 `ZMQ_PROTOCOL.md`、`ROSCO_INTEGRATION.md`、`OFFLINE_PROVIDER.md`、
  `config/induction_mapping.yaml`、`FarmControlInterface.f90`（注释）、
  `offline_fastfarm_provider.c`（注释）、`fcr_protocol.h`（注释）中的 v1.4 旧名。

---

## 3. 本次迁移中发现并修复的真实缺陷

1. **`FcrInductionMapConfig` 字段错位（静默、严重）**
   v1.5 头文件把 `power_cap` 移到第 5 位，而 `induction_supervisor.c` 仍用
   **位置初始化**，导致 `power_cap=0.55 / min_pitch_gain=0.30 / min_pitch_max=1.0`。
   `r_pitch` 被钳到 0.55 → 诱导通道过度降载，会直接破坏 300 s 诱导回归（-15.8%）。
   已恢复字段顺序 + 改用**指定初始化器**，并加 `config_wiring` 回归守卫。

2. **Python 状态帧镜像不兼容（RCP 完全收不到状态）**
   `farm_zmq_client.py` 硬编码 v1.4 几何（56/360/23096），**每一帧状态都被丢弃**；
   `wfcrl/engine/fastfarm_zmq.py` 另有一份独立 v1.4 镜像，把 `n_turbines`
   读在偏移 36（v1.5 为 44）→ 恒读到 0 → `turbines` 恒为空 →
   集成测试全部 `IndexError`。两处均已改为复用权威镜像
   （`farm_protocol.state_frame_from_bytes`）。
   集成测试 **7/21 → 21/21**。

3. **G-4 违背：墙钟被写入仿真时间字段**
   `set_yaw_cmd`/`set_induction_cmd` 在未提供时把 `time.time()`（~1.77e9）写入
   `command_basis_sim_time_s`(S0102)。而 `command_store.c` 用
   `sim_time_s - command_basis_sim_time_s > stale_threshold_s` 判定"命令超龄"，
   墙钟使该差值恒为巨大负数 → **超龄保护永不触发**。已改为由
   `FarmZmqClient` 用最近状态帧的 `state_sample_sim_time_s` 补齐。

4. **Modbus 状态区完整性尾部越界**
   DR-8 的 u64 `timestamp` 原在偏移 125，跨越本机 128 寄存器块（125..128）并与
   CRC16/SEQ 槽重叠。ICD v1.5 §5.3 G-6 明确不约束 Modbus 地址，已重排到 102..112。
   这些槽目前**尚未实现**。

5. **集成用例断言前提错误**（非产品缺陷，但会掩盖真实回归）
   - `test_offline_induction_loop.py` 把 `gen_power_w` 当嵌套结构（实为标量）；
   - 同文件跨机组"a 越小功率越低"断言不成立：取样在 seq 采纳后立即进行，
     降载尚未收敛（a=0.20 与 a=0.30 实测仅差 0.8%，预期 ~15%），且三台位置/
     尾流不同，`a>a_max=1/3` 还会整体 fallback。已改为只断言该用例真正负责的
     **多机通道隔离**，并指向真正负责降载方向的用例。

6. **Modbus AWAE 缓冲与状态区地址冲突（Phase C 发现）**
   AWAE 缓冲原定 0x2000，而状态区是 0x1000..0x2FFF（64 机 × 128）——
   0x2000 恰是状态区第 33 机起始，≥33 机时 AWAE 会被状态写入覆盖。
   G-6 不约束 Modbus 地址，已移到 0x3000（寄存器区随之扩到 0x4000），
   并加 `awae_area_layout` 守卫断言"不与状态区重叠"。

7. **Modbus AWAE 单点布局把 f64 当成 2 寄存器（Phase C 发现，隐蔽）**
   原布局写 `position_xyz: 1 (3×f64, 1..6)` / `wind_xyz: 7`，即按 **2 寄存器/f64**
   计算，而 `mb_put_double` 实际写 **4** 个寄存器。若照此实现，三个坐标会
   互相覆盖：实测 `x` 读回 `10.00000191340223`（bits `4024000040340000`，
   低 32 位残留了下一个坐标的高位）而非 `10.0`。
   已按 f64=4 寄存器重排（1+12+12+4+1+1+1=32，恰好填满 `MB_AWAE_PER_POINT`），
   并**删除 `wind_speed`/`wind_direction`** —— 二者非 ICD v1.5 信号
   （S0505 `wind_uvw` 已含矢量），保留会使单点超过 32 寄存器。
   注：命令区/状态区的 f64/u64 步长本来就是 4，无此问题，仅 AWAE 单点布局有。

---

## 4. 剩余工作

### Phase C — AWAE S04/S05/S12/S13（**已完成**）
- **已完成**：帧组装（`awae_frame_builder`）+ Transport 扩展（`publish_awae`）+
  Modbus 通道实现 + runtime 低速步上送 + 离线可选注入 + 单测（见 §2）。
- **W-08（ZMQ 侧 AWAE 二跳）已完成**：`zmq_transport.cpp` 新增 `zmq_publish_awae`，
  在 state PUB socket 上按冻结尺寸发布 `FcrAWAEObservationFrame`(3096 B) +
  `FcrAWAECatalogFrame`(2584 B)；`wfcrl/transport/farm_protocol.py` 增加对应
  ctypes 镜像/尺寸断言/解析；`farm_zmq_client.py` 接收循环按尺寸分派并缓存，
  新增 `get_awae_observation()` / `get_awae_catalog()`；`check_abi.py` 将 AWAE 帧
  纳入逐字段偏移校验（64→82 字段）；集成测试 `test_awae_zmq_link.py` 端到端
  验证 S12/S13 到达 Python RCP（含 G-4 S0502 无墙钟泄漏、SIMULATED 质量位）。
- **待完成（合同测试）**：AWAE 相关契约用例 T-13/T-14/T-15（见 Phase D）。
- 离线环境下 AWAE 数据必须由 Provider 注入
  （`offline/provider/offline_fastfarm_provider.c` 的
  `FCR_CFG_AWAE_SYNTH_POINTS`，WIN-13）。

### Phase D — 合同测试 T-05…T-20（**全部覆盖**）
- **Python 协议侧**（`tests/integration/test_icd_v15_abi_python.py`，12 例）：
  T-04/T-12（时间戳不混用）＋ W-08 AWAE 帧镜像 4 例 ＋ **T-05**（命令帧
  Python→C→回读全链路：Python 构造 → 编译 C 探针解析回读 → 逐字段一致）。
- **C 单测侧**（`tests/unit/`）：
  - T-06（control_enable=0 回退）：`command_store::control_disable_fallback`、
    `rosco_api::control_disable_fallback`
  - T-07（control_mode 枚举，3 必须 UNSUPPORTED）：`bridge_core::mode3_unsupported`
  - T-08（requested_effective_sim_time_s 排程）：`command_store::requested_effective_time`
  - T-09/T-10（yaw/induction 独立序号）：`command_store::channels_independent`、
    `yaw_action_manager::induction_does_not_affect_yaw`
  - T-11（TTL 到期回退）：`command_store::ttl_expiry`、
    `yaw_action_manager::ttl_expiry_rollback`、`bridge_core::safety`
  - T-13（AWAE Catalog 注入契约）：`awae::awae_catalog_build`（point_catalog_version 保持）
  - T-14（AWAE Observation 注入契约）：`awae::awae_observation_build`（S0502 仿真时刻 +
    quality 透传）
  - T-16（TurbineWindEstimate S06 wind_source 标注）：`bridge_adapter::wind_estimate`
  - T-17（Scheduler_to_Bridge 快照 S07 全字段）：`bridge_adapter::input_from_state` +
    `cmd_from_input`
  - T-18（Bridge_Ack seq + flags）：`rosco_api::ack_roundtrip`
  - T-19（S10 新增字段 S1011-S1017）：**`state_aggregator::aggregate_s10_new_fields`**
    （新增，2026-10-20）
  - T-20（Upstream_Signal_ID 跨两跳追溯）：**`bridge_adapter::trace_upstream_signal_id`**
    （新增，2026-10-20）＋ G-3 静态链校验（`tools/check_icd_gates.py`）
- **验收门禁 G-1/G-2/G-3(静态)**：`tools/check_icd_gates.py`（新增，2026-10-20）
  基于 v1.5 SingleHop CSV（已入库 `docs/farm_control/inputs/`）：
  G-1（Signal_ID 唯一 + 每组固定一对 Sender/Receiver）、G-2（v1.4 组保留 +
  S12/S13 不冲突 + 信号数）、G-3（Upstream_Signal_ID 存在性 + 期望链 + T-15
  二跳接收端分离）。**PASS（88 信号 / 13 组）**。

### Phase E — 300 s FAST.Farm 系统回归（**T-21 / T-22 / T-23 / T-25 / T-27 完成**）
- **已完成**：T-21 基线回归 PASS（4366 kW vs 基线 4366 kW，0.00%）；T-22 诱导
  通道 PASS（T1 a=0.20 → -15.14%，判据 -15.8%±2%）；T-23 偏航同 run 复核完成
  （见 §2，命令执行正确，-27.1% 为 W-11 失稳污染）；T-25 Bridge 模式 PASS；
  T-27 Modbus 吞吐 PASS。
- **待跑**：
  - T-24 多机隔离 300 s：`verify_multiturbine_isolation.py`
  - T-26 全机 3 台回读：由 T-24 的 per-turbine 回读覆盖（T1/T2/T3 均回读
    heading + seq + 功率，验收要求"三台均通过"）。
- **T-28（t≈237s 失速）根因已定位**：见 §2 W-11 —— FAST.Farm wake 平面
  互相超越被移除（f_c 过高），非 runtime 缺陷。T-29 缓解（调低 f_c）属研究侧
  配置。
- ⚠ 跑回归时勿并行重编译，并把 stdout 重定向到文件（见 §2 Phase E 的操作注意）。

### 其他已知问题（未处理，需决策）
- `config/induction_mapping.yaml` 的 `a_max` 已统一为 `1/3`（与 C 默认一致，
  决策 2026-10-20）；该 YAML **仍未被加载**，运行时以 C 默认为准
  （env `FCR_BRIDGE_IND_A_MAX/MIN` 可显式覆盖，W-10）。
- `wfcrl/engine/fastfarm_zmq.py`、`induction_vs_yaw_study/*`、
  `closed-loop-control/*` 中调用 `send_yaw_delta`（CW+ 增量）的脚本**已清理**：
  换算在 `FarmZmqClient.send_yaw_delta` 内部完成（动作语义不变，这是被维护的
  便捷 API，非旧字段）；`ZmqFarmBridge._send_yaw`（持有绝对目标）已改走
  `send_yaw_target`（S0108 正式通道），删除绝对→增量往返与相关死代码；
  replay 的 `yaw_delta_deg` 用法因状态帧 heading 不可靠而**刻意保留**。
- 工作区 `induction_vs_yaw_study/run/_*.py` 5 个调试脚本为未跟踪文件
  （在 `farm_control_runtime` 之外），**决策保留**（2026-10-20）。

---

## 5. 复现验证的一键命令

```bat
cd /d D:\HR_Project\wfcrl-env-HRL
farm_control_runtime\tools\_build_test.bat && farm_control_runtime\test_runner.exe
.venv\Scripts\python.exe farm_control_runtime\tests\integration\test_icd_v15_abi_python.py
.venv\Scripts\python.exe farm_control_runtime\tools\check_abi.py
.venv\Scripts\python.exe -m pytest farm_control_runtime\tests\integration -q
.venv\Scripts\python.exe wfcrl\simulators\fastfarm\src\_compile.py
```

---

## 6. 代码审查修复状态（2026-10-19 审查，21 项）

> 审查结论：Windows 主链路可追踪，但 Mode2 有"错误成功回报"、控制器参数保存/回退不完整、
> 状态回传有物理语义错误；RT 侧有跨机组共享状态/锁内通信/快照并发问题。
> 审查按 Windows（W-*）与后续 LinuxRT（R-*）两阶段划分。

### 已修复（14 项，单测 78/78，DLL 2359.4 KB）

| 项 | 修复 |
|---|---|
| **W-12** | ZMQ 接受 `command_basis_sim_time_s >= 0`（0 是合法仿真时刻）；NaN/负值仍拒 |
| **W-03** | S0709 逐机 `control_mode` 成为唯一权威，静态配置仅作越界兜底；Mode3 透传给 core 显式拒绝，不再被 `cfg.bridge_induction_mode` 覆盖 |
| **W-04** | Mode2 无气动表（`aero_n_lam=0`）或反查失败 → **不采纳**：`induction_enable=0`、seq 不入账、置 `0x200`(CT_LOOKUP_INVALID)/`0x100`(UNACHIEVABLE)+`0x4`(FALLBACK)。新增回归测试 `mode2_no_table_fallback` |
| **W-14** | `a>1/3` 不再一概判 UNACHIEVABLE；`1/3<a<1/2` 区可达性由真实 Ct(λ,β) 表判定。新增回归测试 `mode2_a_above_third_with_table`（a=0.35 可达→采纳） |
| 附带 | 修正旧 `step_modes` 测试的隐藏错误：其气动表只填了 1 行、其余行全 0，旧代码靠无条件 `induction_enable=1` 掩盖——这正是 W-04 所指的"错误成功回报"。现已全行填表 |
| **W-01** | ROSCO Fortran 完整保存/恢复 PRC/PS/VS 配置（含 peak-shaving 表）；禁用时不再直接置 0 破坏原配置 |
| **W-02** | 移除 pitch 通道隐式 `VS_ConstPower=1 + VS_RtPwr=nom*r_pitch` 联动；r_pitch 唯一经 PRC_R_Pitch 生效。**T-22 复测 -15.26% 通过**（判据 -15.8%±2%），证明降载来自桨距通道本身 |
| **W-05** | Mode2 表轴严格递增校验 + 维度下限 + 缺风不退化 lambda=0（新码 -5 NO_WIND）。新增回归测试 `mode2_find_beta_robust` |
| **W-06** | 风速来源如实标 OPENFAST + 低置信（不再谎称 ROSCO_WE）；Ct 仅在真实有限值时输出，否则显式 0 |
| **W-07** | CMakeLists + build_windows.ps1 补 `awae_frame_builder.c` 与 include 目录（gcc 路径 77/77 验证；本机无 CMake） |
| **W-10** | env 配置入口 `FCR_BRIDGE_IND_A_MAX/MIN` + 启动日志打印生效 mode/a_max/a_min（YAML 未加载，C 权威 + env 显式覆盖） |
| **W-13** | 每通道独立 basis time（`yaw/ind_basis_sim_time_s`），stale 评估逐通道解耦——偏航 60s 锁存与诱导 1s 更新互不串扰。新增回归测试 `stale_flag_per_channel` |
| **W-08** | ZMQ 侧 AWAE 二跳发布（S12/S13 到 Python RCP）：`zmq_transport.cpp` 新增 `publish_awae`（state socket 按冻结尺寸发布观测 3096 B + 目录 2584 B）；`farm_protocol.py` 新增 AWAE 帧镜像/解析/类型分派；`farm_zmq_client.py` 新增 `get_awae_observation/get_awae_catalog`；`check_abi.py` 纳入 AWAE 字段偏移（82 字段）；集成测试 `test_awae_zmq_link.py` 端到端 PASS |
| **W-09** | 容量统一：`FCR_MAX_TURBINES=64` / `FCR_MAX_OBSERVATION_POINTS=64` 线协议容量冻结不变；引入版本化容量宏（`FCR_CAPACITY_VERSION`、`*_V1`）；bridge `FCR_BRIDGE_MAX_TURBINES` 128→64、legacy AWAE `FCR_AWAE_MAX_POINTS` 1024→64 全部对齐线协议；新增单测 `abi_capacity_unified`；ABI 尺寸零变化（check_abi PASS） |
| **W-11** | 1s 诱导跟踪验收 PASS（derate -35.6%、seq 5s 确认）+ **t≈237s 失稳根因定位**：FAST.Farm wake 平面互相超越被移除（`Reduce f_c to prevent planes from overtaking`，f_c=0.105651 过高），基线无命令也出现（162 条）→ 非 runtime 缺陷；缓解 = 研究侧调低 f_c |

### 待办（剩余 LinuxRT 阶段；Windows 侧审查项已全部完成）

| 阶段 | 项 | 说明 |
|---|---|---|
| R0-R1 | **R-01~R-07** | LinuxRT 阶段：实例隔离、快照原子性、10ms 无锁、容量版本化、Modbus Client、大结构拷贝、供应商调度核/ROSCO 主源码补交。**需 RT 环境，现阶段不做** |
