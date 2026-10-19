# ICD v1.5 迁移进度（farm_control_runtime）

> 本文件记录 ICD v1.5 迁移的实际完成状态、验证证据与剩余工作。
> 计划与信号清单见 `ICD_V1_5_TASK_PLAN.md` / `ICD_V1_5_IMPLEMENTATION_PLAN.md` /
> `ICD_V1_5_HANDOFF.md`。本文件只记录"事实状态"。

最近更新：Phase A / B / F 已完成并提交（commit `02f349f`、`5f1c187`、`c4fe9a6`）；
Phase C 发布链路（Modbus 侧）已完成。

---

## 1. 当前验证状态（全部为实测）

| 验证项 | 命令 | 结果 |
|---|---|---|
| C 单元测试 | `farm_control_runtime/tools/_build_test.bat && test_runner.exe` | **73/73 PASS** |
| Python 协议测试 | `.venv\Scripts\python.exe farm_control_runtime\tests\integration\test_icd_v15_abi_python.py` | **7/7 PASS** |
| ABI 交叉校验 | `.venv\Scripts\python.exe farm_control_runtime\tools\check_abi.py` | **PASS**（C 尺寸 + 64/64 Python↔C 字段偏移） |
| 集成测试（FAST.Farm 闭环） | `pytest farm_control_runtime\tests\integration -q` | **21/21 PASS** |
| ROSCO DLL | `.venv\Scripts\python.exe wfcrl\simulators\fastfarm\src\_compile.py` | **OK**（2351.7 KB，含 Phase C 模块） |
| **T-21** 300 s 基线回归 | `.venv\Scripts\python.exe -m induction_vs_yaw_study.run.regress_300s` | **PASS**：`mean_farm_power=4364 kW`（基线 4366 kW，偏差 **0.05%**），wall 162.5 s，steps=100 |

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

### Phase E — 系统回归（**T-21 已通过**）

- **T-21（300 s 基线回归）PASS**：`mean_farm_power = 4364 kW`，与基线 **4366 kW**
  偏差 **0.05%**；wall 162.5 s，100 个控制步全部走完。
  意义：Phase A/B/F 的改动（含诱导配置字段错位修复、G-4 时间基准修复、
  Modbus 重排）**没有扰动基线**。
- 该次运行使用 Phase C **之前**的 DLL，因此是对 Phase A/B/F 的干净验证。
- 其余 T-22…T-27 仍待跑（见 §4）。
- ⚠ **操作注意**：首次运行曾卡死（FAST.Farm 已在 303 s 正常结束，但 replay
  进程空转不再推进、不产出 CSV）。同时并行跑 gcc 构建/测试会与其争抢 CPU，
  疑似导致 replay 跟不上仿真步进而卡在等待。**跑 300 s 回归时不要并行做重编译**；
  且**务必把 stdout 重定向到文件**（管道 `| Select-Object` 会缓冲，看不到
  Python 侧诊断输出）。

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

### Phase C — AWAE S04/S05/S12/S13（**部分完成**）
- **已完成**：帧组装（`awae_frame_builder`）+ Transport 扩展（`publish_awae`）+
  Modbus 通道实现 + runtime 低速步上送 + 离线可选注入 + 单测（见 §2）。
- **待完成（ZMQ 侧）**：Python RCP 目前只通过 ZMQ 收 `FarmStateFrame`；
  S12/S13 要真正到达 Python RCP，需要在 ZMQ 上再承载 AWAE 观测/目录帧
  （新消息类型或独立 socket），并在 `wfcrl/transport/farm_protocol.py` 增加对应
  镜像与解析。这是协议扩展，建议单独一次提交。
- **待完成（合同测试）**：AWAE 相关契约用例（见 Phase D）。
- 离线环境下 AWAE 数据必须由 Provider 注入
  （`offline/provider/offline_fastfarm_provider.c` 的
  `FCR_CFG_AWAE_SYNTH_POINTS`，WIN-13）。

### Phase D — 合同测试 T-05…T-20（部分完成）
- 已完成：T-04/T-12 的 Python 侧（`tests/integration/test_icd_v15_abi_python.py`，7 例）。
- 待补：T-05…T-11、T-13…T-20（AWAE、S06/S07/S09 契约、超时/回退等）。

### Phase E — 300 s FAST.Farm 系统回归（**T-21 完成，其余待跑**）
- **已完成**：T-21 基线回归 PASS（4364 kW vs 基线 4366 kW，偏差 0.05%）。
- **待跑**（命令见任务计划 §2.3）：
  - T-22 诱导通道 300 s：`_test_induction_only.py`，判据 T1 a=0.20 → **-15.8% ±2%**
  - T-23 偏航通道 300 s：`_test_yaw_only.py`，判据 T1 +10° → 同 run 惩罚 **-2.8%~-8.1%**
  - T-24 多机隔离 300 s：`verify_multiturbine_isolation.py`
  - T-25 Bridge 模式 300 s：`verify_bridge_modes.py`，判据 Bridge vs Legacy **<0.1%**
  - T-26 全机 3 台回读、T-27 Modbus 通道吞吐
- **T-28**：多机场景 T1 在 t≈237 s 失速（TSR→2.0，BEM 关闭，P→0），
  已记录为**已知限制、根因未定位**；T-29 缓解措施未开始。
  本次 T-21 运行的日志中同样可见大量
  `wake plane ... has overtaken wake plane ... Offending wake plane removed`
  与 `wake plane #N ... has passed the ... boundary` 警告，与该问题同源，
  是后续调查的切入点。
- ⚠ 跑回归时勿并行重编译，并把 stdout 重定向到文件（见 §2 Phase E 的操作注意）。

### 其他已知问题（未处理，需决策）
- `config/induction_mapping.yaml` 的 `a_max=0.45` 与 C 侧默认 `a_max=1/3` 不一致；
  该 YAML 目前**未被加载**，运行时以 C 默认为准。若要真正启用配置加载需先统一。
- `wfcrl/engine/fastfarm_zmq.py`、`induction_vs_yaw_study/*`、
  `closed-loop-control/*` 中仍有直接调用 `send_yaw_delta`（CW+ 增量）的脚本；
  换算已在 `FarmZmqClient.send_yaw_delta` 内部完成，故其动作语义保持不变。
- 工作区 `induction_vs_yaw_study/run/_*.py` 5 个调试脚本为未跟踪文件
  （在 `farm_control_runtime` 之外），未做处理。

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
