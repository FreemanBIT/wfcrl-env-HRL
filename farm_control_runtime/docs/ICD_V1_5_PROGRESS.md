# ICD v1.5 迁移进度（farm_control_runtime）

> 本文件记录 ICD v1.5 迁移的实际完成状态、验证证据与剩余工作。
> 计划与信号清单见 `ICD_V1_5_TASK_PLAN.md` / `ICD_V1_5_IMPLEMENTATION_PLAN.md` /
> `ICD_V1_5_HANDOFF.md`。本文件只记录"事实状态"。

最近更新：Phase A / B / F 已完成并提交（commit `02f349f`、`5f1c187`、`c4fe9a6`）。

---

## 1. 当前验证状态（全部为实测）

| 验证项 | 命令 | 结果 |
|---|---|---|
| C 单元测试 | `farm_control_runtime/tools/_build_test.bat && test_runner.exe` | **68/68 PASS** |
| Python 协议测试 | `.venv\Scripts\python.exe farm_control_runtime\tests\integration\test_icd_v15_abi_python.py` | **7/7 PASS** |
| ABI 交叉校验 | `.venv\Scripts\python.exe farm_control_runtime\tools\check_abi.py` | **PASS**（C 尺寸 + 64/64 Python↔C 字段偏移） |
| 集成测试（FAST.Farm 闭环） | `pytest farm_control_runtime\tests\integration -q` | **21/21 PASS** |
| ROSCO DLL | `.venv\Scripts\python.exe wfcrl\simulators\fastfarm\src\_compile.py` | **OK**（2349.1 KB） |

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

### Phase F — 清理与文档（commit `c4fe9a6`）
- 删除 `tools/_test_yaw_state.c`（引用已删除的 `yaw_delta_rad`/`yaw_enable`）。
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

---

## 4. 剩余工作

### Phase C — AWAE S04/S05/S12/S13（未开始）
- 类型已在 `include/fcr_awae_types.h` 定义；`awae/observation_manager.*`、
  `awae/wind_mapper.*` 已存在。
- **缺**：`FcrFlowState` → `FcrAWAEObservationFrame`(S0501-S0507) /
  `FcrAWAECatalogFrame`(S0401-S0403) 的构造，以及把它们送到 RCP 的通道（S12/S13）。
- 注意：`FcrTransportOps` 目前只有 `mb_poll_command` / `mb_publish_state`，
  要发布 AWAE 帧需要**扩展传输接口**（Modbus 侧地址区已预留 `0x2000`）。
  这是一次接口设计变更，建议单独一个提交完成。
- 离线环境下 AWAE 数据必须由 Provider 注入
  （`offline/provider/offline_fastfarm_provider.c` 已有
  `fcr_offline_provider_generate_synthetic_flow`，WIN-13）。

### Phase D — 合同测试 T-05…T-20（部分完成）
- 已完成：T-04/T-12 的 Python 侧（`tests/integration/test_icd_v15_abi_python.py`，7 例）。
- 待补：T-05…T-11、T-13…T-20（AWAE、S06/S07/S09 契约、超时/回退等）。

### Phase E — 300 s FAST.Farm 系统回归（未执行）
- 计划要求 18:17（本地）之后运行；本次会话未到该窗口。
- 关注项：T-21 诱导 -15.8%±2%、T-22 yaw -5.2%（cos³）、T-25 bridge/legacy 差 <0.1%、
  T-28 多机场景 T1 在 t≈237 s 失速（独立调查项）。

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
