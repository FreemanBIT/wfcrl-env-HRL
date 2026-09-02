# ROSCO_INTEGRATION — runtime ↔ ROSCO 集成说明（Phase 2）

## 1. 目标

打通 `farm_control_runtime` ↔ ROSCO 的 C/Fortran ABI，实现**无扰接入**：
外部控制默认 disabled（`farm_control_enable=0`）时，ROSCO 原控制行为完全不变。

验收（Phase 2）：改造前后**无外部命令**的短时 case 数值一致（<1e-6 级舍入差异）。

## 2. 文件与责任

| 文件 | 内容 |
|---|---|
| `farm_control_runtime/rosco/FarmControlCBindings.f90` | 纯 `bind(C)` 接口声明（镜像 C ABI 结构） |
| `farm_control_runtime/rosco/FarmControlInterface.f90` | Fortran 语义层：初始化/每 10 ms 步进/发布/查询 |
| `farm_control_runtime/src/rosco_api.c` | C 宿主：DLL 内 runtime 单例 + 锁 + ABI 入口 |
| `wfcrl/simulators/fastfarm/src/DISCON.F90` | 主路径接入（宏 `FCR_FARM_CONTROL` 控制） |

## 3. 部署形态（离线）

`farm_control_runtime` 的 C 核心与 Fortran 桥一起编译进 `DISCON_WT1.dll`：

```text
FAST.Farm_x64_OMP.exe（预编译二进制，不修改）
   └─ ServoDyn → DISCON_WT1.dll
        ├─ ROSCO（Controllers/…）
        ├─ FarmControlInterface.f90（bind(C) 桥）
        └─ farm_control_runtime（C 核心，DLL 内单例）
             ├─ fcr_rosco_read_setpoint   ← ROSCO 每 10 ms 读取
             ├─ fcr_rosco_publish_state   ← ROSCO 每 10 ms 发布
             └─ fcr_rosco_inject_command_frame ← 离线 harness/Transport 注入
```

## 4. 10 ms 调用时序（DISCON 每次调用）

```text
ReadAvrSWAP → (首次)FCR_Init(tid) → SetParameters
   → FCR_Step(tid, Time)   [runtime 高速步：命令 refresh + yaw 锁存 + setpoint 生成]
   → ROSCO 控制计算（VariableSpeed/Pitch/Yaw…，可查询 FCR_* 外部 setpoint）
   → 错误/输出处理
   → FCR_PublishState(tid, LocalVar, avrSWAP)   [发布快速状态到共享内存]
```

约束：

- 10 ms 路径内**无 socket、无文件 I/O**，只有锁保护下的共享内存读写；
- `FCR_Step` 先在 runtime 内完成命令处理，于是控制计算读取的 setpoint 是最新值；
- `FCR_PublishState` 中 `avrSWAP(47/48/42-44)` 为 ROSCO 最终命令（控制计算完成后的输出槽）。

## 5. 外部控制语义（目前冻结）

### 偏航（Phase 3 验证后启用映射）

- Farm Controller 下发 `yaw_delta_rad`（CW+）与 `yaw_seq`；
- runtime 在新 seq 生效时用最新 `NacHeading` 锁存绝对目标 `yaw_target_heading_rad`（OpenFAST 内部约定 CCW+）；
- 重复 seq 幂等；乱序拒绝；
- ROSCO 侧保留 yaw-rate limit、deadband、state machine；目标来源在 Phase 3 切换为 `FarmYawTargetHeading`。

### 诱导（Phase 6 完成后启用映射）

- Farm Controller 下发 `induction_ref` 与 `induction_seq`；
- InductionSupervisor 映射为 `ct_ref/power_ratio/speed_ref_ratio/torque_limit_ratio/min_pitch_rad`；
- 当前 Phase 2 透传 `power_ratio=1, min_pitch=0`（额定语义），`induction_enable` 仍为 0 时不改变 ROSCO。

## 6. 编译

```bash
# 新路径（farm_control_runtime 集成）：默认
python wfcrl/simulators/fastfarm/src/_compile.py

# legacy（等价 baseline，无 FCR）
python wfcrl/simulators/fastfarm/src/_compile.py --mode rosco
```

## 7. ROSCO 原控制路径（本阶段不修改的计算）

| 模块 | 说明 |
|---|---|
| VariableSpeedControl | 转矩-转速曲线（VS_ControlMode=2），写 avrSWAP(47) |
| PitchControl | 变桨 PI，写 avrSWAP(42-45) |
| YawRateControl | 偏航速率状态机（Y_ControlMode=1 时），写 avrSWAP(48)；当前模板 Y_ControlMode=0 |
| StateMachine/SetpointSmoother | 状态机与设定点平滑 |
| WindSpeedEstimator | 风速估计（WE_Mode=0 禁用） |

## 8. Phase 2 验收记录（2026-XX）

- 用例：Row3T（3 机）、8 m/s、270° 西风、TMax=18 s、DT_High=0.01 s、DT_Low=3 s；
- 方法：同一运行目录复制两份，分别部署 legacy DLL（无 FCR 宏）与
  FCR 集成 DLL（_compile.py --mode farmcontrol），删除 controls.txt（无外部命令），
  分别运行 FAST.Farm_x64_OMP.exe，解析输出对比：
  - Case.out（全场 45 通道）：**maxdiff = 0.0**（逐位一致）；
  - Case.T1/T2/T3.outb（每机 25 通道 × 181 行）：**maxdiff = 0.0**（逐位一致）。
- 结论：外部控制 disabled 时，FCR 路径与原 ROSCO 路径数值完全等价，无扰接入成立。

## 9. Phase 3 验收记录（偏航增量动作端到端，2026-XX）

- 用例：Row3T（3 机）、8 m/s、270° 西风、TMax=42 s、DT_High=0.01 s；
  模板配置：ServoDyn YCMode=5（DLL yaw 命令生效）、TYCOn=0；ElastoDyn YawDOF=True；
  DISCON.IN：Y_ControlMode=1（ROSCO yaw 状态机启用）、Y_ErrThresh(2)。
- 命令（共享内存通道注入 runtime，DLL 内线程每 10 ms 轮询）：
  - seq1 t≈8s：+10° CW → 锁存目标 -10°（内部约定）；
  - seq2 t≈19s：-15° CW → 重新锁存（heading0 + 15°）；
  - seq3 t≈27s：+5° CW；t≈31s 重复 seq3（幂等，目标不变）。
- T1 YawPzn 实测：-3.0°@12s → -5.0°@20s（+10°CW 方向正确）→ -1.0°@28s →
  +3.0°@36s → +5.5°@41s（seq2 反转方向正确）；min=-6.0° max=+6.0°。
- T2/T3 无命令 → YawPzn≈0（多机通道隔离正确）。
- 结论：yaw_delta（CW+）方向、绝对目标锁存（只在新 seq 生效）、seq 幂等、
  通道独立性均验证通过。复现：farm_control_runtime/offline/harness/p3_yaw_e2e.py。

## 10. 已知边界


- 模板 ServoDyn `YCMode=0`：avrSWAP(48) 偏航速率命令**不驱动 ServoDyn**；
  Phase 3 启动物理偏航时需要 `YCMode=5`（Bladed-style DLL）并解锁 `YawDOF`；
- `ZeroMQInterface.f90` 已从 `FCR_FARM_CONTROL` 主路径退出（保留文件供 legacy 对照）；
- 多机组场景：每台机首次调用 `FCR_Init` 注册自身 turbine_id；runtime 槽位按 id 扩展。