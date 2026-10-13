# FAST.Farm 场控接口与 ROSCO Bridge 开发步骤计划 v2（决策修订版）

> 状态：**M0/M1 执行中**（本文档为实施基线，随 M0/M1 结论更新）
> 上游输入：FASTFarm_WFC_Bridge_Development_Plan_v1_1.md + FASTFarm_WFC_Interface_ICD_v1_1.csv
> 决策来源：场控接口项目实施决策（7 条，已全部纳入）
> 分支：dev/farm-control-runtime（严禁触碰 main/master）

---

## 0. 决策约束（DR 编号，后续所有变更不得违反）

| DR | 决策 | 影响 |
|---|---|---|
| DR-1 | ROSCO 保留在 OpenFAST ServoDyn/DISCON 机组级控制路径；**禁止**调度核自制等效控制器替代 ROSCO | Bridge 输出必须经 DISCON 接口进 ROSCO，不得旁路 |
| DR-2 | 先完成 v2.9.4 基线测试 + 多机实例隔离验证 → 迁移锁定版 **ROSCO v2.10.6** → 迁移回归 → 再开发依赖新版 ROSCO 的 Bridge | 顺序硬依赖：基线 → 迁移 → Bridge |
| DR-3 | Bridge 首版实现 **Mode 0/1/2**；Mode 1 完整支持 R_Pitch/R_Speed/R_Torque + 可配置控制分配策略；Mode 2 用 Ct(lambda,beta) 气动表桨距前馈 + ROSCO 原生 PRC 或独立最小桨距约束；Mode 3 只留反馈接口桩 | 禁止未分配就同时施加多个降载量；保留原 ROSCO 桨距/转矩/偏航/保护 |
| DR-4 | 复用 Runtime 算法（CommandStore/InductionSupervisor/YawActionManager/Watchdog），**拆分出不依赖通信的实时核心**；10ms 路径禁全场互斥锁/socket/动态分配/等待 GPU/Modbus | 实时核心 vs 通信适配分离 |
| DR-5 | 偏航协议升级为**绝对机舱目标方位**，保留旧离线协议兼容适配器 | 双协议并存期 |
| DR-6 | 不改供应商九步数值算法，只加调度生命周期钩子/命令快照读取/状态发布/流场快照接入；跨核用线程安全验证的非阻塞机制；10ms 不等 AWAE/WD/通信 | 调度核改动最小化 |
| DR-7 | Observation Manager 与风机状态分离；配置驱动多点测风，初期 256 点验证、1000 点容量验证；明确区分上游参考/轮盘背景/轮盘扰动/AWAE 观察点风速；**禁止扰动轮盘风速直接用于经典 Ct 归一化而不做模型修正** | AWAE 数据语义分层 |
| DR-8 | 实时机=Modbus TCP Client，RCP=Server；命令/机组状态/AWAE 多点数据区分区；配置版本号/时间戳/序号/质量标志/完整性检查；先测 30/100 台吞吐再冻结布局 | Modbus 冻结前置条件 |
| DR-9 | 验收顺序固定（见 §8）；**1s 内诱导因子跟踪**为动态性能目标，不得仅凭命令下发成功认定达成 | 验收判据 |

---

## 1. 现状基线（M0 已核实）

| 项 | 结论 |
|---|---|
| 本仓 ROSCO fork | **v2.9.4**（Constants.f90:17 'v2.9.0'；ROSCO_Types/ROSCO_IO v2.9.4）|
| 离线中间层 | farm_control_runtime Phase1-11 完成：39/39 单测、离线集成 11/14、ZMQ/Modbus 传输 |
| 供应商 RT 参考 | fastfarm_workspace/_ref_demo/fastfarm_rt_v1.1.0（OpenFAST v5.0.0 + WD + GPU AWAE，9 步闭环调度核 fastfarm_scheduler.c）|
| 供应商 RT 控制器 | **无真实 ROSCO**：ServoDyn 引用 ./DISCON_WT1.so（PCMode=5/VSContrl=5 Bladed DLL），但实际是 **21KB ELF 桩** → 需从本 fork 编 Linux .so 挂入 |
| 300s 门禁用风场 | study FarmInputs/coarse U8 18 盒全部就位（54 文件）|

**M0-0.2 关键结论（DR-1 可行性）**：供应商 RT 的 ServoDyn 走 Bladed-DLL 接口（PCMode=5/VSContrl=5，DLL_FileName=./DISCON_WT1.so）→ ROSCO 可以也必须在 OF 子步内经 ServoDyn 加载真实 DISCON .so（DR-1 成立）。Bridge 位于调度核 Fast Task，通过共享 setpoint 槽与 ROSCO 10ms 交互（复用现 FCR_ROSCO_API 语义）。

---

## 2. 修订后 Phase 划分与依赖图

依赖关系（严格顺序）：

    M0 (基线+可行性) → 门禁G0
       ├── M1 (Bridge 实时核心, 离线单机) → 门禁G1   [DR-3/DR-4/DR-5]
       ├── M2 (AWAE 观察点 + Wind Mapper) → 门禁G2   [DR-7]
       └── M3 (调度核三任务接入 + 多机隔离) → 门禁G3  [DR-6]
            ├── M4 (Modbus 吞吐测量→冻结布局) → 门禁G4 [DR-8]
            └── M5 (ROSCO v2.9.4→v2.10.6 迁移) → 门禁G5 [DR-2]
                 └── M6 (RT 端到端 + 1s 诱导跟踪验收) → 门禁G6 [DR-9]

> **重要修订（依 DR-2）**：原计划“先 Bridge 后迁移”改为**先迁移后 Bridge 依赖新版 ROSCO 的部分**。但 Bridge 的“实时核心拆分 + 离线单机验证”可在 v2.9.4 上先行（不依赖新版 ROSCO 特性），因此 M1 与 M5 可并行——M1 用 v2.9.4 验证实时核心与 Mode 0/1/2 逻辑，M5 迁移后重跑门禁。

---

## 3. Phase 0 — 基线确认与可行性（M0）

| # | 任务 | 源文件/对象 | 交付物 | 验收 |
|---|---|---|---|---|
| 0.1 | ROSCO fork diff 盘点 | wfcrl/simulators/fastfarm/src/{ROSCO_Types,ControllerBlocks,Controllers,DISCON}.f90 + _compile.py + compile_rosco.bat | diff 清单 | 明确 FCR 宏段/DISCON 挂载点/legacy 桥差异 |
| 0.2 | 供应商 RT DISCON 挂载确认 | fastfarm_rt_v1.1.0/config/FFTest_WT1.fst + ServoDyn_WT1.dat + DISCON_WT1.so | 可行性结论 | 已确认：Bladed DLL 接口 + 21KB 桩 → 需编 Linux .so（本阶段落库）|
| 0.3 | v2.9.4 多机实例隔离验证 | 现 ZMQ 链路 + U8 盒 | 隔离验证报告 | 多机 ROSCO 实例独立无串扰（每机独立 DISCON 状态）|
| 0.4 | 300s 基线回归 | regress_300s.py（U8/TI10/seed1）| PASS 基线（~3860kW）| 退出码 0 + 有限均值 |
| 0.5 | 实时核心拆分设计 | farm_control_runtime/{src,include} | RT-core 边界设计 | 10ms 路径无锁/socket/malloc/等待 |

**门禁 G0**：0.1/0.2 结论落库 + 0.3/0.4 PASS + 0.5 设计评审 → 才允许动 Bridge 代码。

---

## 4. Phase 1 — ROSCO Bridge（M1，实时核心，离线可测）

### 4.1 目录与源文件修改清单（新建 farm_control_runtime/bridge/）

| 源文件（新） | 职责 | 复用来源 |
|---|---|---|
| bridge/rt_core.h/.c | **不依赖通信的实时核心**：命令快照/状态快照/超时/watchdog 判定，纯内存 | runtime.c 逻辑拆分 |
| bridge/induction_bridge.c | a→参考映射（DR-3：Mode0/1/2 分配）| InductionSupervisor |
| bridge/control_alloc.c | **控制分配策略**（DR-3）：R_Pitch/R_Speed/R_Torque 三者按配置权重分配降载量，禁止未分配叠加 | 新增（关键新模块）|
| bridge/ct_estimator.c | Ct=2T/(rho A V^2)；**模型修正层**（DR-7：扰动盘面风不可直接经典归一化）| offline_provider 派生式 + 修正 |
| bridge/yaw_adapter.c | 绝对机舱目标方位（DR-5 新协议）+ 旧 delta 协议适配器 | YawActionManager + angle_convention |
| bridge/min_pitch_adapter.c | Mode 2 桨距前馈：Ct(lambda,beta) 气动表反解 beta_target → 最小桨距约束/PRC | 新增（读气动表）|
| bridge/awae_adapter.c | 观察点帧→盘面等效风速/TI 的桥接 | WindMapper 输出消费 |
| bridge/safety_manager.c | TTL/stale/fallback/健康标志 | Watchdog + CommandStore |

### 4.2 Bridge 输入/输出契约（依 ICD v1.1，冻结于 BRIDGE_ICD_v2）

输入（来自调度核快照，10ms 或 1s 按 ICD）：

    turbine_id, induction_ref, yaw_target_heading,
    rotor_speed, generator_speed, rotor_thrust, blade_pitch,
    disk_wind_speed, disk_turbulence

输出（→ROSCO，经 FCR_ROSCO_API setpoint 槽，10ms 只读）：

    r_pitch, r_speed, r_torque, min_pitch_ref, pitch_offset(桩), yaw_target

### 4.3 诱导模式实现范围（DR-3）

| Mode | 实现 | 输出 | 验证 |
|---|---|---|---|
| 0 | 原 ROSCO，所有外部 ratio=1 | 旁路 | 与基线 bit-接近 |
| 1 | 功率比例开环 R_power=a(1-a)^2/[a0(1-a0)^2] → **R_Pitch/R_Speed/R_Torque 三通道 + 控制分配** | 三 ratio | 单机降额单调性 |
| 2 | Ct 前馈：Ct_ref=4a(1-a)，Ct(lambda,beta) 气动表反解 → min_pitch_ref（ROSCO 原生 PRC 或独立最小桨距约束）| min_pitch_ref | 稳态 Ct 响应 |
| 3 | 反馈接口桩（保留 ct_ref/Ct_est 通道）| pitch_offset=0 | 桩编译+ABI |

### 4.4 测试案例（新建 bridge/tests/）

- 单测：control_alloc 分配不变量（总降载=Σ通道加权、无未分配叠加）；yaw_adapter 绝对目标锁存/wrap + 旧协议适配；ct_estimator 修正层（扰动盘面风 vs 上游参考风）数值案例；safety_manager TTL/fallback。
- 集成（离线单机）：Mode0/1/2 在 v2.9.4 上的功率/桨距/转矩响应；Mode1 三通道分配生效验证；Mode2 稳态 Ct 响应 vs 理论。

**门禁 G1**：Bridge 单测全绿 + 单机 Mode0/1/2 行为符合物理预期（1s 诱导跟踪初步达标，见 §8）+ 300s 回归无劣化。

---

## 5. Phase 2 — AWAE 观察点 + Wind Mapper（M2）

### 5.1 源文件修改清单

| 文件（新） | 职责 |
|---|---|
| awae/observation_manager.c/h | 点位注册（配置驱动）、时间同步、环形缓存；与风机状态**分离**（DR-7）|
| awae/wind_mapper.c/h | 观察点→轮盘等效风速/TI；区分四类风速语义（DR-7）|
| awae/awae_point_frame.h | Observation Point Frame 结构（ICD：point_id, xyz, wind_xyz, wind_speed, wind_dir, TI）|
| awae/wind_semantics.h | 上游参考/轮盘背景/轮盘扰动/AWAE 观察点四类定义 + 互转规则 |

### 5.2 测试

- 256 点配置解析/映射；1000 点容量（内存/性能预算）验证；
- 四类风速语义正确分流：**扰动盘面风不得直接用于经典 Ct 归一化**（走修正层）——专项数值测试；
- 映射结果 vs 供应商 GPU 高分辨率场（Vdist_High）对比偏差 < 阈值。

**门禁 G2**：256/1000 点通过 + 语义分流测试通过 + 映射偏差达标。

---

## 6. Phase 3 — 调度核三任务接入 + 多机隔离（M3）

### 6.1 调度核修改边界（DR-6，最小侵入）

不改供应商九步数值算法。只加：

    scheduler/lifecycle_hooks.h   调度生命周期钩子（init/step/teardown）
    scheduler/cmd_snapshot.h      命令快照读取（非阻塞）
    scheduler/state_publish.h     状态发布（非阻塞，线程安全验证）
    scheduler/flow_snapshot.h     流场快照接入（AWAE → WindMapper → Bridge）

三任务（新增，挂在现有 9 步闭环外围）：

| 任务 | 周期 | 内容 | 红线 |
|---|---|---|---|
| Fast | 10ms | 读 OF 状态 → Bridge → 写 ROSCO setpoint | 禁 socket/malloc/锁等待/等 AWAE/WD |
| Slow | 1s | Modbus 收发、RCP 同步、命令落 CommandStore | 与 Fast 双缓冲 |
| Mapping | 1s | AWAE 观察点 → WindMapper → disk 风速/TI | 独立于 Fast |

### 6.2 多机实例隔离（DR-2 前置）

- 每机独立 DISCON 状态（v2.9.4 现架构已多实例）；新增隔离验证用例：T1/T2/T3 不同命令并发，确认无串扰（状态回显 per-turbine）。

**门禁 G3**：Fast 10ms 确定性（无阻塞/无分配）+ 双缓冲无竞争（TSAN/竞态注入）+ 多机隔离 PASS + 200 步 RT smoke。

---

## 7. Phase 4 — Modbus 吞吐测量与冻结（M4）

### 7.1 流程（DR-8）

1. 现有 modbus_transport.c + modbus_map.yaml 上先实现三个数据区（Command / Turbine State / AWAE Frame Buffer）；
2. **先测量 30 台与 100 台实际通信吞吐**（往返延迟、帧率、丢包），记录数据；
3. 依据测量**冻结**刷新策略与寄存器布局（modbus_map_v2.yaml）；
4. 补：版本号/时间戳/序号/质量标志/完整性校验（CRC/seq 检查）。

**门禁 G4**：30/100 台吞吐数据入库 + 布局冻结 + 完整性校验测试通过。

---

## 8. Phase 5 — ROSCO 迁移 v2.9.4 → v2.10.6（M5）

严格按 DR-2 + 计划 P0 规则：

1. 获取锁定版 v2.10.6 源码（不直接覆盖 Fortran，diff/三方合并）；
2. 逐步迁移：ROSCO_Types → ControllerBlocks → Controllers → DISCON（每步单机回归）；
3. 保留 FAST.Farm 集成修改（FCR 宏段、yaw 钩子、legacy 桥适配）；
4. 多机实例隔离回归；
5. 300s 门禁（U8/TI10/seed1）与 v2.9.4 基线对比。

**门禁 G5**：v2.10.6 下所有离线门禁与 v2.9.4 基线一致（功率统计/偏航诱导 identity），无回归。

---

## 9. Phase 6 — RT 端到端与验收（M6）

验收顺序（DR-9 固定）：

1. ROSCO 版本迁移及机组实例隔离（G5）
2. Bridge Ratio 在 ROSCO 内部各控制回路实际生效（验证 r_pitch/r_speed/r_torque 确实改变 ROSCO 内部回路输出）
3. 气动表前馈及稳态 Ct 响应（Mode 2）
4. 离线 ZeroMQ 多机回归
5. 实时机 10ms 控制确定性（Fast Task 周期抖动 < 阈值）
6. Modbus 与 AWAE 多点数据完整性
7. 30 台功能 + 百台扩展能力

**动态性能目标（DR-9）**：**1s 内诱导因子跟踪**——施加 induction_ref 后 1s 内功率/推力响应达到目标（以实际状态测量为准，不以“命令下发成功”判定）。

**门禁 G6**：上述 7 项全部 PASS + 1s 诱导跟踪达标。

---

## 10. 构建脚本清单

| 脚本 | 用途 | 备注 |
|---|---|---|
| farm_control_runtime/tools/build_windows.ps1 | Windows runtime 单测 | 现用，加 bridge 目标 |
| wfcrl/simulators/fastfarm/src/_compile.py --mode farmcontrol | 离线 FCR DISCON_WT1.dll | 现用 |
| _compile.py --mode linux-rt | **新增**：编 Linux DISCON_WT1.so（供应商 RT 挂载）| M5 后 |
| fastfarm_rt_v1.1.0/build.sh | 供应商 RT 三 DSO + 调度核 | 现用，M3 加钩子编译 |
| tools/check_abi.py | ABI 冻结校验 | 现用，加 bridge ABI |

---

## 11. 里程碑与并行性

    M0 基线+可行性        → 当前（300s 基线已在跑）
    M1 Bridge 实时核心     → v2.9.4 先行（与 M5 并行）
    M5 ROSCO 迁移          → 独立并行，完成后重跑 M1 门禁
    M2 AWAE 观察点         → 可与 M1 并行
    M3 调度核三任务        → 依赖 M1 实时核心
    M4 Modbus              → 依赖 M3
    M6 RT 端到端验收       → 依赖 M2/M3/M4/M5

**下一交付**：完成 M0（G0）与 M1（G1）后，提交**新版 Bridge 接口设计**（fcr_bridge_api.h + ICD v2）供审查。
