# 场控接口模块对《风电控制策略实时验证平台子模块开发、接入与测试要求》的符合性审查

> 依据：《风电控制策略实时验证平台子模块开发、接入与测试要求》（2026-10-20 提供）
> 审查对象：`farm_control_runtime`（场控交互模块）及其 ROSCO/Python 侧接口
> 审查方式：逐条对照 + 代码级证据（文件:行 / 测试名），非文档推断
> 审查日期：2026-10-20　基线：`dev/farm-control-runtime` @ `3fe4026`

---

## 0. 结论摘要

| 准入里程碑 | 状态 | 说明 |
|---|---|---|
| **M1 Module Verified** | **≈95%** | 独立功能/边界/参考结果/稳定性已具备；**Reset 已补齐（G-01）**，入口 NaN 校验已补齐（G-02） |
| **M2 Interface Verified** | **≈80%** | 数据字典（88 信号）强；**结构化状态/错误（G-08）、程序化 CONFIGURE（G-11）已实现**；时间语义/数据策略表待成文（G-03/G-05） |
| **M3 Scheduler Verified** | **≈60%** | **调度器接入层 `fcr_rt_adapter` 已实现**（RTimeCtx / 10ms step / 1s SCADA 计数器 / 状态回传）；**10ms 无锁（G-12）与调度时序测试（G-13）待做** |
| **M4 RT Ready** | **0%（采购后）** | 按要求属采购后事项 |

**2026-10-20 实施进展**（本次已完成）：

| 项 | 内容 | 验证 |
|---|---|---|
| **G-01 Reset** | `fcr_runtime_reset()` + `fcr_rosco_api_reset()` + `FCR_Reset()`（Fortran）；重建全部运行时状态、保留配置与 ROSCO 标称副本 | `rt_reset_repeatable` 单测 |
| **G-02 入口校验** | `fcr_command_store_accept_frame` 增加 `isfinite` 校验（偏航目标/诱导因子/时间戳），非法→拒绝并置 `FCR_CMD_FLAG_INVALID_VALUE` | `rt_reject_nan_command` 单测 |
| **G-04 RTimeCtx** | `FcrRtTimeCtx`（n_low/t_low/dt_low/n_fast_low/dt_high/n_high_low）只读传入 | `fcr_rt.h` |
| **G-07 调度器形态** | `fcr_rt_adapter.c`：create/configure/init/set_*/step/get_*/reset/stop/destroy | 9 个 RT 单测 |
| **G-08 状态接口** | `FcrRtStatus` 九态 + `FcrRtStatusInfo`（ErrorCode/ErrorSource/Message/Timestamp） | 同上 |
| **G-11 CONFIGURE** | `fcr_rt_configure()` 结构体入参（含 dt_scada 可整除校验） | `rt_configure_validation` |
| **G-13 SCADA 边界** | 计数器实现（`steps_per_scada = dt_scada/dt_high`），与 DT_Low 解耦 | `rt_scada_boundary_counter`（100 步 = 1 s） |

**一句话**：场控交互模块的**业务语义与降级行为符合**要求（§7.4 六条已实现并有测试）；**平台接入形态已具备**（生命周期/时间/状态/数据接口齐备）；剩余为**10 ms 无锁改造、调度时序测试、策略表成文**及采购后的 M4。

---

## 1. 模块定位（按 §2 要求自述）

### 1.1 模块角色（§2.1）

**交互类（场控）**，并含少量**辅助类**职责：
- 主职责：外部场控命令（RCP/SCADA）与平台状态之间的接口——命令接收/校验/锁存/超时、状态聚合/发布、诱导 setpoint 生成、AWAE 观测二跳。
- 辅助职责：离线合成注入（仅测试用，`FCR_CFG_AWAE_SYNTH_POINTS`，默认关闭）。
- **不承担**：物理模型（FAST.Farm/OpenFAST）、单机控制律（ROSCO）、风场尾流计算。

### 1.2 调度周期中的位置（§2.2）

**当前形态（FAST.Farm 内嵌）**：

| 项 | 现状 |
|---|---|
| 谁调用 | ServoDyn（每 `DT_High`=10 ms 调 `DISCON()`）→ `FarmControlInterface.FCR_Step` |
| 调用位置 | ROSCO 控制链**之前**（`FCR_Step`/`FCR_ApplyReferences`）与**之后**（`FCR_PublishState`/`FCR_PublishExtra`） |
| 调用前输入 | 状态帧（S02/S03/S06/AWAE）、上一周期 ROSCO 状态、ZMQ 命令帧 |
| 调用后输出 | `FcrRoscoExternalSetpoint`(S08) → ROSCO `CntrPar`；S10 状态帧 → RCP；S12/S13 AWAE 帧 |
| 生效周期 | 命令按 `requested_effective_sim_time_s`(S0112) 到点生效；偏航目标锁存后持续；诱导 setpoint 每 10 ms 刷新 |
| 依赖 | 上游 CommandStore/S07 输入；下游 ROSCO 读取；**不直接调用其他模块内部函数** |

**RT 平台目标形态（要求 §2.2/§5.1）**：调度器在 `Phase 3 主循环` 中以 `DT_SCADA` 周期调用本模块，时间由 `RTimeCtx` 统一提供。**当前不具备该接口**（见缺口 G-07）。

---

## 2. 逐章符合性对照

### 2.1 §3 生命周期接口

| 阶段 | 要求 | 现状 | 证据 | 结论 |
|---|---|---|---|---|
| CREATE/LOAD | 资源创建 | `fcr_runtime_create(cfg)` | `src/runtime.c` | ✅ |
| CONFIGURE | 独立配置阶段 | **无 API**，仅 env（`FCR_BRIDGE_ENABLE`/`_INDUCTION_MODE`/`_ALLOC_W_*`/`_IND_A_MAX/MIN`） | `src/rosco_api.c:75-116` | ⚠️ 部分 |
| INITIALIZE | 初始化、失败返回原因 | `fcr_rosco_api_init(n,dt)` 返回 rc；`api_ok_` 门控后续调用 | `src/rosco_api.c:74`、`FarmControlInterface.f90:61-80` | ✅/⚠️（原因仅 rc） |
| RUN/STEP | 时间推进 | `fcr_rosco_step_high(tid, sim_time_s)` | `src/rosco_api.c:231` | ✅ |
| **RESET** | run→reset→run 不重启程序 | **无 `fcr_runtime_reset()`**；仅子系统 reset（catalog/statistics） | 全仓 grep 无 | ❌ **缺口 G-01** |
| STOP | 停止 | `fcr_rosco_api_shutdown()` | `src/rosco_api.c:164` | ✅ |
| DESTROY | 释放 | `fcr_runtime_destroy()` | `src/runtime.c:88` | ✅ |
| 初始化失败不得进入运行 | — | `api_ok_=false` → Fortran 全部 RETURN；`!g_rt` → step 返回 -1 | `FarmControlInterface.f90:90,108` | ✅ |

### 2.2 §4 数据接口

| 要求 | 现状 | 证据 | 结论 |
|---|---|---|---|
| 正式数据字典 | **ICD v1.5 SingleHop CSV，88 信号/13 组**，含 Sender/Receiver/Data_Type/Unit/Update_Rate/无效值与质量要求/Upstream_Signal_ID | `docs/farm_control/inputs/FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv`；`tools/check_icd_gates.py`（G-1/G-2/G-3 PASS） | ✅ **强** |
| NaN/Inf 检查 | 诱导：`bridge/fcr_bridge_core.c:96,105,167,266,331`、`induction_supervisor.c:58` 兜底；**偏航目标在 CommandStore 入口未校验 isfinite** | `src/command_store.c` 无 isfinite | ⚠️ **缺口 G-02** |
| 物理范围 | 诱导 a∈[a_min,a_max]、power_cap、min_pitch 限幅；Ct∈(0,2)；Mode2 表轴校验 | `fcr_bridge_core.c:140-155,266-296` | ✅ |
| StepID | `command_frame_seq`(S0101)+逐通道 seq：重复幂等、乱序拒绝（置 `INVALID_SEQ`） | `test_command_store.c` | ✅ |
| 时间戳超龄 | 逐通道 stale（5 s，W-13）+ TTL 到期 | `command_store.c:147-225`；`stale_flag_per_channel` | ✅ |
| 数据质量 | `wind_quality_flags`/`observation_quality_flags`/SIMULATED 位 | `fcr_awae_types.h`、`state_aggregator.c:138-153` | ✅ |
| 接口版本兼容 | `protocol_version` 帧校验；容量版本 `FCR_CAPACITY_VERSION`+`*_V1` | `zmq_transport.cpp:145`、`fcr_protocol.h` | ✅ |
| **逐情况策略表**（接受/拒绝/限幅/保持/降级/停止） | 行为已实现在代码，但**未形成逐信号策略表** | — | ⚠️ **缺口 G-03** |

### 2.3 §5 时间接口

| 要求 | 现状 | 结论 |
|---|---|---|
| 调度器为唯一主时间源（RTimeCtx） | 模块**接收** `sim_time_s` 不自推（符合"不自行推进"），但**无 RTimeCtx 结构**（n_low/t_low/dt_low/n_fast_low/dt_high/n_high_low） | ⚠️ **缺口 G-04** |
| 声明自身执行周期（DT_SCADA） | 仅 `FCR_CFG_DT_LOW` env（默认 1 s）；实际被调 10 ms、内部 1 s 聚合；**未形式声明** | ⚠️ 缺口 G-04 |
| 输入输出生效时刻 | 有 `requested_effective_sim_time_s`(S0112) 到点生效 + TTL/stale 语义；但**未成统一表** | ⚠️ 缺口 G-05 |
| 执行时间/超时策略 | **无**典型/最大执行时、是否关键路径、跨周期许可、超时处理；仅有锁统计 `fcr_rosco_lock_stats`（WIN-14） | ❌ **缺口 G-06** |
| 实时测试（Deadline/Jitter/Worst/Miss） | 未开展（采购后 M4 + R-03） | ❌ |

### 2.4 §6 状态与错误接口

| 要求 | 现状 | 结论 |
|---|---|---|
| 结构化状态枚举（OK/WARNING/STALE_DATA/INVALID_INPUT/NOT_READY/TIMEOUT/NUMERICAL_ERROR/COMM_ERROR/FATAL） | **无模块级状态枚举**；现有为 ICD 信号位域：`command_status_flags`、`rt_health_flags`、`bridge_status_flags`、`turbine_status_flags` | ❌ **缺口 G-08** |
| ErrorCode/ErrorSource/ErrorMessage/Timestamp | **无**；API 仅返回 `int`（0/-1/-2） | ❌ 缺口 G-08 |

> 说明：位域适合"业务信号追溯"，但满足不了"调度器判断异常来自哪个模块"的要求，需新增模块状态接口，二者并存。

### 2.5 §6.1/§7.4 降级策略（场控交互）

| 要求 | 现状 | 证据 | 结论 |
|---|---|---|---|
| 短时通信异常 → D1（保持最后有效值，须规定最大保持） | TTL 内保持最后 setpoint（偏航 60 s / 诱导 5 s）+ stale 5 s | `command_store.c`；`test_offline_disconnect_fallback.py::test_ttl_expiry_rolls_back_yaw` | ✅（**最大保持时间未成文**，缺口 G-09） |
| 长时间通信失败 → D3（bypass，退出外部控制） | TTL 到期 → 通道 `enable=0` → 纯 ROSCO 回退 | `test_command_store.c::ttl_expiry`、`rosco_api.c::control_disable_fallback` | ✅ |
| 非法指令 → 拒绝并报警 | 乱序拒绝置 `INVALID_SEQ`；`control_enable=0` 全通道回退；Mode2 无表不采纳（W-04） | `test_command_store.c`、`test_bridge_core.c::mode2_no_table_fallback` | ✅（"报警"=状态位，无日志/告警通道 → G-09） |
| 超时命令 → 不执行 | TTL 到期不执行 | 同上 | ✅ |
| 重复/乱序 → 按 StepID/时间戳过滤 | 重复 seq 幂等、乱序拒绝 | `test_command_store.c` | ✅ |
| 场控失效不得导致 FAST.Farm 崩溃 | PUB/SUB 非阻塞、全路径有回退；实测无控制器时仿真继续 | `test_offline_disconnect_fallback.py::test_no_controller_sim_keeps_running` | ✅ |

### 2.6 §8/§13 单元与生命周期测试

| 项 | 现状 |
|---|---|
| 正常/边界/非法输入 | ✅ 80/80 单测覆盖（a_min/a_max、容量守卫、NaN 诱导、乱序、TTL、stale、Mode2 无表） |
| Reset 可重复性 | ❌ 无 Reset 接口，无法测（G-01） |
| 参考结果对比 | ✅ T-21 300 s 基线 4366 kW（0.00%）、T-22 诱导 -15.14%、T-23/T-24/T-25/T-27 |
| create→init→run→stop | ✅ 集成测试 |
| **create→init→run→reset→run** | ❌ 缺 |
| create→invalid init | ⚠️ 仅 `api_init_idempotent`（重复 init），无"非法配置"用例（G-10） |
| run before init / stop→destroy | ⚠️ 行为存在（返回 -1），**无显式断言用例**（G-10） |

### 2.7 §15 时间接口测试 / §16 异常用例

| 要求 | 现状 |
|---|---|
| 调度器周期测试（DT_BASE/DT_CTRL/DT_FARM 调用次数、首末时刻、漏调/重调、生效周期） | ❌ 未做（无调度器，G-07） |
| CASE-SCADA-ERR-01（断连 → 保留 X 秒 → 退出外部控制） | ⚠️ 部分：`test_offline_disconnect_fallback` 覆盖"仿真继续/TTL 回退"，但**未断言 setpoint 实际回退到标称及精确时长**（G-09） |
| 每个降级策略有对应用例 | ⚠️ 5/6 有弱覆盖，D0/D2 无显式用例 |

---

## 3. 缺口清单（按准入里程碑分级）

### M1 — 模块自证（**已基本完成**）

| ID | 缺口 | 要求条款 | 状态 |
|---|---|---|---|
| **G-01** | 无 Reset 接口 | §3.3、§13 | ✅ **已实现**：`fcr_runtime_reset` / `fcr_rosco_api_reset` / `FCR_Reset`；`rt_reset_repeatable` 验证 run→reset→run |
| **G-02** | CommandStore 入口未校验 NaN/Inf | §4.1、§14 | ✅ **已实现**：`accept_turbine` 校验偏航目标/诱导因子/时间戳，拒绝并置 `FCR_CMD_FLAG_INVALID_VALUE` |
| **G-10** | init 非法配置、run-before-init、stop-then-destroy 无显式用例 | §13 | ✅ **已实现**：`rt_configure_validation`（n=0/dt=0/非整除）、`rt_run_before_init`、`rt_stop_then_step` |

### M2 — 接口规范（**大部分已实现**）

| ID | 缺口 | 要求条款 | 状态 |
|---|---|---|---|
| **G-08** | 无结构化状态/错误接口 | §6 | ✅ **已实现**：`FcrRtStatus`（OK/WARNING/STALE_DATA/INVALID_INPUT/NOT_READY/TIMEOUT/NUMERICAL_ERROR/COMM_ERROR/FATAL）+ `FcrRtStatusInfo`（ErrorCode/ErrorSource/Message/Timestamp） |
| **G-11** | 无程序化 CONFIGURE | §3 | ✅ **已实现**：`fcr_rt_configure(FcrRtConfig*)`（含 dt_scada 可整除校验） |
| **G-03** | 无逐信号"接受/拒绝/限幅/保持/降级/停止"策略表 | §14 | ⬜ 待成文（行为已在代码，建议补 `data_policy.csv` 并与 `audit_signals.py` 联动） |
| **G-05** | 时间语义/生效时刻未成表 | §5.3 | ⬜ 待成文（S0112 到点生效 + TTL/stale 语义已实现，需成表） |
| **G-09** | 最大保持时间未成文；D0/D2 无用例；CASE-SCADA-ERR-01 断言弱 | §6.1、§16 | ⚠️ 部分：`rt_ttl_timeout_edge` 已强断言 TTL 边沿与 D3 释放；最大保持时间=TTL（偏航 60 s/诱导 5 s）需写入文档 |
| **G-06** | 无 Step 执行时预算/超时策略 | §5.4 | ⬜ 待做（已有 lock 统计，需扩展为典型/最大执行时 + 超时动作） |

### M3 — 调度器闭环（**接入层已实现，余 2 项**）

| ID | 缺口 | 要求条款 | 状态 |
|---|---|---|---|
| **G-04** | 无 `RTimeCtx` 时间接口 | §5.1、§5.2 | ✅ **已实现**：`FcrRtTimeCtx` 只读传入；10 ms 调用、DT_SCADA=1 s |
| **G-07** | 无调度器可调用形态 | §2.2、§15 | ✅ **已实现**：`fcr_rt_adapter.c`（create/configure/init/set_command/set_state/set_extra/set_flow/set_wind_estimate/set_awae_catalog/step/get_output/get_setpoint/get_awae/reset/stop/destroy） |
| **G-12** | 10 ms 路径含互斥锁 | R-03、§5.4 | ⬜ 待做（`FCR_LOCK` 在 `fcr_rosco_step_high`/`publish_state`；RT 路径 `fcr_rt_step` 经 runtime 也走锁，需改无锁 double-buffer） |
| **G-13** | 无调度时序测试 | §15 | ⚠️ 部分：`rt_scada_boundary_counter` 已验证 100×10 ms = 1 s 边界；完整 1000 周期调度测试待补 |

### M4 — RT Ready（采购后）

RT 框架导入、目标系统编译加载、RT vs WSL 一致性、Deadline/Jitter/WorstCase 实测。

---

## 4. 与平台方的接口约定（已确认 + 待确认）

### 4.1 已确认（2026-10-20）

| # | 问题 | 平台方答复 | 对本模块的含义 |
|---|---|---|---|
| Q1 | 调度周期 / DT_SCADA | **scheduler 周期 = 10 ms；接口模块每 10 ms 被调用一次；DT_SCADA = 1 s，即 RCP ↔ scheduler 通信周期 1 s** | 模块是"10 ms 步进模块"，但**对外数据交换按 1 s**；10 ms 内只做锁存/刷新/聚合，1 s 边界才做 SCADA 收发与状态帧发布 |
| Q4 | 输出生效周期 | **是**：DT_SCADA 内下发、下一个 DT_Low 生效 | 保持现有 S0112 `requested_effective_sim_time_s` 语义，无需改 |
| Q5 | 降级判定归属 | **模块自行判断**（D3 退出场控由模块决定） | 模块需**上报**已降级状态（供调度器记录），但决策权在模块 |

**由此确定的运行时序**：

```
RCP(场级控制器)
  │  ←──── 1 s (DT_SCADA)：命令下发 / 状态回传 ────→  [Scheduler 通信层]
  │
Scheduler（唯一时间源，10 ms 步进）
  ├─ 每 10 ms：fcr_rt_step(handle, RTimeCtx)     ← 本模块被调用
  │     ├─ 命令锁存 / TTL / stale / setpoint 刷新（10 ms）
  │     └─ 仅当 n_low 变化（1 s 边界）：状态聚合窗口闭合 + 输出 S10 状态帧
  └─ 每 1 s：把 RCP 命令送入本模块、把本模块输出发给 RCP
```

### 4.2 Q2 详解：RTimeCtx 怎么传给模块（待确认）

**问题是什么**：要求 §5.1 规定调度器用 `RTimeCtx` 统一维护时间
（`n_low / t_low / dt_low / n_fast_low / dt_high / n_high_low`），
但**没说这个结构如何到达模块**。三种做法：

**方案 A（推荐）：调度器把结构体指针作为 step 入参传入**

```c
typedef struct {
    int32_t n_low;        /* DT_Low 步号，0 起 —— 唯一权威 */
    double  t_low;        /* = n_low * DT_Low [s] */
    double  dt_low;       /* 1.0 s */
    int32_t n_fast_low;   /* 本 DT_Low 内已推进的 10 ms 子步数 */
    double  dt_high;      /* 0.01 s */
    int32_t n_high_low;   /* 本 DT_Low 内风时间片数 */
} FcrRtTimeCtx;

/* 调度器每 10 ms 调用一次，时间上下文只读传入 */
int fcr_rt_step(FcrRtHandle *h, const FcrRtTimeCtx *t);
```

- 优点：模块**不持有全局时钟**（符合"不得自行推进系统时间"）；单元测试可直接构造 ctx 模拟任意时序（M3 调度测试必需）；无隐式全局依赖。
- 缺点：调用方必须提供完整结构。

**方案 B：调度器提供查询函数，模块主动取**

```c
/* 模块内部调用（需要链接/注册到调度器） */
const RTimeCtx *scheduler_get_time(void);
int fcr_rt_step(FcrRtHandle *h);
```

- 优点：接口参数少。
- 缺点：模块与调度器**产生隐式全局耦合**，脱离调度器无法单测，也不便于 Mock。

**方案 C：只传最小标量**

```c
int fcr_rt_step(FcrRtHandle *h, double t_low, int32_t n_low, double dt_low);
```

- 优点：简单。
- 缺点：缺少 `n_fast_low/dt_high`，模块无法判断"10 ms 子步/1 s 边界"的精确位置，也无法对齐风时间片。

**需要你确认的**：选 A / B / C 哪一种？（建议 A）

> 补充说明：因为模块 10 ms 被调、1 s 做 SCADA 交换，模块**必须能可靠识别 1 s 边界**。
> 方案 A 中可用 `t->n_low` 变化作为边界信号（n_low 是权威步号），比用浮点时间比较更可靠。

### 4.3 Q3 详解：状态/错误怎么回传给调度器（待确认）

**问题是什么**：要求 §6 规定模块要向调度器返回**结构化状态**
（OK/WARNING/STALE_DATA/INVALID_INPUT/NOT_READY/TIMEOUT/NUMERICAL_ERROR/COMM_ERROR/FATAL
+ ErrorCode/ErrorSource/ErrorMessage/Timestamp），但**没规定传递形态**。三种做法：

**方案 A（推荐）：同步返回状态枚举 + 可选详情结构**

```c
typedef enum {
    FCR_ST_OK = 0, FCR_ST_WARNING, FCR_ST_STALE_DATA, FCR_ST_INVALID_INPUT,
    FCR_ST_NOT_READY, FCR_ST_TIMEOUT, FCR_ST_NUMERICAL_ERROR,
    FCR_ST_COMM_ERROR, FCR_ST_FATAL
} FcrRtStatus;

typedef struct {
    FcrRtStatus status;
    int32_t     error_code;      /* 模块内细分码 */
    uint16_t    error_source;    /* 子模块编号：命令/状态/桥接/AWAE/传输 */
    char        message[64];
    double      timestamp_s;     /* 发生时刻（仿真时间） */
} FcrRtStatusInfo;

/* 返回值给调度器做快速判断；详情由调度器传入的结构带回 */
FcrRtStatus fcr_rt_step(FcrRtHandle *h, const FcrRtTimeCtx *t,
                        FcrRtStatusInfo *out_status);
```

- 优点：同步、无共享内存、可单测、调度器可立即据 `status` 执行 D0~D4；`out_status` 让"异常来自哪个子模块"可追溯（要求 §6 明确目的）。
- 缺点：每步多写一个结构（可忽略）。

**方案 B：写入共享状态区，调度器每步读**

```c
/* 模块把状态写进自己的共享块，调度器按约定地址读 */
typedef struct { FcrRtStatus status; int32_t code; uint16_t src; char msg[64]; double t; } FcrRtStatusBlock;
/* 模块：fcr_rt_status_write(h, &blk);  调度器：读共享内存 */
```

- 优点：适合**模块独立进程/独立 RT 任务**（跨进程无法用返回值）。
- 缺点：需要共享内存布局约定 + 版本化；单测需模拟共享区。

**方案 C：两者并用**——`step()` 返回枚举（快路径），同时写共享块（供跨进程/诊断）。

**需要你确认的**：本模块在平台里是
（a）**与调度器同进程的库**（选 A 即可），还是
（b）**独立进程/独立 RT 任务**（必须选 B 或 C）？

### 4.4 由此衍生的两个待确认问题

1. **传输归属**：Q1 说"RCP ↔ scheduler 通信周期 1 s"，那么 **ZMQ/Modbus 收发是调度器做，还是本模块做**？
   - 若调度器做：本模块只需 `set_input(命令帧)` / `get_output(状态帧)`，10 ms 路径内**无任何 socket**（更符合 RT 红线，也能消掉现有 10 ms 锁）。
   - 若本模块做：需保留 ZMQ 传输，但应改为**1 s 边界异步收发**，10 ms 路径只读本地缓存。
2. **1 s 边界由谁定义**：用 `n_low` 变化（推荐，权威步号），还是用 `t_low` 跨 1 s 整数倍？若平台 `DT_Low ≠ 1 s`（如 3 s），SCADA 边界应独立于 `DT_Low`（每 1 s 一次），需明确由哪个计数器驱动。

---

## 5. 附：现有证据索引

- 单测 80/80：`farm_control_runtime/tools/_build_test.bat && test_runner.exe`
- 协议 12/12：`farm_control_runtime/tests/integration/test_icd_v15_abi_python.py`
- ABI：`tools/check_abi.py`（82 字段偏移）
- 门禁：`tools/check_icd_gates.py`（G-1/G-2/G-3）、`tools/audit_signals.py`（88/88）
- 集成 26/26：`pytest farm_control_runtime/tests/integration -q`（逐文件）
- 系统回归：T-21 4366 kW（0.00%）、T-22 -15.14%、T-23 同 run 复核、T-24 隔离、T-25 Bridge、T-27 Modbus
- 交付说明：`docs/DELIVERY_NOTE_RT_INTEGRATION.md`
