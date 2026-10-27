# 场控交互模块 — 数据策略 / 时间语义 / 降级映射

> 依据《风电控制策略实时验证平台子模块开发、接入与测试要求》§4.1 / §5.3 / §6.1 / §7.4 / §14 / §16
> 对应实现：`farm_control_runtime/src/{command_store,state_store,state_aggregator,bridge_runtime_adapter,fcr_rt_adapter}.c`
> 接口：`include/fcr_rt.h`（RT 调度器接入）　基线：`dev/farm-control-runtime` @ `39caf7e`+

---

## 1. 数据策略表（§14：对每种情况提前规定动作）

动作含义：**接受** = 采用；**拒绝** = 丢弃该值/该通道；**限幅** = 钳到合法范围；
**保持** = 沿用上一有效值；**降级** = 走 D0~D4；**停止** = 上报 FATAL 由调度器停算例。

### 1.1 S01 命令帧（RCP → 本模块）

| 条件 | 动作 | 告警位 / 上报 |
|---|---|---|
| 帧尺寸 ≠ 5664 B | 拒绝整帧 | 传输层丢弃（ZMQ 严格尺寸校验） |
| `protocol_version` ≠ 2 | 拒绝整帧 | 传输层丢弃 |
| `n_turbines` = 0 或 > 64 | 拒绝整帧 | 传输层丢弃 |
| 无任何有效通道（yaw/ind 均无效 且 basis ∉ [0,1e12)） | 拒绝整帧 | 传输层丢弃（WIN-05） |
| `command_basis_sim_time_s` NaN/Inf | **拒绝整条**（不落地任何字段） | `FCR_CMD_FLAG_INVALID_VALUE` → `INVALID_INPUT` |
| `requested_effective_sim_time_s` NaN/Inf | **拒绝整条** | 同上 |
| `yaw_target_heading_rad` NaN/Inf | **拒绝该通道**（不锁存） | 同上 |
| `induction_ref` NaN/Inf | **拒绝该通道** | 同上 |
| `yaw_seq` / `induction_seq` 重复 | 接受（幂等，不续命 TTL） | — |
| `yaw_seq` / `induction_seq` 落后 | 拒绝该通道 | `FCR_CMD_FLAG_INVALID_SEQ` |
| `control_enable = 0` | 双通道清零，回退 ROSCO 原控制 | `FCR_CMD_FLAG_CONTROL_DISABLED` |
| `induction_ref < a_min` 或 `> a_max`（legacy） | 不采纳，回退额定 | `FCR_CMD_FLAG_FALLBACK_ACTIVE` |
| `a > a0(1/3)`（Mode1，物理不可达） | 不采纳，回退额定 | `FALLBACK` + `0x100 UNACHIEVABLE` |
| Mode2 无气动表 / 反查失败 | 不采纳，seq 不入账 | `FALLBACK` + `0x200 CT_LOOKUP_INVALID` 或 `0x100` |
| Mode3（闭环预留） | 不生效，显式拒绝 | `FALLBACK` + `0x100 UNSUPPORTED` |
| TTL 到期（无新命令） | 该通道失效 → 回退 ROSCO | `FCR_CMD_FLAG_*_TTL_EXPIRED` → `TIMEOUT`（边沿） |
| basis 距当前 > 5 s | 保持上一有效值 | `FCR_CMD_FLAG_*_STALE` → `STALE_DATA` |

### 1.2 S02/S03 平台状态（调度器 → 本模块）

| 条件 | 动作 | 说明 |
|---|---|---|
| `valid = 0` | 该机不参与聚合 | 状态帧对应机 `valid = 0` |
| `gen_power_w` / `nacelle_heading_rad` NaN/Inf（valid=1） | 上报 | `NUMERICAL_ERROR`（D4 建议） |
| `rotor_ct` ∉ (0,2) 或 NaN | `ct_hat = 0`（显式无估计） | 不冒充真实 Ct |
| `tower_base_*` NaN | 该统计保持 invalid | 离线接口不可得 |

### 1.3 S04/S05 AWAE（注入）

| 条件 | 动作 |
|---|---|
| `n_points` > 64 | **限幅**截断到 64 |
| 目录未注入 | 上送**合法空帧**（不删字段、不伪造点） |
| 合成/外部注入 | 质量位 `FCR_AWAE_QUALITY_SIMULATED` 可追溯 |

### 1.4 S06 风速估计

| 条件 | 动作 |
|---|---|
| `wind_quality_flags` 未置 OK | 不采用（保留原值） |
| `estimate_sim_time_s` 超龄（> flow_stale 3 s） | 不采用 + 置 `FCR_WIND_QUALITY_STALE` |

### 1.5 S10 状态帧（本模块 → 调度器 → RCP）

| 条件 | 动作 |
|---|---|
| 1 s 统计窗口未闭合 | 瞬时直通（`n_samples=1`） |
| 某机无有效状态 | 该机 `valid = 0`，其余机正常 |
| `Ct` 无真实值 | `ct_hat = 0`（不伪造） |

---

## 2. 时间语义表（§5.3：明确输入输出生效时刻）

### 2.1 调用周期

| 接口 | 周期 | 说明 |
|---|---|---|
| `fcr_rt_step` | **10 ms**（DT_BASE） | 调度器每周期调用一次；不自推时间 |
| SCADA 交换（`get_output` / `set_command`） | **1 s**（DT_SCADA） | 由 `steps_per_scada = dt_scada/dt_high` 计数器触发，与 DT_Low 解耦 |
| `fcr_rt_get_setpoint` | 10 ms 级 | ROSCO/单机控制在 10 ms 读取 |
| AWAE 二跳（`get_awae`） | 1 s 级 | S12/S13 |

### 2.2 生效时刻

| 输入 | 生效时刻 |
|---|---|
| 命令（`set_command`） | 调度器在 1 s 边界送入 → **下一个 10 ms 步**落地（不回溯） |
| `requested_effective_sim_time_s`(S0112) | 到该仿真时刻才生效（默认 0 = 本步） |
| 偏航目标 | 新 seq 时锁存，持续到 TTL 到期 |
| 诱导 setpoint | 每 10 ms 刷新（ZOH 到下一个命令） |
| 状态帧输出 | **下一个 DT_Low 生效**（平台确认） |
| AWAE 观测/目录 | 1 s 边界发布 |

### 2.3 超时与保持（§6.1 D1 最大保持时间）

| 通道 | TTL（最大保持） | stale 阈值 | 依据 |
|---|---|---|---|
| 偏航 | **60 s** | 5 s | `FcrRuntimeConfig.default_yaw_ttl_s` / `stale_threshold_s` |
| 诱导 | **5 s** | 5 s | `default_induction_ttl_s` |
| 流场（AWAE） | 3 s | — | `flow_stale_threshold_s` |

> **禁止无限保持**：TTL 到期即失效并回退 ROSCO 原控制（D3）。

---

## 3. 降级策略映射（§6.1 / §7.4）

| 级别 | 定义 | 本模块场景 | 模块行为 | 上报状态 |
|---|---|---|---|---|
| **D0** 正常继续 | 异常不影响运行 | AWAE 目录未注入；非关键诊断输出失败 | 上送空帧/跳过诊断 | `OK` / `WARNING` |
| **D1** 保持上一有效值 | 短时数据缺失 | 命令 basis 超龄（< TTL）；短时断连 | 沿用最后 setpoint（≤ TTL） | `STALE_DATA` |
| **D2** 默认/备用策略 | 主策略失效 | `control_enable=0`；Mode2 无表；诱导不可达 | 回退 ROSCO 标称（ratio=1） | `OK`/`WARNING` + `FALLBACK` 位 |
| **D3** Bypass 模块 | 长时通信失败 | 命令 TTL 到期且无新命令 | 退出外部控制（通道 enable=0），FAST.Farm 继续 | `TIMEOUT`（边沿触发一次） |
| **D4** 停止算例 | 物理结果不可信 | 状态量 NaN/Inf；内部不变量破坏 | 不修改输出，立即上报 | `NUMERICAL_ERROR` / `FATAL` |

**关键约束（§7.4）**：场控失效**不得**导致 FAST.Farm 物理模型崩溃——已由 PUB/SUB 非阻塞 +
全路径回退保证，并有实测（`test_offline_disconnect_fallback.py`、`rt_ttl_timeout_edge`）。

---

## 4. 对应测试用例

| 策略/语义 | 用例 |
|---|---|
| NaN 命令拒绝（S01） | `rt_reject_nan_command`、`test_command_store.c` |
| 乱序/重复 seq | `out_of_order_seq_rejected`、`duplicate_seq_idempotent` |
| `control_enable=0`（D2） | `control_disable_fallback`（command_store + rosco_api） |
| TTL 到期（D3） | `rt_ttl_timeout_edge`、`ttl_expiry`、`test_offline_disconnect_fallback.py` |
| stale 保持（D1） | `stale_flag`、`stale_flag_per_channel` |
| 诱导越界/Mode2 无表（D2） | `fallback_on_out_of_range`、`mode2_no_table_fallback` |
| Mode3 拒绝 | `mode3_unsupported` |
| 1 s 边界/调用次数 | `rt_scada_boundary_counter`、`rt_mock_scheduler_1000_cycles` |
| 执行时预算 | `rt_step_perf_budget` |
| Reset 可重复 | `rt_reset_repeatable` |

---

## 5. 待补（后续迭代）

| 项 | 说明 |
|---|---|
| D0 显式用例 | 诊断输出失败仍继续（当前无独立用例） |
| D2 显式用例 | 已有 `control_disable_fallback`，可再补"主控异常→标称"命名用例 |
| CASE-SCADA-ERR-01 强断言 | 需断言"保留最后有效设定 X 秒后退出外部控制"的精确时刻（现覆盖 TTL 边沿） |
| 多线程搬运假设 | 现假设调度器单线程调用 step/set_*；若多线程需补原子/双缓冲 |
