# PROTOCOL_DECISIONS — 冻结的控制协议决策记录（Phase 0/1）

> 本文档冻结 `farm_control_runtime` 与 Farm Controller / ROSCO 之间的**控制语义**。
> 后续 Phase 的编码不得偏离本文档；如需修订，必须先改本文件再改代码。

## 1. 动作语义总则

Farm Controller 对每台机输出**两个独立异步通道**，各自携带独立序号（seq）：

```text
yaw_delta_rad  + yaw_seq       （典型 ~60 s 更新一次）
induction_ref  + induction_seq （典型 ~1 s 更新一次）
```

- 通道之间**互不覆盖、互不依赖**；下发任意一个通道不得影响另一个通道的
  当前生效值、seq 与 TTL。
- 每个通道的字段：

```text
yaw_delta_rad / induction_ref      值
yaw_seq / induction_seq            动作序号（uint64，从 1 开始，单调递增）
yaw_valid / induction_valid        本条命令有效标志
yaw_effective_low_step             命令生效的低速步号（FAST.Farm low_step）
yaw_ttl / induction_ttl            有效时间（s），超时后自动失效并回退
```

## 2. 偏航动作

### 2.1 定义

- `yaw_delta_rad`：相对**动作生效时当前实际机舱位置**的增量角；
- **顺时针为正**（CW+，从机舱上方俯视，北→东为正，即罗盘方位角增加方向）；
- 单位：rad；典型值约 ±15°（约 ±0.26 rad），更新周期约 60 s。

### 2.2 生效规则（runtime 内）

新 `yaw_seq` 生效时**只计算一次**绝对目标并锁存：

```text
heading0       = latest nacelle_heading（动作生效瞬间的机舱方位，OpenFAST 内部约定）
delta_internal = convert(CW+ → OpenFAST 内部约定)
target         = wrap_pm180(heading0 + delta_internal)
```

- 之后 `target` 保持不变，直至下一个新 seq；
- **禁止**每 10 ms 重复执行 `current_heading + yaw_delta`；
- 重复 seq 幂等：相同 seq 不得重新计算 target，不得重复执行；
- 乱序 seq（seq ≤ 已应用 seq）拒绝；
- target 下发到 ROSCO 的 `FarmYawTargetHeading`，ROSCO 保留原有
  yaw-rate limit / deadband / state machine，只替换"目标来源"。

### 2.3 角度约定与转换

| 量 | 约定 |
|---|---|
| NacHeading（OpenFAST/ROSCO LocalVar） | 0°=+X（东），逆时针为正（CCW+），[0, 2π) 或 [−π, π] |
| yaw_delta（Farm Controller） | 顺时针为正（CW+，罗盘方向） |
| yaw_target_heading_rad（下发 ROSCO） | 与 NacHeading 相同内部约定（CCW+） |

转换关系（由 `angle_convention.c` 唯一实现，禁止散落各处）：

```text
delta_internal = -yaw_delta                （CW+ → CCW+ 取反）
target         = wrap(heading0 + delta_internal)
```

RT 交付时明示：**RT 侧不得再次反号**。

## 3. 诱导因子动作

### 3.1 定义

- `induction_ref`：轴向诱导因子目标，无量纲，取值范围 [0, 1)，典型 ~0.1–0.4；
- 更新周期约 1 s。

### 3.2 生效规则

- runtime 收到新 `induction_seq` 时，把 `induction_ref` 交给
  InductionSupervisor 做逐机映射，输出 ROSCO supervisory references：

```text
ct_ref            目标转子推力系数（无量纲）
power_ratio       功率参考比例（无量纲，1.0 = 额定）
speed_ref_ratio   转速参考比例（无量纲，1.0 = 额定）
torque_limit_ratio 转矩限值比例（无量纲，1.0 = 额定）
min_pitch_rad     最小桨距约束（rad）
```

- 1 s 周期之间 ZOH（保持最新值）；
- 越界/NaN 时 fallback 到安全默认值并置 command_status_flags 降级位；
- 实际首版启用哪些输出通道由 ROSCO 控制路径验证结果决定（见
  farm_control_runtime/rosco/ROSCO_INTEGRATION.md）。

## 4. 命令生命周期与安全

- **TTL**：每个通道带 TTL；TTL 到期（期间无新 seq）→ 该通道自动失效，
  视情况回退（yaw 回退 ROSCO 原 yaw 控制器；induction 回退额定参考）。
- **Stale 检测**：source_time 与当前 sim_time 之差超过阈值视为 stale。
- **健康标志**：command_status_flags / controller_status_flags 位定义在
  fcr_protocol.h 冻结，后续 ICD 沿用。

## 5. 多速率与同步

| 数据 | 周期 | 同步策略 |
|---|---|---|
| ROSCO/OF fast state | ~10 ms | 最新快照；载荷量同时做 1 s 统计（mean/RMS/min/max） |
| AWAE flow state | ~1 s | 1 s 之间 ZOH，不伪造 10 ms 时间戳；带 flow_source_time/flow_seq |
| FarmStateFrame | ~1 s | 多源聚合后发布 |
| 命令生效 | low_step 边界 | effective_low_step 记录 |

- 每个状态源带 source_time/source_seq，供 age 与丢帧检测。
- 高速（10 ms）数据在 1 s 聚合窗口：instant/mean/RMS/min/max 全部保留。

## 6. ABI 冻结原则（Phase 1 一次性冻结）

- fcr_protocol.h：帧结构与协议版本号。
- fcr_state_types.h：状态结构（保持 C ABI 稳定，sizeof/align 有测试）。
- fcr_transport.h：poll_command() / publish_state()，ZMQ/Modbus 均只实现
  该接口，不得实现控制算法。
- fcr_provider_api.h：push 型 Provider API（fcr_publish_rt_clock/
  fcr_publish_turbine_extra_state/fcr_publish_flow_state），Offline Provider
  与未来 RT Provider 调用同一组函数。
- fcr_rosco_api.h：runtime ↔ ROSCO 的 C/Fortran ABI。
- fcr_fastfarm_rt_provider.h：给 RT 团队实现的 Provider 入口（Phase 10 冻结）。

## 7. 禁止事项（编码红线）

1. ROSCO 10 ms 控制路径内**禁止任何网络 I/O、文件 I/O**（仅共享内存读写）。
2. yaw_delta 语义 ≠ Y_MErrSet（偏航误差设定），不得混用。
3. Offline Provider **不得通过读 .out/.outb 给闭环提供状态**。
4. AWAE 1 s 数据不得标成 10 ms 新数据。
5. 角度与单位转换一律走 angle_convention.c。
6. yaw/induction 各自独立 seq，不得合并或互相推导。
