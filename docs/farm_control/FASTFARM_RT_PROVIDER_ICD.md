# FAST.Farm RT Provider ICD（Phase 10 — 冻结）

## 1. 范围与参考

本文档冻结 FAST.Farm RT（实时）部署中 **RT Provider** 与 **farm_control_runtime**
的数据契约。RT Provider 是指在 FAST.Farm 调度器侧（FAST.Farm.F90 / FAST_Subs）
实现的采集模块，通过 `fcr_provider_api.h`（Phase 1 ABI 冻结）推送每机状态。

参考：
- `farm_control_runtime/include/fcr_provider_api.h`（接口 ABI）
- `farm_control_runtime/include/fcr_fastfarm_rt_provider.h`（RT 实现契约）
- `farm_control_runtime/offline/provider/OFFLINE_PROVIDER.md`（离线探针实现，语义同源）
- 供应商 demo `fastfarm_rt_interface_variables.csv`（逐变量对照本表）

## 2. 逐标量 ICD（每机，RT 侧 → runtime）

### 2.1 快速量（~1 步/10 ms，FcrTurbineExtraState）

| 字段 | demo 变量 | RT 取数点 | 单位 | 语义 | 冻结状态 |
|---|---|---|---|---|---|
| rotor_thrust_n | LSShftFxa | 低速轴 GL x 分量（对风≈推力） | N | 瞬时 | ✓（离线校验 vs outb） |
| shaft_torque_nm | LSShftMxs/LSSTipMxa | 主轴扭矩 | N·m | 瞬时 | ✓（离线均值偏差 0.1 kNm） |
| load_point_f[0..2][0] | LSShftFxa/Fya/Fza | 轮毂力 GL 坐标 | N | 瞬时 | ✓ |
| load_point_f[4..5][1] | YawBrMyn/Mzn | 偏航轴承弯矩 My/Mz GL | N·m | 瞬时 | ✓ |
| tower_base_fa_moment_nm | TwrBsMyt | **ElastoDyn 塔基弯矩（RT 必须提供）** | N·m | 瞬时 | 离线：valid=0+NaN；RT：valid=1 |
| tower_base_ss_moment_nm | TwrBsMxt | 同上（侧向） | N·m | 瞬时 | 离线：valid=0+NaN；RT：valid=1 |
| hub_wind_speed_mps | Wind1VelX（hub） | 轮毂风 x 分量 | m/s | 瞬时 | ✓ |
| gen_power_w / 其它 ROSCO 量 | 见 FcrRoscoFastState | ROSCO 输出槽 | — | 瞬时 | ✓ |

### 2.2 流场/聚合量（~1 s，FcrFlowState + 聚合器）

| 字段 | 语义 | 冻结状态 |
|---|---|---|
| disk_ambient_wind_mps | 轮盘环境风（AWAE 输出） | ✓（Phase 5 ZOH 语义） |
| disk_disturbed_wind_mps | 扰动风（ADM 输出） | ✓ |
| disk_ambient_ti | 环境湍流强度 | ✓（RT 由 AWAE 提供；离线恒 0 + invalid） |
| gen_power 1s 统计（mean/RMS/min/max） | 1 s 窗口统计 | ✓（Phase 5/6） |
| Ct/Cp | **runtime 派生**（ct=2T/(ρA V²)、cp=P/(0.5ρA V³)，V=disk disturbed） | ✓（离线派生；RT 可直采） |

### 2.3 时钟/健康

| 字段 | 语义 | 冻结状态 |
|---|---|---|
| sim_time_s / fast_step / low_step | 全场一致时钟（RT 侧调度器挂钟） | ✓ |
| rt_health_flags | FCR_RT_HEALTH_OK / WARN / ERROR | ✓ |

## 3. RT 侧实现指南（供 FAST.Farm 源码开发参考）

```fortran
! FAST.Farm 调度器内（FAST_FarmSubstep 末，每 turbine）：
!   call fcr_offline_provider_publish_fast(tid, t, avrSWAP)  ! RT 版：直接填结构
! RT 实现要点：
!   1. 每 10 ms 步调用一次 fcr_provider_publish_fast（=离线同名函数语义）；
!   2. 每 DT_Low 步调用 fcr_provider_publish_flow（tower/轮盘数据齐备）；
!   3. tower_base 弯矩从 ElastoDyn APA 取（离线 DLL 无此量，valid=0）；
!   4. 勿在 ROSCO 10 ms 路径做文件/网络 I/O（红线 1）；
!   5. 时钟以调度器统一时间推进（多机同步），勿依赖各机 LocalVar%Time 累计差。
```

## 4. 测试向量

### 4.1 离线往返（已在 Phase 4 执行）

| 量 | 期望 | 实测 | 偏差 |
|---|---|---|---|
| RotTorq（主轴扭矩） | 无偏 | mean \|Δ\| | 0.1 kNm |
| YawBrMyp | 一致 | 趋势吻合 | — |
| hub wind（StateFrame vs outb） | <3 m/s | 通过（integration） | — |
| thrust（avrSWAP 110） | 在线非零 | 2.4e5 N 量级 | — |

### 4.2 RT 验收建议（现场）

1. 空载 60 s：三机 fast_step 单调、source_time 单调且各机偏差 <0.5 s；
2. 全局 yaw 命令：seq 生效 ≤2 s；目标差 ≤0.5°（含滞后 settle）；
3. 全局 induction 命令：a=0.2 生效 ≤2 s，稳态降载进入 [10%,17%]（min_pitch_gain=0.55 @8 m/s 标定）；
4. 拔网：命令 TTL 到期回退，仿真不中断；
5. tower_base 施加已知弯矩（FAST 单位测试）→ RT Provider 数值一致。

## 5. 冻结声明

- 结构布局（FcrTurbineExtraState / FcrFlowState / FcrFarmTurbineState 等）冻结于
  Phase 1 ABI（fcr_state_types.h + tests/unit/test_abi.c）；
- 字段语义与 valid 组合冻结于本文档；后续变更必须走 ICD 修订流程（版本号 + 兼容性说明）；
- 离线 Provider 与 RT Provider 共享同一数据语义（红线 4 的实现保障）。
