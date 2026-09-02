# OFFLINE_PROVIDER — Offline FAST.Farm Provider 说明（Phase 4）

## 1. 目标

使离线 FAST.Farm 输出与未来 RT Provider 具有**相同的数据语义**（push 型
Provider API），并完成离线闭环所需的多速率状态采集（Phase 4：快速机组状态）。

## 2. 部署约束与探针架构

离线 FAST.Farm 以预编译二进制运行（RI 团队负责的 FAST.Farm RT 才允许修改
scheduler 内部）。因此 Offline Provider 采用 **DISCON DLL 进程内探针**：

```text
FAST.Farm_x64_OMP.exe（预编译）
   └─ ServoDyn → DISCON_WT1.dll
        ├─ ROSCO（10 ms 控制）
        ├─ FarmControlInterface（FCR_Step/Read/Publish）
        ├─ Offline Provider（本模块，每 10 ms）
        │     ├─ fcr_offline_provider_publish_fast()
        │     └─ fcr_publish_rt_clock()
        └─ farm_control_runtime（共享内存状态库）
```

## 3. 快速状态（~10 ms）标量来源（Bladed DLL avrSWAP 记录）

| Provider 字段 | avrSWAP record | 来源（OpenFAST ServoDyn） | 单位 |
|---|---|---|---|
| rotor_thrust_n | 110 | LSShftFxa（低速轴 GL x 分量；对风 ≈ 推力） | N |
| shaft_torque_nm | 109 | LSSTipMxa/LSShftMxs（主轴扭矩） | N·m |
| load_point_f[0..2][0] | 110/111/112 | 轮毂（低速轴）力 Fx/Fy/Fz，GL 坐标 | N |
| load_point_f[4..5][1] | 77/78 | 偏航轴承弯矩 My/Mz，GL 坐标 | N·m |
| sim_time/fast_step | LocalVar%Time / floor(t/0.01) | — | s / count |
| low_step | floor(t/DT_low)，DT_low 来自 env FCR_CFG_DT_LOW | — | count |

**不可得量（DLL 接口限制，已冻结语义）**：

- `tower_base_fa/ss_moment_nm`：valid=0 + NaN（ServoDyn 接口无塔基弯矩记录；
  RT Provider 必须从 ElastoDyn 提供）；
- `rotor_ct/rotor_cp`：由 runtime 侧派生（Phase 5/6：ct=2T/(ρA V²)，
  cp=P/(0.5ρA V³)，V 取 disk disturbed wind），Provider 只发布原始推力与功率。

## 4. 红线合规

- 不读取 `.out/.outb` 给闭环提供状态（Line-12 红线）；
- 10 ms 控制路径无网络/文件 I/O（只有共享内存写入）；
- source_time = LocalVar%Time（与 ROSCO 时间完全一致，无插值）。

## 5. 配置

环境变量（离线 harness 启动 FAST.Farm 前设置）：

```text
FCR_CFG_DT_LOW=3.0     FAST.Farm DT_Low（用于 low_step 推算；缺省 1.0）
FCR_CFG_DT_HIGH=0.01   DT_High（缺省 0.01）
```

## 6. 单元测试与校核（26/26 通过）

- 单元测试：`tests/unit/test_offline_provider.c`（映射/路由/时钟/不可得语义）；
- 实测校核（Row3T 3 机 8 m/s 90 s 仿真）：
  - 主轴扭矩：Provider vs Case.T*.outb `RotTorq`，mean|Δ|=0.1 kNm（一致）；
  - 偏航轴承弯矩：Provider vs outb `YawBrMyp`，稳态 max|Δ|<6 kNm（启动瞬态采样差）；
  - 三机 turbine_id 路由正确；source_time 与 ROSCO 一致。

## 7. 与 RT Provider 的差异（ICD 注明）

| 量 | Offline Provider | RT Provider（RT 团队） |
|---|---|---|
| thrust/shaft | ServoDyn 接口记录 | AeroDyn/传动链内部量 |
| tower base | 不可得（valid=0） | ElastoDyn 塔基载荷 |
| Ct/Cp | runtime 派生（同式） | AeroDyn 直接聚合 |
| clock | DISCON 时间推算 | scheduler 时钟 |

数据语义（单位/坐标/source time/seq）保持一致。
