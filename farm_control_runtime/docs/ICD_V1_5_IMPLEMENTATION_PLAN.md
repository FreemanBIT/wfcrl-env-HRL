# ICD v1.5 信号接口修改计划

## 目标
按照 FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv 重新定义工程信号接口，
确保所有 13 个 Interface_Group、70 条信号 ID 均得到正确实现。

---

## 修改阶段

### Phase 1: 命令帧重构 (RCP_Command S0101-S0112)

#### 1.1 FcrTurbineCommand 变更

| 当前字段 | 操作 | 新字段 | ICD 信号 |
|----------|------|--------|----------|
| yaw_delta_rad | **删除** | — | — |
| yaw_seq | 保留 | yaw_seq | S0109 |
| yaw_valid | 保留 | yaw_valid | S0110 |
| yaw_effective_low_step | **替换** | requested_effective_sim_time_s | S0112 |
| yaw_ttl_s | 保留 | yaw_ttl_s | 内部实现 |
| yaw_absolute_heading_rad | **重命名** | yaw_target_heading_rad | S0108 |
| yaw_absolute_valid | **合并** | →yaw_valid | S0110 |
| induction_ref | 保留 | induction_ref | S0105 |
| induction_seq | 保留 | induction_seq | S0106 |
| induction_valid | 保留 | induction_valid | S0107 |
| induction_effective_low_step | **替换** | requested_effective_sim_time_s | S0112 |
| induction_ttl_s | 保留 | induction_ttl_s | 内部实现 |
| source_time_s | **重命名** | command_basis_sim_time_s | S0102 |
| — | **新增** | control_enable | S0104 |
| — | **新增** | control_mode | S0111 |

#### 1.2 FarmCommandFrame 头变更

| 当前字段 | 操作 | 新字段 | ICD 信号 |
|----------|------|--------|----------|
| protocol_version | 保留 | protocol_version | — |
| frame_seq | **重命名** | command_frame_seq | S0101 |
| commit_seq | **删除** | — | — |
| n_turbines | 保留 | n_turbines | — |
| receive_time_s | 保留 | receive_time_s | 内部实现 |

---

### Phase 2: 状态帧重构

#### 2.1 FcrRoscoFastState → S0301-S0303

| 当前字段 | 新字段 | ICD 信号 |
|----------|--------|----------|
| controller_status_flags | rosco_controller_status_flags | S0301 |
| gen_torque_cmd_nm | generator_torque_command_nm | S0302 |
| pitch_cmd_rad[3] | pitch_command_rad[3] | S0303 |

#### 2.2 FcrTurbineExtraState → S0203-S0210

已有字段映射：
- rotor_thrust_n → S0208 rotor_aero_thrust_n
- shaft_torque_nm → S0209
- tower_base_fa_moment_nm → S0210

需新增：
- blade_pitch_actual_rad[3] (S0206) — 从 FcrRoscoFastState 迁移

#### 2.3 FcrFarmTurbineState → S1001-S1017

已有字段：
- gen_power_w.instant → S1005 generator_power_instant_w
- gen_power_w.mean → S1004 generator_power_mean_1s_w
- rotor_thrust_n → S1010 rotor_aero_thrust_n
- nacelle_heading_rad → S1009
- yaw_seq_applied → S1013
- induction_seq_applied → S1014

需新增：
- estimated_wind_mps (S1011) — 来自 TurbineWindEstimate
- ct_hat (S1012) — 从推力和风估计计算
- turbine_status_flags (S1015) — ROSCO+Bridge 综合状态
- wind_source (S1016) — 0=unknown, 1=ROSCO_WE, 2=OpenFAST, 3=AWAE
- wind_quality_flags (S1017) — 风估计质量

---

### Phase 3: 新增接口组结构

#### 3.1 AWAE_Catalog (S0401-S0403)

```c
typedef struct {
    uint32_t  point_id;                    /* S0401 */
    double    point_position_xyz_m[3];     /* S0402 */
    uint32_t  point_catalog_version;       /* S0403 */
} FcrAWAECatalogEntry;

typedef struct {
    uint32_t  protocol_version;
    uint64_t  frame_seq;
    uint32_t  n_points;
    FcrAWAECatalogEntry entries[FCR_MAX_OBSERVATION_POINTS];
} FcrAWAECatalogFrame;
```

#### 3.2 AWAE_Observation (S0501-S0507)

```c
typedef struct {
    uint64_t  observation_frame_seq;       /* S0501 */
    double    observation_sample_sim_time_s; /* S0502 */
    uint32_t  observation_point_count;     /* S0503 */
    uint32_t  point_id;                    /* S0504 */
    float     wind_uvw_mps[3];             /* S0505 */
    uint16_t  observation_quality_flags;   /* S0506 */
    float     turbulence_intensity;        /* S0507 */
} FcrAWAEObservation;

typedef struct {
    uint32_t  protocol_version;
    uint64_t  frame_seq;
    uint32_t  n_observations;
    FcrAWAEObservation observations[FCR_MAX_OBSERVATION_POINTS];
} FcrAWAEObservationFrame;
```

#### 3.3 TurbineWindEstimate (S0601-S0604)

```c
typedef struct {
    int32_t   turbine_id;
    double    estimated_wind_mps;          /* S0601 */
    double    estimate_sim_time_s;         /* S0602 */
    uint16_t  wind_source;                 /* S0603: 0=unk,1=ROSCO_WE,2=OF,3=AWAE */
    uint16_t  wind_quality_flags;          /* S0604 */
} FcrTurbineWindEstimate;
```

#### 3.4 Bridge_Ack (S0901-S0904)

新增到 FcrRoscoFastState 或独立结构：
- bridge_status_flags (S0903) — 桥接器健康/状态位
- ct_ref (S0904) — 跟踪的等效推力系数目标

---

### Phase 4: Bridge_to_ROSCO 字段重命名 (S0801-S0807)

| 当前字段 | 新字段 | ICD 信号 |
|----------|--------|----------|
| — | external_yaw_enable | S0801 |
| yaw_target_heading_rad | yaw_target_heading_rad | S0802 |
| power_ratio | r_pitch | S0803 |
| speed_ref_ratio | r_speed | S0804 |
| torque_limit_ratio | r_torque | S0805 |
| min_pitch_rad | min_pitch_ref_rad | S0806 |
| — | collective_pitch_offset_rad | S0807 |

---

### Phase 5: Scheduler_to_Bridge (S0701-S0710)

```c
typedef struct {
    double    bridge_step_sim_time_s;      /* S0701 */
    uint16_t  bridge_command_valid_flags;  /* S0702 */
    uint32_t  turbine_id;                  /* S0703 */
    uint8_t   control_enable;              /* S0704 */
    double    induction_ref;               /* S0705 */
    uint64_t  induction_seq;               /* S0706 */
    double    yaw_target_heading_rad;      /* S0707 */
    uint64_t  yaw_seq;                     /* S0708 */
    uint16_t  control_mode;                /* S0709 */
    double    command_basis_sim_time_s;    /* S0710 */
} FcrBridgeInput;
```

---

### Phase 6: ZMQ Transport 更新

- 更新帧验证逻辑（新结构大小）
- 更新 Python ctypes 镜像
- 更新单元测试 (test_abi.c)

---

### Phase 7: Bridge Core & ROSCO API 更新

- fcr_bridge_core.c: 使用新字段名
- FarmControlInterface.f90: 更新 FCR_ApplyReferences
- command_store.c: 处理 control_enable, control_mode

---

## 修改文件清单

### 修改 (13 个文件)

| 文件 | 修改内容 |
|------|----------|
| include/fcr_state_types.h | 重大重构：命令帧、状态帧、新增 AWAE 结构 |
| include/fcr_protocol.h | 版本号升级、新常量 |
| src/command_store.c | 处理 control_enable, control_mode |
| src/induction_supervisor.c | 字段重命名 |
| bridge/fcr_bridge_core.c | 字段重命名、新增字段 |
| bridge/yaw_adapter.c | 删除 delta 支持（ICD v1.5 仅绝对目标） |
| rosco/FarmControlInterface.f90 | 更新 FCR_ApplyReferences |
| transport/zmq/zmq_transport.cpp | 新帧大小验证 |
| src/rosco_api.c | 映射新字段 |
| src/runtime.c | 处理新状态字段 |
| wfcrl/transport/farm_protocol.py | 更新 ctypes 镜像 |
| tests/unit/test_abi.c | 更新大小/偏移 |
| offline/provider/offline_fastfarm_provider.c | 新状态字段 |

### 新增 (3 个文件)

| 文件 | 说明 |
|------|------|
| include/fcr_awae_types.h | AWAE catalog/observation 类型 |
| awae/observation_manager.c | S04/S05/S12/S13 处理 |
| awae/wind_mapper.c | S06 风估计映射 |

---

## 验收标准

1. 所有 13 个 Interface_Group 均有对应 C 结构实现
2. 所有 70 个 Signal_ID 字段均可通过 ZMQ 帧正确传输
3. 单元测试 100% 通过（更新后的 test_abi.c）
4. 300s FAST.Farm 回归测试通过
5. Python ctypes 镜像与 C 结构完全一致
6. 多机隔离测试通过（无串扰）
