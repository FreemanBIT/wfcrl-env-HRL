/*
 * fcr_state_types.h — farm_control_runtime 状态/命令帧类型（Phase 1 冻结）
 *
 * 所有结构保持 C ABI（无 vtable、无 STL）；多速率来源各自带 source_time/source_seq。
 * 命名与 fastfarm_rt_interface_variables.csv 的标量名一一对应（见 ICD）。
 */
#ifndef FCR_STATE_TYPES_H
#define FCR_STATE_TYPES_H

#include "fcr_protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/* 1. 命令帧（Farm Controller → runtime）                             */
/* ================================================================== */

/* 每台机的独立异步通道命令（yaw / induction 互不覆盖）               */
typedef struct {
    /* ---- 偏航通道 ---- */
    double    yaw_delta_rad;           /* 相对生效时实际机舱位置的增量角，CW+ */
    uint64_t  yaw_seq;                 /* 偏航动作序号（单调递增，从 1 开始） */
    uint32_t  yaw_valid;               /* 本条偏航命令有效标志               */
    int64_t   yaw_effective_low_step;  /* 目标生效的 FAST.Farm 低速步号     */
    double    yaw_ttl_s;               /* 偏航命令有效时长（s），0 = 无限    */

    /* ---- 诱导通道 ---- */
    double    induction_ref;           /* 轴向诱导因子目标（无量纲 [0,1)）  */
    uint64_t  induction_seq;           /* 诱导动作序号（单调递增，从 1 开始） */
    uint32_t  induction_valid;         /* 本条诱导命令有效标志               */
    int64_t   induction_effective_low_step; /* 目标生效低速步号             */
    double    induction_ttl_s;         /* 诱导命令有效时长（s），0 = 无限    */

    double    source_time_s;           /* 本条命令的发出时间（控制器侧）     */
} FcrTurbineCommand;

/* 整场命令帧（Transport 层原子帧）                                    */
typedef struct {
    uint32_t  protocol_version;        /* = FCR_PROTOCOL_VERSION            */
    uint64_t  frame_seq;               /* 帧序号（transport 单调递增）      */
    uint64_t  commit_seq;              /* 原子提交序号（Modbus commit 用）  */
    uint32_t  n_turbines;              /* 本帧包含的机组数                  */
    double    receive_time_s;          /* 帧到达时间（runtime 侧，s）       */
    FcrTurbineCommand turbines[FCR_MAX_TURBINES];
} FarmCommandFrame;

/* ================================================================== */
/* 2. 信号统计（1 s 窗口）                                            */
/* ================================================================== */
typedef struct {
    double    instant;                 /* 最新瞬时值                        */
    double    mean;                    /* 窗口均值                          */
    double    rms;                     /* 窗口 RMS（含均值偏移）            */
    double    min;                     /* 窗口最小值                        */
    double    max;                     /* 窗口最大值                        */
    uint32_t  n_samples;               /* 窗口内采样数                      */
    uint32_t  valid;                   /* 统计有效标志                      */
} FcrSignalStats;

/* ================================================================== */
/* 3. ROSCO 快速状态（ROSCO → runtime，~10 ms）                       */
/* ================================================================== */
typedef struct {
    int32_t   turbine_id;
    uint32_t  valid;
    double    source_time_s;           /* rosco_time_s */
    uint64_t  source_seq;              /* fast_step（可选由 clock 提供）   */

    double    mech_gen_power_w;        /* VS_MechGenPwr                     */
    double    gen_power_w;             /* VS_GenPwr                         */
    double    gen_speed_rad_s;         /* GenSpeed                           */
    double    rotor_speed_rad_s;       /* RotSpeed                           */
    double    gen_torque_meas_nm;      /* GenTqMeas                          */
    double    nacelle_heading_rad;     /* NacHeading（OpenFAST 内部约定）   */
    double    nacelle_vane_rad;        /* NacVane                            */
    double    hub_wind_speed_mps;      /* HorWindV                           */
    double    blade_pitch_rad[3];      /* avrSWAP blade pitch               */
    double    blade_root_moop_nm[3];   /* rootMOOP(1:3)                      */
    double    tower_top_fa_acc_mps2;   /* FA_Acc                             */
    double    nacelle_imu_fa_acc_mps2; /* NacIMU_FA_Acc                      */
    double    rotor_azimuth_rad;       /* Azimuth                            */
    double    gen_torque_cmd_nm;       /* 最终转矩命令                       */
    double    yaw_rate_cmd_rad_s;      /* 最终偏航速率命令                   */
    double    pitch_cmd_rad[3];        /* 最终桨距命令                       */
    double    yaw_target_heading_rad;  /* 当前外部目标机舱方位（锁存值）    */
    uint64_t  yaw_seq_applied;         /* 已应用偏航动作序号                 */
    uint64_t  induction_seq_applied;   /* 已应用诱导动作序号                 */
    uint32_t  controller_status_flags; /* FCR_CTRL_FLAG_*                    */
} FcrRoscoFastState;

/* ================================================================== */
/* 4. OpenFAST 额外机组状态（Provider → runtime，优选 ~10 ms）        */
/* ================================================================== */
typedef struct {
    int32_t   turbine_id;
    uint32_t  valid;
    double    source_time_s;
    uint64_t  source_seq;

    double    rotor_thrust_n;          /* 转子总推力                         */
    double    shaft_torque_nm;         /* 低速轴转矩                         */
    double    tower_base_fa_moment_nm; /* 塔基前后向弯矩                     */
    double    tower_base_ss_moment_nm; /* 塔基侧向弯矩                       */
    double    rotor_ct;                /* 转子推力系数（无量纲）            */
    double    rotor_cp;                /* 转子功率系数（无量纲）            */

    /* 可配置载荷测点：行 = [Fx,Fy,Fz,Mx,My,Mz]，列 = point_id           */
    double    load_point_f[6][FCR_MAX_LOAD_POINTS];
} FcrTurbineExtraState;

/* ================================================================== */
/* 5. AWAE/流场状态（Provider → runtime，~1 s，ZOH）                  */
/* ================================================================== */
typedef struct {
    uint32_t  valid;
    double    flow_source_time_s;      /* AWAE 快照对应仿真时间             */
    uint64_t  flow_seq;                /* AWAE 快照序号（单调递增）         */
    uint32_t  n_points;                /* 实际观察点数                      */
    uint32_t  n_turbines;              /* 实际机组数                        */

    double    ambient_u[FCR_MAX_OBSERVATION_POINTS];
    double    ambient_v[FCR_MAX_OBSERVATION_POINTS];
    double    ambient_w[FCR_MAX_OBSERVATION_POINTS];
    double    disturbed_u[FCR_MAX_OBSERVATION_POINTS];
    double    disturbed_v[FCR_MAX_OBSERVATION_POINTS];
    double    disturbed_w[FCR_MAX_OBSERVATION_POINTS];
    double    point_ti[FCR_MAX_OBSERVATION_POINTS];

    double    disk_ambient_wind_mps[FCR_MAX_TURBINES];
    double    disk_disturbed_wind_mps[FCR_MAX_TURBINES];
    double    disk_ambient_ti[FCR_MAX_TURBINES];
} FcrFlowState;

/* ================================================================== */
/* 6. FarmStateFrame（runtime → Farm Controller，~1 s 聚合发布）       */
/* ================================================================== */
typedef struct {
    int32_t   turbine_id;
    uint32_t  valid;
    double    source_time_s;           /* 本机组快照时间                    */

    /* 高速量：瞬时 + 1 s 统计                                           */
    FcrSignalStats gen_power_w;
    FcrSignalStats rotor_thrust_n;
    FcrSignalStats tower_base_fa_moment_nm;
    FcrSignalStats tower_base_ss_moment_nm;

    double    nacelle_heading_rad;     /* 瞬时                              */
    double    nacelle_vane_rad;
    double    hub_wind_speed_mps;
    double    rotor_speed_rad_s;
    double    gen_speed_rad_s;
    double    rotor_ct;                /* 瞬时                              */
    double    rotor_cp;
    double    blade_pitch_rad[3];      /* 瞬时                              */
    double    yaw_target_heading_rad;  /* 锁存目标                          */
    uint64_t  yaw_seq_applied;
    uint64_t  induction_seq_applied;
    uint32_t  controller_status_flags;

    /* 低速量（AWAE，1 s 之间 ZOH）                                      */
    double    disk_ambient_wind_mps;
    double    disk_disturbed_wind_mps;
    double    disk_ambient_ti;
    double    flow_source_time_s;      /* 流场快照时间（age 计算）         */
    uint64_t  flow_seq;
} FcrFarmTurbineState;

typedef struct {
    uint32_t  protocol_version;
    uint32_t  frame_seq;               /* 状态帧序号（transport 单调递增） */
    double    sim_time_s;              /* 帧生成仿真时间                    */
    uint64_t  fast_step;
    uint64_t  low_step;
    uint32_t  rt_health_flags;         /* FCR_RT_HEALTH_*                   */
    uint32_t  n_turbines;
    double    flow_source_time_s;      /* 最新流场时间（age 计算）         */
    uint64_t  flow_seq;
    FcrFarmTurbineState turbines[FCR_MAX_TURBINES];
} FarmStateFrame;

#ifdef __cplusplus
}
#endif

#endif /* FCR_STATE_TYPES_H */
