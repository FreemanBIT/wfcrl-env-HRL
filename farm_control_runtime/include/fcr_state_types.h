/*
 * fcr_state_types.h — farm_control_runtime 状态/命令帧类型（ICD v1.5）
 *
 * 对应 FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv 信号定义:
 *   S0101-S0112  RCP_Command
 *   S0201-S0210  OpenFAST_RawState
 *   S0301-S0303  ROSCO_RawState
 *   S0701-S0710  Scheduler_to_Bridge
 *   S0801-S0807  Bridge_to_ROSCO
 *   S0901-S0904  Bridge_Ack_to_Scheduler
 *   S1001-S1017  Scheduler_to_RCP_Turbine
 *
 * 所有结构保持 C ABI（无 vtable、无 STL）；多速率来源各自带 source_time/source_seq。
 */
#ifndef FCR_STATE_TYPES_H
#define FCR_STATE_TYPES_H

#include "fcr_protocol.h"
#include "fcr_awae_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/* 1. RCP_Command (S0101-S0112) — Farm Controller → runtime           */
/* ================================================================== */

/* 每台机的独立异步通道命令（yaw / induction 互不覆盖）               */
typedef struct {
    /* ---- 偏航通道 (S0108-S0110) ---- */
    double    yaw_target_heading_rad;      /* S0108: 绝对机舱目标方位角 */
    uint64_t  yaw_seq;                     /* S0109: 偏航指令序号 */
    uint32_t  yaw_valid;                   /* S0110: 偏航目标有效标记 */

    /* ---- 诱导通道 (S0105-S0107) ---- */
    double    induction_ref;               /* S0105: 目标轴向诱导因子 */
    uint64_t  induction_seq;               /* S0106: 诱导指令序号 */
    uint32_t  induction_valid;             /* S0107: 诱导目标有效标记 */

    /* ---- 通用控制字段 ---- */
    uint32_t  control_enable;              /* S0104: 1=允许外部场控, 0=回退ROSCO */
    uint16_t  control_mode;                /* S0111: 0=基线,1=PRC,2=Ct前馈,3=预留 */
    double    requested_effective_sim_time_s; /* S0112: 期望生效仿真时刻 */

    /* ---- 时间戳 ---- */
    double    command_basis_sim_time_s;    /* S0102: 命令依据的仿真时刻 */

    /* ---- 内部实现字段（不在 ICD 中但必需） ---- */
    double    yaw_ttl_s;                   /* 偏航命令有效时长（s），0 = 无限 */
    double    induction_ttl_s;             /* 诱导命令有效时长（s），0 = 无限 */
} FcrTurbineCommand;

/* control_mode 枚举 */
#define FCR_CONTROL_MODE_BASELINE   0u   /* 基线：ROSCO 原始控制 */
#define FCR_CONTROL_MODE_PRC        1u   /* ROSCO PRC 比例开环 */
#define FCR_CONTROL_MODE_CT_FEED    2u   /* Ct 气动表前馈 */
#define FCR_CONTROL_MODE_CLOSEDLOOP 3u   /* 闭环预留（不得报告成功） */

/* 整场命令帧（Transport 层原子帧）                                    */
typedef struct {
    uint32_t  protocol_version;            /* = FCR_PROTOCOL_VERSION_V2 */
    uint64_t  command_frame_seq;           /* S0101: 全场命令快照序号 */
    uint32_t  n_turbines;                  /* 本帧包含的机组数 */
    double    receive_time_s;              /* 帧到达时间（runtime 侧，内部） */
    FcrTurbineCommand turbines[FCR_MAX_TURBINES];
} FarmCommandFrame;

/* ================================================================== */
/* 2. OpenFAST_RawState (S0201-S0210) — Provider → runtime            */
/* ================================================================== */

typedef struct {
    int32_t   turbine_id;                  /* S0201: 机组编号 */
    uint32_t  valid;
    double    of_sample_sim_time_s;        /* S0202: OpenFAST状态采样仿真时刻 */
    uint64_t  source_seq;                  /* 内部：fast_step */

    double    rotor_speed_rad_s;           /* S0203: 风轮转速（低速轴） */
    double    generator_speed_rad_s;       /* S0204: 发电机转速（高速轴） */
    double    generator_electrical_power_w; /* S0205: 发电机实际电功率 */
    double    blade_pitch_actual_rad[3];   /* S0206: 三叶片实际桨距角数组 */
    double    nacelle_heading_actual_rad;  /* S0207: 机舱实际绝对方位角 */
    double    rotor_aero_thrust_n;         /* S0208: 风轮轴向气动推力 */
    double    shaft_torque_nm;             /* S0209: 低速轴机械转矩 */
    double    tower_base_fa_moment_nm;     /* S0210: 塔基前后向弯矩 */
} FcrOpenFASTRawState;

/* ================================================================== */
/* 3. ROSCO_RawState (S0301-S0303) — ROSCO → runtime                  */
/* ================================================================== */

typedef struct {
    int32_t   turbine_id;
    uint32_t  valid;
    double    source_time_s;               /* rosco_time_s */
    uint64_t  source_seq;                  /* fast_step */

    uint32_t  rosco_controller_status_flags; /* S0301: ROSCO运行/保护状态标志 */
    double    generator_torque_command_nm;   /* S0302: ROSCO发电机转矩命令 */
    double    pitch_command_rad[3];          /* S0303: ROSCO三叶片桨距命令数组 */

    /* 扩展字段（用于 Bridge 内部逻辑，不在 ICD S03 组中） */
    double    mech_gen_power_w;            /* VS_MechGenPwr */
    double    gen_power_w;                 /* VS_GenPwr */
    double    gen_speed_rad_s;             /* GenSpeed */
    double    rotor_speed_rad_s;           /* RotSpeed */
    double    gen_torque_meas_nm;          /* GenTqMeas */
    double    nacelle_heading_rad;         /* NacHeading（OpenFAST 内部约定） */
    double    nacelle_vane_rad;            /* NacVane */
    double    hub_wind_speed_mps;          /* HorWindV */
    double    blade_pitch_rad[3];          /* avrSWAP blade pitch */
    double    blade_root_moop_nm[3];       /* rootMOOP(1:3) */
    double    tower_top_fa_acc_mps2;       /* FA_Acc */
    double    nacelle_imu_fa_acc_mps2;     /* NacIMU_FA_Acc */
    double    rotor_azimuth_rad;           /* Azimuth */
    double    yaw_rate_cmd_rad_s;          /* 最终偏航速率命令 */
    double    yaw_target_heading_rad;      /* 当前外部目标机舱方位（锁存值） */
    uint64_t  yaw_seq_applied;             /* S0902: 已被ROSCO采纳的偏航序号 */
    uint64_t  induction_seq_applied;       /* S0901: 已被ROSCO采纳的诱导序号 */
} FcrRoscoRawState;

/* ================================================================== */
/* 4. FcrSignalStats — 信号统计（1 s 窗口）                            */
/* ================================================================== */

typedef struct {
    double    instant;                     /* 最新瞬时值 */
    double    mean;                        /* 窗口均值 */
    double    rms;                         /* 窗口 RMS（含均值偏移） */
    double    min;                         /* 窗口最小值 */
    double    max;                         /* 窗口最大值 */
    uint32_t  n_samples;                   /* 窗口内采样数 */
    uint32_t  valid;                       /* 统计有效标志 */
} FcrSignalStats;

/* ================================================================== */
/* 5. FcrTurbineExtraState — OpenFAST 额外机组状态（Provider → runtime） */
/* ================================================================== */

typedef struct {
    int32_t   turbine_id;
    uint32_t  valid;
    double    source_time_s;
    uint64_t  source_seq;

    /* S0208-S0210: OpenFAST 原始状态 */
    double    rotor_aero_thrust_n;         /* S0208: 风轮轴向气动推力 */
    double    shaft_torque_nm;             /* S0209: 低速轴机械转矩 */
    double    tower_base_fa_moment_nm;     /* S0210: 塔基前后向弯矩 */

    double    tower_base_ss_moment_nm;     /* 塔基侧向弯矩 */
    double    rotor_ct;                    /* 转子推力系数（无量纲） */
    double    rotor_cp;                    /* 转子功率系数（无量纲） */

    /* 可配置载荷测点：行 = [Fx,Fy,Fz,Mx,My,Mz]，列 = point_id */
    double    load_point_f[6][FCR_MAX_LOAD_POINTS];
} FcrTurbineExtraState;

/* ================================================================== */
/* 6. FcrFlowState — AWAE/流场状态（Provider → runtime，~1 s，ZOH）     */
/* ================================================================== */

typedef struct {
    uint32_t  valid;
    double    flow_source_time_s;          /* AWAE 快照对应仿真时间 */
    uint64_t  flow_seq;                    /* AWAE 快照序号（单调递增） */
    uint32_t  n_points;                    /* 实际观察点数 */
    uint32_t  n_turbines;                  /* 实际机组数 */

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
/* 7. FcrBridgeInput — Scheduler_to_Bridge (S0701-S0710)               */
/* ================================================================== */

typedef struct {
    double    bridge_step_sim_time_s;      /* S0701: 桥接控制步仿真时刻 */
    uint16_t  bridge_command_valid_flags;  /* S0702: 桥接命令有效标记 */
    uint32_t  turbine_id;                  /* S0703: 命令目标机组编号 */
    uint8_t   control_enable;              /* S0704: 外部控制允许开关 */
    double    induction_ref;               /* S0705: 已调度的诱导因子目标 */
    uint64_t  induction_seq;               /* S0706: 诱导命令执行追踪序号 */
    double    yaw_target_heading_rad;      /* S0707: 已调度的绝对机舱方位目标 */
    uint64_t  yaw_seq;                     /* S0708: 偏航命令执行追踪序号 */
    uint16_t  control_mode;                /* S0709: 诱导控制方式 */
    double    command_basis_sim_time_s;    /* S0710: 下发指令所依据的状态仿真时刻 */
} FcrBridgeInput;

/* ================================================================== */
/* 8. FcrBridgeOutput — Bridge_to_ROSCO (S0801-S0807)                  */
/* ================================================================== */

typedef struct {
    uint8_t   external_yaw_enable;         /* S0801: 外部偏航使能 */
    double    yaw_target_heading_rad;      /* S0802: ROSCO外部绝对方位目标 */
    double    r_pitch;                     /* S0803: PRC桨距降载比例 */
    double    r_speed;                     /* S0804: PRC转速参考比例 */
    double    r_torque;                    /* S0805: PRC转矩参考/额定比例 */
    double    min_pitch_ref_rad;           /* S0806: 额外最小桨距约束 */
    double    collective_pitch_offset_rad; /* S0807: 集体桨距修正量 */
} FcrBridgeOutput;

/* ================================================================== */
/* 9. FcrBridgeAck — Bridge_Ack_to_Scheduler (S0901-S0904)             */
/* ================================================================== */

typedef struct {
    uint64_t  induction_seq_applied;       /* S0901: 已被ROSCO采纳的诱导序号 */
    uint64_t  yaw_seq_applied;             /* S0902: 已被ROSCO采纳的偏航序号 */
    uint32_t  bridge_status_flags;         /* S0903: 桥接器状态 */
    double    ct_ref;                      /* S0904: 等效推力系数目标 */
} FcrBridgeAck;

/* bridge_status_flags 位定义 */
#define FCR_BRIDGE_STATUS_OK                (1u << 0)  /* 桥接器正常 */
#define FCR_BRIDGE_STATUS_CMD_ACTIVE        (1u << 1)  /* 有活动命令 */
#define FCR_BRIDGE_STATUS_FALLBACK          (1u << 2)  /* 处于回退模式 */
#define FCR_BRIDGE_STATUS_YAW_TRACKING      (1u << 3)  /* 偏航跟踪中 */
#define FCR_BRIDGE_STATUS_IND_TRACKING      (1u << 4)  /* 诱导跟踪中 */
#define FCR_BRIDGE_STATUS_CMD_STALE         (1u << 5)  /* 命令超龄 */
#define FCR_BRIDGE_STATUS_FAULT             (1u << 6)  /* 桥接器故障 */

/* ================================================================== */
/* 10. FcrFarmTurbineState — Scheduler_to_RCP_Turbine (S1001-S1017)    */
/* ================================================================== */

typedef struct {
    int32_t   turbine_id;                  /* S1003: 状态所属机组编号 */
    uint32_t  valid;
    double    source_time_s;               /* 本机组快照时间 */

    /* 高速量：瞬时 + 1 s 统计 */
    FcrSignalStats gen_power_w;            /* S1004/S1005: mean=1s, instant=瞬时 */
    FcrSignalStats rotor_aero_thrust_n;    /* S1010: 风轮气动轴向推力 */
    FcrSignalStats tower_base_fa_moment_nm;
    FcrSignalStats tower_base_ss_moment_nm;

    double    rotor_speed_rad_s;           /* S1006: 风轮低速轴转速 */
    double    gen_speed_rad_s;             /* S1007: 发电机高速轴转速 */
    double    collective_pitch_actual_rad; /* S1008: 实际集体桨距角 */
    double    nacelle_heading_actual_rad;  /* S1009: 机舱实际绝对方位角 */

    /* 风估计 (S1011, S1016, S1017) */
    double    estimated_wind_mps;          /* S1011: 机组等效来流风速估计值 */
    uint16_t  wind_source;                 /* S1016: 风估计来源 */
    uint16_t  wind_quality_flags;          /* S1017: 风估计质量 */

    /* 推力系数估计 */
    double    ct_hat;                      /* S1012: 等效气动推力系数估计 */

    /* 状态确认 */
    uint64_t  yaw_seq_applied;             /* S1013: 偏航控制采纳确认序号 */
    uint64_t  induction_seq_applied;       /* S1014: 诱导控制采纳确认序号 */
    uint32_t  turbine_status_flags;        /* S1015: 机组/桥接综合状态字 */

    /* 扩展字段 */
    double    nacelle_vane_rad;
    double    hub_wind_speed_mps;
    double    rotor_ct;
    double    rotor_cp;
    double    blade_pitch_rad[3];
    double    yaw_target_heading_rad;      /* 锁存目标 */
    uint32_t  controller_status_flags;

    /* 低速量（AWAE，1 s 之间 ZOH） */
    double    disk_ambient_wind_mps;
    double    disk_disturbed_wind_mps;
    double    disk_ambient_ti;
    double    flow_source_time_s;          /* 流场快照时间（age 计算） */
    uint64_t  flow_seq;
} FcrFarmTurbineState;

/* ================================================================== */
/* 11. FarmStateFrame — runtime → Farm Controller (~1 s 聚合发布)      */
/* ================================================================== */

typedef struct {
    uint32_t  protocol_version;
    uint64_t  state_frame_seq;             /* S1001: 场站状态快照序号 */
    double    state_sample_sim_time_s;     /* S1002: 场站状态采样仿真时刻 */
    uint64_t  fast_step;
    uint64_t  low_step;
    uint32_t  rt_health_flags;             /* FCR_RT_HEALTH_* */
    uint32_t  n_turbines;
    double    flow_source_time_s;          /* 最新流场时间（age 计算） */
    uint64_t  flow_seq;
    FcrFarmTurbineState turbines[FCR_MAX_TURBINES];
} FarmStateFrame;

/* ================================================================== */
/* 12. FcrInductionMapConfig/Output — 诱导映射配置与输出                */
/* ================================================================== */

typedef struct {
    double    a0;                          /* 基线诱导因子（通常 1/3） */
    double    a_min;                       /* 可实现最小诱导因子 */
    double    a_max;                       /* 可实现最大诱导因子 */
    double    ct_coef;                     /* Ct 修正系数 */
    double    min_pitch_gain;              /* 最小桨距增益 */
    double    min_pitch_max;               /* 最小桨距上限（rad） */
    double    power_cap;                   /* 功率比上限 */
} FcrInductionMapConfig;

typedef struct {
    double    ct_ref;                      /* 推力系数目标 */
    double    r_pitch;                     /* S0803: PRC桨距比例 */
    double    r_speed;                     /* S0804: PRC转速比例 */
    double    r_torque;                    /* S0805: PRC转矩比例 */
    double    min_pitch_ref_rad;           /* S0806: 最小桨距约束 */
    double    collective_pitch_offset_rad; /* S0807: 集体桨距修正 */
    uint32_t  fallback;                    /* 是否处于 fallback */
} FcrInductionMapOutput;

/* FcrInductionState — 诱导监督器内部状态（per-turbine） */
typedef struct {
    uint64_t seq_applied;
    uint32_t valid;
    uint32_t fallback;
    double   induction_ref;
    double   ct_ref;
    double   r_pitch;
    double   r_speed;
    double   r_torque;
    double   min_pitch_ref_rad;
    double   collective_pitch_offset_rad;
    double   last_update_time_s;
} FcrInductionState;


#ifdef __cplusplus
}
#endif

#endif /* FCR_STATE_TYPES_H */