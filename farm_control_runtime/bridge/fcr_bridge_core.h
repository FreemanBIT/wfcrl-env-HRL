/*
 * fcr_bridge_core.h — ROSCO Bridge 实时核心（M1，DR-3/DR-4/DR-5）
 *
 * 设计原则（对应 BRIDGE_DEV_PLAN_v2 §4）：
 *   - 纯内存、无通信、无锁、无动态分配、无 I/O —— 可离线单测、可在 RT Fast Task 用。
 *   - 显式输入 → 显式输出：不依赖 farm_control_runtime 的 transport/store。
 *   - 输入经"命令快照 + 测量快照"传入；输出为 FcrRoscoExternalSetpoint（与现
 *     fcr_rosco_api.h 对齐，Bridge 输出可无缝进 ROSCO 10ms 只读路径）。
 *
 * 诱导模式（DR-3）：
 *   Mode 0: 原 ROSCO（外部 ratio 全 1，旁路）
 *   Mode 1: 功率比例开环 R_power = a(1-a)^2 / [a0(1-a0)^2]，经控制分配输出
 *            r_pitch / r_speed / r_torque（可配置权重，禁止未分配叠加）
 *   Mode 2: Ct 前馈 Ct_ref = 4a(1-a)，由 Ct(lambda,beta) 气动表反解 beta_target
 *            → min_pitch_ref（ROSCO 原生 PRC 或独立最小桨距约束）
 *   Mode 3: 反馈接口桩（保留 ct_ref/Ct_est 通道，本期不实现闭环）
 *
 * 偏航（DR-5）：绝对机舱目标方位（OpenFAST 内部约定，CCW+）为正式通道；
 *   旧 delta 协议由 yaw_adapter 适配（转换仅在适配层，rt_core 内部统一绝对目标）。
 *
 * 红线（DR-4）：本核心 10ms 步进 fcr_bridge_core_step() 内禁止：
 *   全场互斥锁 / socket / malloc / 等待 GPU/Modbus/AWAE。
 */
#ifndef FCR_BRIDGE_CORE_H
#define FCR_BRIDGE_CORE_H

#include "fcr_rosco_api.h"   /* FcrRoscoExternalSetpoint */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FCR_BRIDGE_MAX_TURBINES 128   /* 初期 30 台，扩展 100+（DR-8） */
#define FCR_BRIDGE_MAX_AERO_TABLE 200 /* Ct(lambda,beta) 表最大点数 */

/* ------------------------------------------------------------------ */
/* 1. 测量快照（Fast Task 10ms 输入；OpenFAST/ROSCO 状态）             */
/* ------------------------------------------------------------------ */
typedef struct {
    int32_t   turbine_id;
    uint32_t  valid;
    double    source_time_s;
    double    rotor_speed_rad_s;     /* ICD: rotor_speed */
    double    generator_speed_rad_s; /* ICD: generator_speed */
    double    rotor_thrust_n;        /* ICD: rotor_thrust */
    double    blade_pitch_rad[3];    /* ICD: blade_pitch */
    double    nacelle_heading_rad;   /* 机舱方位（OpenFAST 内部约定 CCW+）*/
    /* ---- DR-7 四类风速语义 ----
     * ambient_upstream_wind_mps : 上游参考风（AWAE 观察点映射，未受本机扰动）
     * disk_background_wind_mps  : 轮盘背景风速（含他机尾流、不含本机诱导）
     * disk_wind_speed_mps       : 轮盘扰动风速（盘面实际；含本机诱导效应）
     * 经典 Ct 归一化必须用修正后的参考风，不得直接用扰动盘面风。 */
    double    ambient_upstream_wind_mps; /* 上游参考风（DR-7） */
    double    disk_background_wind_mps;  /* 轮盘背景风速（DR-7） */
    double    disk_wind_speed_mps;   /* 映射后盘面等效风速（WindMapper 输出）*/
    double    disk_turbulence;       /* 映射后盘面湍流 */
    double    gen_power_w;           /* ICD: generator_power */
} FcrBridgeMeas;

/* ------------------------------------------------------------------ */
/* 2. 命令快照（Slow Task 写入；含 DR-3 诱导模式 + DR-5 绝对偏航）     */
/* ------------------------------------------------------------------ */
typedef struct {
    int32_t   turbine_id;
    uint32_t  valid;
    /* ---- 诱导通道 ---- */
    uint32_t  induction_mode;        /* 0/1/2/3（DR-3）*/
    uint32_t  induction_valid;       /* 本条诱导命令有效标志 */
    double    induction_ref;         /* 轴向诱导 a [0,1) */
    uint64_t  induction_seq;
    double    induction_ttl_s;
    /* ---- 偏航通道（DR-5 绝对目标）---- */
    uint32_t  yaw_valid;             /* 本条偏航命令有效标志 */
    double    yaw_target_heading_rad;/* 绝对机舱目标（OpenFAST 内部约定）*/
    uint64_t  yaw_seq;
    double    yaw_ttl_s;
    /* ---- 控制分配（Mode 1，DR-3）---- */
    double    alloc_w_pitch;         /* R_Pitch 权重 [0,1]，和为 1 校验 */
    double    alloc_w_speed;         /* R_Speed 权重 */
    double    alloc_w_torque;        /* R_Torque 权重 */
    /* ---- 气动表（Mode 2；静态配置，Slow 一次性加载）---- */
    double    aero_blade_radius_m;
    int32_t   aero_n_lam;
    int32_t   aero_n_beta;
    /* Ct(lambda,beta) 表：lam 行 x beta 列。
     * 优先用共享指针 aero_ct/aero_lam/aero_beta_deg（Slow 一次性加载，所有机共用，
     * 避免 320KB/机 的嵌入拷贝；DR-4：10ms 路径只读指针不做分配）。
     * 若指针为空，回退到内嵌小数组 aero_ct_small（供测试/小表）。 */
    const double *aero_lam_ptr;
    const double *aero_beta_deg_ptr;
    const double *aero_ct_ptr;
    double    aero_lam_small[FCR_BRIDGE_MAX_AERO_TABLE];
    double    aero_beta_deg_small[FCR_BRIDGE_MAX_AERO_TABLE];
    double    aero_ct_small[FCR_BRIDGE_MAX_AERO_TABLE * FCR_BRIDGE_MAX_AERO_TABLE];
    double    source_time_s;
} FcrBridgeCmd;

/* ------------------------------------------------------------------ */
/* 3. 诱导映射配置（可由 yaml 注入；缺省 1/3 基线）                    */
/* ------------------------------------------------------------------ */
typedef struct {
    double a0;                /* 基线轴向诱导（默认 1/3） */
    double a_min;             /* 最小允许诱导 */
    double a_max;             /* 最大允许诱导 */
    double ct_coef;           /* Ct 标定系数（动量理论修正） */
    double power_cap;         /* power_ratio 上限 */
    double min_pitch_max_rad; /* 最小桨距约束上限（rad） */
} FcrBridgeIndConfig;

/* ------------------------------------------------------------------ */
/* 4. Ct 估计器配置（DR-7：扰动盘面风不得直接用于经典 Ct 归一化）      */
/* ------------------------------------------------------------------ */
typedef struct {
    double air_density;       /* 1.225 kg/m^3 */
    double rotor_area;        /* pi R^2 */
    double gen_eff;           /* 发电机效率 */
    double v_disk_ref_corr;   /* 盘面风模型修正系数（<1：扰动风偏高时下调） */
    uint32_t use_model_corr;  /* 1 = 对扰动盘面风做模型修正后再归一化 */
} FcrBridgeCtConfig;

/* ------------------------------------------------------------------ */
/* 5. 偏航适配配置（DR-5：绝对目标为正式通道；delta 仅 legacy）        */
/* ------------------------------------------------------------------ */
typedef struct {
    double deadband_rad;      /* 到达判定死区 */
} FcrBridgeYawConfig;

/* ------------------------------------------------------------------ */
/* 6. 核心状态（每机；编译期分配）                                     */
/* ------------------------------------------------------------------ */
typedef struct {
    /* 命令/测量快照（无锁单写单读，由外部 double-buffer 保证） */
    FcrBridgeCmd  cmd;
    FcrBridgeMeas meas;
    uint32_t      cmd_valid;
    uint32_t      meas_valid;
    /* 诱导映射结果 */
    double        power_ratio;
    double        speed_ref_ratio;
    double        torque_limit_ratio;
    double        min_pitch_rad;
    double        ct_ref;
    uint32_t      induction_fallback;
    uint64_t      induction_seq_applied;
    /* 偏航锁存 */
    double        yaw_target_heading_rad;
    uint64_t      yaw_seq_applied;
    /* Ct 估计（修正后） */
    double        ct_est;
    uint32_t      ct_est_valid;
    /* 健康 */
    uint32_t      cmd_stale;   /* 命令 TTL 失效/超时 */
    uint32_t      fallback_active;
} FcrBridgeTurbineState;

typedef struct FcrBridgeCore FcrBridgeCore;

/* ------------------------------------------------------------------ */
/* 7. 生命周期与步进 API                                                */
/* ------------------------------------------------------------------ */

/* 创建核心（n_turbines 编译期分配；NULL 配置用默认） */
FcrBridgeCore *fcr_bridge_core_create(uint32_t n_turbines,
                                      const FcrBridgeIndConfig *ind,
                                      const FcrBridgeCtConfig *ct,
                                      const FcrBridgeYawConfig *yaw);

void fcr_bridge_core_destroy(FcrBridgeCore *core);

/* Slow Task：写入命令快照（含诱导模式/绝对偏航/分配权重/气动表） */
int fcr_bridge_core_set_command(FcrBridgeCore *core, const FcrBridgeCmd *cmd);

/* Fast Task（10ms）：喂测量快照 */
int fcr_bridge_core_set_meas(FcrBridgeCore *core, const FcrBridgeMeas *meas);

/* Fast Task（10ms）：对指定 turbine_id 执行一步（诱导映射 + 偏航锁存 +
 * Ct 估计 + 安全），输出该机 setpoint（对应 ROSCO fcr_rosco_read_setpoint
 * 语义的单机槽）。禁止锁/socket/malloc/等待 —— 本函数内部仅纯内存运算。 */
int fcr_bridge_core_step(FcrBridgeCore *core, int32_t turbine_id,
                         double sim_time_s, FcrRoscoExternalSetpoint *out);

/* 查询单机内部状态（诊断） */
int fcr_bridge_core_get(FcrBridgeCore *core, int32_t turbine_id,
                        FcrBridgeTurbineState *out);

/* 工具：诱导映射纯函数（供单测；Mode1 R_power、Mode2 Ct_ref） */
double fcr_bridge_mode1_power_ratio(double a, double a0);
double fcr_bridge_mode2_ct_ref(double a);
int    fcr_bridge_mode2_find_beta(const FcrBridgeCmd *cmd, double lam,
                                  double ct_ref, double *beta_deg_out);

/* 偏航适配（DR-5）：旧 delta 协议 → 绝对目标；绝对目标校验 */
double fcr_yaw_adapter_delta_to_target(double yaw_delta_rad, double heading_now_rad);
int    fcr_yaw_adapter_validate_abs(double target_rad);

#ifdef __cplusplus
}
#endif

#endif /* FCR_BRIDGE_CORE_H */
