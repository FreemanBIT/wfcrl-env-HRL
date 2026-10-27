/*
 * fcr_internal.h — farm_control_runtime 内部共享声明（不构成 ABI，不进交付头）
 */
#ifndef FCR_INTERNAL_H
#define FCR_INTERNAL_H

#include "fcr_protocol.h"
#include "fcr_state_types.h"
#include "fcr_runtime.h"
#include "fcr_rosco_api.h"
#include "awae_frame_builder.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* angle_convention.c                                                  */
/* ------------------------------------------------------------------ */

double fcr_wrap_pm_pi(double a);
double fcr_wrap_0_2pi(double a);
double fcr_deg2rad(double d);
double fcr_rad2deg(double r);
double fcr_cw_delta_to_internal(double delta_cw_rad);
double fcr_compute_yaw_target(double heading_now_rad, double delta_cw_rad);
int fcr_is_finite(double x);

/* ------------------------------------------------------------------ */
/* command_store.c (ICD v1.5 S0101-S0112)                              */
/* ------------------------------------------------------------------ */

typedef enum {
    FCR_CH_IDLE = 0,
    FCR_CH_ACCEPTED,
    FCR_CH_SCHEDULED,
    FCR_CH_ACTIVE,
    FCR_CH_EXPIRED
} FcrChannelState;

typedef struct FcrTurbineCommandState {
    /* ---- yaw 通道 (S0108-S0110) ---- */
    uint64_t yaw_seq_rx;
    uint64_t yaw_seq_applied;
    double   yaw_target_heading_rad;    /* S0108: 绝对目标（内部约定） */
    uint32_t yaw_valid;                 /* S0110: 偏航目标有效标记 */
    uint32_t yaw_target_latched;
    double   yaw_ttl_s;
    double   yaw_issue_time_s;
    double   yaw_basis_sim_time_s;      /* W-13: yaw 通道自身的命令依据时刻 */
    FcrChannelState yaw_state;
    
    /* ---- induction 通道 (S0105-S0107) ---- */
    uint64_t ind_seq_rx;
    uint64_t ind_seq_applied;
    double   induction_ref;             /* S0105: 目标轴向诱导因子 */
    uint32_t ind_valid;                 /* S0107: 诱导目标有效标记 */
    double   ind_ttl_s;
    double   ind_issue_time_s;
    double   ind_basis_sim_time_s;      /* W-13: induction 通道自身的命令依据时刻 */
    FcrChannelState ind_state;
    
    /* ---- 通用字段 (S0102, S0104, S0111, S0112) ---- */
    double   command_basis_sim_time_s;  /* S0102: 命令依据的仿真时刻 */
    uint32_t control_enable;            /* S0104: 1=允许外部场控 */
    uint16_t control_mode;              /* S0111: 0=基线,1=PRC,2=Ct,3=闭环 */
    double   requested_effective_sim_time_s; /* S0112: 期望生效仿真时刻 */
    
    /* ---- 诊断 ---- */
    uint32_t command_status_flags;
} FcrTurbineCommandState;

struct FcrCommandStore {
    uint32_t n_turbines;
    double   default_yaw_ttl_s;
    double   default_induction_ttl_s;
    double   stale_threshold_s;
    double   dt_low_s;
    FcrTurbineCommandState t[FCR_MAX_TURBINES];
};

FcrCommandStore *fcr_command_store_create(uint32_t n_turbines,
                                          double default_yaw_ttl_s,
                                          double default_induction_ttl_s,
                                          double stale_threshold_s,
                                          double dt_low_s);
void fcr_command_store_destroy(FcrCommandStore *cs);
int fcr_command_store_accept_frame(FcrCommandStore *cs, const FarmCommandFrame *frame);
void fcr_command_store_refresh(FcrCommandStore *cs, double sim_time_s,
                               const double *heading_now_rad,
                               uint64_t low_step);
void fcr_command_store_refresh_one(FcrCommandStore *cs, int32_t turbine_id,
                                   double sim_time_s, double heading_now_rad,
                                   uint64_t low_step);
void fcr_command_store_get_setpoint(const FcrCommandStore *cs, int32_t turbine_id,
                                    FcrRoscoExternalSetpoint *sp);
FcrTurbineCommandState *fcr_command_store_turbine(FcrCommandStore *cs, int32_t turbine_id);
void fcr_command_store_debug_counters(uint32_t *yaw_accept, uint32_t *ind_accept,
                                      uint32_t *yaw_latch, uint32_t *ind_latch);
/* RT 平台要求 §3.3：reset 时清零诊断计数（可重复性） */
void fcr_command_store_debug_counters_reset(void);

/* ------------------------------------------------------------------ */
/* state_store.c (ICD v1.5 S0201-S0210, S0301-S0303)                   */
/* ------------------------------------------------------------------ */

struct FcrStateStore {
    uint32_t n_turbines;
    FcrRoscoRawState       fast[FCR_MAX_TURBINES];     /* ROSCO 10 ms 槽 */
    FcrOpenFASTRawState    of[FCR_MAX_TURBINES];       /* S02 OpenFAST 10 ms 槽 */
    FcrTurbineExtraState   extra[FCR_MAX_TURBINES];    /* OF 额外 10 ms 槽 */
    FcrTurbineWindEstimate wind_est[FCR_MAX_TURBINES]; /* S06 风速估计 1 s 槽 */
    FcrFlowState           flow;                       /* AWAE 1 s ZOH 槽 */
    FcrAWAECatalog         awae_cat;                   /* GG-04 静态测点目录（注入） */
    uint16_t               awae_extra_quality_flags;   /* GG-05 注入方声明的质量位 */
    double   sim_time_s;
    uint64_t fast_step;
    uint64_t low_step;
    uint32_t rt_health_flags;
};

FcrStateStore *fcr_state_store_create(uint32_t n_turbines);
void fcr_state_store_destroy(FcrStateStore *ss);
int fcr_state_store_set_clock(FcrStateStore *ss, double sim_time_s,
                              uint64_t fast_step, uint64_t low_step,
                              uint32_t rt_health_flags);
int fcr_state_store_set_fast(FcrStateStore *ss, int32_t turbine_id,
                             const FcrRoscoRawState *st);
/* S02（OpenFAST 原始状态）：字段级归属——OF 拥有物理量测，ROSCO 拥有命令/状态/seq。
 * 写入 of 槽并把 OF 拥有的字段合并进 fast/extra 槽（不覆盖 ROSCO 侧字段）。 */
int fcr_state_store_set_openfast(FcrStateStore *ss,
                                 const FcrOpenFASTRawState *st);
int fcr_state_store_get_openfast(const FcrStateStore *ss, int32_t turbine_id,
                                 FcrOpenFASTRawState *out);
int fcr_state_store_set_extra(FcrStateStore *ss, const FcrTurbineExtraState *st);
int fcr_state_store_set_flow(FcrStateStore *ss, const FcrFlowState *flow);
int fcr_state_store_get_fast(const FcrStateStore *ss, int32_t turbine_id,
                             FcrRoscoRawState *out);
const FcrFlowState *fcr_state_store_get_flow(const FcrStateStore *ss);
int fcr_state_store_get_extra(const FcrStateStore *ss, int32_t turbine_id,
                              FcrTurbineExtraState *out);
/* S06: TurbineWindEstimate_to_Bridge（离线由 Provider 注入） */
int fcr_state_store_set_wind_estimate(FcrStateStore *ss,
                                      const FcrTurbineWindEstimate *est);
int fcr_state_store_get_wind_estimate(const FcrStateStore *ss, int32_t turbine_id,
                                      FcrTurbineWindEstimate *out);

/* ICD v1.5 GG-04/GG-05：AWAE 静态测点目录与注入方质量位。
 * FAST.Farm 离线无法产生真实 AWAE 量测，故目录由 Provider/配置注入；
 * 合成数据请把 extra_quality_flags 置 FCR_AWAE_QUALITY_SIMULATED。 */
int fcr_state_store_set_awae_catalog(FcrStateStore *ss, const FcrAWAECatalog *cat);
int fcr_state_store_set_awae_quality(FcrStateStore *ss, uint16_t extra_quality_flags);

/* ------------------------------------------------------------------ */
/* signal_statistics.c + state_aggregator.c                            */
/* ------------------------------------------------------------------ */

FcrSignalStatistics *fcr_signal_stat_create(double window_s, uint32_t capacity);
void fcr_signal_stat_destroy(FcrSignalStatistics *s);
void fcr_signal_stat_reset(FcrSignalStatistics *s);
int  fcr_signal_stat_update(FcrSignalStatistics *s, double sim_time_s, double x);
int  fcr_signal_stat_get(const FcrSignalStatistics *s, FcrSignalStats *out);

struct FcrStateAggregator;

FcrStateAggregator *fcr_state_aggregator_create(uint32_t n_turbines, double window_s);
void fcr_state_aggregator_destroy(FcrStateAggregator *a);
void fcr_state_aggregator_feed_fast(FcrStateAggregator *a, int32_t turbine_id,
                                    const FcrRoscoRawState *st);
void fcr_state_aggregator_feed_extra(FcrStateAggregator *a, const FcrTurbineExtraState *st);
void fcr_runtime_feed_fast(FcrRuntime *rt, int32_t turbine_id, const FcrRoscoRawState *st);
void fcr_runtime_feed_extra_stats(FcrRuntime *rt, const FcrTurbineExtraState *st);
void fcr_state_aggregator_build_frame(FcrStateAggregator *a, const FcrStateStore *ss,
                                      FarmStateFrame *out);

/* ------------------------------------------------------------------ */
/* yaw_action_manager.c                                                */
/* ------------------------------------------------------------------ */

typedef struct {
    int32_t  turbine_id;
    uint32_t valid;
    uint64_t seq;
    double   target_heading_rad;
    double   heading_now_rad;
    double   error_rad;
    uint32_t tracking;
    uint32_t reached;
    double   last_update_time_s;
} FcrYawActionState;

struct FcrYawActionManager {
    uint32_t n_turbines;
    double   deadband_rad;
    FcrYawActionState t[FCR_MAX_TURBINES];
};

FcrYawActionManager *fcr_yaw_manager_create(uint32_t n_turbines, double deadband_rad);
void fcr_yaw_manager_destroy(FcrYawActionManager *mgr);
void fcr_yaw_manager_update(FcrYawActionManager *mgr, int32_t turbine_id,
                            const FcrTurbineCommandState *tc,
                            double heading_now_rad, double sim_time_s);
int  fcr_yaw_manager_get(const FcrYawActionManager *mgr, int32_t turbine_id,
                         FcrYawActionState *out);

/* ------------------------------------------------------------------ */
/* FcrInductionMapConfig and FcrInductionState defined in fcr_state_types.h */

struct FcrInductionSupervisor {
    uint32_t n_turbines;
    FcrInductionMapConfig cfg;
    FcrInductionState t[FCR_MAX_TURBINES];
};

FcrInductionSupervisor *fcr_induction_supervisor_create(uint32_t n_turbines,
                                                        const FcrInductionMapConfig *cfg);
void fcr_induction_supervisor_destroy(FcrInductionSupervisor *s);
void fcr_induction_supervisor_update(FcrInductionSupervisor *s, int32_t turbine_id,
                                     const FcrTurbineCommandState *tc, double sim_time_s);
void fcr_induction_supervisor_apply(const FcrInductionSupervisor *s, int32_t turbine_id,
                                    FcrRoscoExternalSetpoint *sp);
int  fcr_induction_supervisor_get(const FcrInductionSupervisor *s, int32_t turbine_id,
                                  FcrInductionState *out);
const FcrInductionMapConfig *fcr_induction_supervisor_cfg(const FcrInductionSupervisor *s);

/* ------------------------------------------------------------------ */
/* watchdog.c                                                          */
/* ------------------------------------------------------------------ */

struct FcrWatchdog {
    double flow_stale_threshold_s;
    double command_stale_threshold_s;
    uint32_t health_flags;
    double last_clock_time_s;
    uint64_t last_fast_step;
    int clock_seen;
};

FcrWatchdog *fcr_watchdog_create(double flow_stale_threshold_s);
void fcr_watchdog_destroy(FcrWatchdog *wd);
void fcr_watchdog_update(FcrWatchdog *wd, double sim_time_s,
                         const FcrStateStore *ss,
                         const FcrCommandStore *cs);
uint32_t fcr_watchdog_health(const FcrWatchdog *wd);

/* ------------------------------------------------------------------ */
/* runtime.c (internal helpers)                                        */
/* ------------------------------------------------------------------ */

struct FcrRuntime {
    FcrRuntimeConfig cfg;
    FcrCommandStore *cs;
    FcrStateStore   *ss;
    FcrWatchdog     *wd;
    void *aggregator;
    void *yaw_manager;
    void *induction_supervisor;
    void *signal_statistics;
    void *rosco_api;
    void *bridge_core;
    FcrRoscoExternalSetpoint setpoint[FCR_MAX_TURBINES];
    double last_low_step_time_s;
    uint64_t state_frame_seq;   /* ICD v1.5 S1001 */
};

void fcr_runtime_update_setpoints(FcrRuntime *rt, double sim_time_s, uint64_t low_step);
void fcr_runtime_update_setpoint_one(FcrRuntime *rt, int32_t turbine_id);
/* RT 平台要求 §3.3：构造 1 s 聚合状态帧（不发布；供 step_low 与 RT 适配层共用） */
int fcr_runtime_build_state_frame(FcrRuntime *rt, FarmStateFrame *out);

/* ------------------------------------------------------------------ */
/* rosco_api.c — RT 集成（fastfarmRT：调度器 + ROSCO .a 同核）          */
/* ------------------------------------------------------------------ */
/* 调度器创建 runtime 后注入单例，使 Fortran 侧读到同一份 setpoint。      */
FcrRuntime *fcr_rosco_api_runtime(void);
int  fcr_rosco_api_adopt_runtime(FcrRuntime *rt);
void fcr_rosco_api_detach_runtime(FcrRuntime *rt);
int  fcr_rosco_api_rt_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* FCR_INTERNAL_H */