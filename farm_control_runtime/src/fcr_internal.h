/*
 * fcr_internal.h — farm_control_runtime 内部共享声明（不构成 ABI，不进交付头）
 */
#ifndef FCR_INTERNAL_H
#define FCR_INTERNAL_H

#include "fcr_protocol.h"
#include "fcr_state_types.h"
#include "fcr_runtime.h"
#include "fcr_rosco_api.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* angle_convention.c                                                  */
/* ------------------------------------------------------------------ */

/* 把角度 wrap 到 (-PI, PI]                                           */
double fcr_wrap_pm_pi(double a);
/* 把角度 wrap 到 [0, 2*PI)                                           */
double fcr_wrap_0_2pi(double a);
/* 度 → 弧度 / 弧度 → 度                                              */
double fcr_deg2rad(double d);
double fcr_rad2deg(double r);
/* 罗盘顺时针增量 → OpenFAST 内部增量（CCW+）：取反                   */
double fcr_cw_delta_to_internal(double delta_cw_rad);
/* 由当前机舱方位（内部约定）与 CW+ 增量计算锁存目标                   */
double fcr_compute_yaw_target(double heading_now_rad, double delta_cw_rad);
/* NaN/Inf 检查（1 = 非有限，0 = 有限）                               */
int fcr_is_finite(double x);

/* ------------------------------------------------------------------ */
/* command_store.c                                                    */
/* ------------------------------------------------------------------ */

typedef struct FcrTurbineCommandState {
    /* ---- yaw 通道 ---- */
    uint64_t yaw_seq_rx;              /* 最近收到（成功校验）的 seq       */
    uint64_t yaw_seq_applied;         /* 已生效 seq                       */
    double   yaw_delta_rad;           /* 最新有效 delta（CW+）            */
    uint32_t yaw_valid;               /* 通道有效标志                     */
    uint32_t yaw_target_latched;      /* 绝对目标已锁存                   */
    double   yaw_target_heading;      /* 锁存绝对目标（内部约定）         */
    int64_t  yaw_effective_low_step;  /* 生效低速步号                     */
    double   yaw_ttl_s;               /* 生效 TTL（0 = 无限）             */
    double   yaw_issue_time_s;        /* 生效时刻（sim_time）             */
    /* ---- induction 通道 ---- */
    uint64_t ind_seq_rx;
    uint64_t ind_seq_applied;
    double   induction_ref;           /* 最新有效 ref（无量纲）           */
    uint32_t ind_valid;
    int64_t  ind_effective_low_step;
    double   ind_ttl_s;
    double   ind_issue_time_s;
    double   source_time_s;           /* 命令发出时间（控制器侧）         */
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

/* 接收一帧命令（按通道独立处理；返回 0 成功）                         */
int fcr_command_store_accept_frame(FcrCommandStore *cs, const FarmCommandFrame *frame);

/* 在高速步调用：对每台机检查 TTL/stale，并（若需要）用最新机舱方位    *
 * 锁存 yaw 目标；其结果反映在 setpoint 中                              */
void fcr_command_store_refresh(FcrCommandStore *cs, double sim_time_s,
                               const double *heading_now_rad, /* [n_turbines] 或 NULL */
                               uint64_t low_step);

/* 单机版（DLL 内每台机进程/线程调用；heading 为 NULL 时等待方位就绪） */
void fcr_command_store_refresh_one(FcrCommandStore *cs, int32_t turbine_id,
                                   double sim_time_s, double heading_now_rad,
                                   uint64_t low_step);

/* 生成当前生效 setpoint（每机）                                       */
void fcr_command_store_get_setpoint(const FcrCommandStore *cs, int32_t turbine_id,
                                    FcrRoscoExternalSetpoint *sp);

/* 供测试使用：通道状态直接访问                                        */
FcrTurbineCommandState *fcr_command_store_turbine(FcrCommandStore *cs, int32_t turbine_id);

/* ------------------------------------------------------------------ */
/* state_store.c                                                      */
/* ------------------------------------------------------------------ */

struct FcrStateStore {
    uint32_t n_turbines;
    FcrRoscoFastState      fast[FCR_MAX_TURBINES];   /* ROSCO 10 ms 槽    */
    FcrTurbineExtraState   extra[FCR_MAX_TURBINES];  /* OF 额外 10 ms 槽  */
    FcrFlowState           flow;                     /* AWAE 1 s ZOH 槽   */
    double   sim_time_s;
    uint64_t fast_step;
    uint64_t low_step;
    uint32_t rt_health_flags;
};

FcrStateStore *fcr_state_store_create(uint32_t n_turbines);
void fcr_state_store_destroy(FcrStateStore *ss);

/* Provider push 实现（也供 Offline/RT Provider 使用）                 */
int fcr_state_store_set_clock(FcrStateStore *ss, double sim_time_s,
                              uint64_t fast_step, uint64_t low_step,
                              uint32_t rt_health_flags);
int fcr_state_store_set_fast(FcrStateStore *ss, int32_t turbine_id,
                             const FcrRoscoFastState *st);
int fcr_state_store_set_extra(FcrStateStore *ss, const FcrTurbineExtraState *st);
int fcr_state_store_set_flow(FcrStateStore *ss, const FcrFlowState *flow);

/* 读取最新快照（供聚合/测试）                                         */
int fcr_state_store_get_fast(const FcrStateStore *ss, int32_t turbine_id,
                             FcrRoscoFastState *out);
const FcrFlowState *fcr_state_store_get_flow(const FcrStateStore *ss);
int fcr_state_store_get_extra(const FcrStateStore *ss, int32_t turbine_id,
                              FcrTurbineExtraState *out);

/* ------------------------------------------------------------------ */
/* yaw_action_manager.c                                               */
/* ------------------------------------------------------------------ */

/* 偏航动作执行状态（每机）                                            */
typedef struct {
    int32_t  turbine_id;
    uint32_t valid;                   /* 状态有效                          */
    uint64_t seq;                     /* 已应用 seq                        */
    double   target_heading_rad;      /* 锁存绝对目标（内部约定）          */
    double   heading_now_rad;         /* 最近一次机舱方位                  */
    double   error_rad;               /* wrap_pm_pi(target - heading)      */
    uint32_t tracking;                /* 正在执行（|err| > deadband）      */
    uint32_t reached;                 /* 已到达（|err| <= deadband）       */
    double   last_update_time_s;
} FcrYawActionState;

struct FcrYawActionManager {
    uint32_t n_turbines;
    double   deadband_rad;            /* 到达判定死区                      */
    FcrYawActionState t[FCR_MAX_TURBINES];
};

FcrYawActionManager *fcr_yaw_manager_create(uint32_t n_turbines, double deadband_rad);
void fcr_yaw_manager_destroy(FcrYawActionManager *mgr);
/* 每高速步更新单机状态：读取 CommandStore 通道状态 + 最新机舱方位      */
void fcr_yaw_manager_update(FcrYawActionManager *mgr, int32_t turbine_id,
                            const FcrTurbineCommandState *tc,
                            double heading_now_rad, double sim_time_s);
int  fcr_yaw_manager_get(const FcrYawActionManager *mgr, int32_t turbine_id,
                         FcrYawActionState *out);

/* ------------------------------------------------------------------ */
/* watchdog.c                                                         */
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
/* 更新健康标志：输入最新 sim_time、state store 与 command store       */
void fcr_watchdog_update(FcrWatchdog *wd, double sim_time_s,
                         const FcrStateStore *ss,
                         const FcrCommandStore *cs);
uint32_t fcr_watchdog_health(const FcrWatchdog *wd);

/* ------------------------------------------------------------------ */
/* runtime.c (internal helpers)                                       */
/* ------------------------------------------------------------------ */

struct FcrRuntime {
    FcrRuntimeConfig cfg;
    FcrCommandStore *cs;
    FcrStateStore   *ss;
    FcrWatchdog     *wd;
    /* 后续 Phase 挂载：aggregator / yaw_manager / induction_supervisor /
     * signal_statistics / rosco_api 在此扩展                                */
    void *aggregator;         /* Phase 5 */
    void *yaw_manager;        /* Phase 3 */
    void *induction_supervisor; /* Phase 6 */
    void *signal_statistics;  /* Phase 5 */
    void *rosco_api;          /* Phase 2 */
    FcrRoscoExternalSetpoint setpoint[FCR_MAX_TURBINES]; /* 各机当前生效 setpoint */
    double last_low_step_time_s;
};

/* runtime 内部：更新每台机 setpoint（供 step_high 使用）              */
void fcr_runtime_update_setpoints(FcrRuntime *rt, double sim_time_s, uint64_t low_step);
void fcr_runtime_update_setpoint_one(FcrRuntime *rt, int32_t turbine_id);

#ifdef __cplusplus
}
#endif

#endif /* FCR_INTERNAL_H */