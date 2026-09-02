/*
 * runtime.c — farm_control_runtime 核心装配与步进（Phase 1）
 *
 * 高速步（~10 ms）：
 *   1. 从 Transport 拉取命令帧 → CommandStore；
 *   2. CommandStore refresh（TTL/stale/yaw 目标锁存，使用最新机舱方位）；
 *   3. 生成各机 ROSCO setpoint（YawActionManager/InductionSupervisor 在
 *      Phase 3/6 挂载，当前经 CommandStore 直接生成）。
 * 低速步（~1 s）：
 *   1. 从 StateStore 聚合各源最新快照 → FarmStateFrame；
 *   2. Watchdog 健康检查；
 *   3. Transport publish_state。
 */
#include "fcr_internal.h"
#include <string.h>
#include <stdlib.h>

FcrRuntime *fcr_runtime_create(const FcrRuntimeConfig *cfg)
{
    FcrRuntime *rt;
    if (!cfg) return NULL;
    if (cfg->n_turbines == 0 || cfg->n_turbines > FCR_MAX_TURBINES) return NULL;

    rt = (FcrRuntime *)calloc(1, sizeof(FcrRuntime));
    if (!rt) return NULL;
    rt->cfg = *cfg;
    if (rt->cfg.dt_high_s <= 0.0) rt->cfg.dt_high_s = 0.01;
    if (rt->cfg.dt_low_s  <= 0.0) rt->cfg.dt_low_s  = 1.0;
    if (rt->cfg.stale_threshold_s <= 0.0) rt->cfg.stale_threshold_s = 5.0;
    if (rt->cfg.flow_stale_threshold_s <= 0.0) rt->cfg.flow_stale_threshold_s = 3.0;

    rt->aggregator = fcr_state_aggregator_create(rt->cfg.n_turbines, 1.0);
    if (!rt->aggregator) { fcr_runtime_destroy(rt); return NULL; }
    rt->induction_supervisor = fcr_induction_supervisor_create(rt->cfg.n_turbines, NULL);
    if (!rt->induction_supervisor) { fcr_runtime_destroy(rt); return NULL; }
    rt->cs = fcr_command_store_create(rt->cfg.n_turbines,
                                      rt->cfg.default_yaw_ttl_s,
                                      rt->cfg.default_induction_ttl_s,
                                      rt->cfg.stale_threshold_s,
                                      rt->cfg.dt_low_s);
    rt->ss = fcr_state_store_create(rt->cfg.n_turbines);
    rt->wd = fcr_watchdog_create(rt->cfg.flow_stale_threshold_s);
    if (!rt->cs || !rt->ss || !rt->wd) {
        fcr_runtime_destroy(rt);
        return NULL;
    }
    rt->last_low_step_time_s = -1.0;
    return rt;
}

int fcr_runtime_destroy(FcrRuntime *rt)
{
    if (!rt) return 0;
    if (rt->aggregator) fcr_state_aggregator_destroy((FcrStateAggregator *)rt->aggregator);
    if (rt->induction_supervisor)
        fcr_induction_supervisor_destroy((FcrInductionSupervisor *)rt->induction_supervisor);
    fcr_command_store_destroy(rt->cs);
    fcr_state_store_destroy(rt->ss);
    fcr_watchdog_destroy(rt->wd);
    free(rt);
    return 0;
}

FcrCommandStore        *fcr_runtime_command_store(FcrRuntime *rt) { return rt ? rt->cs : NULL; }
FcrStateStore          *fcr_runtime_state_store(FcrRuntime *rt)   { return rt ? rt->ss : NULL; }
FcrStateAggregator     *fcr_runtime_aggregator(FcrRuntime *rt)    { return rt ? (FcrStateAggregator *)rt->aggregator : NULL; }
FcrYawActionManager    *fcr_runtime_yaw_manager(FcrRuntime *rt)   { return rt ? (FcrYawActionManager *)rt->yaw_manager : NULL; }
FcrInductionSupervisor *fcr_runtime_induction_supervisor(FcrRuntime *rt) { return rt ? (FcrInductionSupervisor *)rt->induction_supervisor : NULL; }
FcrSignalStatistics    *fcr_runtime_signal_statistics(FcrRuntime *rt) { return rt ? (FcrSignalStatistics *)rt->signal_statistics : NULL; }
FcrWatchdog            *fcr_runtime_watchdog(FcrRuntime *rt)      { return rt ? rt->wd : NULL; }
FcrRoscoApi            *fcr_runtime_rosco_api(FcrRuntime *rt)     { return rt ? (FcrRoscoApi *)rt->rosco_api : NULL; }

int fcr_runtime_set_transport(FcrRuntime *rt, FcrTransport *t)
{
    if (!rt) return -1;
    rt->cfg.transport = t;
    return 0;
}

int fcr_runtime_bind_turbines(FcrRuntime *rt, uint32_t n_turbines)
{
    if (!rt || n_turbines == 0 || n_turbines > FCR_MAX_TURBINES) return -1;
    rt->cfg.n_turbines = n_turbines;
    if (rt->cs) rt->cs->n_turbines = n_turbines;
    if (rt->ss) rt->ss->n_turbines = n_turbines;
    return 0;
}

/* 生成单机 ROSCO setpoint（供 Fortran 侧 10 ms 读取）                 */
void fcr_runtime_update_setpoint_one(FcrRuntime *rt, int32_t turbine_id)
{
    FcrRoscoExternalSetpoint sp;
    if (!rt || !rt->cs) return;
    fcr_command_store_get_setpoint(rt->cs, turbine_id, &sp);
    /* InductionSupervisor（Phase 6）：新 seq 计算映射并叠加到 setpoint */
    if (rt->induction_supervisor) {
        FcrTurbineCommandState *tc = fcr_command_store_turbine(rt->cs, turbine_id);
        FcrInductionSupervisor *sup = (FcrInductionSupervisor *)rt->induction_supervisor;
        if (tc) {
            fcr_induction_supervisor_update(sup, turbine_id, tc, rt->ss->sim_time_s);
            fcr_induction_supervisor_apply(sup, turbine_id, &sp);
        }
    }
    rt->setpoint[turbine_id - 1] = sp;
}

/* 生成各机 ROSCO setpoint（整场版）                                   */
void fcr_runtime_update_setpoints(FcrRuntime *rt, double sim_time_s, uint64_t low_step)
{
    uint32_t i;
    if (!rt || !rt->cs) return;
    for (i = 0; i < rt->cfg.n_turbines; ++i) {
        fcr_runtime_update_setpoint_one(rt, (int32_t)(i + 1));
    }
    (void)sim_time_s; (void)low_step;
}

int fcr_runtime_step_high(FcrRuntime *rt, double sim_time_s)
{
    FarmCommandFrame frame;
    double heading[FCR_MAX_TURBINES];
    uint32_t i;
    if (!rt) return -1;

    /* 1) Transport 拉取命令帧 */
    if (rt->cfg.transport) {
        while (fcr_transport_poll(rt->cfg.transport, &frame) == 1) {
            fcr_command_store_accept_frame(rt->cs, &frame);
        }
    }

    /* 2) 取最新机舱方位（ROSCO fast state 槽） */
    for (i = 0; i < rt->cfg.n_turbines; ++i) {
        FcrRoscoFastState st;
        memset(&st, 0, sizeof(st));
        fcr_state_store_get_fast(rt->ss, (int32_t)(i + 1), &st);
        heading[i] = st.valid ? st.nacelle_heading_rad : 0.0;
        /* valid=0 时传入 NaN，store 会等待方位就绪 */
        if (!st.valid) heading[i] = 0.0 / 0.0; /* NaN */
    }

    /* 3) CommandStore refresh + setpoint 生成 */
    fcr_command_store_refresh(rt->cs, sim_time_s, heading, rt->ss->low_step);
    fcr_runtime_update_setpoints(rt, sim_time_s, rt->ss->low_step);

    return 0;
}

/* 聚合（Phase 5 完善统计；当前直通瞬时值）                            */
static void aggregate_state(FcrRuntime *rt, FarmStateFrame *out, double sim_time_s)
{
    uint32_t i;
    memset(out, 0, sizeof(*out));
    out->protocol_version = FCR_PROTOCOL_VERSION;
    out->sim_time_s = sim_time_s;
    out->fast_step = rt->ss->fast_step;
    out->low_step = rt->ss->low_step;
    out->rt_health_flags = fcr_watchdog_health(rt->wd);
    out->n_turbines = rt->cfg.n_turbines;
    out->flow_source_time_s = rt->ss->flow.valid ? rt->ss->flow.flow_source_time_s : -1.0;
    out->flow_seq = rt->ss->flow.flow_seq;

    for (i = 0; i < rt->cfg.n_turbines; ++i) {
        FcrFarmTurbineState *ts = &out->turbines[i];
        const FcrRoscoFastState *fast = &rt->ss->fast[i];
        const FcrTurbineExtraState *extra = &rt->ss->extra[i];
        ts->turbine_id = (int32_t)(i + 1);
        if (!fast->valid) continue;
        ts->valid = 1;
        ts->source_time_s = fast->source_time_s;
        ts->nacelle_heading_rad = fast->nacelle_heading_rad;
        ts->nacelle_vane_rad = fast->nacelle_vane_rad;
        ts->hub_wind_speed_mps = fast->hub_wind_speed_mps;
        ts->rotor_speed_rad_s = fast->rotor_speed_rad_s;
        ts->gen_speed_rad_s = fast->gen_speed_rad_s;
        ts->rotor_ct = extra->valid ? extra->rotor_ct : 0.0;
        ts->rotor_cp = extra->valid ? extra->rotor_cp : 0.0;
        memcpy(ts->blade_pitch_rad, fast->blade_pitch_rad, sizeof(ts->blade_pitch_rad));
        ts->yaw_target_heading_rad = fast->yaw_target_heading_rad;
        ts->yaw_seq_applied = fast->yaw_seq_applied;
        ts->induction_seq_applied = fast->induction_seq_applied;
        ts->controller_status_flags = fast->controller_status_flags;

        /* 1 s 统计：Phase 5 由 SignalStatistics 填充；当前直通瞬时 */
        ts->gen_power_w.instant = fast->gen_power_w;
        ts->gen_power_w.valid = fast->valid;
        ts->gen_power_w.n_samples = 1;
        ts->gen_power_w.mean = fast->gen_power_w;
        ts->gen_power_w.rms = fast->gen_power_w;
        ts->gen_power_w.min = fast->gen_power_w;
        ts->gen_power_w.max = fast->gen_power_w;
        if (extra->valid) {
            ts->rotor_thrust_n.instant = extra->rotor_thrust_n;
            ts->rotor_thrust_n.valid = 1; ts->rotor_thrust_n.n_samples = 1;
            ts->rotor_thrust_n.mean = extra->rotor_thrust_n;
            ts->rotor_thrust_n.rms = extra->rotor_thrust_n;
            ts->rotor_thrust_n.min = extra->rotor_thrust_n;
            ts->rotor_thrust_n.max = extra->rotor_thrust_n;
            ts->tower_base_fa_moment_nm.instant = extra->tower_base_fa_moment_nm;
            ts->tower_base_fa_moment_nm.valid = 1; ts->tower_base_fa_moment_nm.n_samples = 1;
            ts->tower_base_fa_moment_nm.mean = extra->tower_base_fa_moment_nm;
            ts->tower_base_fa_moment_nm.rms = extra->tower_base_fa_moment_nm;
            ts->tower_base_fa_moment_nm.min = extra->tower_base_fa_moment_nm;
            ts->tower_base_fa_moment_nm.max = extra->tower_base_fa_moment_nm;
            ts->tower_base_ss_moment_nm.instant = extra->tower_base_ss_moment_nm;
            ts->tower_base_ss_moment_nm.valid = 1; ts->tower_base_ss_moment_nm.n_samples = 1;
            ts->tower_base_ss_moment_nm.mean = extra->tower_base_ss_moment_nm;
            ts->tower_base_ss_moment_nm.rms = extra->tower_base_ss_moment_nm;
            ts->tower_base_ss_moment_nm.min = extra->tower_base_ss_moment_nm;
            ts->tower_base_ss_moment_nm.max = extra->tower_base_ss_moment_nm;
        }
        /* 低速量 ZOH */
        if (rt->ss->flow.valid) {
            ts->disk_ambient_wind_mps = rt->ss->flow.disk_ambient_wind_mps[i];
            ts->disk_disturbed_wind_mps = rt->ss->flow.disk_disturbed_wind_mps[i];
            ts->disk_ambient_ti = rt->ss->flow.disk_ambient_ti[i];
            ts->flow_source_time_s = rt->ss->flow.flow_source_time_s;
            ts->flow_seq = rt->ss->flow.flow_seq;
        }
    }
}

int fcr_runtime_step_low(FcrRuntime *rt, double sim_time_s)
{
    FarmStateFrame out;
    (void)sim_time_s;
    if (!rt) return -1;

    fcr_watchdog_update(rt->wd, rt->ss->sim_time_s, rt->ss, rt->cs);
    if (rt->aggregator) {
        fcr_state_aggregator_build_frame((FcrStateAggregator *)rt->aggregator, rt->ss, &out);
    } else {
        aggregate_state(rt, &out, rt->ss->sim_time_s);
    }
    if (rt->cfg.transport) {
        fcr_transport_publish(rt->cfg.transport, &out);
    }
    rt->last_low_step_time_s = rt->ss->sim_time_s;
    return 0;
}

/* ------------------------------------------------------------------ */
/* 高速样本喂入（发布事件驱动 1 s 统计；Phase 5）                      */
/* ------------------------------------------------------------------ */
void fcr_runtime_feed_fast(FcrRuntime *rt, int32_t turbine_id,
                           const FcrRoscoFastState *st)
{
    FcrStateAggregator *a;
    if (!rt || !st) return;
    a = (FcrStateAggregator *)rt->aggregator;
    if (a) fcr_state_aggregator_feed_fast(a, turbine_id, st);
}

void fcr_runtime_feed_extra_stats(FcrRuntime *rt, const FcrTurbineExtraState *st)
{
    FcrStateAggregator *a;
    if (!rt || !st) return;
    a = (FcrStateAggregator *)rt->aggregator;
    if (a) fcr_state_aggregator_feed_extra(a, st);
}

int fcr_publish_turbine_extra_state(FcrRuntime *rt, const FcrTurbineExtraState *st)
{
    if (!rt) return -1;
    fcr_runtime_feed_extra_stats(rt, st);
    return fcr_state_store_set_extra(rt->ss, st);
}

int fcr_publish_flow_state(FcrRuntime *rt, const FcrFlowState *flow)
{
    if (!rt) return -1;
    return fcr_state_store_set_flow(rt->ss, flow);
}
int fcr_publish_rt_clock(FcrRuntime *rt, double sim_time_s,
                         uint64_t fast_step, uint64_t low_step,
                         uint32_t rt_health_flags)
{
    if (!rt) return -1;
    return fcr_state_store_set_clock(rt->ss, sim_time_s, fast_step, low_step,
                                     rt_health_flags);
}