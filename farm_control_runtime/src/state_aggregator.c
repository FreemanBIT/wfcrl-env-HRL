/*
 * state_aggregator.c — 多速率状态聚合（Phase 5）
 *
 * 在 1 s 低速步把多个来源的最新完整快照聚合为 FarmStateFrame：
 *   - ROSCO fast state    (~10 ms)：瞬时直通 + 1 s 统计（SignalStatistics）
 *   - OF extra state      (~10 ms)：瞬时直通 + 1 s 统计
 *   - AWAE flow state     (~1 s)：ZOH（来源 time/seq 透传）
 *   - rt clock            (10 ms)：sim_time/fast_step/low_step/health
 *
 * 同步规则（PROTOCOL_DECISIONS §5）：latest-complete-snapshot；不插值伪造；
 * 每个来源保留 source_time/source_seq 供 age/丢帧检测。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

struct FcrStateAggregator {
    uint32_t n_turbines;
    double   window_s;           /* 统计窗口（1 s） */
    /* 每机统计（10 ms 高速量） */
    FcrSignalStatistics *gen_power[FCR_MAX_TURBINES];
    FcrSignalStatistics *rotor_thrust[FCR_MAX_TURBINES];
    FcrSignalStatistics *tower_base_fa[FCR_MAX_TURBINES];
    FcrSignalStatistics *tower_base_ss[FCR_MAX_TURBINES];
    FcrSignalStatistics *hub_wind[FCR_MAX_TURBINES];
    uint64_t frame_seq;
};

FcrStateAggregator *fcr_state_aggregator_create(uint32_t n_turbines, double window_s)
{
    FcrStateAggregator *a;
    uint32_t i;
    if (n_turbines == 0 || n_turbines > FCR_MAX_TURBINES) return NULL;
    a = (FcrStateAggregator *)calloc(1, sizeof(FcrStateAggregator));
    if (!a) return NULL;
    a->n_turbines = n_turbines;
    a->window_s = (window_s > 0.0) ? window_s : 1.0;
    for (i = 0; i < n_turbines; ++i) {
        a->gen_power[i] = fcr_signal_stat_create(a->window_s, 0);
        a->rotor_thrust[i] = fcr_signal_stat_create(a->window_s, 0);
        a->tower_base_fa[i] = fcr_signal_stat_create(a->window_s, 0);
        a->tower_base_ss[i] = fcr_signal_stat_create(a->window_s, 0);
        a->hub_wind[i] = fcr_signal_stat_create(a->window_s, 0);
        if (!a->gen_power[i] || !a->rotor_thrust[i] || !a->tower_base_fa[i] ||
            !a->tower_base_ss[i] || !a->hub_wind[i]) {
            fcr_state_aggregator_destroy(a);
            return NULL;
        }
    }
    return a;
}

void fcr_state_aggregator_destroy(FcrStateAggregator *a)
{
    uint32_t i;
    if (!a) return;
    for (i = 0; i < a->n_turbines; ++i) {
        fcr_signal_stat_destroy(a->gen_power[i]);
        fcr_signal_stat_destroy(a->rotor_thrust[i]);
        fcr_signal_stat_destroy(a->tower_base_fa[i]);
        fcr_signal_stat_destroy(a->tower_base_ss[i]);
        fcr_signal_stat_destroy(a->hub_wind[i]);
    }
    free(a);
}

/* 发布事件驱动（10 ms）：ROSCO fast state 样本 */
void fcr_state_aggregator_feed_fast(FcrStateAggregator *a, int32_t turbine_id,
                                    const FcrRoscoFastState *st)
{
    FcrSignalStatistics *s;
    if (!a || !st) return;
    if (turbine_id < 1 || turbine_id > (int32_t)a->n_turbines) return;
    if (!st->valid) return;
    s = a->gen_power[turbine_id - 1];
    fcr_signal_stat_update(s, st->source_time_s, st->gen_power_w);
    fcr_signal_stat_update(a->hub_wind[turbine_id - 1], st->source_time_s,
                           st->hub_wind_speed_mps);
}

/* 发布事件驱动（10 ms）：OF extra 样本 */
void fcr_state_aggregator_feed_extra(FcrStateAggregator *a, const FcrTurbineExtraState *st)
{
    if (!a || !st) return;
    if (st->turbine_id < 1 || st->turbine_id > (int32_t)a->n_turbines) return;
    if (!st->valid) return;
    fcr_signal_stat_update(a->rotor_thrust[st->turbine_id - 1], st->source_time_s,
                           st->rotor_thrust_n);
    /* tower base 不可得（NaN）→ 不进入统计（保持 invalid） */
    if (isfinite(st->tower_base_fa_moment_nm)) {
        fcr_signal_stat_update(a->tower_base_fa[st->turbine_id - 1], st->source_time_s,
                               st->tower_base_fa_moment_nm);
    }
    if (isfinite(st->tower_base_ss_moment_nm)) {
        fcr_signal_stat_update(a->tower_base_ss[st->turbine_id - 1], st->source_time_s,
                               st->tower_base_ss_moment_nm);
    }
}

/* 1 s 低速步：构造聚合帧 */
void fcr_state_aggregator_build_frame(FcrStateAggregator *a, const FcrStateStore *ss,
                                      FarmStateFrame *out)
{
    uint32_t i;
    if (!a || !ss || !out) return;
    memset(out, 0, sizeof(*out));
    out->protocol_version = FCR_PROTOCOL_VERSION;
    out->frame_seq = (uint32_t)(++a->frame_seq);
    out->sim_time_s = ss->sim_time_s;
    out->fast_step = ss->fast_step;
    out->low_step = ss->low_step;
    out->rt_health_flags = ss->rt_health_flags;
    out->n_turbines = a->n_turbines;
    out->flow_source_time_s = ss->flow.valid ? ss->flow.flow_source_time_s : -1.0;
    out->flow_seq = ss->flow.flow_seq;

    for (i = 0; i < a->n_turbines && i < ss->n_turbines; ++i) {
        FcrFarmTurbineState *ts = &out->turbines[i];
        const FcrRoscoFastState *f = &ss->fast[i];
        const FcrTurbineExtraState *e = &ss->extra[i];
        ts->turbine_id = (int32_t)(i + 1);
        if (!f->valid) continue;
        ts->valid = 1;
        ts->source_time_s = f->source_time_s;
        ts->nacelle_heading_rad = f->nacelle_heading_rad;
        ts->nacelle_vane_rad = f->nacelle_vane_rad;
        ts->hub_wind_speed_mps = f->hub_wind_speed_mps;
        ts->rotor_speed_rad_s = f->rotor_speed_rad_s;
        ts->gen_speed_rad_s = f->gen_speed_rad_s;
        memcpy(ts->blade_pitch_rad, f->blade_pitch_rad, sizeof(ts->blade_pitch_rad));
        ts->yaw_target_heading_rad = f->yaw_target_heading_rad;
        ts->yaw_seq_applied = f->yaw_seq_applied;
        ts->induction_seq_applied = f->induction_seq_applied;
        ts->controller_status_flags = f->controller_status_flags;
        ts->rotor_ct = e->valid ? e->rotor_ct : 0.0;
        ts->rotor_cp = e->valid ? e->rotor_cp : 0.0;

        /* 1 s 统计结果 */
        fcr_signal_stat_get(a->gen_power[i], &ts->gen_power_w);
        fcr_signal_stat_get(a->rotor_thrust[i], &ts->rotor_thrust_n);
        fcr_signal_stat_get(a->tower_base_fa[i], &ts->tower_base_fa_moment_nm);
        fcr_signal_stat_get(a->tower_base_ss[i], &ts->tower_base_ss_moment_nm);
        if (!ts->gen_power_w.valid) {
            /* 统计未闭合：瞬时直通 */
            ts->gen_power_w.valid = 1;
            ts->gen_power_w.instant = f->gen_power_w;
            ts->gen_power_w.mean = f->gen_power_w;
            ts->gen_power_w.rms = f->gen_power_w;
            ts->gen_power_w.min = f->gen_power_w;
            ts->gen_power_w.max = f->gen_power_w;
            ts->gen_power_w.n_samples = 1;
        }

        /* 低速量 ZOH（flow 快照） */
        if (ss->flow.valid) {
            ts->disk_ambient_wind_mps = ss->flow.disk_ambient_wind_mps[i];
            ts->disk_disturbed_wind_mps = ss->flow.disk_disturbed_wind_mps[i];
            ts->disk_ambient_ti = ss->flow.disk_ambient_ti[i];
            ts->flow_source_time_s = ss->flow.flow_source_time_s;
            ts->flow_seq = ss->flow.flow_seq;
        }
    }
}