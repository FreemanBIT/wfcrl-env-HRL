/*
 * watchdog.c — 健康监视（Phase 1）
 *
 * 聚合健康标志（FCR_RT_HEALTH_*）：
 *   - 时钟未见 → 无 clock；
 *   - 流场年龄超过阈值 → FLOW_STALE；
 *   - 命令通道超龄 → 分别置 YAW/INDUCTION_STALE（command store 内处理，
 *     这里汇总到 fps）。
 * Phase 5 起增加流场丢帧（flow_seq 跳变）检测。
 */
#include "fcr_internal.h"
#include <stdlib.h>

FcrWatchdog *fcr_watchdog_create(double flow_stale_threshold_s)
{
    FcrWatchdog *wd = (FcrWatchdog *)calloc(1, sizeof(FcrWatchdog));
    if (!wd) return NULL;
    wd->flow_stale_threshold_s = flow_stale_threshold_s;
    wd->command_stale_threshold_s = 0.0;  /* 由 command store 管理 */
    wd->health_flags = 0u;
    wd->clock_seen = 0;
    return wd;
}

void fcr_watchdog_destroy(FcrWatchdog *wd)
{
    if (wd) free(wd);
}

void fcr_watchdog_update(FcrWatchdog *wd, double sim_time_s,
                         const FcrStateStore *ss,
                         const FcrCommandStore *cs)
{
    uint32_t flags = FCR_RT_HEALTH_OK;
    (void)cs;
    if (!wd) return;
    if (!ss) { wd->health_flags = 0u; return; }

    if (wd->clock_seen && ss->sim_time_s >= 0.0) {
        /* 流场 age 检查 */
        if (ss->flow.valid &&
            (sim_time_s - ss->flow.flow_source_time_s) > wd->flow_stale_threshold_s) {
            flags |= FCR_RT_HEALTH_FLOW_STALE;
        }
    } else {
        flags = 0u;   /* 时钟尚未就绪，不报 OK */
    }

    /* 快速步丢步检测（fast_step 非单调时置位） */
    if (wd->clock_seen && ss->fast_step > 0) {
        if (ss->fast_step <= wd->last_fast_step && ss->fast_step != wd->last_fast_step) {
            flags |= FCR_RT_HEALTH_FAST_DROPPED;
        }
        wd->last_fast_step = ss->fast_step;
    }

    wd->last_clock_time_s = sim_time_s;
    wd->clock_seen = 1;
    wd->health_flags = flags;
}

uint32_t fcr_watchdog_health(const FcrWatchdog *wd)
{
    return wd ? wd->health_flags : 0u;
}
