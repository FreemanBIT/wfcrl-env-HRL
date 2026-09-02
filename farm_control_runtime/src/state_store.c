/*
 * state_store.c — 多速率状态存储（Phase 1）
 *
 * 保存各来源的最新完整快照（latest-complete-snapshot 语义）：
 *   - ROSCO fast state     ~10 ms（每机槽位）
 *   - OF extra state       优选 ~10 ms（每机槽位）
 *   - AWAE flow state      ~1 s（ZOH 到下一次快照）
 *   - rt clock             sim_time / fast_step / low_step / health
 *
 * 不做插值、不伪造时间戳。线程安全：publish 采用"写后置有效标志 +
 * 消费者读快照"策略（单写者单读者场景，Volatile 屏障足够；
 * 多写者场景由宿主保证 Provider 线程互斥）。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>

FcrStateStore *fcr_state_store_create(uint32_t n_turbines)
{
    FcrStateStore *ss;
    if (n_turbines == 0 || n_turbines > FCR_MAX_TURBINES) return NULL;
    ss = (FcrStateStore *)calloc(1, sizeof(FcrStateStore));
    if (!ss) return NULL;
    ss->n_turbines = n_turbines;
    return ss;
}

void fcr_state_store_destroy(FcrStateStore *ss)
{
    if (ss) free(ss);
}

int fcr_state_store_set_clock(FcrStateStore *ss, double sim_time_s,
                              uint64_t fast_step, uint64_t low_step,
                              uint32_t rt_health_flags)
{
    if (!ss) return -1;
    ss->sim_time_s = sim_time_s;
    ss->fast_step = fast_step;
    ss->low_step = low_step;
    ss->rt_health_flags = rt_health_flags;
    return 0;
}

int fcr_state_store_set_fast(FcrStateStore *ss, int32_t turbine_id,
                             const FcrRoscoFastState *st)
{
    if (!ss || !st) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)ss->n_turbines) return -2;
    ss->fast[turbine_id - 1] = *st;
    return 0;
}

int fcr_state_store_set_extra(FcrStateStore *ss, const FcrTurbineExtraState *st)
{
    if (!ss || !st) return -1;
    if (st->turbine_id < 1 || st->turbine_id > (int32_t)ss->n_turbines) return -2;
    ss->extra[st->turbine_id - 1] = *st;
    return 0;
}

int fcr_state_store_set_flow(FcrStateStore *ss, const FcrFlowState *flow)
{
    if (!ss || !flow) return -1;
    ss->flow = *flow;   /* ZOH：保持到下一次 AWAE 快照 */
    return 0;
}

int fcr_state_store_get_fast(const FcrStateStore *ss, int32_t turbine_id,
                             FcrRoscoFastState *out)
{
    if (!ss || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)ss->n_turbines) return -2;
    *out = ss->fast[turbine_id - 1];
    return 0;
}

const FcrFlowState *fcr_state_store_get_flow(const FcrStateStore *ss)
{
    return ss ? &ss->flow : NULL;
}
