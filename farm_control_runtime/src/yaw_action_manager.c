/*
 * yaw_action_manager.c — 偏航增量动作执行状态管理（Phase 3）
 *
 * 职责（对应 development_plan §3.1 / Phase 3）：
 *   - 跟踪每机偏航动作执行状态：目标锁存值、seq、跟踪误差、reached 判定；
 *   - 目标锁存的**数值来源**是 CommandStore（yaw_delta → 绝对目标只计算一次）；
 *   - 本模块只做状态监督与诊断（供 FarmStateFrame/测试使用），不重复计算目标；
 *   - 执行期间目标固定不变（latch 语义由 CommandStore 保证）。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

FcrYawActionManager *fcr_yaw_manager_create(uint32_t n_turbines, double deadband_rad)
{
    FcrYawActionManager *mgr;
    if (n_turbines == 0 || n_turbines > FCR_MAX_TURBINES) return NULL;
    mgr = (FcrYawActionManager *)calloc(1, sizeof(FcrYawActionManager));
    if (!mgr) return NULL;
    mgr->n_turbines = n_turbines;
    mgr->deadband_rad = (deadband_rad > 0.0) ? deadband_rad : 0.0;
    return mgr;
}

void fcr_yaw_manager_destroy(FcrYawActionManager *mgr)
{
    if (mgr) free(mgr);
}

void fcr_yaw_manager_update(FcrYawActionManager *mgr, int32_t turbine_id,
                            const FcrTurbineCommandState *tc,
                            double heading_now_rad, double sim_time_s)
{
    FcrYawActionState *st;
    uint32_t was_tracking, was_reached;
    if (!mgr || !tc) return;
    if (turbine_id < 1 || turbine_id > (int32_t)mgr->n_turbines) return;
    st = &mgr->t[turbine_id - 1];

    was_tracking = st->tracking;
    was_reached = st->reached;

    /* 通道无效（TTL 到期/从未下发）→ 状态清空（回退） */
    if (!tc->yaw_valid) {
        memset(st, 0, sizeof(*st));
        st->turbine_id = turbine_id;
        st->last_update_time_s = sim_time_s;
        st->valid = 0;
        return;
    }

    st->valid = 1;
    st->seq = tc->yaw_seq_applied ? tc->yaw_seq_applied : tc->yaw_seq_rx;
    st->last_update_time_s = sim_time_s;

    /* 目标锁存后固定；未锁存（方位未知）则不更新目标 */
    if (tc->yaw_target_latched) {
        st->target_heading_rad = tc->yaw_target_heading;
    }

    if (fcr_is_finite(heading_now_rad)) {
        st->heading_now_rad = heading_now_rad;
        st->error_rad = fcr_wrap_pm_pi(st->target_heading_rad - heading_now_rad);
        st->reached = (mgr->deadband_rad > 0.0)
                          ? (fabs(st->error_rad) <= mgr->deadband_rad ? 1u : 0u)
                          : 0u;
        st->tracking = st->reached ? 0u : 1u;
    }

    /* 状态变化诊断（可上报） */
    (void)was_tracking; (void)was_reached;
}

int fcr_yaw_manager_get(const FcrYawActionManager *mgr, int32_t turbine_id,
                        FcrYawActionState *out)
{
    if (!mgr || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)mgr->n_turbines) return -2;
    *out = mgr->t[turbine_id - 1];
    return 0;
}