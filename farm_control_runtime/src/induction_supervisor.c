/*
 * induction_supervisor.c — 诱导因子动作监督与 ROSCO 参考映射（Phase 6）
 *
 * induction_ref（无量纲轴向诱导 a）逐机映射为 ROSCO supervisory references：
 *   ct_ref             = ct_coef * 4a(1-a)                 （动量理论 + 标定系数）
 *   power_ratio        = [a(1-a)^2] / [a0(1-a0)^2]          （相对基线 a0=1/3 的功率提取比）
 *   speed_ref_ratio    = power_ratio^(1/3)
 *   torque_limit_ratio = power_ratio^(2/3)
 *   min_pitch_rad      = clamp(min_pitch_gain*(1-power_ratio), 0, min_pitch_max)
 *
 * 规则（对应 PROTOCOL_DECISIONS §3）：
 *   - 新 induction_seq 才重算并锁存；seq 之间 ZOH；
 *   - 越界 / NaN → fallback（安全默认）并置 FALLBACK 标志；
 *   - 通道失效（TTL 到期）→ 回退未使能（由 command_store 语义保证）。
 * V1：确定性标定映射；后续可替换为 Ct 闭环 / PI / 机型 map。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const FcrInductionMapConfig default_cfg = {
    0.3333333333333333,  /* a0 */
    0.05,                /* a_min */
    0.45,                /* a_max */
    1.0,                 /* ct_coef */
    0.55,                /* min_pitch_gain (rad)；region2 标定：8m/s 下 1rad≈-5%~-7% 功率 */
    0.30,                /* min_pitch_max (rad) */
    1.0                  /* power_cap */
};

FcrInductionSupervisor *fcr_induction_supervisor_create(uint32_t n_turbines,
                                                        const FcrInductionMapConfig *cfg)
{
    FcrInductionSupervisor *s;
    if (n_turbines == 0 || n_turbines > FCR_MAX_TURBINES) return NULL;
    s = (FcrInductionSupervisor *)calloc(1, sizeof(FcrInductionSupervisor));
    if (!s) return NULL;
    s->n_turbines = n_turbines;
    s->cfg = cfg ? *cfg : default_cfg;
    return s;
}

void fcr_induction_supervisor_destroy(FcrInductionSupervisor *s)
{
    if (s) free(s);
}

static int map_ref(const FcrInductionSupervisor *s, double a, FcrInductionState *out)
{
    const FcrInductionMapConfig *c = &s->cfg;
    double pe, pe0, pr;

    if (!isfinite(a) || a < c->a_min || a > c->a_max) {
        /* fallback：安全默认（额定语义） */
        memset(out, 0, sizeof(*out));
        out->ct_ref = 4.0 * c->a0 * (1.0 - c->a0);
        out->power_ratio = 1.0;
        out->speed_ref_ratio = 1.0;
        out->torque_limit_ratio = 1.0;
        out->min_pitch_rad = 0.0;
        out->fallback = 1;
        return 1;
    }
    out->ct_ref = c->ct_coef * 4.0 * a * (1.0 - a);
    pe = a * (1.0 - a) * (1.0 - a);
    pe0 = c->a0 * (1.0 - c->a0) * (1.0 - c->a0);
    pr = pe / pe0;
    if (pr > c->power_cap) pr = c->power_cap;
    if (pr < 0.05) pr = 0.05;
    out->power_ratio = pr;
    out->speed_ref_ratio = pow(pr, 1.0 / 3.0);
    out->torque_limit_ratio = pow(pr, 2.0 / 3.0);
    out->min_pitch_rad = c->min_pitch_gain * (1.0 - pr);
    if (out->min_pitch_rad < 0.0) out->min_pitch_rad = 0.0;
    if (out->min_pitch_rad > c->min_pitch_max) out->min_pitch_rad = c->min_pitch_max;
    out->fallback = 0;
    return 0;
}

/* 新 seq 计算并锁存；同 seq 幂等（ZOH）；通道失效清空 */
void fcr_induction_supervisor_update(FcrInductionSupervisor *s, int32_t turbine_id,
                                     const FcrTurbineCommandState *tc,
                                     double sim_time_s)
{
    FcrInductionState *st;
    if (!s || !tc) return;
    if (turbine_id < 1 || turbine_id > (int32_t)s->n_turbines) return;
    st = &s->t[turbine_id - 1];

    if (!tc->ind_valid) {
        /* 通道失效：监督状态清空（回退） */
        memset(st, 0, sizeof(*st));
        return;
    }
    if (tc->ind_seq_rx != st->seq_applied) {
        /* 新 seq：重算映射并锁存 */
        memset(st, 0, sizeof(*st));
        st->seq_applied = tc->ind_seq_rx;
        st->induction_ref = tc->induction_ref;
        map_ref(s, tc->induction_ref, st);
        st->valid = st->fallback ? 0u : 1u;   /* fallback 时也锁存（诊断），但标记 */
        st->last_update_time_s = sim_time_s;
        return;
    }
    /* 同 seq：ZOH（不做任何改动） */
    st->last_update_time_s = sim_time_s;
}

/* 应用到 setpoint（覆盖 ROSCO 参考比例） */
void fcr_induction_supervisor_apply(const FcrInductionSupervisor *s, int32_t turbine_id,
                                    FcrRoscoExternalSetpoint *sp)
{
    const FcrInductionState *st;
    if (!s || !sp) return;
    if (turbine_id < 1 || turbine_id > (int32_t)s->n_turbines) return;
    st = &s->t[turbine_id - 1];
    if (!st->valid && !st->fallback) return;   /* 无生效映射 */
    sp->induction_enable = 1;
    sp->induction_seq = st->seq_applied;
    sp->ct_ref = st->ct_ref;
    sp->power_ratio = st->power_ratio;
    sp->speed_ref_ratio = st->speed_ref_ratio;
    sp->torque_limit_ratio = st->torque_limit_ratio;
    sp->min_pitch_rad = st->min_pitch_rad;
    if (st->fallback) sp->command_status_flags |= FCR_CMD_FLAG_FALLBACK_ACTIVE;
    sp->command_status_flags |= FCR_CMD_FLAG_INDUCTION_VALID;
}

int fcr_induction_supervisor_get(const FcrInductionSupervisor *s, int32_t turbine_id,
                                 FcrInductionState *out)
{
    if (!s || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)s->n_turbines) return -2;
    *out = s->t[turbine_id - 1];
    return 0;
}

const FcrInductionMapConfig *fcr_induction_supervisor_cfg(const FcrInductionSupervisor *s)
{
    return s ? &s->cfg : NULL;
}