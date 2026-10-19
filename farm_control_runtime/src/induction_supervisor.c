/*
 * induction_supervisor.c — 诱导因子动作监督与 ROSCO 参考映射（ICD v1.5）
 *
 * induction_ref（无量纲轴向诱导 a）逐机映射为 ROSCO supervisory references：
 *   ct_ref                      = ct_coef * 4a(1-a)          （动量理论 + 标定系数）
 *   r_pitch (S0803)             = [a(1-a)^2] / [a0(1-a0)^2]  （相对基线 a0=1/3 的功率提取比）
 *   r_speed (S0804)             = r_pitch^(1/3)
 *   r_torque (S0805)            = r_pitch^(2/3)
 *   min_pitch_ref_rad (S0806)   = clamp(min_pitch_gain*(1-r_pitch), 0, min_pitch_max)
 *   collective_pitch_offset (S0807) = 0 （当前实现不使用）
 *
 * 规则（对应 ICD v1.5 S0705-S0709）：
 *   - 新 induction_seq 才重算并锁存；seq 之间 ZOH；
 *   - 越界 / NaN → fallback（安全默认）并置 FALLBACK 标志；
 *   - 通道失效（TTL 到期）→ 回退未使能（由 command_store 语义保证）。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* 默认配置。
 * 注意：必须使用**指定初始化器**——v1.5 重构中本结构体字段顺序曾被调整，
 * 位置初始化会把 power_cap/min_pitch_gain/min_pitch_max 装错，
 * 导致 r_pitch 被错误地钳到 0.55（诱导通道被严重过度降载）。 */
static const FcrInductionMapConfig default_cfg = {
    .a0             = 0.3333333333333333,  /* 基线诱导 a0 = 1/3 */
    .a_min          = 0.05,                /* 可实现最小诱导 */
    .a_max          = 0.3333333333333333,  /* 可实现最大诱导 = a0 (WIN-12) */
    .ct_coef        = 1.0,                 /* Ct 标定系数 */
    .min_pitch_gain = 0.55,                /* 最小桨距增益 (rad) */
    .min_pitch_max  = 0.30,                /* 最小桨距上限 (rad) */
    .power_cap      = 1.0                  /* r_pitch 上限 */
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
        /* fallback：安全默认（额定语义）。
         * 注意：不得 memset 整个 out —— 上层已经写入 seq_applied/induction_ref。 */
        out->ct_ref = 4.0 * c->a0 * (1.0 - c->a0);
        out->r_pitch = 1.0;
        out->r_speed = 1.0;
        out->r_torque = 1.0;
        out->min_pitch_ref_rad = 0.0;
        out->collective_pitch_offset_rad = 0.0;
        out->fallback = 1;
        return 1;
    }
    out->ct_ref = c->ct_coef * 4.0 * a * (1.0 - a);
    pe = a * (1.0 - a) * (1.0 - a);
    pe0 = c->a0 * (1.0 - c->a0) * (1.0 - c->a0);
    pr = pe / pe0;
    if (pr > c->power_cap) pr = c->power_cap;
    if (pr < 0.05) pr = 0.05;
    out->r_pitch = pr;
    out->r_speed = pow(pr, 1.0 / 3.0);
    out->r_torque = pow(pr, 2.0 / 3.0);
    out->min_pitch_ref_rad = c->min_pitch_gain * (1.0 - pr);
    if (out->min_pitch_ref_rad < 0.0) out->min_pitch_ref_rad = 0.0;
    if (out->min_pitch_ref_rad > c->min_pitch_max) out->min_pitch_ref_rad = c->min_pitch_max;
    out->collective_pitch_offset_rad = 0.0;
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
    sp->r_pitch = st->r_pitch;
    sp->r_speed = st->r_speed;
    sp->r_torque = st->r_torque;
    sp->min_pitch_ref_rad = st->min_pitch_ref_rad;
    sp->collective_pitch_offset_rad = st->collective_pitch_offset_rad;
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
