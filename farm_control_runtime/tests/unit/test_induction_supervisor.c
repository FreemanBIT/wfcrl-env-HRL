/*
 * test_induction_supervisor.c — 诱导映射测试（Phase 6）
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include <string.h>
#include <math.h>

static void make_ind_cmd(FcrTurbineCommandState *tc, uint64_t seq, double ref, double ttl)
{
    memset(tc, 0, sizeof(*tc));
    tc->ind_seq_rx = seq;
    tc->ind_seq_applied = seq;
    tc->induction_ref = ref;
    tc->ind_valid = 1;
    tc->ind_ttl_s = ttl;
    tc->ind_issue_time_s = 0.0;
}

static void test_baseline_mapping(void)
{
    FcrInductionSupervisor *s = fcr_induction_supervisor_create(1, NULL);
    FcrTurbineCommandState tc;
    FcrInductionState st;
    FcrRoscoExternalSetpoint sp;
    ASSERT(s != NULL);

    /* a = 1/3（基线）：power_ratio≈1, ct=8/9 */
    make_ind_cmd(&tc, 1, 0.333333333, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 0.0);
    fcr_induction_supervisor_get(s, 1, &st);
    ASSERT(st.valid == 1);
    ASSERT(st.seq_applied == 1);
    ASSERT_NEAR(st.ct_ref, 8.0 / 9.0, 1e-9);
    ASSERT_NEAR(st.r_pitch, 1.0, 1e-6);
    ASSERT_NEAR(st.r_speed, 1.0, 1e-6);
    ASSERT_NEAR(st.r_torque, 1.0, 1e-6);
    ASSERT_NEAR(st.min_pitch_ref_rad, 0.0, 1e-9);

    memset(&sp, 0, sizeof(sp));
    fcr_induction_supervisor_apply(s, 1, &sp);
    ASSERT(sp.induction_enable == 1);
    ASSERT(sp.induction_seq == 1);
    ASSERT_NEAR(sp.ct_ref, 8.0 / 9.0, 1e-9);

    fcr_induction_supervisor_destroy(s);
}

static void test_derating_mapping(void)
{
    FcrInductionSupervisor *s = fcr_induction_supervisor_create(1, NULL);
    FcrTurbineCommandState tc;
    FcrInductionState st;
    double pr, pe, pe0;

    /* a = 0.2（降载）：r_pitch < 1, min_pitch > 0 */
    make_ind_cmd(&tc, 1, 0.2, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 0.0);
    fcr_induction_supervisor_get(s, 1, &st);
    ASSERT(st.valid == 1);
    ASSERT(st.fallback == 0);
    ASSERT(st.r_pitch < 1.0 && st.r_pitch > 0.0);
    ASSERT(st.ct_ref < 8.0 / 9.0);

    /* 解析校验：pr = [a(1-a)^2] / [a0(1-a0)^2]，a0 = 1/3 → pr = 0.864 */
    pe  = 0.2 * 0.8 * 0.8;
    pe0 = (1.0 / 3.0) * (2.0 / 3.0) * (2.0 / 3.0);
    pr  = pe / pe0;
    ASSERT_NEAR(st.r_pitch, pr, 1e-12);
    ASSERT_NEAR(st.r_speed, pow(pr, 1.0 / 3.0), 1e-12);
    ASSERT_NEAR(st.r_torque, pow(pr, 2.0 / 3.0), 1e-12);
    ASSERT_NEAR(st.ct_ref, 4.0 * 0.2 * 0.8, 1e-12);
    /* P ∝ ω^3 → r_speed = pr^(1/3)；T = P/ω → r_torque = pr^(2/3) */
    ASSERT(st.r_speed > st.r_torque);
    ASSERT(st.r_torque > st.r_pitch);
    ASSERT_NEAR(st.min_pitch_ref_rad, 0.55 * (1.0 - pr), 1e-12);
    ASSERT(st.min_pitch_ref_rad > 0.0);

    /* a = a_max = 1/3（基线边界）：不降载 */
    make_ind_cmd(&tc, 2, 1.0 / 3.0, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 0.5);
    fcr_induction_supervisor_get(s, 1, &st);
    ASSERT(st.valid == 1);
    ASSERT_NEAR(st.r_pitch, 1.0, 1e-9);
    ASSERT_NEAR(st.ct_ref, 8.0 / 9.0, 1e-9);
    ASSERT_NEAR(st.min_pitch_ref_rad, 0.0, 1e-12);

    /* a = 0.45 > a_max → fallback（安全默认：不降载，valid=0） */
    make_ind_cmd(&tc, 3, 0.45, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 1.0);
    fcr_induction_supervisor_get(s, 1, &st);
    ASSERT(st.fallback == 1);
    ASSERT(st.valid == 0);
    ASSERT_NEAR(st.r_pitch, 1.0, 1e-9);
    ASSERT_NEAR(st.ct_ref, 8.0 / 9.0, 1e-9);

    fcr_induction_supervisor_destroy(s);
}

/* 回归门禁：FcrInductionMapConfig 默认配置装配必须正确。
 * 背景（真实缺陷）：v1.5 重构把 power_cap 移到结构体中部，但
 * induction_supervisor.c 仍用位置初始化，导致 power_cap=0.55、
 * min_pitch_gain=0.30、min_pitch_max=1.0 —— r_pitch 被错误钳到 0.55，
 * 诱导通道严重过度降载（会直接破坏 300s 诱导回归 -15.8% 的判据）。 */
static void test_config_wiring(void)
{
    FcrInductionSupervisor *s = fcr_induction_supervisor_create(1, NULL);
    const FcrInductionMapConfig *c = fcr_induction_supervisor_cfg(s);
    ASSERT(s != NULL);
    ASSERT(c != NULL);
    ASSERT_NEAR(c->a0, 1.0 / 3.0, 1e-12);
    ASSERT_NEAR(c->a_min, 0.05, 1e-12);
    ASSERT_NEAR(c->a_max, 1.0 / 3.0, 1e-12);
    ASSERT_NEAR(c->ct_coef, 1.0, 1e-12);
    ASSERT_NEAR(c->min_pitch_gain, 0.55, 1e-12);
    ASSERT_NEAR(c->min_pitch_max, 0.30, 1e-12);
    ASSERT_NEAR(c->power_cap, 1.0, 1e-12);   /* 关键：不得为 0.55 */
    fcr_induction_supervisor_destroy(s);
}

static void test_fallback_on_out_of_range(void)
{
    FcrInductionSupervisor *s = fcr_induction_supervisor_create(1, NULL);
    FcrTurbineCommandState tc;
    FcrInductionState st;

    /* NaN 回退：安全默认（rated）并置 fallback */
    make_ind_cmd(&tc, 1, 0.0 / 0.0, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 0.0);
    fcr_induction_supervisor_get(s, 1, &st);
    ASSERT(st.fallback == 1);
    ASSERT_NEAR(st.r_pitch, 1.0, 1e-9);
    ASSERT_NEAR(st.min_pitch_ref_rad, 0.0, 1e-9);

    /* 越界（a=0.8）同样 fallback */
    make_ind_cmd(&tc, 2, 0.8, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 0.1);
    fcr_induction_supervisor_get(s, 1, &st);
    ASSERT(st.fallback == 1);

    /* apply 时 fallback 仍下发（带 FALLBACK flag） */
    {
        FcrRoscoExternalSetpoint sp;
        memset(&sp, 0, sizeof(sp));
        fcr_induction_supervisor_apply(s, 1, &sp);
        ASSERT((sp.command_status_flags & FCR_CMD_FLAG_FALLBACK_ACTIVE) != 0);
    }
    fcr_induction_supervisor_destroy(s);
}

static void test_seq_zoh_and_disable(void)
{
    FcrInductionSupervisor *s = fcr_induction_supervisor_create(1, NULL);
    FcrTurbineCommandState tc;
    FcrInductionState st1, st2;

    make_ind_cmd(&tc, 1, 0.2, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 0.0);
    fcr_induction_supervisor_get(s, 1, &st1);
    ASSERT(st1.seq_applied == 1);

    /* 同 seq（重复帧）：ZOH，不重算 */
    make_ind_cmd(&tc, 1, 0.4, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 0.5);
    fcr_induction_supervisor_get(s, 1, &st2);
    ASSERT(st2.seq_applied == 1);
    ASSERT_NEAR(st2.induction_ref, 0.2, 1e-12);   /* 保持旧值 */

    /* 通道失效：状态清空 */
    tc.ind_valid = 0;
    fcr_induction_supervisor_update(s, 1, &tc, 1.0);
    fcr_induction_supervisor_get(s, 1, &st2);
    ASSERT(st2.valid == 0);
    ASSERT(st2.seq_applied == 0);

    fcr_induction_supervisor_destroy(s);
}

static void test_multi_turbine(void)
{
    FcrInductionSupervisor *s = fcr_induction_supervisor_create(2, NULL);
    FcrTurbineCommandState tc1, tc2;
    FcrInductionState st;
    FcrRoscoExternalSetpoint sp;

    make_ind_cmd(&tc1, 1, 0.20, 10.0);
    make_ind_cmd(&tc2, 1, 0.30, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc1, 0.0);
    fcr_induction_supervisor_update(s, 2, &tc2, 0.0);

    memset(&sp, 0, sizeof(sp));
    fcr_induction_supervisor_apply(s, 1, &sp);
    ASSERT((sp.r_pitch > 0.0) && (sp.r_pitch < 1.0));
    memset(&sp, 0, sizeof(sp));
    fcr_induction_supervisor_apply(s, 2, &sp);
    fcr_induction_supervisor_get(s, 2, &st);
    ASSERT_NEAR(st.induction_ref, 0.30, 1e-12);
    (void)st;
    fcr_induction_supervisor_destroy(s);
}

void test_induction_supervisor_register(void)
{
    RUN_TEST("baseline_mapping", test_baseline_mapping);
    RUN_TEST("derating_mapping", test_derating_mapping);
    RUN_TEST("config_wiring", test_config_wiring);
    RUN_TEST("fallback_on_out_of_range", test_fallback_on_out_of_range);
    RUN_TEST("seq_zoh_and_disable", test_seq_zoh_and_disable);
    RUN_TEST("multi_turbine", test_multi_turbine);
}