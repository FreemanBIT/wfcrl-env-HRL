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
    ASSERT_NEAR(st.power_ratio, 1.0, 1e-6);
    ASSERT_NEAR(st.speed_ref_ratio, 1.0, 1e-6);
    ASSERT_NEAR(st.torque_limit_ratio, 1.0, 1e-6);
    ASSERT_NEAR(st.min_pitch_rad, 0.0, 1e-9);

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

    /* a = 0.2（降载）：power_ratio < 1, min_pitch > 0 */
    make_ind_cmd(&tc, 1, 0.2, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 0.0);
    fcr_induction_supervisor_get(s, 1, &st);
    ASSERT(st.valid == 1);
    ASSERT(st.power_ratio < 1.0 && st.power_ratio > 0.0);
    ASSERT(st.ct_ref < 8.0 / 9.0);
    /* P ∝ ω^3 → speed_ratio = pr^(1/3) > pr；T = P/ω → torque_ratio = pr^(2/3) */
    ASSERT(st.speed_ref_ratio > st.power_ratio);
    ASSERT(st.torque_limit_ratio > st.power_ratio);
    ASSERT(st.speed_ref_ratio > st.torque_limit_ratio);
    ASSERT(st.min_pitch_rad > 0.0);

    /* a = 0.45（边界）：ct = 0.99 封顶？ 4*0.45*0.55=0.99 */
    make_ind_cmd(&tc, 2, 0.45, 10.0);
    fcr_induction_supervisor_update(s, 1, &tc, 0.5);
    fcr_induction_supervisor_get(s, 1, &st);
    ASSERT(st.valid == 1);
    ASSERT_NEAR(st.ct_ref, 4.0 * 0.45 * 0.55, 1e-9);

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
    ASSERT_NEAR(st.power_ratio, 1.0, 1e-9);
    ASSERT_NEAR(st.min_pitch_rad, 0.0, 1e-9);

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
    ASSERT((sp.power_ratio > 0.0) && (sp.power_ratio < 1.0));
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
    RUN_TEST("fallback_on_out_of_range", test_fallback_on_out_of_range);
    RUN_TEST("seq_zoh_and_disable", test_seq_zoh_and_disable);
    RUN_TEST("multi_turbine", test_multi_turbine);
}