/*
 * test_yaw_action_manager.c — 偏航动作监督测试（ICD v1.5）
 *
 * ICD v1.5 语义变更（重要）：
 *   - 命令侧**只有绝对目标** `yaw_target_heading_rad`(S0108)，`yaw_delta_rad` 已删除。
 *     旧测试的 "+5° CW" 现在等价于直接下发绝对目标 -5°（CCW+ 约定）。
 *   - CommandStore 不再做 delta→heading 计算与 wrap；目标原样锁存。
 *   - 所有命令必须带 `control_enable = 1`(S0104)，否则 refresh 会走回退分支清空通道。
 *
 * 覆盖：绝对目标锁存 / 误差 wrap / 重复 seq / induction 不影响 yaw /
 *       执行中目标固定 / TTL 到期回退。
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include <string.h>

#define PI 3.14159265358979323846
#define D2R(d) ((d) * PI / 180.0)

/* 构造单机命令帧（绝对偏航目标 + 可选诱导） */
static void mk_frame(FarmCommandFrame *fr, uint64_t frame_seq, uint64_t yaw_seq,
                     double yaw_target_rad, double yaw_ttl_s)
{
    memset(fr, 0, sizeof(*fr));
    fr->protocol_version = FCR_PROTOCOL_VERSION;
    fr->command_frame_seq = frame_seq;      /* S0101 */
    fr->n_turbines = 1;
    fr->turbines[0].control_enable = 1;     /* S0104 */
    fr->turbines[0].yaw_valid = 1;          /* S0110 */
    fr->turbines[0].yaw_seq = yaw_seq;      /* S0109 */
    fr->turbines[0].yaw_target_heading_rad = yaw_target_rad; /* S0108 绝对目标 */
    fr->turbines[0].yaw_ttl_s = yaw_ttl_s;
}

static void test_absolute_target_negative(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = 0.0;

    /* 绝对目标 -5°（等价于旧协议的 "+5° CW"） */
    mk_frame(&fr, 1, 1, D2R(-5.0), 60.0);
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);

    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT(st.valid == 1);
    ASSERT(st.seq == 1);
    ASSERT_NEAR(st.target_heading_rad, D2R(-5.0), 1e-12);   /* 原样锁存，不取反 */
    ASSERT(st.tracking == 1);                                /* 误差 -5° > deadband */

    /* 机舱转到 -5° → reached */
    heading = D2R(-5.0);
    fcr_command_store_refresh(cs, 1.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 1.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT(st.reached == 1);
    ASSERT(st.tracking == 0);
    ASSERT_NEAR(st.error_rad, 0.0, 1e-12);

    fcr_yaw_manager_destroy(mgr);
    fcr_command_store_destroy(cs);
}

static void test_absolute_target_positive(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = 0.0;

    mk_frame(&fr, 1, 1, D2R(5.0), 60.0);
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT_NEAR(st.target_heading_rad, D2R(5.0), 1e-12);
    ASSERT(st.tracking == 1);

    fcr_yaw_manager_destroy(mgr);
    fcr_command_store_destroy(cs);
}

/* 误差 wrap：目标 160°，机舱 -170° → 误差 330° → wrap 到 -30° */
static void test_error_wrap_180(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = D2R(170.0);

    /* 绝对目标 140°，机舱 170° → 误差 -30°（无 wrap 跨越） */
    mk_frame(&fr, 1, 1, D2R(140.0), 60.0);
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT_NEAR(st.target_heading_rad, D2R(140.0), 1e-9);
    ASSERT_NEAR(st.error_rad, D2R(-30.0), 1e-9);

    /* wrap：绝对目标 160°，机舱 -170° → 160-(-170)=330 → wrap 到 -30° */
    heading = D2R(-170.0);
    mk_frame(&fr, 2, 2, D2R(160.0), 60.0);
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT_NEAR(st.target_heading_rad, D2R(160.0), 1e-9);
    ASSERT_NEAR(st.error_rad, D2R(-30.0), 1e-9);

    fcr_yaw_manager_destroy(mgr);
    fcr_command_store_destroy(cs);
}

static void test_induction_does_not_affect_yaw(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = D2R(10.0);

    /* 绝对目标 -10° + 同帧 induction（独立通道） */
    mk_frame(&fr, 1, 1, D2R(-10.0), 60.0);
    fr.turbines[0].induction_valid = 1;
    fr.turbines[0].induction_seq = 1;
    fr.turbines[0].induction_ref = 0.2;
    fr.turbines[0].induction_ttl_s = 1.0;
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT_NEAR(st.target_heading_rad, D2R(-10.0), 1e-12);

    /* 只发 induction seq=2 —— yaw target 与 seq 均不变 */
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION;
    fr.command_frame_seq = 2; fr.n_turbines = 1;
    fr.turbines[0].control_enable = 1;
    fr.turbines[0].induction_valid = 1; fr.turbines[0].induction_seq = 2;
    fr.turbines[0].induction_ref = 0.3; fr.turbines[0].induction_ttl_s = 1.0;
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.5, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.5);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT(st.seq == 1);
    ASSERT_NEAR(st.target_heading_rad, D2R(-10.0), 1e-12);

    fcr_yaw_manager_destroy(mgr);
    fcr_command_store_destroy(cs);
}

static void test_target_fixed_during_execution(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = 0.0;
    int step;

    mk_frame(&fr, 1, 1, D2R(-10.0), 60.0);
    fcr_command_store_accept_frame(cs, &fr);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);

    /* 执行中机舱逐步接近，目标不变（seq 相同） */
    for (step = 1; step <= 10; ++step) {
        heading = D2R(-1.0 * step);   /* 每次 -1° */
        fcr_command_store_refresh(cs, step * 0.1, &heading, 1);
        fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, step * 0.1);
        fcr_yaw_manager_get(mgr, 1, &st);
        ASSERT_NEAR(st.target_heading_rad, D2R(-10.0), 1e-12);
        ASSERT(st.seq == 1);
    }
    /* 最终到达 -10° → reached */
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT(st.reached == 1);

    fcr_yaw_manager_destroy(mgr);
    fcr_command_store_destroy(cs);
}

static void test_ttl_expiry_rollback(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = 0.0;

    mk_frame(&fr, 1, 1, D2R(5.0), 0.5);   /* 短 TTL */
    fcr_command_store_accept_frame(cs, &fr);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT(st.valid == 1);

    /* TTL 到期（0.6s > 0.5s）→ 通道失效 → 监督状态清空（回退原 ROSCO） */
    fcr_command_store_refresh(cs, 0.6, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.6);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT(st.valid == 0);
    /* TTL 到期标志置位（S0108-S0110 回退语义） */
    ASSERT((fcr_command_store_turbine(cs, 1)->command_status_flags &
            FCR_CMD_FLAG_YAW_TTL_EXPIRED) != 0);

    fcr_yaw_manager_destroy(mgr);
    fcr_command_store_destroy(cs);
}

void test_yaw_action_manager_register(void)
{
    RUN_TEST("absolute_target_negative", test_absolute_target_negative);
    RUN_TEST("absolute_target_positive", test_absolute_target_positive);
    RUN_TEST("error_wrap_180", test_error_wrap_180);
    RUN_TEST("induction_does_not_affect_yaw", test_induction_does_not_affect_yaw);
    RUN_TEST("target_fixed_during_execution", test_target_fixed_during_execution);
    RUN_TEST("ttl_expiry_rollback", test_ttl_expiry_rollback);
}
