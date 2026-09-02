/*
 * test_yaw_action_manager.c — 偏航动作监督测试（Phase 3）
 * 覆盖：0° +5° CW / 0° -5° / ±180° wrap / 重复 seq /
 *       induction 不影响 yaw / 执行中目标固定 / disabled 回退。
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include <string.h>

#define PI 3.14159265358979323846
#define D2R(d) ((d) * PI / 180.0)

static void test_plus5_cw(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = 0.0;

    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION; fr.frame_seq = 1; fr.n_turbines = 1;
    fr.turbines[0].yaw_valid = 1; fr.turbines[0].yaw_seq = 1;
    fr.turbines[0].yaw_delta_rad = D2R(5.0); fr.turbines[0].yaw_ttl_s = 60.0;
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);

    /* 锁存：heading=0 + 5°CW → target=-5°（内部） */
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT(st.valid == 1);
    ASSERT(st.seq == 1);
    ASSERT_NEAR(st.target_heading_rad, D2R(-5.0), 1e-12);
    ASSERT(st.tracking == 1);  /* 0 → -5°，误差 -5° > deadband */

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

static void test_minus5_cw(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = 0.0;

    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION; fr.frame_seq = 1; fr.n_turbines = 1;
    fr.turbines[0].yaw_valid = 1; fr.turbines[0].yaw_seq = 1;
    fr.turbines[0].yaw_delta_rad = D2R(-5.0); fr.turbines[0].yaw_ttl_s = 60.0;
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT_NEAR(st.target_heading_rad, D2R(5.0), 1e-12);
    fcr_yaw_manager_destroy(mgr);
    fcr_command_store_destroy(cs);
}

static void test_wrap_180(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = D2R(170.0);

    /* 机舱 170°，+30° CW → 140°（无 wrap） */
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION; fr.frame_seq = 1; fr.n_turbines = 1;
    fr.turbines[0].yaw_valid = 1; fr.turbines[0].yaw_seq = 1;
    fr.turbines[0].yaw_delta_rad = D2R(30.0); fr.turbines[0].yaw_ttl_s = 60.0;
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT_NEAR(st.target_heading_rad, D2R(140.0), 1e-9);

    /* wrap：机舱 -170°，+30° CW → -200° → wrap → 160° */
    heading = D2R(-170.0);
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION; fr.frame_seq = 2; fr.n_turbines = 1;
    fr.turbines[0].yaw_valid = 1; fr.turbines[0].yaw_seq = 2;
    fr.turbines[0].yaw_delta_rad = D2R(30.0); fr.turbines[0].yaw_ttl_s = 60.0;
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT_NEAR(st.target_heading_rad, D2R(160.0), 1e-9);
    /* 误差应为 wrap：160 - (-170) = 330 → -30° */
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

    /* yaw seq=1 + 20° CW，同时 induction（同帧，ind 通道） */
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION; fr.frame_seq = 1; fr.n_turbines = 1;
    fr.turbines[0].yaw_valid = 1; fr.turbines[0].yaw_seq = 1;
    fr.turbines[0].yaw_delta_rad = D2R(20.0); fr.turbines[0].yaw_ttl_s = 60.0;
    fr.turbines[0].induction_valid = 1; fr.turbines[0].induction_seq = 1;
    fr.turbines[0].induction_ref = 0.2; fr.turbines[0].induction_ttl_s = 1.0;
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);
    fcr_yaw_manager_update(mgr, 1, fcr_command_store_turbine(cs, 1), heading, 0.0);
    fcr_yaw_manager_get(mgr, 1, &st);
    ASSERT_NEAR(st.target_heading_rad, D2R(-10.0), 1e-12); /* 10 - 20 = -10 */

    /* 只发 induction seq=2 —— yaw target 与 seq 均不变 */
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION; fr.frame_seq = 2; fr.n_turbines = 1;
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

    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION; fr.frame_seq = 1; fr.n_turbines = 1;
    fr.turbines[0].yaw_valid = 1; fr.turbines[0].yaw_seq = 1;
    fr.turbines[0].yaw_delta_rad = D2R(10.0); fr.turbines[0].yaw_ttl_s = 60.0;
    fcr_command_store_accept_frame(cs, &fr);
    fcr_command_store_refresh(cs, 0.0, &heading, 1);

    /* 执行中机舱逐步接近，目标不变（seq 相同） */
    for (int step = 1; step <= 10; ++step) {
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

static void test_disabled_rollback(void)
{
    FcrCommandStore *cs = fcr_command_store_create(1, 60.0, 1.0, 5.0, 1.0);
    FcrYawActionManager *mgr = fcr_yaw_manager_create(1, D2R(0.5));
    FarmCommandFrame fr;
    FcrYawActionState st;
    double heading = 0.0;

    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION; fr.frame_seq = 1; fr.n_turbines = 1;
    fr.turbines[0].yaw_valid = 1; fr.turbines[0].yaw_seq = 1;
    fr.turbines[0].yaw_delta_rad = D2R(5.0);
    fr.turbines[0].yaw_ttl_s = 0.5;   /* 短 TTL */
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
    fcr_yaw_manager_destroy(mgr);
    fcr_command_store_destroy(cs);
}

void test_yaw_action_manager_register(void)
{
    RUN_TEST("plus5_cw", test_plus5_cw);
    RUN_TEST("minus5_cw", test_minus5_cw);
    RUN_TEST("wrap_180", test_wrap_180);
    RUN_TEST("induction_does_not_affect_yaw", test_induction_does_not_affect_yaw);
    RUN_TEST("target_fixed_during_execution", test_target_fixed_during_execution);
    RUN_TEST("disabled_rollback", test_disabled_rollback);
}