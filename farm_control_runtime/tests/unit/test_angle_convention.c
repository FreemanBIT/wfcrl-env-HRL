/*
 * test_angle_convention.c — wrap / 单位 / CW+↔CCW+ 转换测试（Phase 1）
 */
#include "test_framework.h"
#include "fcr_internal.h"

#define PI 3.14159265358979323846

static void test_wrap_pm_pi(void)
{
    ASSERT_NEAR(fcr_wrap_pm_pi(0.0), 0.0, 1e-12);
    ASSERT_NEAR(fcr_wrap_pm_pi(PI), PI, 1e-12);
    ASSERT_NEAR(fcr_wrap_pm_pi(-PI + 1e-9), -PI + 1e-9, 1e-12);
    ASSERT_NEAR(fcr_wrap_pm_pi(3.0 * PI), PI, 1e-12);
    ASSERT_NEAR(fcr_wrap_pm_pi(-3.0 * PI), PI, 1e-12);  /* -3π mod 2π → +π */
    ASSERT_NEAR(fcr_wrap_pm_pi(2.0 * PI + 0.5), 0.5, 1e-12);
    ASSERT_NEAR(fcr_wrap_pm_pi(-0.5), -0.5, 1e-12);
    ASSERT_NEAR(fcr_wrap_pm_pi(100.0 * PI + 0.3), 0.3, 1e-9);
}

static void test_wrap_0_2pi(void)
{
    ASSERT_NEAR(fcr_wrap_0_2pi(0.0), 0.0, 1e-12);
    ASSERT_NEAR(fcr_wrap_0_2pi(2.0 * PI), 0.0, 1e-12);
    ASSERT_NEAR(fcr_wrap_0_2pi(-0.1), 2.0 * PI - 0.1, 1e-12);
    ASSERT_NEAR(fcr_wrap_0_2pi(4.0 * PI + 0.7), 0.7, 1e-12);
}

static void test_deg_rad(void)
{
    ASSERT_NEAR(fcr_deg2rad(180.0), PI, 1e-12);
    ASSERT_NEAR(fcr_rad2deg(PI), 180.0, 1e-12);
    ASSERT_NEAR(fcr_deg2rad(-90.0), -PI / 2, 1e-12);
}

static void test_cw_to_internal_sign(void)
{
    /* CW+ 增量 → 内部 CCW+ 增量取反 */
    ASSERT_NEAR(fcr_cw_delta_to_internal(0.5), -0.5, 1e-12);
    ASSERT_NEAR(fcr_cw_delta_to_internal(-0.5), 0.5, 1e-12);
}

static void test_yaw_target_latch(void)
{
    /* 0 rad 机舱 +5° CW → 目标 -5°（内部） */
    double t1 = fcr_compute_yaw_target(0.0, fcr_deg2rad(5.0));
    ASSERT_NEAR(t1, fcr_deg2rad(-5.0), 1e-12);

    /* 0 rad 机舱 -5° CW（即 CCW 5°）→ 目标 +5° */
    double t2 = fcr_compute_yaw_target(0.0, fcr_deg2rad(-5.0));
    ASSERT_NEAR(t2, fcr_deg2rad(5.0), 1e-12);

    /* ±180° wrap：机舱 170°，+20° CW → 170-20=150° 无 wrap */
    double t3 = fcr_compute_yaw_target(fcr_deg2rad(170.0), fcr_deg2rad(20.0));
    ASSERT_NEAR(t3, fcr_deg2rad(150.0), 1e-12);

    /* 机舱 170°，+30° CW → 170-30=140° */
    double t4 = fcr_compute_yaw_target(fcr_deg2rad(170.0), fcr_deg2rad(30.0));
    ASSERT_NEAR(t4, fcr_deg2rad(140.0), 1e-12);

    /* 机舱 175°，+20° CW → 175-20=155° */
    double t5 = fcr_compute_yaw_target(fcr_deg2rad(175.0), fcr_deg2rad(20.0));
    ASSERT_NEAR(t5, fcr_deg2rad(155.0), 1e-12);

    /* wrap across ±180：机舱 175°，+30° CW → 145°？错了有 wrap 场景：
     * 机舱 170°，+40° CW → 170-40=130° */
    double t6 = fcr_compute_yaw_target(fcr_deg2rad(170.0), fcr_deg2rad(40.0));
    ASSERT_NEAR(t6, fcr_deg2rad(130.0), 1e-12);

    /* wrap 场景：机舱 -170°（=190°），+30° CW → -170-30=-200 → wrap → 160° */
    double t7 = fcr_compute_yaw_target(fcr_deg2rad(-170.0), fcr_deg2rad(30.0));
    ASSERT_NEAR(t7, fcr_deg2rad(160.0), 1e-12);

    /* wrap 场景：机舱 170°，-30° CW（CCW 30°）→ 170+30=200 → wrap → -160° */
    double t8 = fcr_compute_yaw_target(fcr_deg2rad(170.0), fcr_deg2rad(-30.0));
    ASSERT_NEAR(t8, fcr_deg2rad(-160.0), 1e-12);
}

static void test_finite(void)
{
    ASSERT(fcr_is_finite(1.0) == 1);
    ASSERT(fcr_is_finite(0.0 / 0.0) == 0);   /* NaN */
    ASSERT(fcr_is_finite(1.0 / 0.0) == 0);   /* +Inf */
    ASSERT(fcr_is_finite(-1.0 / 0.0) == 0);  /* -Inf */
}

void test_angle_convention_register(void)
{
    RUN_TEST("wrap_pm_pi", test_wrap_pm_pi);
    RUN_TEST("wrap_0_2pi", test_wrap_0_2pi);
    RUN_TEST("deg_rad", test_deg_rad);
    RUN_TEST("cw_to_internal_sign", test_cw_to_internal_sign);
    RUN_TEST("yaw_target_latch", test_yaw_target_latch);
    RUN_TEST("finite", test_finite);
}
