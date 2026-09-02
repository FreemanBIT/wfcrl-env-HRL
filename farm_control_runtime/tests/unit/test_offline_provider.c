/*
 * test_offline_provider.c — Offline Provider 快速量发布测试（Phase 4）
 * 验证：avrSWAP record → FcrTurbineExtraState 映射正确；
 *       clock（fast_step/low_step）发布正确；tower_base 不可得语义。
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include <string.h>

extern int fcr_rosco_api_init(uint32_t n_turbines_max, double dt_high_s);
extern int fcr_rosco_api_register_turbine(int32_t turbine_id);
extern int fcr_rosco_api_shutdown(void);
extern FcrRuntime *fcr_rosco_api_runtime(void);
extern int fcr_offline_provider_init(void);
extern int fcr_offline_provider_publish_fast(int32_t turbine_id, double sim_time_s,
                                             const float *avrSWAP);

static void test_provider_extra_mapping(void)
{
    float swap[200];
    FcrRuntime *rt;
    FcrTurbineExtraState st;
    int i;

    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    ASSERT(fcr_offline_provider_init() == 0);
    ASSERT(fcr_rosco_api_register_turbine(1) == 0);
    rt = fcr_rosco_api_runtime();
    ASSERT(rt != NULL);

    for (i = 0; i < 200; ++i) swap[i] = 0.0f;
    swap[109 - 1] = 3.0e6f;    /* shaft torque Nm */
    swap[110 - 1] = 5.5e5f;    /* rotor thrust N (GL x) */
    swap[111 - 1] = 1.2e4f;    /* LSShftFys */
    swap[112 - 1] = -3.0e3f;   /* LSShftFzs */
    swap[77 - 1]  = 9.0e6f;    /* YawBrMyn Nm */
    swap[78 - 1]  = 1.5e5f;    /* YawBrMzn Nm */

    ASSERT(fcr_offline_provider_publish_fast(1, 2.5, swap) == 0);

    memset(&st, 0, sizeof(st));
    ASSERT(fcr_state_store_get_extra(rt->ss, 1, &st) == 0);
    ASSERT(st.valid == 1);
    ASSERT_NEAR(st.shaft_torque_nm, 3.0e6, 1.0);
    ASSERT_NEAR(st.rotor_thrust_n, 5.5e5, 1.0);
    ASSERT_NEAR(st.load_point_f[0][0], 5.5e5, 1.0);   /* Fx = thrust */
    ASSERT_NEAR(st.load_point_f[1][0], 1.2e4, 1.0);
    ASSERT_NEAR(st.load_point_f[2][0], -3.0e3, 1.0);
    ASSERT_NEAR(st.load_point_f[4][1], 9.0e6, 1.0);   /* yaw bearing My */
    ASSERT_NEAR(st.load_point_f[5][1], 1.5e5, 1.0);   /* yaw bearing Mz */
    /* tower base 不可得：NaN 语义 */
    ASSERT(!fcr_is_finite(st.tower_base_fa_moment_nm));
    ASSERT(!fcr_is_finite(st.tower_base_ss_moment_nm));

    /* clock 发布检查：fast_step=250 @ 2.5s/0.01s; low_step=2 @ 2.5s/1.0s */
    ASSERT(rt->ss->fast_step == 250);
    ASSERT(rt->ss->low_step == 2);
    ASSERT_NEAR(rt->ss->sim_time_s, 2.5, 1e-9);

    fcr_rosco_api_shutdown();
}

static void test_provider_turbine_routing(void)
{
    FcrRuntime *rt;
    float swap[200];
    FcrTurbineExtraState st;
    int i;
    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    ASSERT(fcr_rosco_api_register_turbine(1) == 0);
    ASSERT(fcr_rosco_api_register_turbine(2) == 0);
    rt = fcr_rosco_api_runtime();
    for (i = 0; i < 200; ++i) swap[i] = 0.0f;
    swap[110 - 1] = 4.0e5f;
    swap[109 - 1] = 2.0e6f;

    /* T1: thrust 4e5/torque 2e6；T2 不发布 -> valid=0 */
    ASSERT(fcr_offline_provider_publish_fast(1, 1.0, swap) == 0);
    memset(&st, 0, sizeof(st));
    ASSERT(fcr_state_store_get_extra(rt->ss, 1, &st) == 0);
    ASSERT(st.valid == 1);
    ASSERT_NEAR(st.rotor_thrust_n, 4.0e5, 1.0);
    memset(&st, 0, sizeof(st));
    ASSERT(fcr_state_store_get_extra(rt->ss, 2, &st) == 0);
    ASSERT(st.valid == 0);
    fcr_rosco_api_shutdown();
}

void test_offline_provider_register(void)
{
    RUN_TEST("provider_extra_mapping", test_provider_extra_mapping);
    RUN_TEST("provider_turbine_routing", test_provider_turbine_routing);
}