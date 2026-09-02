/*
 * test_rosco_api.c — rosco_api ABI 全链路测试（Phase 2）
 * 验证：init → register → inject frame → step_high → read_setpoint
 *       以及 publish_state 之后锁存使用最新机舱方位。
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include "fcr_rosco_api.h"
#include "fcr_provider_api.h"
#include <string.h>

extern int fcr_rosco_api_init(uint32_t n_turbines_max, double dt_high_s);
extern int fcr_rosco_api_register_turbine(int32_t turbine_id);
extern int fcr_rosco_api_shutdown(void);
extern int fcr_rosco_step_high(int32_t turbine_id, double sim_time_s);
extern int fcr_rosco_inject_command_frame(const FarmCommandFrame *frame);
extern FcrRuntime *fcr_rosco_api_runtime(void);

static void test_full_abi_chain(void)
{
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;
    FcrRoscoFastState st;

    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    ASSERT(fcr_rosco_api_register_turbine(1) == 0);

    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION;
    fr.frame_seq = 1;
    fr.n_turbines = 1;
    fr.turbines[0].yaw_valid = 1;
    fr.turbines[0].yaw_seq = 1;
    fr.turbines[0].yaw_delta_rad = 5.0 * 3.141592653589793 / 180.0;
    fr.turbines[0].yaw_ttl_s = 60.0;
    fr.turbines[0].source_time_s = 0.0;
    ASSERT(fcr_rosco_inject_command_frame(&fr) == 0);

    memset(&st, 0, sizeof(st));
    st.turbine_id = 1;
    st.valid = 1;
    st.source_time_s = 0.0;
    st.nacelle_heading_rad = 0.0;
    st.gen_power_w = 4.5e6;
    st.rotor_speed_rad_s = 1.2;
    ASSERT(fcr_rosco_publish_state(1, &st) == 0);

    ASSERT(fcr_rosco_step_high(1, 0.01) == 0);
    ASSERT(fcr_rosco_read_setpoint(1, &sp) == 0);
    ASSERT(sp.farm_control_enable == 1);
    ASSERT(sp.yaw_enable == 1);
    ASSERT(sp.yaw_seq == 1);
    ASSERT_NEAR(sp.yaw_target_heading_rad, -5.0 * 3.141592653589793 / 180.0, 1e-9);

    {
        FcrRoscoFastState back;
        FcrRuntime *rt = fcr_rosco_api_runtime();
        ASSERT(rt != NULL);
        memset(&back, 0, sizeof(back));
        ASSERT(fcr_state_store_get_fast(fcr_runtime_state_store(rt), 1, &back) == 0);
        ASSERT(back.valid == 1);
        ASSERT_NEAR(back.gen_power_w, 4.5e6, 1.0);
    }

    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION;
    fr.frame_seq = 2;
    fr.n_turbines = 1;
    fr.turbines[0].yaw_valid = 1;
    fr.turbines[0].yaw_seq = 1;   /* 重复 seq：幂等 */
    fr.turbines[0].yaw_delta_rad = 0.5;
    fr.turbines[0].source_time_s = 0.0;
    ASSERT(fcr_rosco_inject_command_frame(&fr) == 0);
    ASSERT(fcr_rosco_step_high(1, 0.02) == 0);
    ASSERT(fcr_rosco_read_setpoint(1, &sp) == 0);
    ASSERT_NEAR(sp.yaw_target_heading_rad, -5.0 * 3.141592653589793 / 180.0, 1e-9);

    fcr_rosco_api_shutdown();
    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    fcr_rosco_api_shutdown();
}

static void test_api_init_idempotent(void)
{
    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    fcr_rosco_api_shutdown();
}

void test_rosco_api_register(void)
{
    RUN_TEST("full_abi_chain", test_full_abi_chain);
    RUN_TEST("api_init_idempotent", test_api_init_idempotent);
}