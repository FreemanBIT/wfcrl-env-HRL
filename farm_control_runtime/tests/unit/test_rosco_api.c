/*
 * test_rosco_api.c — rosco_api ABI 全链路测试（ICD v1.5）
 *
 * 验证：init → register → inject frame → step_high → read_setpoint
 *       以及 publish_state 之后锁存使用最新机舱方位。
 *
 * ICD v1.5 变更：
 *   - FcrRoscoFastState → FcrRoscoRawState
 *   - FarmCommandFrame.frame_seq → command_frame_seq
 *   - yaw_delta_rad（增量）→ yaw_target_heading_rad（绝对目标，不再取反）
 *   - source_time_s → command_basis_sim_time_s
 *   - FcrRoscoExternalSetpoint.yaw_enable → external_yaw_enable
 *   - 新增 control_enable（不置 1 会被 S0104 回退分支清空通道）
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

#define DEG2RAD (3.141592653589793 / 180.0)

static void test_full_abi_chain(void)
{
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;
    FcrRoscoRawState st;

    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    ASSERT(fcr_rosco_api_register_turbine(1) == 0);

    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION;
    fr.command_frame_seq = 1;                      /* S0101 */
    fr.n_turbines = 1;
    fr.turbines[0].control_enable = 1;             /* S0104: 必须置 1 */
    fr.turbines[0].yaw_valid = 1;                  /* S0110 */
    fr.turbines[0].yaw_seq = 1;                    /* S0109 */
    fr.turbines[0].yaw_target_heading_rad = -5.0 * DEG2RAD;  /* S0108: 绝对目标 */
    fr.turbines[0].yaw_ttl_s = 60.0;
    fr.turbines[0].command_basis_sim_time_s = 0.0; /* S0102 */
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
    ASSERT(sp.valid == 1);
    ASSERT(sp.farm_control_enable == 1);
    ASSERT(sp.external_yaw_enable == 1);           /* S0801 */
    ASSERT(sp.yaw_seq == 1);
    /* ICD v1.5: 绝对目标原样下发，不做 delta→heading 取反 */
    ASSERT_NEAR(sp.yaw_target_heading_rad, -5.0 * DEG2RAD, 1e-9);

    {
        FcrRoscoRawState back;
        FcrRuntime *rt = fcr_rosco_api_runtime();
        ASSERT(rt != NULL);
        memset(&back, 0, sizeof(back));
        ASSERT(fcr_state_store_get_fast(fcr_runtime_state_store(rt), 1, &back) == 0);
        ASSERT(back.valid == 1);
        ASSERT_NEAR(back.gen_power_w, 4.5e6, 1.0);
    }

    /* 重复 seq：幂等，目标不变（新帧携带不同目标也不得生效） */
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION;
    fr.command_frame_seq = 2;
    fr.n_turbines = 1;
    fr.turbines[0].control_enable = 1;
    fr.turbines[0].yaw_valid = 1;
    fr.turbines[0].yaw_seq = 1;                    /* 重复 seq */
    fr.turbines[0].yaw_target_heading_rad = 0.5;   /* 不应生效 */
    fr.turbines[0].command_basis_sim_time_s = 0.0;
    ASSERT(fcr_rosco_inject_command_frame(&fr) == 0);
    ASSERT(fcr_rosco_step_high(1, 0.02) == 0);
    ASSERT(fcr_rosco_read_setpoint(1, &sp) == 0);
    ASSERT_NEAR(sp.yaw_target_heading_rad, -5.0 * DEG2RAD, 1e-9);

    fcr_rosco_api_shutdown();
    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    fcr_rosco_api_shutdown();
}

/* ICD v1.5 S0901/S0902: seq_applied 必须回读（历史 bug：恒为 0） */
static void test_ack_roundtrip(void)
{
    FcrRoscoAck ack_in, ack_out;

    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    ASSERT(fcr_rosco_api_register_turbine(1) == 0);

    memset(&ack_in, 0, sizeof(ack_in));
    ack_in.induction_seq_applied = 7;
    ack_in.yaw_seq_applied = 3;
    ack_in.bridge_status_flags = FCR_BRIDGE_STATUS_OK | FCR_BRIDGE_STATUS_CMD_ACTIVE;
    ack_in.ct_ref = 0.42;
    ASSERT(fcr_rosco_publish_ack(1, &ack_in) == 0);

    memset(&ack_out, 0, sizeof(ack_out));
    ASSERT(fcr_rosco_get_ack(1, &ack_out) == 0);
    ASSERT(ack_out.induction_seq_applied == 7);
    ASSERT(ack_out.yaw_seq_applied == 3);
    ASSERT(ack_out.bridge_status_flags ==
           (FCR_BRIDGE_STATUS_OK | FCR_BRIDGE_STATUS_CMD_ACTIVE));
    ASSERT_NEAR(ack_out.ct_ref, 0.42, 1e-12);

    fcr_rosco_api_shutdown();
}

/* ICD v1.5 S0104: control_enable=0 时必须回退 ROSCO（通道清空 + 标志置位） */
static void test_control_disable_fallback(void)
{
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;
    FcrRoscoRawState st;

    ASSERT(fcr_rosco_api_init(64, 0.01) == 0);
    ASSERT(fcr_rosco_api_register_turbine(1) == 0);

    memset(&st, 0, sizeof(st));
    st.turbine_id = 1;
    st.valid = 1;
    st.nacelle_heading_rad = 0.0;
    ASSERT(fcr_rosco_publish_state(1, &st) == 0);

    /* 先下发使能命令并生效 */
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION;
    fr.command_frame_seq = 1;
    fr.n_turbines = 1;
    fr.turbines[0].control_enable = 1;
    fr.turbines[0].yaw_valid = 1;
    fr.turbines[0].yaw_seq = 1;
    fr.turbines[0].yaw_target_heading_rad = 0.1;
    fr.turbines[0].induction_valid = 1;
    fr.turbines[0].induction_seq = 1;
    fr.turbines[0].induction_ref = 0.2;
    fr.turbines[0].command_basis_sim_time_s = 0.0;
    ASSERT(fcr_rosco_inject_command_frame(&fr) == 0);
    ASSERT(fcr_rosco_step_high(1, 0.01) == 0);
    ASSERT(fcr_rosco_read_setpoint(1, &sp) == 0);
    ASSERT(sp.external_yaw_enable == 1);
    ASSERT(sp.induction_enable == 1);

    /* control_enable=0 → 全通道回退 */
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION;
    fr.command_frame_seq = 2;
    fr.n_turbines = 1;
    fr.turbines[0].control_enable = 0;             /* S0104 = 0 */
    fr.turbines[0].yaw_valid = 1;
    fr.turbines[0].yaw_seq = 2;
    fr.turbines[0].yaw_target_heading_rad = 0.3;
    fr.turbines[0].induction_valid = 1;
    fr.turbines[0].induction_seq = 2;
    fr.turbines[0].induction_ref = 0.3;
    fr.turbines[0].command_basis_sim_time_s = 0.0;
    ASSERT(fcr_rosco_inject_command_frame(&fr) == 0);
    ASSERT(fcr_rosco_step_high(1, 0.02) == 0);
    ASSERT(fcr_rosco_read_setpoint(1, &sp) == 0);

    ASSERT(sp.farm_control_enable == 0);
    ASSERT(sp.external_yaw_enable == 0);
    ASSERT(sp.induction_enable == 0);
    ASSERT((sp.command_status_flags & FCR_CMD_FLAG_CONTROL_DISABLED) != 0);
    ASSERT((sp.command_status_flags & FCR_CMD_FLAG_YAW_VALID) == 0);
    ASSERT((sp.command_status_flags & FCR_CMD_FLAG_INDUCTION_VALID) == 0);

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
    RUN_TEST("ack_roundtrip", test_ack_roundtrip);
    RUN_TEST("control_disable_fallback", test_control_disable_fallback);
    RUN_TEST("api_init_idempotent", test_api_init_idempotent);
}
