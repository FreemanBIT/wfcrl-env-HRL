/*
 * test_bridge_adapter.c — Bridge runtime adapter 单元测试（DR-3/DR-4 接入）
 * 覆盖：命令状态→bridge 命令转换、fast 状态→bridge 测量转换、
 *       bridge core step 合并 setpoint、bridge_enable 分支。
 */
#include "test_framework.h"
#include "bridge_runtime_adapter.h"
#include <string.h>
#include <math.h>

/* ---- 命令状态 → bridge 命令 ---- */
static void test_cmd_from_state(void) {
    FcrTurbineCommandState tc;
    FcrBridgeCmd cmd;
    memset(&tc, 0, sizeof(tc));
    tc.ind_valid = 1;
    tc.induction_ref = 0.20;
    tc.ind_seq_rx = 7;
    tc.ind_ttl_s = 5.0;
    tc.command_basis_sim_time_s = 12.0;   /* ICD v1.5 S0102 */
    fcr_bridge_adapter_cmd_from_state(&tc, 1, 400, 300, 300, &cmd);
    ASSERT(cmd.induction_valid == 1);
    ASSERT_NEAR(cmd.induction_ref, 0.20, 1e-9);
    ASSERT(cmd.induction_seq == 7);
    ASSERT_NEAR(cmd.induction_ttl_s, 5.0, 1e-9);
    ASSERT(cmd.induction_mode == 1);
    /* 千分比 → 归一化 */
    ASSERT_NEAR(cmd.alloc_w_pitch, 0.4, 1e-9);
    ASSERT_NEAR(cmd.alloc_w_speed, 0.3, 1e-9);
    ASSERT_NEAR(cmd.alloc_w_torque, 0.3, 1e-9);
    /* 无效命令 → induction_valid=0 */
    memset(&tc, 0, sizeof(tc));
    fcr_bridge_adapter_cmd_from_state(&tc, 1, 0, 0, 0, &cmd);
    ASSERT(cmd.induction_valid == 0);
}

/* ---- fast 状态 → bridge 测量 ---- */
static void test_meas_from_fast(void) {
    FcrRoscoRawState fast;
    FcrBridgeMeas meas;
    memset(&fast, 0, sizeof(fast));
    fast.valid = 1;
    fast.rotor_speed_rad_s = 1.2;
    fast.gen_speed_rad_s = 120.0;
    fast.gen_power_w = 2e6;
    fast.nacelle_heading_rad = 0.5;
    fast.hub_wind_speed_mps = 8.0;
    fast.blade_pitch_rad[0] = 0.1; fast.blade_pitch_rad[1] = 0.1; fast.blade_pitch_rad[2] = 0.1;
    fcr_bridge_adapter_meas_from_fast(&fast, &meas);
    ASSERT(meas.valid == 1);
    ASSERT_NEAR(meas.rotor_speed_rad_s, 1.2, 1e-9);
    ASSERT_NEAR(meas.gen_power_w, 2e6, 1e-6);
    ASSERT_NEAR(meas.ambient_upstream_wind_mps, 8.0, 1e-9);
    ASSERT_NEAR(meas.blade_pitch_rad[1], 0.1, 1e-9);
}

/* ---- bridge core step 合并 setpoint（诱导部分覆盖，yaw 保留）---- */
static void test_adapter_apply(void) {
    FcrBridgeCore *core = fcr_bridge_core_create(1, NULL, NULL, NULL);
    FcrBridgeCmd cmd;
    FcrBridgeMeas meas;
    FcrRoscoExternalSetpoint base, merged;
    ASSERT(core != NULL);
    /* 基础 setpoint：yaw 由 CommandStore 生成 */
    memset(&base, 0, sizeof(base));
    base.valid = 1;
    base.farm_control_enable = 1;
    base.external_yaw_enable = 1;              /* ICD v1.5 S0801 */
    base.yaw_seq = 3;
    base.yaw_target_heading_rad = 0.5;
    base.r_pitch = 1.0;                        /* ICD v1.5 S0803 */
    /* 诱导命令 a=0.2（Mode 1） */
    memset(&cmd, 0, sizeof(cmd));
    cmd.turbine_id = 1; cmd.valid = 1;
    cmd.induction_mode = 1; cmd.induction_valid = 1;
    cmd.induction_ref = 0.2; cmd.induction_seq = 1;
    cmd.induction_ttl_s = 1e6; cmd.source_time_s = 0.0;
    /* 测量 */
    memset(&meas, 0, sizeof(meas));
    meas.turbine_id = 1; meas.valid = 1;
    meas.rotor_speed_rad_s = 1.2; meas.gen_power_w = 2e6;
    meas.ambient_upstream_wind_mps = 8.0; meas.disk_wind_speed_mps = 8.0;
    meas.rotor_thrust_n = 4e5;
    merged = base;
    int rc = fcr_bridge_adapter_apply(core, 1, 10.0, &cmd, &meas, &merged);
    ASSERT(rc == 0);
    /* yaw 保留 */
    ASSERT(merged.external_yaw_enable == 1);
    ASSERT(merged.yaw_seq == 3);
    ASSERT_NEAR(merged.yaw_target_heading_rad, 0.5, 1e-9);
    /* 诱导被 bridge 覆盖（a=0.2 → r_pitch < 1） */
    ASSERT(merged.induction_enable == 1);
    ASSERT(merged.r_pitch < 1.0);
    ASSERT_NEAR(merged.r_pitch, fcr_bridge_mode1_power_ratio(0.2, 1.0/3.0), 1e-9);
    fcr_bridge_core_destroy(core);
}

/* ---- 空指针安全 ---- */
static void test_adapter_null(void) {
    ASSERT(fcr_bridge_adapter_apply(NULL, 1, 0.0, NULL, NULL, NULL) == -1);
}

void test_bridge_adapter_register(void)
{
    RUN_TEST("cmd_from_state", test_cmd_from_state);
    RUN_TEST("meas_from_fast", test_meas_from_fast);
    RUN_TEST("adapter_apply", test_adapter_apply);
    RUN_TEST("adapter_null", test_adapter_null);
}
