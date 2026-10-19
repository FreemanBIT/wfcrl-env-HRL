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

/* ---- S07: runtime 命令状态 → FcrBridgeInput（S0701-S0710）---- */
static void test_input_from_state(void) {
    FcrTurbineCommandState tc;
    FcrBridgeInput in;
    memset(&tc, 0, sizeof(tc));
    tc.control_enable = 1;
    tc.control_mode = FCR_CONTROL_MODE_PRC;
    tc.command_basis_sim_time_s = 12.5;
    tc.yaw_valid = 1; tc.yaw_seq_rx = 3;
    tc.yaw_target_heading_rad = -0.2; tc.yaw_ttl_s = 60.0;
    tc.ind_valid = 1; tc.ind_seq_rx = 7; tc.induction_ref = 0.2; tc.ind_ttl_s = 5.0;

    fcr_bridge_adapter_input_from_state(&tc, 2, 30.0, &in);
    ASSERT_NEAR(in.bridge_step_sim_time_s, 30.0, 1e-12);      /* S0701 */
    ASSERT(in.turbine_id == 2);                                /* S0703 */
    ASSERT(in.control_enable == 1);                            /* S0704 */
    ASSERT(in.control_mode == FCR_CONTROL_MODE_PRC);           /* S0709 */
    ASSERT_NEAR(in.command_basis_sim_time_s, 12.5, 1e-12);     /* S0710 */
    ASSERT_NEAR(in.induction_ref, 0.2, 1e-12);                 /* S0705 */
    ASSERT(in.induction_seq == 7);                             /* S0706 */
    ASSERT_NEAR(in.yaw_target_heading_rad, -0.2, 1e-12);      /* S0707 */
    ASSERT(in.yaw_seq == 3);                                   /* S0708 */
    ASSERT((in.bridge_command_valid_flags & FCR_BRIDGE_CMD_VALID) != 0);
    ASSERT((in.bridge_command_valid_flags & FCR_BRIDGE_CMD_YAW_VALID) != 0);
    ASSERT((in.bridge_command_valid_flags & FCR_BRIDGE_CMD_INDUCTION_VALID) != 0);
    ASSERT((in.bridge_command_valid_flags & FCR_BRIDGE_CMD_CONTROL_ENABLED) != 0);

    /* S0704=0：回退 ROSCO，不得带任何通道有效标记 */
    tc.control_enable = 0;
    fcr_bridge_adapter_input_from_state(&tc, 2, 30.0, &in);
    ASSERT(in.control_enable == 0);
    ASSERT(in.bridge_command_valid_flags == 0);
    ASSERT(in.yaw_seq == 0 && in.induction_seq == 0);
    ASSERT_NEAR(in.yaw_target_heading_rad, 0.0, 1e-12);
}

/* ---- S07: FcrBridgeInput → FcrBridgeCmd ---- */
static void test_cmd_from_input(void) {
    FcrBridgeInput in;
    FcrBridgeCmd cmd;
    memset(&in, 0, sizeof(in));
    in.turbine_id = 3;
    in.control_enable = 1;
    in.control_mode = FCR_CONTROL_MODE_PRC;
    in.bridge_command_valid_flags = FCR_BRIDGE_CMD_VALID | FCR_BRIDGE_CMD_CONTROL_ENABLED
                                  | FCR_BRIDGE_CMD_INDUCTION_VALID | FCR_BRIDGE_CMD_YAW_VALID;
    in.induction_ref = 0.2; in.induction_seq = 5;
    in.yaw_target_heading_rad = -0.3; in.yaw_seq = 9;
    in.command_basis_sim_time_s = 4.0;

    fcr_bridge_adapter_cmd_from_input(&in, 1, 400, 300, 300, 5.0, 60.0, &cmd);
    ASSERT(cmd.turbine_id == 3);
    ASSERT(cmd.valid == 1);
    ASSERT(cmd.induction_mode == 1);
    ASSERT(cmd.induction_valid == 1);
    ASSERT_NEAR(cmd.induction_ref, 0.2, 1e-12);
    ASSERT(cmd.induction_seq == 5);
    ASSERT_NEAR(cmd.induction_ttl_s, 5.0, 1e-12);
    ASSERT(cmd.yaw_valid == 1);
    ASSERT(cmd.yaw_seq == 9);
    ASSERT_NEAR(cmd.yaw_target_heading_rad, -0.3, 1e-12);
    ASSERT_NEAR(cmd.alloc_w_pitch, 0.4, 1e-9);
    ASSERT_NEAR(cmd.alloc_w_speed, 0.3, 1e-9);
    ASSERT_NEAR(cmd.alloc_w_torque, 0.3, 1e-9);
    /* S0710 是仿真时刻，绝不能被墙钟替换（ICD v1.5 G-4） */
    ASSERT_NEAR(cmd.source_time_s, 4.0, 1e-12);

    /* 无有效标记 → valid=0 且通道全部无效 */
    memset(&in.bridge_command_valid_flags, 0, sizeof(in.bridge_command_valid_flags));
    fcr_bridge_adapter_cmd_from_input(&in, 1, 400, 300, 300, 0.0, 0.0, &cmd);
    ASSERT(cmd.valid == 0);
    ASSERT(cmd.induction_valid == 0);
    ASSERT(cmd.yaw_valid == 0);
}

/* ---- S06: TurbineWindEstimate → FcrBridgeMeas ---- */
static void test_wind_estimate(void) {
    FcrBridgeMeas meas;
    FcrTurbineWindEstimate est;
    memset(&meas, 0, sizeof(meas));
    meas.valid = 1; meas.ambient_upstream_wind_mps = 7.0;
    memset(&est, 0, sizeof(est));
    est.turbine_id = 1; est.estimated_wind_mps = 8.4; est.estimate_sim_time_s = 100.0;
    est.wind_source = FCR_WIND_SOURCE_ROSCO_WE;
    est.wind_quality_flags = FCR_WIND_QUALITY_OK;

    /* 质量 OK 且未超龄 → 采用 S0601 并回填 S0603/S0604 */
    ASSERT(fcr_bridge_adapter_meas_apply_wind_estimate(&meas, &est, 100.5, 2.0) == 0);
    ASSERT_NEAR(meas.ambient_upstream_wind_mps, 8.4, 1e-12);
    ASSERT(meas.wind_source == FCR_WIND_SOURCE_ROSCO_WE);
    ASSERT(meas.wind_quality_flags == FCR_WIND_QUALITY_OK);

    /* 超龄（S0602）→ 不采用，保留旧值并置 STALE */
    meas.ambient_upstream_wind_mps = 7.0;
    meas.wind_quality_flags = 0;
    ASSERT(fcr_bridge_adapter_meas_apply_wind_estimate(&meas, &est, 105.0, 2.0) == -1);
    ASSERT_NEAR(meas.ambient_upstream_wind_mps, 7.0, 1e-12);
    ASSERT((meas.wind_quality_flags & FCR_WIND_QUALITY_STALE) != 0);

    /* 质量未置 OK → 不采用 */
    est.wind_quality_flags = 0;
    ASSERT(fcr_bridge_adapter_meas_apply_wind_estimate(&meas, &est, 100.5, 2.0) == -1);
    ASSERT_NEAR(meas.ambient_upstream_wind_mps, 7.0, 1e-12);

    /* stale_threshold=0 → 关闭超龄判定 */
    est.wind_quality_flags = FCR_WIND_QUALITY_OK;
    ASSERT(fcr_bridge_adapter_meas_apply_wind_estimate(&meas, &est, 1e6, 0.0) == 0);

    /* 空指针安全 */
    ASSERT(fcr_bridge_adapter_meas_apply_wind_estimate(NULL, &est, 0.0, 1.0) == -1);
    ASSERT(fcr_bridge_adapter_meas_apply_wind_estimate(&meas, NULL, 0.0, 1.0) == -1);
}

void test_bridge_adapter_register(void)
{
    RUN_TEST("cmd_from_state", test_cmd_from_state);
    RUN_TEST("meas_from_fast", test_meas_from_fast);
    RUN_TEST("adapter_apply", test_adapter_apply);
    RUN_TEST("adapter_null", test_adapter_null);
    RUN_TEST("input_from_state", test_input_from_state);
    RUN_TEST("cmd_from_input", test_cmd_from_input);
    RUN_TEST("wind_estimate", test_wind_estimate);
}
