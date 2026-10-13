/*
 * test_bridge_core.c — ROSCO Bridge 实时核心单元测试（M1）
 * 覆盖：Mode0/1/2、控制分配、Ct 估计（DR-7 修正）、偏航适配（DR-5）、安全（TTL/fallback）
 */
#include "test_framework.h"
#include "fcr_bridge_core.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static FcrBridgeCore *mk_core(void) {
    return fcr_bridge_core_create(3, NULL, NULL, NULL);
}

/* ---- Mode 1：功率比例开环 ---- */
static void test_mode1_power_ratio(void) {
    /* a=1/3 → ratio=1（greedy）；a=0.2 → 理论 R_power=a(1-a)^2/a0(1-a0)^2 */
    double r_g = fcr_bridge_mode1_power_ratio(1.0 / 3.0, 1.0 / 3.0);
    double a = 0.2, a0 = 1.0 / 3.0;
    double expect = a * (1 - a) * (1 - a) / (a0 * (1 - a0) * (1 - a0));
    ASSERT_NEAR(r_g, 1.0, 1e-9);
    ASSERT_NEAR(fcr_bridge_mode1_power_ratio(a, a0), expect, 1e-9);
    /* a 越界 → 安全默认 1.0 */
    ASSERT_NEAR(fcr_bridge_mode1_power_ratio(0.0, a0), 1.0, 1e-9);
    ASSERT_NEAR(fcr_bridge_mode1_power_ratio(1.5, a0), 1.0, 1e-9);
}

/* ---- Mode 2：Ct_ref = 4a(1-a) ---- */
static void test_mode2_ct_ref(void) {
    ASSERT_NEAR(fcr_bridge_mode2_ct_ref(1.0 / 3.0), 8.0 / 9.0, 1e-9);
    ASSERT_NEAR(fcr_bridge_mode2_ct_ref(0.2), 4 * 0.2 * 0.8, 1e-9);
    ASSERT_NEAR(fcr_bridge_mode2_ct_ref(0.45), 4 * 0.45 * 0.55, 1e-9);
}

/* ---- Mode 2：气动表反解 beta ---- */
static void test_mode2_find_beta(void) {
    FcrBridgeCmd cmd;
    double beta = 0.0;
    int i;
    memset(&cmd, 0, sizeof(cmd));
    cmd.aero_n_lam = 3;
    cmd.aero_n_beta = 5;
    cmd.aero_lam[0] = 5.0; cmd.aero_lam[1] = 7.0; cmd.aero_lam[2] = 9.0;
    /* beta: 0,2,4,6,8 deg；Ct 随 beta 增大下降 */
    for (i = 0; i < cmd.aero_n_beta; ++i) {
        double b = cmd.aero_beta_deg[i] = 2.0 * i;      /* 0,2,4,6,8 */
        cmd.aero_ct[1 * cmd.aero_n_beta + i] = 0.9 - 0.05 * b; /* lam=7 行 */
    }
    /* Ct_ref≈0.70 → beta≈4 deg 附近 */
    ASSERT(fcr_bridge_mode2_find_beta(&cmd, 7.0, 0.70, &beta) == 0);
    ASSERT_NEAR(beta, 4.0, 1e-9);
    /* lam 不在表内 → 取最近行（应仍工作） */
    ASSERT(fcr_bridge_mode2_find_beta(&cmd, 7.4, 0.70, &beta) == 0);
    /* 表覆盖不足（Ct 表范围 0.5-0.9，ct_ref=1.5 明显超出）→ 返回错误 */
    ASSERT(fcr_bridge_mode2_find_beta(&cmd, 7.0, 1.5, &beta) != 0);
}

/* ---- 端到端：Mode0 旁路 / Mode1 控制分配 / Mode2 前馈 ---- */
static void test_step_modes(void) {
    FcrBridgeCore *core = mk_core();
    FcrBridgeCmd cmd;
    FcrBridgeMeas meas;
    FcrRoscoExternalSetpoint sp;
    int rc;

    /* T1 = Mode1 induction a=0.2，默认分配（纯 r_pitch） */
    memset(&cmd, 0, sizeof(cmd));
    cmd.turbine_id = 1; cmd.valid = 1;
    cmd.induction_mode = 1; cmd.induction_ref = 0.2;
    cmd.induction_seq = 1; cmd.induction_valid = 1;
    cmd.induction_ttl_s = 1e6; cmd.yaw_ttl_s = 1e6;
    cmd.source_time_s = 0.0;
    memset(&meas, 0, sizeof(meas));
    meas.turbine_id = 1; meas.valid = 1; meas.source_time_s = 1.0;
    meas.rotor_speed_rad_s = 1.2; meas.disk_wind_speed_mps = 8.0;
    meas.rotor_thrust_n = 400000.0; meas.gen_power_w = 2e6;

    fcr_bridge_core_set_command(core, &cmd);
    fcr_bridge_core_set_meas(core, &meas);
    rc = fcr_bridge_core_step(core, 1, 10.0, &sp);
    ASSERT(rc == 0);
    ASSERT(sp.induction_enable == 1);
    /* 默认纯 r_pitch：power_ratio = R_power < 1；speed/torque = 1 */
    ASSERT(sp.power_ratio < 1.0);
    ASSERT_NEAR(sp.speed_ref_ratio, 1.0, 1e-9);
    ASSERT_NEAR(sp.torque_limit_ratio, 1.0, 1e-9);

    /* 控制分配：显式三通道权重 0.4/0.3/0.3 */
    cmd.alloc_w_pitch = 0.4; cmd.alloc_w_speed = 0.3; cmd.alloc_w_torque = 0.3;
    cmd.induction_seq = 2;
    fcr_bridge_core_set_command(core, &cmd);
    fcr_bridge_core_step(core, 1, 11.0, &sp);
    ASSERT(sp.power_ratio < 1.0);
    ASSERT(sp.speed_ref_ratio < 1.0);
    ASSERT(sp.torque_limit_ratio < 1.0);
    /* 分配守恒：总降载 = 0.4*dr_p + 0.3*dr_s + 0.3*dr_t（近似） */

    /* T2 = Mode2 Ct 前馈 */
    memset(&cmd, 0, sizeof(cmd));
    cmd.turbine_id = 2; cmd.valid = 1;
    cmd.induction_mode = 2; cmd.induction_ref = 0.2;
    cmd.induction_seq = 1; cmd.induction_valid = 1;
    cmd.induction_ttl_s = 1e6; cmd.yaw_ttl_s = 1e6;
    cmd.source_time_s = 0.0;
    cmd.aero_blade_radius_m = 63.0;
    cmd.aero_n_lam = 3; cmd.aero_n_beta = 5;
    cmd.aero_lam[0] = 5.0; cmd.aero_lam[1] = 7.0; cmd.aero_lam[2] = 9.0;
    {
        int i;
        for (i = 0; i < cmd.aero_n_beta; ++i) {
            double b = cmd.aero_beta_deg[i] = 2.0 * i;
            cmd.aero_ct[1 * cmd.aero_n_beta + i] = 0.9 - 0.05 * b;
        }
    }
    memset(&meas, 0, sizeof(meas));
    meas.turbine_id = 2; meas.valid = 1; meas.source_time_s = 1.0;
    meas.rotor_speed_rad_s = 8.0; meas.disk_wind_speed_mps = 8.0; /* lam≈63 → 取最近 9? 实际 lam=8*63/8=63 超表 → 取 lam=9 行（最接近） */
    meas.rotor_thrust_n = 400000.0;
    fcr_bridge_core_set_command(core, &cmd);
    fcr_bridge_core_set_meas(core, &meas);
    fcr_bridge_core_step(core, 2, 10.0, &sp);
    ASSERT(sp.induction_enable == 1);
    /* Mode2 输出 min_pitch_rad（可能有值）且 power_ratio=1 */
    ASSERT_NEAR(sp.power_ratio, 1.0, 1e-9);

    fcr_bridge_core_destroy(core);
}

/* ---- Mode0：旁路（induction_enable=0，ratio 全 1）---- */
static void test_step_mode0(void) {
    FcrBridgeCore *core = mk_core();
    FcrBridgeCmd cmd;
    FcrRoscoExternalSetpoint sp;
    memset(&cmd, 0, sizeof(cmd));
    cmd.turbine_id = 1; cmd.valid = 1;
    cmd.induction_mode = 0; cmd.induction_ref = 0.2;
    cmd.induction_seq = 1; cmd.induction_valid = 1;
    cmd.induction_ttl_s = 1e6; cmd.yaw_ttl_s = 1e6;
    cmd.source_time_s = 0.0;
    fcr_bridge_core_set_command(core, &cmd);
    fcr_bridge_core_step(core, 1, 5.0, &sp);
    ASSERT(sp.induction_enable == 0);
    ASSERT_NEAR(sp.power_ratio, 1.0, 1e-9);
    ASSERT_NEAR(sp.min_pitch_rad, 0.0, 1e-9);
    fcr_bridge_core_destroy(core);
}

/* ---- Ct 估计（DR-7 模型修正）---- */
static void test_ct_estimator(void) {
    FcrBridgeCore *core = mk_core();
    FcrBridgeCtConfig ctcfg;
    FcrBridgeMeas meas;
    FcrBridgeTurbineState st;
    double rho = 1.225, A = M_PI * 63.0 * 63.0;
    /* 无修正：T=0.5*rho*A*V^2*Ct → Ct 精确还原 */
    memset(&ctcfg, 0, sizeof(ctcfg));
    ctcfg.air_density = rho; ctcfg.rotor_area = A; ctcfg.gen_eff = 0.944;
    ctcfg.use_model_corr = 0; ctcfg.v_disk_ref_corr = 1.0;
    fcr_bridge_core_destroy(core);
    core = fcr_bridge_core_create(1, NULL, &ctcfg, NULL);
    memset(&meas, 0, sizeof(meas));
    meas.turbine_id = 1; meas.valid = 1;
    meas.disk_wind_speed_mps = 8.0;
    meas.rotor_thrust_n = 0.5 * rho * A * 64.0 * 0.7; /* Ct=0.7 */
    fcr_bridge_core_set_meas(core, &meas);
    fcr_bridge_core_step(core, 1, 1.0, &(FcrRoscoExternalSetpoint){0});
    fcr_bridge_core_get(core, 1, &st);
    ASSERT_NEAR(st.ct_est, 0.7, 1e-6);
    ASSERT(st.ct_est_valid == 1);

    /* 模型修正：扰动风 8.0 按 /1.05 折算（参考风速略低）→ Ct 重新归一化 */
    memset(&ctcfg, 0, sizeof(ctcfg));
    ctcfg.air_density = rho; ctcfg.rotor_area = A; ctcfg.gen_eff = 0.944;
    ctcfg.use_model_corr = 1; ctcfg.v_disk_ref_corr = 1.05;
    fcr_bridge_core_destroy(core);
    core = fcr_bridge_core_create(1, NULL, &ctcfg, NULL);
    memset(&meas, 0, sizeof(meas));
    meas.turbine_id = 1; meas.valid = 1;
    meas.disk_wind_speed_mps = 8.0;
    meas.rotor_thrust_n = 0.5 * rho * A * 64.0 * 0.7;
    fcr_bridge_core_set_meas(core, &meas);
    fcr_bridge_core_step(core, 1, 1.0, &(FcrRoscoExternalSetpoint){0});
    fcr_bridge_core_get(core, 1, &st);
    /* V_ref = 8/1.05 → Ct = 0.7 * (8/(8/1.05))^2 = 0.7*1.05^2 */
    ASSERT_NEAR(st.ct_est, 0.7 * 1.05 * 1.05, 1e-6);
    fcr_bridge_core_destroy(core);
}

/* ---- 偏航适配（DR-5）---- */
static void test_yaw_adapter(void) {
    double h0 = 0.5;  /* 当前机舱方位（内部约定）*/
    double delta_cw = 0.1745; /* +10° CW（罗盘）*/
    double target = fcr_yaw_adapter_delta_to_target(delta_cw, h0);
    /* delta_internal = -delta_cw → target = wrap(h0 - 0.1745) */
    ASSERT_NEAR(target, h0 - 0.1745, 1e-9);
    ASSERT(fcr_yaw_adapter_validate_abs(target) == 0);
    ASSERT(fcr_yaw_adapter_validate_abs(1.0 / 0.0) != 0); /* NaN 拒绝 */
}

/* ---- 安全：TTL 过期回退额定 + fallback ---- */
static void test_safety(void) {
    FcrBridgeCore *core = mk_core();
    FcrBridgeCmd cmd;
    FcrRoscoExternalSetpoint sp;
    memset(&cmd, 0, sizeof(cmd));
    cmd.turbine_id = 1; cmd.valid = 1;
    cmd.induction_mode = 1; cmd.induction_ref = 0.2;
    cmd.induction_seq = 1; cmd.induction_valid = 1;
    cmd.induction_ttl_s = 5.0; cmd.yaw_ttl_s = 1e6;
    cmd.source_time_s = 0.0;
    fcr_bridge_core_set_command(core, &cmd);
    /* TTL 内 → 生效 */
    fcr_bridge_core_step(core, 1, 3.0, &sp);
    ASSERT(sp.induction_enable == 1);
    /* TTL 后（sim_time - source_time = 6 > 5）→ 回退额定 */
    fcr_bridge_core_step(core, 1, 6.0, &sp);
    ASSERT(sp.induction_enable == 0);
    fcr_bridge_core_destroy(core);
}

void test_bridge_core_register(void)
{
    RUN_TEST("mode1_power_ratio", test_mode1_power_ratio);
    RUN_TEST("mode2_ct_ref", test_mode2_ct_ref);
    RUN_TEST("mode2_find_beta", test_mode2_find_beta);
    RUN_TEST("step_modes", test_step_modes);
    RUN_TEST("step_mode0", test_step_mode0);
    RUN_TEST("ct_estimator", test_ct_estimator);
    RUN_TEST("yaw_adapter", test_yaw_adapter);
    RUN_TEST("safety", test_safety);
}
