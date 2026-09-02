/*
 * test_command_store.c — 命令通道独立 / seq 幂等与乱序 / TTL / 锁存（Phase 1）
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include <string.h>

#define PI 3.14159265358979323846

static FcrCommandStore *make_store(void)
{
    return fcr_command_store_create(2, 60.0, 1.0, 5.0, 1.0);
}

static void fill_yaw(FcrTurbineCommand *cmd, uint64_t seq, double delta_rad)
{
    memset(cmd, 0, sizeof(*cmd));
    cmd->yaw_valid = 1;
    cmd->yaw_seq = seq;
    cmd->yaw_delta_rad = delta_rad;
    cmd->yaw_ttl_s = 60.0;
    cmd->source_time_s = 0.0;
}

static void fill_ind(FcrTurbineCommand *cmd, uint64_t seq, double ref)
{
    memset(cmd, 0, sizeof(*cmd));
    cmd->induction_valid = 1;
    cmd->induction_seq = seq;
    cmd->induction_ref = ref;
    cmd->induction_ttl_s = 1.0;
    cmd->source_time_s = 0.0;
}

static FarmCommandFrame make_frame(FcrTurbineCommand *t1, FcrTurbineCommand *t2)
{
    FarmCommandFrame fr;
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION;
    fr.frame_seq = 1;
    fr.n_turbines = 2;
    if (t1) fr.turbines[0] = *t1;
    if (t2) fr.turbines[1] = *t2;
    return fr;
}

/* --- yaw/induction 互不覆盖 --- */
static void test_channels_independent(void)
{
    FcrCommandStore *cs = make_store();
    FcrTurbineCommand yaw, ind;
    FcrRoscoExternalSetpoint sp;
    double heading[2] = {0.0, 0.0};

    fill_yaw(&yaw, 1, fcr_deg2rad(5.0));
    fill_ind(&ind, 1, 0.25);
    /* 同一帧里 yaw+induction 同时下发 T1 */
    yaw.induction_valid = 1;
    yaw.induction_seq = 1;
    yaw.induction_ref = 0.25;
    yaw.induction_ttl_s = 1.0;
    FarmCommandFrame fr = make_frame(&yaw, NULL);
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);

    /* 锁存需要机舱方位 */
    fcr_command_store_refresh(cs, 0.0, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT(sp.yaw_enable == 1);
    ASSERT(sp.induction_enable == 1);
    ASSERT_NEAR(sp.yaw_target_heading_rad, fcr_deg2rad(-5.0), 1e-12);

    /* 再只发 induction 新 seq —— yaw 目标不得变化 */
    FcrTurbineCommand ind2;
    fill_ind(&ind2, 2, 0.30);
    FarmCommandFrame fr2 = make_frame(&ind2, NULL);
    ASSERT(fcr_command_store_accept_frame(cs, &fr2) == 0);
    heading[0] = fcr_deg2rad(10.0);   /* 机舱动了，但 yaw 目标应仍是旧锁存 */
    fcr_command_store_refresh(cs, 0.0, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT_NEAR(sp.yaw_target_heading_rad, fcr_deg2rad(-5.0), 1e-12);
    ASSERT(sp.induction_enable == 1);
    ASSERT(sp.induction_seq == 2);

    /* 再只发 yaw 新 seq —— induction 引用不得变化 */
    FcrTurbineCommand yaw2;
    fill_yaw(&yaw2, 2, fcr_deg2rad(-5.0));
    FarmCommandFrame fr3 = make_frame(&yaw2, NULL);
    ASSERT(fcr_command_store_accept_frame(cs, &fr3) == 0);
    fcr_command_store_refresh(cs, 0.0, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT_NEAR(sp.yaw_target_heading_rad, fcr_deg2rad(15.0) /* heading 10 + (-5 CW → +5 CCW) = 15 */, 1e-12);
    ASSERT(sp.yaw_seq == 2);
    fcr_command_store_destroy(cs);
}

/* --- 重复 seq 幂等 --- */
static void test_duplicate_seq_idempotent(void)
{
    FcrCommandStore *cs = make_store();
    FcrTurbineCommand cmd;
    FcrRoscoExternalSetpoint sp;
    double heading[2] = {0.0, 0.0};

    fill_yaw(&cmd, 1, fcr_deg2rad(5.0));
    FarmCommandFrame fr = make_frame(&cmd, NULL);
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, heading, 1);

    /* 机舱位置变到 30°；重复 seq=1 不得重新锁存 */
    heading[0] = fcr_deg2rad(30.0);
    FarmCommandFrame fr2 = make_frame(&cmd, NULL);
    ASSERT(fcr_command_store_accept_frame(cs, &fr2) == 0);
    fcr_command_store_refresh(cs, 0.5, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT_NEAR(sp.yaw_target_heading_rad, fcr_deg2rad(-5.0), 1e-12);
    fcr_command_store_destroy(cs);
}

/* --- 乱序 seq 拒绝 --- */
static void test_out_of_order_seq_rejected(void)
{
    FcrCommandStore *cs = make_store();
    FcrTurbineCommand cmd;
    FcrRoscoExternalSetpoint sp;
    double heading[2] = {0.0, 0.0};

    fill_yaw(&cmd, 2, fcr_deg2rad(5.0));
    FarmCommandFrame fr = make_frame(&cmd, NULL);
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT(sp.yaw_enable == 1);
    ASSERT(sp.yaw_seq == 2);

    /* seq=1 落后 → 拒绝，且目标保持 seq=2 的锁存 */
    fill_yaw(&cmd, 1, fcr_deg2rad(20.0));
    FarmCommandFrame fr2 = make_frame(&cmd, NULL);
    ASSERT(fcr_command_store_accept_frame(cs, &fr2) == 0);
    fcr_command_store_refresh(cs, 0.0, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT(sp.yaw_enable == 1);
    ASSERT(sp.yaw_seq == 2);
    ASSERT_NEAR(sp.yaw_target_heading_rad, fcr_deg2rad(-5.0), 1e-12);
    ASSERT((sp.command_status_flags & FCR_CMD_FLAG_INVALID_SEQ) != 0);
    fcr_command_store_destroy(cs);
}

/* --- TTL 到期 --- */
static void test_ttl_expiry(void)
{
    FcrCommandStore *cs = make_store();
    FcrTurbineCommand cmd;
    FcrRoscoExternalSetpoint sp;
    double heading[2] = {0.0, 0.0};

    /* induction TTL = 1 s */
    fill_ind(&cmd, 1, 0.25);
    FarmCommandFrame fr = make_frame(&cmd, NULL);
    ASSERT(fcr_command_store_accept_frame(cs, &fr) == 0);
    fcr_command_store_refresh(cs, 0.0, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT(sp.induction_enable == 1);

    /* 1.5 s 后（超过 TTL 1 s）→ 通道失效 */
    fcr_command_store_refresh(cs, 1.5, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT(sp.induction_enable == 0);
    ASSERT((sp.command_status_flags & FCR_CMD_FLAG_INDUCTION_TTL_EXPIRED) != 0);

    /* yaw TTL = 60 s，5 s 后仍有效 */
    fill_yaw(&cmd, 1, fcr_deg2rad(5.0));
    FarmCommandFrame fr2 = make_frame(&cmd, NULL);
    fcr_command_store_accept_frame(cs, &fr2);
    fcr_command_store_refresh(cs, 0.0, heading, 1);
    fcr_command_store_refresh(cs, 5.0, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT(sp.yaw_enable == 1);
    fcr_command_store_destroy(cs);
}

/* --- stale 标志 --- */
static void test_stale_flag(void)
{
    FcrCommandStore *cs = make_store();
    FcrTurbineCommand cmd;
    FcrRoscoExternalSetpoint sp;
    double heading[2] = {0.0, 0.0};

    fill_yaw(&cmd, 1, fcr_deg2rad(5.0));
    cmd.source_time_s = 1.0;   /* 命令发出于 t=1 s */
    FarmCommandFrame fr = make_frame(&cmd, NULL);
    fcr_command_store_accept_frame(cs, &fr);
    fcr_command_store_refresh(cs, 10.0, heading, 1);  /* 距 source 9 s > 5 s 阈值 */
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT((sp.command_status_flags & FCR_CMD_FLAG_YAW_STALE) != 0);
    fcr_command_store_destroy(cs);
}

/* --- 目标锁存只发生在新 seq 生效时（heading 已知）--- */
static void test_yaw_target_not_reevaluated(void)
{
    FcrCommandStore *cs = make_store();
    FcrTurbineCommand cmd;
    FcrRoscoExternalSetpoint sp;
    double heading[2] = {0.0, 0.0};

    fill_yaw(&cmd, 1, fcr_deg2rad(5.0));
    FarmCommandFrame fr = make_frame(&cmd, NULL);
    fcr_command_store_accept_frame(cs, &fr);
    fcr_command_store_refresh(cs, 0.0, heading, 1);       /* 锁存：heading=0 → target=-5° */
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT_NEAR(sp.yaw_target_heading_rad, fcr_deg2rad(-5.0), 1e-12);

    /* 后续每次 refresh 即使机舱方位变化，目标保持不变 */
    heading[0] = fcr_deg2rad(40.0);
    fcr_command_store_refresh(cs, 0.1, heading, 1);
    fcr_command_store_refresh(cs, 0.2, heading, 1);
    fcr_command_store_get_setpoint(cs, 1, &sp);
    ASSERT_NEAR(sp.yaw_target_heading_rad, fcr_deg2rad(-5.0), 1e-12);
    fcr_command_store_destroy(cs);
}

void test_command_store_register(void)
{
    RUN_TEST("channels_independent", test_channels_independent);
    RUN_TEST("duplicate_seq_idempotent", test_duplicate_seq_idempotent);
    RUN_TEST("out_of_order_seq_rejected", test_out_of_order_seq_rejected);
    RUN_TEST("ttl_expiry", test_ttl_expiry);
    RUN_TEST("stale_flag", test_stale_flag);
    RUN_TEST("yaw_target_not_reevaluated", test_yaw_target_not_reevaluated);
}