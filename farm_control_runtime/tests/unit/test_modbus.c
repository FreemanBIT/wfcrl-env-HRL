/*
 * test_modbus.c — Modbus TCP Transport 单元测试（Phase 9）
 * 验证：寄存器编解码、命令区→FarmCommandFrame、FarmStateFrame→状态区、
 * frame_seq 门控语义。
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include "modbus_transport.h"
#include "modbus_register_map.h"
#include <string.h>
#include <math.h>

/* 静态链接到实现内部函数（同一编译单元直接引用） */
extern FcrTransport *modbus_transport_create(int port, uint32_t n_turbines);
extern void modbus_transport_destroy(FcrTransport *t);

static void test_register_encode_decode(void)
{
    FcrTransport *tr = modbus_transport_create(15502, 2);
    ASSERT(tr != NULL);
    struct FcrModbusTransport *mt;
    uint64_t u;
    double d;
    mt = (struct FcrModbusTransport *)tr->impl;
    /* double 往返 */
    mb_put_double(mt->regs, 10, -0.0872665);
    d = mb_get_double(mt->regs, 10);
    ASSERT_NEAR(d, -0.0872665, 1e-15);
    /* u64 往返 */
    mb_put_u64(mt->regs, 20, 0xDEADBEEFCAFEF00DULL);
    u = mb_get_u64(mt->regs, 20);
    ASSERT(u == 0xDEADBEEFCAFEF00DULL);
    /* i64 往返 */
    mb_put_i64(mt->regs, 30, -1234567890123LL);
    ASSERT(mb_get_i64(mt->regs, 30) == -1234567890123LL);
    modbus_transport_destroy(tr);
}

static void test_command_region_to_frame(void)
{
    FcrTransport *tr = modbus_transport_create(15503, 2);
    FarmCommandFrame cmd;
    struct FcrModbusTransport *mt;
    int b;
    ASSERT(tr != NULL);
    mt = (struct FcrModbusTransport *)tr->impl;

    /* 写命令区 T1：yaw seq=1 delta=+5°CW */
    b = MB_CMD_BASE + 0 * MB_CMD_PER_TURBINE;
    mb_put_double(mt->regs, b + MB_CMD_YAW_DELTA_RAD, 5.0 * 3.141592653589793 / 180.0);
    mb_put_u64(mt->regs, b + MB_CMD_YAW_SEQ, 1);
    mb_put_u16(mt->regs, b + MB_CMD_YAW_VALID, 1);
    mb_put_double(mt->regs, b + MB_CMD_YAW_TTL_S, 60.0);
    /* T2 induction */
    b = MB_CMD_BASE + 1 * MB_CMD_PER_TURBINE;
    mb_put_double(mt->regs, b + MB_CMD_IND_REF, 0.2);
    mb_put_u64(mt->regs, b + MB_CMD_IND_SEQ, 1);
    mb_put_u16(mt->regs, b + MB_CMD_IND_VALID, 1);
    mb_put_double(mt->regs, b + MB_CMD_IND_TTL_S, 10.0);

    /* 设置 frame_seq 门控（时钟区） */
    mb_put_u64(mt->regs, MB_CLK_FRAME_SEQ_SLOT, 7);
    mt->last_frame_seq = 0;

    memset(&cmd, 0, sizeof(cmd));
    ASSERT(tr->ops->poll_command(tr, &cmd) == 1);
    ASSERT(cmd.frame_seq == 7);
    ASSERT(cmd.n_turbines == 2);
    ASSERT(cmd.turbines[0].yaw_valid == 1);
    ASSERT(cmd.turbines[0].yaw_seq == 1);
    ASSERT_NEAR(cmd.turbines[0].yaw_delta_rad, 5.0 * 3.141592653589793 / 180.0, 1e-12);
    ASSERT_NEAR(cmd.turbines[0].yaw_ttl_s, 60.0, 1e-12);
    ASSERT(cmd.turbines[1].induction_valid == 1);
    ASSERT(cmd.turbines[1].induction_seq == 1);
    ASSERT_NEAR(cmd.turbines[1].induction_ref, 0.2, 1e-12);
    ASSERT_NEAR(cmd.turbines[1].induction_ttl_s, 10.0, 1e-12);

    /* 相同帧序号幂等（不重复返回） */
    memset(&cmd, 0, sizeof(cmd));
    ASSERT(tr->ops->poll_command(tr, &cmd) == 0);
    modbus_transport_destroy(tr);
}

static void test_state_region_from_frame(void)
{
    FcrTransport *tr = modbus_transport_create(15504, 2);
    FarmStateFrame st;
    struct FcrModbusTransport *mt;
    int b;
    ASSERT(tr != NULL);
    mt = (struct FcrModbusTransport *)tr->impl;

    memset(&st, 0, sizeof(st));
    st.protocol_version = FCR_PROTOCOL_VERSION;
    st.sim_time_s = 12.5;
    st.fast_step = 1250;
    st.low_step = 4;
    st.rt_health_flags = FCR_RT_HEALTH_OK;
    st.n_turbines = 2;
    st.turbines[0].turbine_id = 1;
    st.turbines[0].valid = 1;
    st.turbines[0].source_time_s = 12.5;
    st.turbines[0].gen_power_w.instant = 4.05e6;
    st.turbines[0].gen_power_w.mean = 4.0e6;
    st.turbines[0].gen_power_w.rms = 4.02e6;
    st.turbines[0].rotor_thrust_n.instant = 4.5e5;
    st.turbines[0].nacelle_heading_rad = 0.1;
    st.turbines[0].yaw_seq_applied = 3;
    st.turbines[0].induction_seq_applied = 2;

    ASSERT(tr->ops->publish_state(tr, &st) == 1);

    ASSERT_NEAR(mb_get_double(mt->regs, MB_CLK_SIM_TIME_S), 12.5, 1e-12);
    ASSERT(mb_get_u64(mt->regs, MB_CLK_FAST_STEP) == 1250);
    b = MB_STATE_BASE + 0 * MB_STATE_PER_TURBINE;
    ASSERT(mb_get_u16(mt->regs, b + MB_ST_TURBINE_ID) == 1);
    ASSERT(mb_get_u16(mt->regs, b + MB_ST_VALID) == 1);
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_GEN_POWER_W), 4.05e6, 1e-3);
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_GEN_POWER_MEAN), 4.0e6, 1e-3);
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_ROTOR_THRUST_N), 4.5e5, 1e-3);
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_NACELLE_HEADING), 0.1, 1e-12);
    ASSERT(mb_get_u64(mt->regs, b + MB_ST_YAW_SEQ_APPLIED) == 3);
    ASSERT(mb_get_u64(mt->regs, b + MB_ST_IND_SEQ_APPLIED) == 2);
    modbus_transport_destroy(tr);
}

void test_modbus_register(void)
{
    RUN_TEST("register_encode_decode", test_register_encode_decode);
    RUN_TEST("command_region_to_frame", test_command_region_to_frame);
    RUN_TEST("state_region_from_frame", test_state_region_from_frame);
}


/* 辅助：不启动 socket 的纯编解码测试（崩溃隔离用） */
void test_modbus_encode_only(void)
{
    uint16_t regs[64];
    double d;
    memset(regs, 0, sizeof(regs));
    mb_put_double(regs, 2, -0.0872665);
    d = mb_get_double(regs, 2);
    printf("[modbus] enc-only d=%.6f\n", d);
    fflush(stdout);
}