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
#include <stdio.h>

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

    /* 写命令区 T1：yaw seq=1 绝对目标 -5°（CCW+） */
    b = MB_CMD_BASE + 0 * MB_CMD_PER_TURBINE;
    mb_put_double(mt->regs, b + MB_CMD_YAW_TARGET_HEADING, -5.0 * 3.141592653589793 / 180.0);
    mb_put_u64(mt->regs, b + MB_CMD_YAW_SEQ, 1);
    mb_put_u16(mt->regs, b + MB_CMD_YAW_VALID, 1);
    mb_put_u16(mt->regs, b + MB_CMD_CONTROL_ENABLE, 1);   /* ICD v1.5 S0104 */
    mb_put_double(mt->regs, b + MB_CMD_YAW_TTL_S, 60.0);
    /* T2 induction */
    b = MB_CMD_BASE + 1 * MB_CMD_PER_TURBINE;
    mb_put_double(mt->regs, b + MB_CMD_IND_REF, 0.2);
    mb_put_u64(mt->regs, b + MB_CMD_IND_SEQ, 1);
    mb_put_u16(mt->regs, b + MB_CMD_IND_VALID, 1);
    mb_put_u16(mt->regs, b + MB_CMD_CONTROL_ENABLE, 1);
    mb_put_double(mt->regs, b + MB_CMD_IND_TTL_S, 10.0);

    /* 设置 command_frame_seq 门控（时钟区） */
    mb_put_u64(mt->regs, MB_CLK_CMD_FRAME_SEQ_SLOT, 7);
    mt->last_frame_seq = 0;

    memset(&cmd, 0, sizeof(cmd));
    ASSERT(tr->ops->poll_command(tr, &cmd) == 1);
    ASSERT(cmd.command_frame_seq == 7);                   /* ICD v1.5 S0101 */
    ASSERT(cmd.n_turbines == 2);
    ASSERT(cmd.turbines[0].yaw_valid == 1);
    ASSERT(cmd.turbines[0].yaw_seq == 1);
    ASSERT(cmd.turbines[0].control_enable == 1);
    ASSERT_NEAR(cmd.turbines[0].yaw_target_heading_rad,
                -5.0 * 3.141592653589793 / 180.0, 1e-12);
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
    st.state_sample_sim_time_s = 12.5;    /* ICD v1.5 S1002 */
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
    st.turbines[0].rotor_aero_thrust_n.instant = 4.5e5;   /* ICD v1.5 S1010 */
    st.turbines[0].nacelle_heading_actual_rad = 0.1;      /* ICD v1.5 S1009 */
    st.turbines[0].yaw_seq_applied = 3;
    st.turbines[0].induction_seq_applied = 2;
    /* ICD v1.5 S1011/S1012/S1015-S1017 新增字段 */
    st.turbines[0].estimated_wind_mps = 8.1;
    st.turbines[0].ct_hat = 0.78;
    st.turbines[0].wind_source = 1;
    st.turbines[0].wind_quality_flags = 3;
    st.turbines[0].turbine_status_flags = 0x0005;
    st.turbines[0].tower_base_fa_moment_nm.instant = 9.9e6;

    ASSERT(tr->ops->publish_state(tr, &st) == 1);

    ASSERT_NEAR(mb_get_double(mt->regs, MB_CLK_STATE_SAMPLE_TIME), 12.5, 1e-12);
    ASSERT(mb_get_u64(mt->regs, MB_CLK_FAST_STEP) == 1250);
    b = MB_STATE_BASE + 0 * MB_STATE_PER_TURBINE;
    ASSERT(mb_get_u16(mt->regs, b + MB_ST_TURBINE_ID) == 1);
    ASSERT(mb_get_u16(mt->regs, b + MB_ST_VALID) == 1);
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_GEN_POWER_W), 4.05e6, 1e-3);
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_GEN_POWER_MEAN), 4.0e6, 1e-3);
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_ROTOR_AERO_THRUST), 4.5e5, 1e-3);
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_NACELLE_HEADING), 0.1, 1e-12);
    ASSERT(mb_get_u64(mt->regs, b + MB_ST_YAW_SEQ_APPLIED) == 3);
    ASSERT(mb_get_u64(mt->regs, b + MB_ST_IND_SEQ_APPLIED) == 2);
    /* ICD v1.5 新增状态寄存器 */
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_EST_WIND_MPS), 8.1, 1e-12);
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_CT_HAT), 0.78, 1e-12);
    ASSERT(mb_get_u16(mt->regs, b + MB_ST_WIND_SOURCE) == 1);
    ASSERT(mb_get_u16(mt->regs, b + MB_ST_WIND_QUALITY) == 3);
    ASSERT(mb_get_u16(mt->regs, b + MB_ST_TURBINE_STATUS) == 5);
    /* 原误标的 shaft_torque 槽现承载塔基前后向弯矩 */
    ASSERT_NEAR(mb_get_double(mt->regs, b + MB_ST_TOWER_BASE_FA_MOM), 9.9e6, 1e-3);
    modbus_transport_destroy(tr);
}

/* 寄存器布局守卫：命令区字段不得互相覆盖或越界到下一机 */
static void test_command_area_layout(void)
{
    ASSERT(MB_CMD_YAW_TARGET_HEADING + 4 <= MB_CMD_YAW_SEQ);
    ASSERT(MB_CMD_YAW_SEQ + 4 <= MB_CMD_YAW_VALID);
    ASSERT(MB_CMD_YAW_VALID + 1 <= MB_CMD_REQ_EFFECTIVE_TIME);
    ASSERT(MB_CMD_REQ_EFFECTIVE_TIME + 4 <= MB_CMD_YAW_TTL_S);
    ASSERT(MB_CMD_YAW_TTL_S + 4 <= MB_CMD_IND_REF);
    ASSERT(MB_CMD_IND_REF + 4 <= MB_CMD_IND_SEQ);
    ASSERT(MB_CMD_IND_SEQ + 4 <= MB_CMD_IND_VALID);
    ASSERT(MB_CMD_IND_VALID + 1 <= MB_CMD_CONTROL_ENABLE);
    ASSERT(MB_CMD_CONTROL_ENABLE + 1 <= MB_CMD_CONTROL_MODE);
    ASSERT(MB_CMD_CONTROL_MODE + 1 <= MB_CMD_IND_TTL_S);
    ASSERT(MB_CMD_IND_TTL_S + 4 <= MB_CMD_COMMAND_BASIS_TIME);
    ASSERT(MB_CMD_COMMAND_BASIS_TIME + 4 <= MB_CMD_COMMAND_FRAME_SEQ);
    ASSERT(MB_CMD_COMMAND_FRAME_SEQ + 4 <= (int)MB_CMD_PER_TURBINE);
}

/* 寄存器布局守卫：状态区（含 DR-8 完整性尾部）必须落在本机 128 寄存器块内 */
static void test_state_area_layout(void)
{
    ASSERT(MB_ST_TURBINE_STATUS < (int)MB_STATE_PER_TURBINE);
    ASSERT(MB_ST_FLOW_SEQ + 4 <= MB_ST_EST_WIND_MPS);
    ASSERT(MB_ST_CT_HAT + 4 <= MB_ST_WIND_SOURCE);
    ASSERT(MB_ST_WIND_SOURCE + 1 <= MB_ST_WIND_QUALITY);
    ASSERT(MB_ST_WIND_QUALITY + 1 <= MB_ST_TURBINE_STATUS);
    ASSERT(MB_ST_TURBINE_STATUS + 1 <= MB_ST_DATA_VERSION);
    /* u64 槽必须完整落在本机块内（修复前 0x7D 会溢出到 128） */
    ASSERT(MB_ST_TIMESTAMP_SLOT + 4 <= (int)MB_STATE_PER_TURBINE);
    ASSERT(MB_ST_SEQ_SLOT + 4 <= (int)MB_STATE_PER_TURBINE);
    ASSERT(MB_ST_QUALITY_FLAGS + 1 <= MB_ST_TIMESTAMP_SLOT);
    ASSERT(MB_ST_TIMESTAMP_SLOT + 4 <= MB_ST_SEQ_SLOT);
    ASSERT(MB_ST_SEQ_SLOT + 4 <= MB_ST_CRC16);
    ASSERT(MB_ST_CRC16 < (int)MB_STATE_PER_TURBINE);
}

/* 寄存器布局守卫：AWAE 缓冲必须整体落在寄存器区内，且不与状态区重叠 */
static void test_awae_area_layout(void)
{
    int last = MB_AWAE_POINT_0 + (FCR_MAX_OBSERVATION_POINTS - 1) * MB_AWAE_PER_POINT
               + MB_AWAE_POINT_CRC16;
    /* 不与状态区（0x1000..0x2FFF = 64 机 × 128）重叠：
     * 修复前 AWAE 置 0x2000，恰是状态区第 33 机起始。 */
    ASSERT(MB_AWAE_BASE >= (int)MB_STATE_BASE
                          + (int)MB_STATE_PER_TURBINE * (int)FCR_MAX_TURBINES);
    /* 单点字段不互相覆盖（f64 = 4 寄存器；修复前按 2 寄存器排布会重叠） */
    ASSERT(MB_AWAE_POINT_POINT_ID + 1 <= MB_AWAE_POINT_POS_XYZ);
    ASSERT(MB_AWAE_POINT_POS_XYZ + 12 <= MB_AWAE_POINT_WIND_XYZ);
    ASSERT(MB_AWAE_POINT_WIND_XYZ + 12 <= MB_AWAE_POINT_TI);
    ASSERT(MB_AWAE_POINT_TI + 4 <= MB_AWAE_POINT_QUALITY);
    ASSERT(MB_AWAE_POINT_QUALITY + 1 <= MB_AWAE_POINT_CRC16);
    ASSERT(MB_AWAE_POINT_CRC16 + 1 <= (int)MB_AWAE_PER_POINT);
    /* 头部不与点 0 重叠 */
    ASSERT(MB_AWAE_FRAME_SEQ_SLOT + 4 <= MB_AWAE_SOURCE_TIME);
    ASSERT(MB_AWAE_SOURCE_TIME + 4 <= MB_AWAE_POINT_0);
    /* 末点仍落在 modbus_transport_create 分配的 0x4000 寄存器区内 */
    ASSERT(last < MB_TOTAL_REGS);
}

/* ICD v1.5 GG-12/GG-13：publish_awae 把观测/目录写入 AWAE 缓冲 */
static void test_publish_awae(void)
{
    FcrTransport *tr = modbus_transport_create(15505, 2);
    struct FcrModbusTransport *mt;
    FcrAWAEObservationFrame obs;
    FcrAWAECatalogFrame cat;
    FcrAWAECatalog catalog;
    FcrFlowState flow;
    int i;

    ASSERT(tr != NULL);
    mt = (struct FcrModbusTransport *)tr->impl;
    ASSERT(mt != NULL);
    ASSERT(tr->ops->publish_awae != NULL);

    memset(&flow, 0, sizeof(flow));
    flow.valid = 1;
    flow.flow_seq = 77;
    flow.flow_source_time_s = 42.0;
    flow.n_points = 3;
    for (i = 0; i < 3; ++i) {
        flow.ambient_u[i] = 8.0 + i;
        flow.ambient_v[i] = 0.25 * i;
        flow.ambient_w[i] = 0.0;
        flow.point_ti[i] = 0.10;
    }
    fcr_awae_catalog_reset(&catalog, 3, 5u);
    for (i = 0; i < 3; ++i)
        fcr_awae_catalog_set_point(&catalog, i, (uint32_t)(900 + i),
                                   10.0 * i, 20.0 * i, 90.0);

    ASSERT(fcr_awae_build_observation_frame(&flow, &catalog,
                                            FCR_AWAE_QUALITY_SIMULATED, &obs) == 3);
    ASSERT(fcr_awae_build_catalog_frame(&catalog, 77, &cat) == 3);
    ASSERT(fcr_transport_publish_awae(tr, &obs, &cat) == 1);

    /* 头部：S0503 / S0501 / S0502 */
    ASSERT(mb_get_u16(mt->regs, MB_AWAE_N_POINTS_SLOT) == 3);
    ASSERT(mb_get_u64(mt->regs, MB_AWAE_FRAME_SEQ_SLOT) == 77);
    ASSERT_NEAR(mb_get_double(mt->regs, MB_AWAE_SOURCE_TIME), 42.0, 1e-12);

    /* 点 1：ID / 坐标(S0402) / 风矢量(S0505) / TI(S0507) / 质量(S0506) */
    {
        int b = MB_AWAE_POINT_0 + 1 * MB_AWAE_PER_POINT;
        ASSERT(mb_get_u16(mt->regs, b + MB_AWAE_POINT_POINT_ID) == 901);
        ASSERT_NEAR(mb_get_double(mt->regs, b + MB_AWAE_POINT_POS_XYZ), 10.0, 1e-12);
        ASSERT_NEAR(mb_get_double(mt->regs, b + MB_AWAE_POINT_POS_XYZ + 4), 20.0, 1e-12);
        ASSERT_NEAR(mb_get_double(mt->regs, b + MB_AWAE_POINT_POS_XYZ + 8), 90.0, 1e-12);
        ASSERT_NEAR(mb_get_double(mt->regs, b + MB_AWAE_POINT_WIND_XYZ), 9.0, 1e-6);
        ASSERT_NEAR(mb_get_double(mt->regs, b + MB_AWAE_POINT_WIND_XYZ + 4), 0.25, 1e-6);
        ASSERT_NEAR(mb_get_double(mt->regs, b + MB_AWAE_POINT_TI), 0.10, 1e-6);
        ASSERT((mb_get_u16(mt->regs, b + MB_AWAE_POINT_QUALITY)
                & FCR_AWAE_QUALITY_SIMULATED) != 0);
    }

    /* NULL 观测帧：头部清零，不崩溃（Transport 可选能力语义） */
    ASSERT(fcr_transport_publish_awae(tr, NULL, NULL) == 1);
    ASSERT(mb_get_u16(mt->regs, MB_AWAE_N_POINTS_SLOT) == 0);
    ASSERT(mb_get_u64(mt->regs, MB_AWAE_FRAME_SEQ_SLOT) == 0);

    modbus_transport_destroy(tr);
}

void test_modbus_register(void)
{
    RUN_TEST("register_encode_decode", test_register_encode_decode);
    RUN_TEST("command_region_to_frame", test_command_region_to_frame);
    RUN_TEST("state_region_from_frame", test_state_region_from_frame);
    RUN_TEST("command_area_layout", test_command_area_layout);
    RUN_TEST("state_area_layout", test_state_area_layout);
    RUN_TEST("awae_area_layout", test_awae_area_layout);
    RUN_TEST("publish_awae", test_publish_awae);
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