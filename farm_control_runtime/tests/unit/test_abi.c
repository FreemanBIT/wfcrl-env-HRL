/*
 * test_abi.c — ABI 尺寸/对齐冻结检查（ICD v1.5）
 *
 * ABI 变化必须同步更新本文件与 ICD；CI 中断即表明 ABI 破坏。
 * 尺寸由 abi_dump（64 位 TDM-GCC-64 10.3.0）实测，2026-10-19。
 * 同时必须与 wfcrl/transport/farm_protocol.py 的 assert 保持一致。
 */
#include "test_framework.h"
#include "fcr_protocol.h"
#include "fcr_state_types.h"
#include "fcr_awae_types.h"
#include "fcr_transport.h"
#include "fcr_rosco_api.h"
#include "fcr_bridge_core.h"
#include "awae_point_frame.h"
#include <stddef.h>

#define CHECK_ABI(type, expected_size) do {                          \
    if (sizeof(type) != (expected_size)) {                           \
        fprintf(stderr, "  [ABI] sizeof(%s)=%zu expected %d\n",      \
                #type, sizeof(type), (int)(expected_size));           \
        longjmp(g_env, 1);                                            \
    }                                                                 \
} while (0)

static void test_abi_sizes(void)
{
    /* ---- 命令链路 (S01) ---- */
    /* ICD v1.5: yaw_delta_rad/commit_seq 删除, control_enable/control_mode/
     * requested_effective_sim_time_s 新增 → 104 → 88 bytes */
    CHECK_ABI(FcrTurbineCommand, 88);
    CHECK_ABI(FarmCommandFrame, 5664);          /* 32 + 88*64 */

    /* ---- 状态链路 (S02/S03) ---- */
    CHECK_ABI(FcrOpenFASTRawState, 104);
    CHECK_ABI(FcrRoscoRawState, 232);           /* 原 FcrRoscoFastState，尺寸不变 */
    CHECK_ABI(FcrSignalStats, 48);
    CHECK_ABI(FcrTurbineExtraState, 840);
    CHECK_ABI(FcrFlowState, 5152);

    /* ---- Bridge 链路 (S07/S08/S09) ---- */
    CHECK_ABI(FcrBridgeInput, 72);
    CHECK_ABI(FcrBridgeOutput, 56);
    CHECK_ABI(FcrBridgeAck, 32);

    /* ---- ROSCO setpoint/ack (S08/S09) ---- */
    CHECK_ABI(FcrRoscoExternalSetpoint, 112);   /* S0807 新增 → 104；S1101 保留接口 → 112 */
    CHECK_ABI(FcrRoscoAck, 32);

    /* ---- 状态帧 (S10) ---- */
    CHECK_ABI(FcrFarmTurbineState, 400);        /* S1011/S1012/S1015-S1017 新增 */
    CHECK_ABI(FarmStateFrame, 25664);

    /* ---- AWAE (S04/S05) + 风估计 (S06) ---- */
    CHECK_ABI(FcrAWAECatalogEntry, 40);
    CHECK_ABI(FcrAWAECatalogFrame, 2584);
    CHECK_ABI(FcrAWAEObservation, 48);
    CHECK_ABI(FcrAWAEObservationFrame, 3096);
    CHECK_ABI(FcrTurbineWindEstimate, 32);

    /* ---- 诱导映射 ---- */
    CHECK_ABI(FcrInductionMapConfig, 56);
    CHECK_ABI(FcrInductionMapOutput, 56);
    CHECK_ABI(FcrInductionState, 80);
}

static void test_abi_offsets(void)
{
    /* 关键字段偏移（帮助发现误改/静默字段错位） */

    /* FcrTurbineCommand — ICD v1.5 布局（88 B） */
    ASSERT(offsetof(FcrTurbineCommand, yaw_target_heading_rad) == 0);
    ASSERT(offsetof(FcrTurbineCommand, yaw_seq) == 8);
    ASSERT(offsetof(FcrTurbineCommand, yaw_valid) == 16);
    ASSERT(offsetof(FcrTurbineCommand, induction_ref) == 24);
    ASSERT(offsetof(FcrTurbineCommand, induction_seq) == 32);
    ASSERT(offsetof(FcrTurbineCommand, induction_valid) == 40);
    ASSERT(offsetof(FcrTurbineCommand, control_enable) == 44);
    ASSERT(offsetof(FcrTurbineCommand, control_mode) == 48);
    ASSERT(offsetof(FcrTurbineCommand, requested_effective_sim_time_s) == 56);
    ASSERT(offsetof(FcrTurbineCommand, command_basis_sim_time_s) == 64);
    ASSERT(offsetof(FcrTurbineCommand, yaw_ttl_s) == 72);
    ASSERT(offsetof(FcrTurbineCommand, induction_ttl_s) == 80);

    /* FarmCommandFrame — 头 32 B + 88×64 */
    ASSERT(offsetof(FarmCommandFrame, protocol_version) == 0);
    ASSERT(offsetof(FarmCommandFrame, command_frame_seq) == 8);
    ASSERT(offsetof(FarmCommandFrame, n_turbines) == 16);
    ASSERT(offsetof(FarmCommandFrame, receive_time_s) == 24);
    ASSERT(offsetof(FarmCommandFrame, turbines) == 32);

    /* FcrRoscoRawState — 回读字段 */
    ASSERT(offsetof(FcrRoscoRawState, nacelle_heading_rad) == 104);
    ASSERT(offsetof(FcrRoscoRawState, yaw_seq_applied) == 216);
    ASSERT(offsetof(FcrRoscoRawState, induction_seq_applied) == 224);

    /* FcrRoscoExternalSetpoint — S08 布局 */
    ASSERT(offsetof(FcrRoscoExternalSetpoint, yaw_target_heading_rad) == 24);
    ASSERT(offsetof(FcrRoscoExternalSetpoint, r_pitch) == 56);
    ASSERT(offsetof(FcrRoscoExternalSetpoint, collective_pitch_offset_rad) == 88);
    /* S1101 保留接口：追加在末尾，既有字段偏移不变 */
    ASSERT(offsetof(FcrRoscoExternalSetpoint, ct_tracking_error) == 104);

    /* FarmStateFrame — S1001/S1002 改名后偏移不变 */
    ASSERT(offsetof(FarmStateFrame, state_frame_seq) == 8);
    ASSERT(offsetof(FarmStateFrame, state_sample_sim_time_s) == 16);
    ASSERT(offsetof(FarmStateFrame, turbines) == 64);

    /* FcrFarmTurbineState — 新增 S1011/S1012/S1015-S1017 字段存在性 */
    ASSERT(offsetof(FcrFarmTurbineState, estimated_wind_mps) < sizeof(FcrFarmTurbineState));
    ASSERT(offsetof(FcrFarmTurbineState, ct_hat) < sizeof(FcrFarmTurbineState));
    ASSERT(offsetof(FcrFarmTurbineState, wind_source) < sizeof(FcrFarmTurbineState));
    ASSERT(offsetof(FcrFarmTurbineState, wind_quality_flags) < sizeof(FcrFarmTurbineState));
    ASSERT(offsetof(FcrFarmTurbineState, turbine_status_flags) < sizeof(FcrFarmTurbineState));

    /* 8 字节对齐不变量：64 位/双精度成员必须 8 字节对齐 */
    ASSERT((offsetof(FcrTurbineCommand, yaw_seq) % 8) == 0);
    ASSERT((offsetof(FcrTurbineCommand, command_basis_sim_time_s) % 8) == 0);
    ASSERT((offsetof(FarmCommandFrame, turbines) % 8) == 0);
    ASSERT((sizeof(FcrTurbineCommand) % 8) == 0);
}

/* 协议版本门禁：v2 为当前版本，v1 兼容宏保留 */
static void test_abi_protocol_version(void)
{
    ASSERT(FCR_PROTOCOL_VERSION == FCR_PROTOCOL_VERSION_V2);
    ASSERT(FCR_PROTOCOL_VERSION_V2 == 2u);
    ASSERT(FCR_PROTOCOL_VERSION_V1 == 1u);
}

/* W-09 容量统一守卫：线协议容量（冻结 ABI）与所有内部消费者一致，
 * 版本化宏（*_V1）指向当前 ABI 值。 */
static void test_abi_capacity_unified(void)
{
    /* 版本化容量宏：当前版本 = V1 冻结值 */
    ASSERT(FCR_CAPACITY_VERSION == 1u);
    ASSERT(FCR_MAX_TURBINES == FCR_MAX_TURBINES_V1);
    ASSERT(FCR_MAX_OBSERVATION_POINTS == FCR_MAX_OBSERVATION_POINTS_V1);
    ASSERT(FCR_MAX_TURBINES_V1 == 64u);
    ASSERT(FCR_MAX_OBSERVATION_POINTS_V1 == 64u);

    /* bridge 容量与协议机组容量一致（原 128 → 64） */
    ASSERT(FCR_BRIDGE_MAX_TURBINES == FCR_MAX_TURBINES);

    /* legacy AWAE 帧容量与线协议观察点容量一致（原 1024 → 64） */
    ASSERT(FCR_AWAE_MAX_POINTS == FCR_MAX_OBSERVATION_POINTS);
}

void test_abi_register(void)
{
    RUN_TEST("abi_sizes", test_abi_sizes);
    RUN_TEST("abi_offsets", test_abi_offsets);
    RUN_TEST("abi_protocol_version", test_abi_protocol_version);
    RUN_TEST("abi_capacity_unified", test_abi_capacity_unified);
}
