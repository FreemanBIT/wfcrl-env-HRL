/*
 * test_abi.c — ABI 尺寸/对齐冻结检查（Phase 1）
 * ABI 变化必须同步更新本文件与 ICD；CI 中断即表明 ABI 破坏。
 */
#include "test_framework.h"
#include "fcr_protocol.h"
#include "fcr_state_types.h"
#include "fcr_transport.h"
#include "fcr_rosco_api.h"
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
    /* 冻结尺寸（64 位，double/uint64 对齐 8；由 abi_dump 实测 2026-xx） */
    CHECK_ABI(FcrTurbineCommand, 88);
    CHECK_ABI(FarmCommandFrame, 5672);
    CHECK_ABI(FcrSignalStats, 48);
    CHECK_ABI(FcrRoscoFastState, 232);
    CHECK_ABI(FcrRoscoExternalSetpoint, 96);
    CHECK_ABI(FcrTurbineExtraState, 840);
    CHECK_ABI(FcrFlowState, 5152);
    CHECK_ABI(FcrFarmTurbineState, 360);
    CHECK_ABI(FarmStateFrame, 23096);
}

static void test_abi_offsets(void)
{
    /* 关键字段偏移（帮助发现误改） */
    ASSERT(offsetof(FcrTurbineCommand, yaw_seq) == 8);
    ASSERT(offsetof(FcrTurbineCommand, induction_ref) == 40);
    ASSERT(offsetof(FcrRoscoFastState, nacelle_heading_rad) == 64);
    ASSERT(offsetof(FcrRoscoExternalSetpoint, yaw_target_heading_rad) == 24);
}

void test_abi_register(void)
{
    RUN_TEST("abi_sizes", test_abi_sizes);
    RUN_TEST("abi_offsets", test_abi_offsets);
}