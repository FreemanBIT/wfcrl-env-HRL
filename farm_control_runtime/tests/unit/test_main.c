/*
 * test_main.c — farm_control_runtime 单元测试入口
 * 编译全部 test_*.c（含本文件）→ 运行。
 */
#include "test_framework.h"

void test_angle_convention_register(void);
void test_command_store_register(void);
void test_abi_register(void);
void test_watchdog_register(void);
void test_rosco_api_register(void);
void test_yaw_action_manager_register(void);
void test_offline_provider_register(void);
void test_signal_statistics_register(void);
void test_state_aggregator_register(void);
void test_induction_supervisor_register(void);

int main(void)
{
    int failed = 0;
    test_angle_convention_register();
    test_command_store_register();
    test_abi_register();
    test_watchdog_register();
    test_rosco_api_register();
    test_yaw_action_manager_register();
    test_offline_provider_register();
    test_signal_statistics_register();
    test_state_aggregator_register();
    test_induction_supervisor_register();
    failed += test_run_all("farm_control_runtime unit tests");
    return failed ? 1 : 0;
}