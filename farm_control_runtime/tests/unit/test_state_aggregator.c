/*
 * test_state_aggregator.c — 多速率状态聚合测试（Phase 5）
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include <string.h>
#include <math.h>

static void test_aggregate_multi_rate(void)
{
    FcrStateStore *ss = fcr_state_store_create(2);
    FcrStateAggregator *a = fcr_state_aggregator_create(2, 1.0);
    FarmStateFrame fr;
    FcrRoscoFastState f;
    FcrTurbineExtraState e;
    FcrFlowState fl;
    double t;

    ASSERT(ss && a);

    /* 填充 10 ms 序列 1 s（T1 功率 4e6..4.1e6）——发布事件驱动统计 */
    for (t = 0.0; t <= 1.0; t += 0.01) {
        memset(&f, 0, sizeof(f));
        f.turbine_id = 1; f.valid = 1; f.source_time_s = t;
        f.gen_power_w = 4.0e6 + t * 1.0e5;
        f.nacelle_heading_rad = 0.0;
        f.rotor_speed_rad_s = 1.2;
        fcr_state_store_set_fast(ss, 1, &f);
        fcr_state_aggregator_feed_fast(a, 1, &f);
    }
    /* 10 ms extra（T1 推力恒定 4e5） */
    for (t = 0.0; t <= 1.0; t += 0.01) {
        memset(&e, 0, sizeof(e));
        e.turbine_id = 1; e.valid = 1; e.source_time_s = t;
        e.rotor_thrust_n = 4.0e5;
        fcr_state_store_set_extra(ss, &e);
        fcr_state_aggregator_feed_extra(a, &e);
    }
    /* clock */
    fcr_state_store_set_clock(ss, 1.0, 100, 1, FCR_RT_HEALTH_OK);
    /* flow（1 s 快照） */
    memset(&fl, 0, sizeof(fl));
    fl.valid = 1; fl.flow_source_time_s = 1.0; fl.flow_seq = 7;
    fl.n_turbines = 2;
    fl.disk_disturbed_wind_mps[0] = 7.5;
    fl.disk_ambient_wind_mps[0] = 8.0;
    fl.disk_ambient_ti[0] = 0.08;
    fcr_state_store_set_flow(ss, &fl);

    /* 低速帧 */
    fcr_state_aggregator_build_frame(a, ss, &fr);

    ASSERT(fr.protocol_version == FCR_PROTOCOL_VERSION);
    ASSERT(fr.n_turbines == 2);
    ASSERT(fr.fast_step == 100);
    ASSERT(fr.low_step == 1);
    ASSERT(fr.flow_seq == 7);
    ASSERT_NEAR(fr.flow_source_time_s, 1.0, 1e-9);
    ASSERT(fr.turbines[0].valid == 1);
    /* 功率统计：[4.0e6, 4.1e6] mean≈4.05e6 */
    ASSERT_NEAR(fr.turbines[0].gen_power_w.mean, 4.05e6, 2e4);
    ASSERT_NEAR(fr.turbines[0].gen_power_w.min, 4.0e6, 1e3);
    ASSERT_NEAR(fr.turbines[0].gen_power_w.max, 4.1e6, 1e3);
    ASSERT(fr.turbines[0].gen_power_w.n_samples > 0);
    /* 推力统计 */
    ASSERT_NEAR(fr.turbines[0].rotor_thrust_n.mean, 4.0e5, 1e3);
    /* ZOH 流场 */
    ASSERT_NEAR(fr.turbines[0].disk_disturbed_wind_mps, 7.5, 1e-9);
    ASSERT_NEAR(fr.turbines[0].disk_ambient_wind_mps, 8.0, 1e-9);
    ASSERT_NEAR(fr.turbines[0].disk_ambient_ti, 0.08, 1e-9);
    ASSERT_NEAR(fr.turbines[0].flow_source_time_s, 1.0, 1e-9);
    /* T2 无数据 → invalid */
    ASSERT(fr.turbines[1].valid == 0);

    fcr_state_aggregator_destroy(a);
    fcr_state_store_destroy(ss);
}

static void test_aggregate_flow_zoh(void)
{
    FcrStateStore *ss = fcr_state_store_create(1);
    FcrStateAggregator *a = fcr_state_aggregator_create(1, 1.0);
    FarmStateFrame fr;
    FcrRoscoFastState f;
    FcrFlowState fl;

    memset(&f, 0, sizeof(f));
    f.turbine_id = 1; f.valid = 1; f.source_time_s = 5.5;
    f.gen_power_w = 3.6e6;
    fcr_state_store_set_fast(ss, 1, &f);
    fcr_state_store_set_clock(ss, 5.5, 550, 5, FCR_RT_HEALTH_OK);

    memset(&fl, 0, sizeof(fl));
    fl.valid = 1; fl.flow_source_time_s = 5.0; fl.flow_seq = 5;
    fl.n_turbines = 1;
    fl.disk_disturbed_wind_mps[0] = 7.9;
    fcr_state_store_set_flow(ss, &fl);
    fcr_state_aggregator_build_frame(a, ss, &fr);

    /* flow 是 5.0s 的快照：保留 source time（ZOH 语义），不伪造 5.5s */
    ASSERT_NEAR(fr.turbines[0].flow_source_time_s, 5.0, 1e-9);
    ASSERT_NEAR(fr.turbines[0].disk_disturbed_wind_mps, 7.9, 1e-9);
    ASSERT_NEAR(fr.sim_time_s, 5.5, 1e-9);
    ASSERT_NEAR(fr.turbines[0].source_time_s, 5.5, 1e-9);

    fcr_state_aggregator_destroy(a);
    fcr_state_store_destroy(ss);
}

void test_state_aggregator_register(void)
{
    RUN_TEST("aggregate_multi_rate", test_aggregate_multi_rate);
    RUN_TEST("aggregate_flow_zoh", test_aggregate_flow_zoh);
}