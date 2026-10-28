/*
 * test_mock_command_source.c — 命令源（Modbus/RCP 替代）测试
 *
 * 覆盖：事件表合成、全场/单机作用域、通道独立性、duration 过期、
 *       CSV 加载、与 fcr_rt_set_command 的联调（setpoint 生效）。
 */
#include "test_framework.h"
#include "fcr_mock_command_source.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static FcrMockEvent mk_event(double t, int32_t tid, int en, double yaw, double ind,
                             double dur)
{
    FcrMockEvent e;
    memset(&e, 0, sizeof(e));
    e.t_s = t;
    e.turbine_id = tid;
    e.control_enable = (uint32_t)en;
    e.control_mode = FCR_CONTROL_MODE_PRC;
    e.yaw_target_heading_rad = yaw;
    e.induction_ref = ind;
    e.ttl_s = 0.0;
    e.duration_s = dur;
    return e;
}

/* ---- 全场事件：3 台机同时拿到同一偏航目标 ---- */
static void test_mock_all_turbines(void)
{
    FcrMockSource *s = fcr_mock_source_create(3);
    FcrMockEvent e;
    FarmCommandFrame fr;
    int i;

    ASSERT(s != NULL);
    e = mk_event(0.0, 0, 1, 0.1, NAN, 0.0);
    ASSERT(fcr_mock_source_add_event(s, &e) >= 0);

    ASSERT(fcr_mock_source_tick(s, NULL, 0.0) == 0);
    ASSERT(fcr_mock_source_frame(s, &fr) == 0);
    ASSERT(fr.n_turbines == 3);
    ASSERT(fr.command_frame_seq == 1);
    for (i = 0; i < 3; ++i) {
        ASSERT(fr.turbines[i].control_enable == 1);
        ASSERT(fr.turbines[i].yaw_valid == 1);
        ASSERT_NEAR(fr.turbines[i].yaw_target_heading_rad, 0.1, 1e-12);
        ASSERT(fr.turbines[i].induction_valid == 0);   /* 本事件未下发诱导 */
    }
    fcr_mock_source_destroy(s);
}

/* ---- 单机作用域 + 通道独立 ---- */
static void test_mock_per_turbine_channels(void)
{
    FcrMockSource *s = fcr_mock_source_create(3);
    FcrMockEvent e;
    FarmCommandFrame fr;

    ASSERT(s != NULL);
    e = mk_event(0.0, 1, 1, 0.0, 0.20, 0.0);      /* T1：偏航 0 + 诱导 0.2 */
    ASSERT(fcr_mock_source_add_event(s, &e) >= 0);
    e = mk_event(0.0, 2, 1, 0.15, NAN, 0.0);       /* T2：仅偏航 */
    ASSERT(fcr_mock_source_add_event(s, &e) >= 0);

    ASSERT(fcr_mock_source_tick(s, NULL, 0.0) == 0);
    ASSERT(fcr_mock_source_frame(s, &fr) == 0);

    ASSERT(fr.turbines[0].yaw_valid == 1);
    ASSERT(fr.turbines[0].induction_valid == 1);
    ASSERT_NEAR(fr.turbines[0].induction_ref, 0.20, 1e-12);

    ASSERT(fr.turbines[1].yaw_valid == 1);
    ASSERT_NEAR(fr.turbines[1].yaw_target_heading_rad, 0.15, 1e-12);
    ASSERT(fr.turbines[1].induction_valid == 0);   /* T2 未下发诱导 */

    ASSERT(fr.turbines[2].yaw_valid == 0);         /* T3 无事件 */
    ASSERT(fr.turbines[2].control_enable == 0);    /* 默认关闭外部控制 */
    fcr_mock_source_destroy(s);
}

/* ---- 时间语义：未到时刻不生效、duration 到期失效 ---- */
static void test_mock_timing_and_duration(void)
{
    FcrMockSource *s = fcr_mock_source_create(2);
    FcrMockEvent e;
    FarmCommandFrame fr;

    ASSERT(s != NULL);
    e = mk_event(5.0, 0, 1, 0.2, NAN, 0.0);        /* t=5 起生效 */
    ASSERT(fcr_mock_source_add_event(s, &e) >= 0);
    e = mk_event(8.0, 0, 1, NAN, 0.25, 2.0);       /* t=8 起、仅 2 s 有效 */
    ASSERT(fcr_mock_source_add_event(s, &e) >= 0);

    /* t=4：两事件都未生效 */
    ASSERT(fcr_mock_source_tick(s, NULL, 4.0) == 0);
    ASSERT(fcr_mock_source_frame(s, &fr) == 0);
    ASSERT(fr.turbines[0].yaw_valid == 0);
    ASSERT(fr.turbines[0].induction_valid == 0);

    /* t=5：偏航生效 */
    ASSERT(fcr_mock_source_tick(s, NULL, 5.0) == 0);
    ASSERT(fcr_mock_source_frame(s, &fr) == 0);
    ASSERT(fr.turbines[0].yaw_valid == 1);

    /* t=8：诱导生效 */
    ASSERT(fcr_mock_source_tick(s, NULL, 8.0) == 0);
    ASSERT(fcr_mock_source_frame(s, &fr) == 0);
    ASSERT(fr.turbines[0].induction_valid == 1);

    /* t=11：诱导已过期（8+2），偏航仍在 */
    ASSERT(fcr_mock_source_tick(s, NULL, 11.0) == 0);
    ASSERT(fcr_mock_source_frame(s, &fr) == 0);
    ASSERT(fr.turbines[0].induction_valid == 0);
    ASSERT(fr.turbines[0].yaw_valid == 1);
    fcr_mock_source_destroy(s);
}

/* ---- CSV 加载：表头跳过、空值表示"不改变" ---- */
static void test_mock_csv_load(void)
{
    const char *path = "test_mock_cmd.csv";
    FILE *f;
    FcrMockSource *s = fcr_mock_source_create(2);
    FarmCommandFrame fr;
    int n;

    ASSERT(s != NULL);
    f = fopen(path, "w");
    ASSERT(f != NULL);
    fprintf(f, "t_s,turbine_id,control_enable,control_mode,yaw_target_heading_rad,"
               "induction_ref,ttl_s,duration_s\n");
    fprintf(f, "0,0,1,1,0.10,,0,0\n");       /* 全场偏航，诱导空 */
    fprintf(f, "3,2,1,1,,0.20,0,0\n");       /* T2 诱导 */
    fclose(f);

    n = fcr_mock_source_load_csv(s, path);
    ASSERT(n == 2);
    ASSERT(fcr_mock_source_tick(s, NULL, 3.0) == 0);
    ASSERT(fcr_mock_source_frame(s, &fr) == 0);
    ASSERT(fr.turbines[0].yaw_valid == 1);
    ASSERT(fr.turbines[0].induction_valid == 0);
    ASSERT(fr.turbines[1].induction_valid == 1);
    ASSERT_NEAR(fr.turbines[1].induction_ref, 0.20, 1e-12);
    remove(path);
    fcr_mock_source_destroy(s);
}

/* ---- 与场控模块联调：注入的命令经 CommandStore/TTL 生效为 setpoint ---- */
static void test_mock_feeds_runtime(void)
{
    FcrRtConfig cfg;
    FcrRtHandle *h;
    FcrMockSource *s;
    FcrMockEvent e;
    FcrRtStatusInfo si;
    FcrRoscoExternalSetpoint sp;
    FcrRtTimeCtx t;
    FcrRoscoRawState rs;
    int k;

    memset(&cfg, 0, sizeof(cfg));
    cfg.n_turbines = 2;
    cfg.dt_high_s = 0.01;
    cfg.dt_low_s = 1.0;
    cfg.dt_scada_s = 1.0;
    cfg.default_yaw_ttl_s = 60.0;
    cfg.default_induction_ttl_s = 5.0;
    cfg.stale_threshold_s = 5.0;
    cfg.flow_stale_threshold_s = 3.0;

    h = fcr_rt_create();
    ASSERT(h != NULL);
    ASSERT(fcr_rt_configure(h, &cfg) == 0);
    ASSERT(fcr_rt_init(h) == 0);

    s = fcr_mock_source_create(2);
    ASSERT(s != NULL);
    e = mk_event(0.0, 0, 1, 0.0, 0.20, 0.0);
    ASSERT(fcr_mock_source_add_event(s, &e) >= 0);

    /* 100 × 10 ms = 1 s */
    for (k = 1; k <= 100; ++k) {
        memset(&rs, 0, sizeof(rs));
        rs.turbine_id = 1;
        rs.valid = 1;
        rs.nacelle_heading_rad = 0.0;   /* 与目标一致 */
        rs.gen_power_w = 2.0e6;
        ASSERT(fcr_rt_set_state(h, 1, &rs) == 0);

        if (k % 100 == 0) ASSERT(fcr_mock_source_tick(s, h, (double)(k / 100)) == 0);

        memset(&t, 0, sizeof(t));
        t.n_low = k / 100;
        t.n_fast_low = k % 100;
        t.dt_low = 1.0;
        t.dt_high = 0.01;
        t.n_high_low = 100;
        t.t_low = (double)k * 0.01;
        ASSERT(fcr_rt_step(h, &t, &si) == FCR_RT_OK);
    }

    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.induction_enable == 1);
    ASSERT_NEAR(sp.ct_ref, 4.0 * 0.20 * 0.80, 1e-9);   /* Ct = 4a(1-a)（legacy 映射） */
    ASSERT(sp.yaw_seq == 1);

    fcr_mock_source_destroy(s);
    fcr_rt_destroy(h);
}

void test_mock_command_source_register(void)
{
    RUN_TEST("mock_all_turbines", test_mock_all_turbines);
    RUN_TEST("mock_per_turbine_channels", test_mock_per_turbine_channels);
    RUN_TEST("mock_timing_and_duration", test_mock_timing_and_duration);
    RUN_TEST("mock_csv_load", test_mock_csv_load);
    RUN_TEST("mock_feeds_runtime", test_mock_feeds_runtime);
}
