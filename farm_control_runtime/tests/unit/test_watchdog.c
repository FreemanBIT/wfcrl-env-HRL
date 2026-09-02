/*
 * test_watchdog.c — 健康标志/流场超龄测试（Phase 1）
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include <string.h>

static void test_watchdog_flow_stale(void)
{
    FcrWatchdog *wd = fcr_watchdog_create(3.0);
    FcrStateStore *ss = fcr_state_store_create(1);
    FcrFlowState flow;
    memset(&flow, 0, sizeof(flow));
    flow.valid = 1;
    flow.flow_source_time_s = 10.0;
    flow.flow_seq = 1;
    fcr_state_store_set_flow(ss, &flow);
    fcr_state_store_set_clock(ss, 11.0, 100, 1, FCR_RT_HEALTH_OK);

    /* 11.0 - 10.0 = 1 s < 3 s → OK */
    fcr_watchdog_update(wd, 11.0, ss, NULL);
    ASSERT((fcr_watchdog_health(wd) & FCR_RT_HEALTH_FLOW_STALE) == 0);

    /* 15.0 - 10.0 = 5 s > 3 s → FLOW_STALE */
    fcr_watchdog_update(wd, 15.0, ss, NULL);
    ASSERT((fcr_watchdog_health(wd) & FCR_RT_HEALTH_FLOW_STALE) != 0);

    fcr_watchdog_destroy(wd);
    fcr_state_store_destroy(ss);
}

static void test_watchdog_clock_not_seen(void)
{
    FcrWatchdog *wd = fcr_watchdog_create(3.0);
    FcrStateStore *ss = fcr_state_store_create(1);
    fcr_watchdog_update(wd, 0.0, ss, NULL);
    /* 时钟未就绪 → 不报 OK */
    ASSERT(fcr_watchdog_health(wd) == 0u);
    fcr_watchdog_destroy(wd);
    fcr_state_store_destroy(ss);
}

void test_watchdog_register(void)
{
    RUN_TEST("watchdog_flow_stale", test_watchdog_flow_stale);
    RUN_TEST("watchdog_clock_not_seen", test_watchdog_clock_not_seen);
}