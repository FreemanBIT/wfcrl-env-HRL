/*
 * test_signal_statistics.c — 1 s 窗口统计测试（Phase 5）
 */
#include "test_framework.h"
#include "fcr_internal.h"
#include <string.h>
#include <math.h>

static void test_constant_signal(void)
{
    FcrSignalStatistics *s = fcr_signal_stat_create(1.0, 128);
    FcrSignalStats st;
    int t;
    ASSERT(s != NULL);
    for (t = 0; t < 100; ++t) {
        fcr_signal_stat_update(s, t * 0.01, 4.5e6);
    }
    fcr_signal_stat_get(s, &st);
    ASSERT(st.n_samples == 100);
    ASSERT_NEAR(st.mean, 4.5e6, 1);
    ASSERT_NEAR(st.rms, 4.5e6, 1);
    ASSERT_NEAR(st.min, 4.5e6, 1);
    ASSERT_NEAR(st.max, 4.5e6, 1);
    ASSERT(st.valid == 1);
    fcr_signal_stat_destroy(s);
}

static void test_varying_signal(void)
{
    FcrSignalStatistics *s = fcr_signal_stat_create(1.0, 256);
    FcrSignalStats st;
    double x;
    int t;
    for (t = 0; t < 1000; ++t) {
        x = 1000.0 * sin(t * 0.01) + 2000.0;
        fcr_signal_stat_update(s, t * 0.01, x);
    }
    fcr_signal_stat_get(s, &st);
    ASSERT(st.valid == 1);
    ASSERT(st.n_samples > 0);
    ASSERT(st.min <= st.mean && st.mean <= st.max);
    ASSERT(st.rms >= 0.0);
    ASSERT(st.max - st.min <= 2000.0 + 1000.0 - (2000.0 - 1000.0) + 1.0);
    fcr_signal_stat_destroy(s);
}

static void test_window_closure_semantics(void)
{
    FcrSignalStatistics *s = fcr_signal_stat_create(1.0, 128);
    double x = 1.0;
    int t, closed;
    for (t = 0; t < 100; ++t) {   /* sim 0.00..0.99 */
        closed = fcr_signal_stat_update(s, t * 0.01, x);
        ASSERT(closed == 0);
        x += 1.0;
    }
    /* sim=1.00 跨过窗口边界 → 闭合 */
    closed = fcr_signal_stat_update(s, 1.00, x);
    ASSERT(closed == 1);
    fcr_signal_stat_destroy(s);
}

void test_signal_statistics_register(void)
{
    RUN_TEST("constant_signal", test_constant_signal);
    RUN_TEST("varying_signal", test_varying_signal);
    RUN_TEST("window_closure_semantics", test_window_closure_semantics);
}
