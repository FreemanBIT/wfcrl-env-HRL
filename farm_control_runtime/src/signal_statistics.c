/*
 * signal_statistics.c — 1 s 窗口信号统计（Phase 5）
 *
 * 对 10 ms 高频序列做 1 s 窗口的 instant/mean/RMS/min/max 统计。
 * 环形缓冲（窗口闭合后输出统计并继续）。窗口时间由 sim_time 判定：
 * 当 sim_time 越过窗口边界时输出上一窗口结果。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

struct FcrSignalStatistics {
    double   *buf;              /* 环形缓冲 */
    uint32_t  cap;              /* 容量（>= window/dt） */
    uint32_t  head;
    uint32_t  count;
    double    window_s;
    double    window_start_s;   /* 当前窗口起始时间 */
    FcrSignalStats out;         /* 上一个闭合窗口结果 */
    uint32_t  out_valid;
    double    last_t;
};

FcrSignalStatistics *fcr_signal_stat_create(double window_s, uint32_t capacity)
{
    FcrSignalStatistics *s;
    if (window_s <= 0.0) return NULL;
    if (capacity == 0) capacity = 128;
    s = (FcrSignalStatistics *)calloc(1, sizeof(FcrSignalStatistics));
    if (!s) return NULL;
    s->buf = (double *)calloc(capacity, sizeof(double));
    if (!s->buf) { free(s); return NULL; }
    s->cap = capacity;
    s->window_s = window_s;
    s->window_start_s = 0.0;
    s->last_t = -1.0;
    return s;
}

void fcr_signal_stat_destroy(FcrSignalStatistics *s)
{
    if (!s) return;
    free(s->buf);
    free(s);
}

void fcr_signal_stat_reset(FcrSignalStatistics *s)
{
    if (!s) return;
    s->head = 0;
    s->count = 0;
    s->window_start_s = 0.0;
    s->out_valid = 0;
    s->last_t = -1.0;
    memset(&s->out, 0, sizeof(s->out));
}

/* 计算当前缓冲窗口统计 */
static void compute_stats(FcrSignalStatistics *s, FcrSignalStats *out)
{
    uint32_t i;
    double sum = 0.0, sumsq = 0.0, mn = 1e300, mx = -1e300;
    double x;
    if (s->count == 0) {
        memset(out, 0, sizeof(*out));
        return;
    }
    for (i = 0; i < s->count; ++i) {
        x = s->buf[(s->head + s->cap - s->count + i) % s->cap];
        sum += x;
        sumsq += x * x;
        if (x < mn) mn = x;
        if (x > mx) mx = x;
    }
    out->mean = sum / (double)s->count;
    out->rms = sqrt(sumsq / (double)s->count);
    out->min = mn;
    out->max = mx;
    out->n_samples = s->count;
    out->valid = 1;
}

/* 更新：sim_time 单调递推；跨窗口时输出上一窗口统计并开始新窗口。
 * 返回 1 表示有窗口刚闭合（可读取 out），否则 0。 */
int fcr_signal_stat_update(FcrSignalStatistics *s, double sim_time_s, double x)
{
    int closed = 0;
    if (!s) return 0;
    if (s->last_t < 0.0) {
        s->window_start_s = sim_time_s;
    }
    /* 跨窗口边界：闭合当前窗口并输出 */
    while (s->count > 0 && (sim_time_s - s->window_start_s) >= s->window_s) {
        compute_stats(s, &s->out);
        s->out.instant = s->buf[(s->head + s->cap - 1) % s->cap];
        s->out_valid = 1;
        closed = 1;
        /* 开始新窗口：扔掉旧样本 */
        s->window_start_s += s->window_s;
        s->head = 0;
        s->count = 0;
    }
    /* 采样入环 */
    s->buf[s->head] = x;
    s->head = (s->head + 1) % s->cap;
    if (s->count < s->cap) s->count++;
    s->out.instant = x;   /* instant 始终为最新 */
    s->last_t = sim_time_s;
    return closed;
}

int fcr_signal_stat_get(const FcrSignalStatistics *s, FcrSignalStats *out)
{
    FcrSignalStats cur;
    if (!s || !out) return -1;
    if (s->out_valid) {
        *out = s->out;
    } else if (s->count > 0) {
        /* 窗口未闭合：实时计算当前缓冲的即时统计（partial window） */
        compute_stats((FcrSignalStatistics *)s, &cur);
        cur.valid = 1;
        *out = cur;
    } else {
        memset(out, 0, sizeof(*out));
        return 0;
    }
    out->instant = s->buf[(s->head + s->cap - 1) % s->cap];
    return 1;
}