/*
 * fcr_mock_command_source.c — 场控命令源实现（替代 Modbus/RCP 的测试输入）
 * 详见 fcr_mock_command_source.h。
 */
#include "fcr_mock_command_source.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FCR_MOCK_MAX_EVENTS 256

struct FcrMockSource {
    uint32_t n_turbines;
    FcrMockEvent events[FCR_MOCK_MAX_EVENTS];
    int n_events;

    FarmCommandFrame frame;
    uint64_t seq;
    double last_t_s;
    int has_frame;
};

/* ---------------- 生命周期 ---------------- */
FcrMockSource *fcr_mock_source_create(uint32_t n_turbines)
{
    FcrMockSource *s;
    if (n_turbines == 0 || n_turbines > FCR_MAX_TURBINES) return NULL;
    s = (FcrMockSource *)calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->n_turbines = n_turbines;
    s->last_t_s = -1.0;
    return s;
}

void fcr_mock_source_destroy(FcrMockSource *src)
{
    free(src);
}

void fcr_mock_source_reset(FcrMockSource *src)
{
    if (!src) return;
    src->n_events = 0;
    src->seq = 0;
    src->last_t_s = -1.0;
    src->has_frame = 0;
    memset(&src->frame, 0, sizeof(src->frame));
}

int fcr_mock_source_add_event(FcrMockSource *src, const FcrMockEvent *ev)
{
    if (!src || !ev) return -1;
    if (src->n_events >= FCR_MOCK_MAX_EVENTS) return -1;
    if (ev->turbine_id < 0 || ev->turbine_id > (int32_t)src->n_turbines) return -1;
    src->events[src->n_events] = *ev;
    return src->n_events++;
}

/* ---------------- CSV 加载 ---------------- */
static int parse_double_field(const char *tok, double *out)
{
    char *end = NULL;
    double v;
    if (!tok || tok[0] == '\0') { *out = NAN; return 0; }   /* 空 = 不改变 */
    v = strtod(tok, &end);
    if (end == tok) { *out = NAN; return 0; }
    *out = v;
    return 1;
}

int fcr_mock_source_load_csv(FcrMockSource *src, const char *path)
{
    FILE *f;
    char line[512];
    int loaded = 0, lineno = 0;

    if (!src || !path) return -1;
    f = fopen(path, "r");
    if (!f) return -1;

    while (fgets(line, sizeof(line), f)) {
        char *p = line, *tok[8];
        int ntok = 0;
        FcrMockEvent ev;

        ++lineno;
        /* 去行尾 */
        line[strcspn(line, "\r\n")] = '\0';
        /* 跳过空行与注释 */
        if (line[0] == '\0' || line[0] == '#') continue;
        /* 跳过表头（首字符非数字/符号） */
        if (lineno == 1 && !(line[0] == '-' || line[0] == '+' || line[0] == '.' ||
                             (line[0] >= '0' && line[0] <= '9'))) continue;

        /* 按逗号切分（最多 8 列） */
        while (ntok < 8) {
            tok[ntok++] = p;
            p = strchr(p, ',');
            if (!p) break;
            *p++ = '\0';
        }
        if (ntok < 2) continue;

        memset(&ev, 0, sizeof(ev));
        ev.yaw_target_heading_rad = NAN;
        ev.induction_ref = NAN;
        parse_double_field(tok[0], &ev.t_s);
        {
            double tid = 0.0;
            parse_double_field(ntok > 1 ? tok[1] : NULL, &tid);
            ev.turbine_id = (int32_t)tid;
        }
        {
            double v = 1.0;
            parse_double_field(ntok > 2 ? tok[2] : NULL, &v);
            ev.control_enable = (v == v) ? (uint32_t)(v != 0.0) : 1u;
        }
        {
            double v = (double)FCR_CONTROL_MODE_PRC;
            parse_double_field(ntok > 3 ? tok[3] : NULL, &v);
            ev.control_mode = (v == v) ? (uint32_t)v : FCR_CONTROL_MODE_PRC;
        }
        parse_double_field(ntok > 4 ? tok[4] : NULL, &ev.yaw_target_heading_rad);
        parse_double_field(ntok > 5 ? tok[5] : NULL, &ev.induction_ref);
        {
            double v = 0.0;
            parse_double_field(ntok > 6 ? tok[6] : NULL, &v);
            ev.ttl_s = (v == v) ? v : 0.0;
        }
        {
            double v = 0.0;
            parse_double_field(ntok > 7 ? tok[7] : NULL, &v);
            ev.duration_s = (v == v) ? v : 0.0;
        }
        if (ev.t_s != ev.t_s) continue;              /* 无时刻则丢弃 */
        if (fcr_mock_source_add_event(src, &ev) < 0) break;
        ++loaded;
    }
    fclose(f);
    return loaded;
}

/* ---------------- 生效选择 ---------------- */
/* 对某台机、某通道，选 t_s 最大且仍有效的事件；返回下标或 -1。
 * 通道类型：0 = 任意（用于 control_enable/mode），1 = 偏航，2 = 诱导 */
static int pick_event(const FcrMockSource *src, int32_t tid, double t, int channel)
{
    int i, best = -1;
    double best_t = -1.0;
    for (i = 0; i < src->n_events; ++i) {
        const FcrMockEvent *e = &src->events[i];
        int match_turbine = (e->turbine_id == 0 || e->turbine_id == tid);
        int active;
        if (!match_turbine) continue;
        if (e->t_s > t + 1e-9) continue;                      /* 尚未生效 */
        if (e->duration_s > 0.0 && t > e->t_s + e->duration_s + 1e-9) continue;  /* 已过期 */
        if (channel == 1 && !isfinite(e->yaw_target_heading_rad)) continue;
        if (channel == 2 && !isfinite(e->induction_ref)) continue;
        active = (e->t_s >= best_t);
        if (active) { best = i; best_t = e->t_s; }
    }
    return best;
}

/* ---------------- 每 SCADA 边界合成一帧 ---------------- */
int fcr_mock_source_tick(FcrMockSource *src, FcrRtHandle *h, double t_low)
{
    uint32_t i;
    if (!src) return -1;
    if (!isfinite(t_low)) return -1;

    memset(&src->frame, 0, sizeof(src->frame));
    src->frame.protocol_version = FCR_PROTOCOL_VERSION;
    src->frame.command_frame_seq = ++src->seq;
    src->frame.n_turbines = src->n_turbines;
    src->frame.receive_time_s = t_low;

    for (i = 0; i < src->n_turbines; ++i) {
        int32_t tid = (int32_t)(i + 1);
        FcrTurbineCommand *c = &src->frame.turbines[i];
        int e_common = pick_event(src, tid, t_low, 0);
        int e_yaw = pick_event(src, tid, t_low, 1);
        int e_ind = pick_event(src, tid, t_low, 2);

        /* 通用字段：取最近一条作用于该机的事件；无事件则关闭外部控制 */
        if (e_common >= 0) {
            c->control_enable = src->events[e_common].control_enable;
            c->control_mode = (uint16_t)src->events[e_common].control_mode;
        } else {
            c->control_enable = 0;
            c->control_mode = FCR_CONTROL_MODE_BASELINE;
        }
        c->command_basis_sim_time_s = t_low;      /* 保持新鲜，避免 stale */
        c->requested_effective_sim_time_s = 0.0;  /* 立即生效 */

        if (e_yaw >= 0) {
            c->yaw_valid = 1;
            c->yaw_seq = src->seq;
            c->yaw_target_heading_rad = src->events[e_yaw].yaw_target_heading_rad;
            c->yaw_ttl_s = src->events[e_yaw].ttl_s;
        }
        if (e_ind >= 0) {
            c->induction_valid = 1;
            c->induction_seq = src->seq;
            c->induction_ref = src->events[e_ind].induction_ref;
            c->induction_ttl_s = src->events[e_ind].ttl_s;
        }
    }

    src->last_t_s = t_low;
    src->has_frame = 1;

    if (h) return fcr_rt_set_command(h, &src->frame);
    return 0;
}

int fcr_mock_source_frame(const FcrMockSource *src, FarmCommandFrame *out)
{
    if (!src || !out || !src->has_frame) return -1;
    *out = src->frame;
    return 0;
}

double fcr_mock_source_last_time_s(const FcrMockSource *src)
{
    return src ? src->last_t_s : -1.0;
}

uint64_t fcr_mock_source_frame_seq(const FcrMockSource *src)
{
    return src ? src->seq : 0;
}
