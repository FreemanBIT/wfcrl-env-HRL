/*
 * fcr_rt_adapter.c — 场控交互模块的实时平台（调度器）接入适配层
 *
 * 把 farm_control_runtime 包装成"调度器可调用"的模块（要求 §2.2/§3/§5/§6）：
 *   - 生命周期：create → configure → init → step → reset → stop → destroy
 *   - 时间：RTimeCtx 只读传入；本模块不推进/不修改调度器时间
 *   - 数据：set_* 输入 / get_* 输出；传输由调度器负责（本模块无 socket）
 *   - 状态：同步返回 FcrRtStatus + 详情（ErrorCode/ErrorSource/Message/Timestamp）
 *   - 周期：step 每 10 ms；SCADA（对外交换）按 dt_scada 边界（计数器实现，
 *     不绑定 DT_Low——DT_SCADA 变化时仍正确）
 *
 * 红线：step 内无 malloc / 无 I/O / 无阻塞。
 */
#include "fcr_rt.h"
#include "fcr_internal.h"
#include "awae_frame_builder.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

struct FcrRtHandle {
    FcrRtConfig cfg;
    FcrRuntime *rt;

    int inited;
    int stopped;

    /* SCADA 边界（计数器；与 DT_Low 解耦） */
    uint64_t step_index;       /* 每次 step() 自增 */
    uint32_t steps_per_low;    /* = round(dt_low / dt_high) */
    uint32_t steps_per_scada;  /* = round(dt_scada / dt_high) */

    /* 待落地命令（调度器 1 s 送入；下一个 10 ms 步生效） */
    FarmCommandFrame pending;
    int pending_valid;

    /* 输出 */
    FarmStateFrame last_frame;
    int output_valid;

    /* 状态 */
    FcrRtStatusInfo last_status;
    double last_t_low;

    /* TTL 到期边沿检测（每机 1 bit；≤64 机） */
    uint64_t prev_yaw_ttl;
    uint64_t prev_ind_ttl;
};

/* ---------------- 工具 ---------------- */
static int status_rank(FcrRtStatus s)
{
    switch (s) {
    case FCR_RT_OK:              return 0;
    case FCR_RT_WARNING:         return 1;
    case FCR_RT_STALE_DATA:      return 2;
    case FCR_RT_TIMEOUT:         return 3;
    case FCR_RT_INVALID_INPUT:   return 4;
    case FCR_RT_NUMERICAL_ERROR: return 5;
    case FCR_RT_COMM_ERROR:      return 6;
    case FCR_RT_FATAL:           return 7;
    case FCR_RT_NOT_READY:       return 8;
    }
    return 0;
}

static void set_status(FcrRtHandle *h, FcrRtStatusInfo *si,
                       FcrRtStatus st, uint16_t src, int32_t code, const char *msg)
{
    if (!si) si = &h->last_status;
    si->status = st;
    si->error_source = src;
    si->error_code = code;
    si->timestamp_s = h->last_t_low;
    if (msg) {
        strncpy(si->message, msg, sizeof(si->message) - 1);
        si->message[sizeof(si->message) - 1] = '\0';
    } else {
        si->message[0] = '\0';
    }
}

static void bump_status(FcrRtHandle *h, FcrRtStatusInfo *si,
                        FcrRtStatus st, uint16_t src, int32_t code, const char *msg)
{
    if (status_rank(st) > status_rank(si->status))
        set_status(h, si, st, src, code, msg);
}

/* 状态帧（S10）——仅 1 s 边界产生，由调度器取走 */
static int build_output(FcrRtHandle *h)
{
    if (!h->rt) return -1;
    if (fcr_runtime_build_state_frame(h->rt, &h->last_frame) != 0) return -1;
    h->output_valid = 1;
    return 0;
}

/* ---------------- 生命周期 ---------------- */
FcrRtHandle *fcr_rt_create(void)
{
    FcrRtHandle *h = (FcrRtHandle *)calloc(1, sizeof(FcrRtHandle));
    return h;
}

int fcr_rt_configure(FcrRtHandle *h, const FcrRtConfig *cfg)
{
    double n_low_steps, n_scada_steps;
    if (!h || !cfg) return -1;
    if (h->inited) return -3;                    /* 已初始化：先 reset/stop */
    if (cfg->n_turbines == 0 || cfg->n_turbines > FCR_MAX_TURBINES) return -2;
    if (!(cfg->dt_high_s > 0.0) || !(cfg->dt_low_s > 0.0) || !(cfg->dt_scada_s > 0.0))
        return -2;

    n_low_steps = cfg->dt_low_s / cfg->dt_high_s;
    n_scada_steps = cfg->dt_scada_s / cfg->dt_high_s;
    /* 必须是整数步（dt_high 能整除 dt_low / dt_scada） */
    if (fabs(n_low_steps - floor(n_low_steps + 0.5)) > 1e-6) return -2;
    if (fabs(n_scada_steps - floor(n_scada_steps + 0.5)) > 1e-6) return -2;

    h->cfg = *cfg;
    h->steps_per_low = (uint32_t)(n_low_steps + 0.5);
    h->steps_per_scada = (uint32_t)(n_scada_steps + 0.5);
    if (h->steps_per_scada == 0) return -2;
    return 0;
}

int fcr_rt_init(FcrRtHandle *h)
{
    FcrRuntimeConfig rc;
    if (!h) return -1;
    if (h->inited) return -3;                    /* 重复 init 拒绝（要求 §13） */
    if (h->cfg.n_turbines == 0) return -2;       /* 未 configure */

    memset(&rc, 0, sizeof(rc));
    rc.n_turbines = h->cfg.n_turbines;
    rc.dt_high_s = h->cfg.dt_high_s;
    rc.dt_low_s = h->cfg.dt_low_s;
    rc.default_yaw_ttl_s = h->cfg.default_yaw_ttl_s;
    rc.default_induction_ttl_s = h->cfg.default_induction_ttl_s;
    rc.stale_threshold_s = h->cfg.stale_threshold_s;
    rc.flow_stale_threshold_s = h->cfg.flow_stale_threshold_s;
    rc.transport = NULL;                         /* 传输归调度器 */
    rc.bridge_enable = h->cfg.bridge_enable;
    rc.bridge_induction_mode = h->cfg.bridge_induction_mode;
    rc.bridge_alloc_w_pitch = h->cfg.bridge_alloc_w_pitch;
    rc.bridge_alloc_w_speed = h->cfg.bridge_alloc_w_speed;
    rc.bridge_alloc_w_torque = h->cfg.bridge_alloc_w_torque;
    rc.bridge_ind_a_max = h->cfg.bridge_ind_a_max;
    rc.bridge_ind_a_min = h->cfg.bridge_ind_a_min;

    h->rt = fcr_runtime_create(&rc);
    if (!h->rt) return -2;
    h->inited = 1;
    h->stopped = 0;
    h->step_index = 0;
    h->pending_valid = 0;
    h->output_valid = 0;
    h->last_t_low = -1.0;
    h->prev_yaw_ttl = 0;
    h->prev_ind_ttl = 0;
    set_status(h, &h->last_status, FCR_RT_OK, FCR_RT_SRC_NONE, 0, "");
    return 0;
}

int fcr_rt_reset(FcrRtHandle *h)
{
    if (!h || !h->rt) return -1;
    if (fcr_runtime_reset(h->rt) != 0) return -1;
    h->stopped = 0;
    h->step_index = 0;
    h->pending_valid = 0;
    h->output_valid = 0;
    h->last_t_low = -1.0;
    h->prev_yaw_ttl = 0;
    h->prev_ind_ttl = 0;
    memset(&h->last_frame, 0, sizeof(h->last_frame));
    set_status(h, &h->last_status, FCR_RT_OK, FCR_RT_SRC_NONE, 0, "");
    return 0;
}

int fcr_rt_stop(FcrRtHandle *h)
{
    if (!h) return -1;
    h->stopped = 1;
    return 0;
}

void fcr_rt_destroy(FcrRtHandle *h)
{
    if (!h) return;
    if (h->rt) fcr_runtime_destroy(h->rt);
    free(h);
}

/* ---------------- 输入 ---------------- */
int fcr_rt_set_command(FcrRtHandle *h, const FarmCommandFrame *frame)
{
    if (!h || !frame) return -1;
    if (!h->inited || h->stopped) return -2;
    if (frame->protocol_version != FCR_PROTOCOL_VERSION) return -2;
    if (frame->n_turbines == 0 || frame->n_turbines > h->cfg.n_turbines) return -2;
    h->pending = *frame;
    h->pending_valid = 1;
    return 0;
}

int fcr_rt_set_state(FcrRtHandle *h, int32_t turbine_id, const FcrRoscoRawState *st)
{
    if (!h || !st || !h->rt) return -1;
    if (fcr_state_store_set_fast(h->rt->ss, turbine_id, st) != 0) return -2;
    fcr_runtime_feed_fast(h->rt, turbine_id, st);   /* 1 s 统计样本 */
    return 0;
}

int fcr_rt_set_extra(FcrRtHandle *h, const FcrTurbineExtraState *st)
{
    if (!h || !st || !h->rt) return -1;
    if (fcr_state_store_set_extra(h->rt->ss, st) != 0) return -2;
    fcr_runtime_feed_extra_stats(h->rt, st);
    return 0;
}

int fcr_rt_set_flow(FcrRtHandle *h, const FcrFlowState *flow)
{
    if (!h || !flow || !h->rt) return -1;
    return fcr_state_store_set_flow(h->rt->ss, flow);
}

int fcr_rt_set_wind_estimate(FcrRtHandle *h, const FcrTurbineWindEstimate *est)
{
    if (!h || !est || !h->rt) return -1;
    return fcr_state_store_set_wind_estimate(h->rt->ss, est);
}

int fcr_rt_set_awae_catalog(FcrRtHandle *h, const FcrAWAECatalogFrame *cat)
{
    FcrAWAECatalog c;
    uint32_t i, n;
    if (!h || !cat || !h->rt) return -1;
    n = cat->n_points;
    if (n > FCR_MAX_OBSERVATION_POINTS) n = FCR_MAX_OBSERVATION_POINTS;
    fcr_awae_catalog_reset(&c, n, cat->entries[0].point_catalog_version);
    for (i = 0; i < n; ++i) {
        fcr_awae_catalog_set_point(&c, i, cat->entries[i].point_id,
                                   cat->entries[i].point_position_xyz_m[0],
                                   cat->entries[i].point_position_xyz_m[1],
                                   cat->entries[i].point_position_xyz_m[2]);
    }
    return fcr_state_store_set_awae_catalog(h->rt->ss, &c);
}

/* ---------------- 步进 ---------------- */
static FcrRtStatus eval_status(FcrRtHandle *h, FcrRtStatusInfo *si,
                               uint64_t *yaw_ttl_now, uint64_t *ind_ttl_now)
{
    uint32_t i, n = h->cfg.n_turbines;
    uint64_t yaw_ttl = 0, ind_ttl = 0;

    for (i = 0; i < n; ++i) {
        FcrTurbineCommandState *tc =
            fcr_command_store_turbine(h->rt->cs, (int32_t)(i + 1));
        const FcrRoscoRawState *f = &h->rt->ss->fast[i];
        if (!tc) continue;

        /* 非法值（NaN/Inf 被 CommandStore 拒绝）→ INVALID_INPUT */
        if (tc->command_status_flags & FCR_CMD_FLAG_INVALID_VALUE)
            bump_status(h, si, FCR_RT_INVALID_INPUT, FCR_RT_SRC_COMMAND, 10,
                        "command value invalid (NaN/Inf) rejected");

        /* TTL 到期：边沿触发（释放外部控制的那一步报一次） */
        if (tc->command_status_flags & FCR_CMD_FLAG_YAW_TTL_EXPIRED) yaw_ttl |= (1ull << i);
        if (tc->command_status_flags & FCR_CMD_FLAG_INDUCTION_TTL_EXPIRED) ind_ttl |= (1ull << i);
        if ((yaw_ttl & ~h->prev_yaw_ttl) || (ind_ttl & ~h->prev_ind_ttl))
            bump_status(h, si, FCR_RT_TIMEOUT, FCR_RT_SRC_COMMAND, 20,
                        "command TTL expired: external control released (D3)");

        /* 命令超龄（仍保持上一有效值，D1）→ STALE_DATA（持续） */
        if (tc->command_status_flags & (FCR_CMD_FLAG_YAW_STALE | FCR_CMD_FLAG_INDUCTION_STALE))
            bump_status(h, si, FCR_RT_STALE_DATA, FCR_RT_SRC_COMMAND, 30,
                        "command stale: holding last valid value (D1)");

        /* 数值异常：有效状态量出现 NaN/Inf → NUMERICAL_ERROR（D4） */
        if (f->valid && (!isfinite(f->gen_power_w) || !isfinite(f->nacelle_heading_rad)))
            bump_status(h, si, FCR_RT_NUMERICAL_ERROR, FCR_RT_SRC_STATE, 40,
                        "state NaN/Inf detected");
    }
    *yaw_ttl_now = yaw_ttl;
    *ind_ttl_now = ind_ttl;
    return si->status;
}

FcrRtStatus fcr_rt_step(FcrRtHandle *h, const FcrRtTimeCtx *t,
                        FcrRtStatusInfo *out_status)
{
    FcrRtStatusInfo local;
    FcrRtStatusInfo *si = out_status ? out_status : &local;
    uint64_t yaw_ttl = 0, ind_ttl = 0;
    uint64_t fast_step;

    if (!h || !h->rt || !h->inited) {
        if (h) set_status(h, si, FCR_RT_NOT_READY, FCR_RT_SRC_INTERNAL, 1, "not initialized");
        return FCR_RT_NOT_READY;
    }
    if (h->stopped) {
        set_status(h, si, FCR_RT_NOT_READY, FCR_RT_SRC_INTERNAL, 2, "module stopped");
        return FCR_RT_NOT_READY;
    }

    /* ---- 时间上下文校验（要求 §4.1/§14）---- */
    if (!t) {
        set_status(h, si, FCR_RT_INVALID_INPUT, FCR_RT_SRC_TIME, 3, "null time context");
        return FCR_RT_INVALID_INPUT;
    }
    if (t->n_low < 0 || !(t->dt_low > 0.0) || !isfinite(t->t_low)) {
        set_status(h, si, FCR_RT_INVALID_INPUT, FCR_RT_SRC_TIME, 4, "invalid time context");
        return FCR_RT_INVALID_INPUT;
    }
    if (h->last_t_low >= 0.0 && t->t_low < h->last_t_low - 1e-9) {
        set_status(h, si, FCR_RT_INVALID_INPUT, FCR_RT_SRC_TIME, 5, "time went backwards");
        return FCR_RT_INVALID_INPUT;
    }

    /* ---- 1) 命令落地（调度器 1 s 送入 → 本步生效）---- */
    if (h->pending_valid) {
        fcr_command_store_accept_frame(h->rt->cs, &h->pending);
        h->pending_valid = 0;
    }

    /* ---- 2) 时钟同步（调度器为唯一时间源）---- */
    fast_step = (uint64_t)t->n_low * (uint64_t)h->steps_per_low +
                (uint64_t)((t->n_fast_low > 0) ? t->n_fast_low : 0);
    fcr_state_store_set_clock(h->rt->ss, t->t_low, fast_step,
                              (uint64_t)t->n_low, FCR_RT_HEALTH_OK);

    /* ---- 3) 10 ms 高速步（命令 refresh / TTL / stale / setpoint）---- */
    if (fcr_runtime_step_high(h->rt, t->t_low) != 0) {
        set_status(h, si, FCR_RT_FATAL, FCR_RT_SRC_INTERNAL, 6, "step_high failed");
        return FCR_RT_FATAL;
    }

    /* ---- 4) SCADA 边界：1 s 帧（计数器，与 DT_Low 解耦）---- */
    h->step_index++;
    if (h->steps_per_scada > 0 && (h->step_index % h->steps_per_scada) == 0) {
        if (build_output(h) != 0) {
            set_status(h, si, FCR_RT_FATAL, FCR_RT_SRC_INTERNAL, 7, "output build failed");
            return FCR_RT_FATAL;
        }
    }

    /* ---- 5) 状态判定 ---- */
    set_status(h, si, FCR_RT_OK, FCR_RT_SRC_NONE, 0, "");
    eval_status(h, si, &yaw_ttl, &ind_ttl);
    h->prev_yaw_ttl = yaw_ttl;
    h->prev_ind_ttl = ind_ttl;
    h->last_t_low = t->t_low;
    h->last_status = *si;
    return si->status;
}

/* ---------------- 输出 ---------------- */
int fcr_rt_get_output(FcrRtHandle *h, FarmStateFrame *out)
{
    if (!h || !out) return -1;
    if (!h->output_valid) return 0;
    *out = h->last_frame;
    h->output_valid = 0;
    return 1;
}

int fcr_rt_get_setpoint(FcrRtHandle *h, int32_t turbine_id,
                        FcrRoscoExternalSetpoint *out)
{
    if (!h || !out || !h->rt) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)h->cfg.n_turbines) return -2;
    *out = h->rt->setpoint[turbine_id - 1];
    return 0;
}

int fcr_rt_get_awae(FcrRtHandle *h, FcrAWAEObservationFrame *obs,
                    FcrAWAECatalogFrame *cat)
{
    if (!h || !h->rt) return -1;
    if (cat)
        fcr_awae_build_catalog_frame(&h->rt->ss->awae_cat, h->rt->ss->low_step, cat);
    if (obs)
        fcr_awae_build_observation_frame(&h->rt->ss->flow, &h->rt->ss->awae_cat,
                                         h->rt->ss->awae_extra_quality_flags, obs);
    return 0;
}

int fcr_rt_get_status(const FcrRtHandle *h, FcrRtStatusInfo *out)
{
    if (!h || !out) return -1;
    *out = h->last_status;
    return 0;
}

const char *fcr_rt_status_str(FcrRtStatus s)
{
    switch (s) {
    case FCR_RT_OK:              return "OK";
    case FCR_RT_WARNING:         return "WARNING";
    case FCR_RT_STALE_DATA:      return "STALE_DATA";
    case FCR_RT_INVALID_INPUT:   return "INVALID_INPUT";
    case FCR_RT_NOT_READY:       return "NOT_READY";
    case FCR_RT_TIMEOUT:         return "TIMEOUT";
    case FCR_RT_NUMERICAL_ERROR: return "NUMERICAL_ERROR";
    case FCR_RT_COMM_ERROR:      return "COMM_ERROR";
    case FCR_RT_FATAL:           return "FATAL";
    }
    return "UNKNOWN";
}
