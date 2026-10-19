/*
 * command_store.c — 命令存储与通道独立维护（ICD v1.5 S0101-S0112）
 *
 * 关键规则（对应 ICD v1.5）：
 *   1. yaw / induction 通道完全独立，互不覆盖互不推导；
 *   2. 新 seq 才更新生效值；重复 seq 幂等；乱序 seq（<= 已应用）拒绝；
 *   3. TTL 到期自动失效；
 *   4. 命令超龄（command_basis_sim_time_s 距当前 sim_time 超过阈值）置 stale 标志。
 *   5. yaw 绝对目标只在新 seq 生效时用最新机舱方位锁存一次。
 *   6. control_enable=0 时回退 ROSCO 原始控制（S0104）。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* debug counters */
uint32_t g_dbg_yaw_accept = 0, g_dbg_ind_accept = 0;
uint32_t g_dbg_yaw_latch = 0, g_dbg_ind_latch = 0;

FcrCommandStore *fcr_command_store_create(uint32_t n_turbines,
                                          double default_yaw_ttl_s,
                                          double default_induction_ttl_s,
                                          double stale_threshold_s,
                                          double dt_low_s)
{
    FcrCommandStore *cs;
    if (n_turbines == 0 || n_turbines > FCR_MAX_TURBINES) return NULL;
    cs = (FcrCommandStore *)calloc(1, sizeof(FcrCommandStore));
    if (!cs) return NULL;
    cs->n_turbines = n_turbines;
    cs->default_yaw_ttl_s = default_yaw_ttl_s;
    cs->default_induction_ttl_s = default_induction_ttl_s;
    cs->stale_threshold_s = stale_threshold_s;
    cs->dt_low_s = dt_low_s;
    return cs;
}

void fcr_command_store_destroy(FcrCommandStore *cs)
{
    if (cs) free(cs);
}

FcrTurbineCommandState *fcr_command_store_turbine(FcrCommandStore *cs, int32_t turbine_id)
{
    if (!cs || turbine_id < 1 || turbine_id > (int32_t)cs->n_turbines) return NULL;
    return &cs->t[turbine_id - 1];
}

/* 处理单条 turbine 命令（通道状态机：IDLE→ACCEPTED→SCHEDULED→ACTIVE→EXPIRED） */
static void accept_turbine(FcrCommandStore *cs, const FcrTurbineCommand *cmd,
                           int32_t turbine_id)
{
    FcrTurbineCommandState *tc = fcr_command_store_turbine(cs, turbine_id);
    if (!tc) return;

    /* ---- 通用控制字段 (S0104, S0111, S0112) ---- */
    tc->control_enable = cmd->control_enable;
    tc->control_mode = cmd->control_mode;
    tc->requested_effective_sim_time_s = cmd->requested_effective_sim_time_s;
    tc->command_basis_sim_time_s = cmd->command_basis_sim_time_s;

    /* ---- yaw 通道 (S0108-S0110) ---- */
    if (cmd->yaw_valid) {
        if (cmd->yaw_seq > tc->yaw_seq_rx) {
            /* 新 seq：更新待生效值，进入 ACCEPTED 状态 */
            tc->yaw_seq_rx = cmd->yaw_seq;
            tc->yaw_target_heading_rad = cmd->yaw_target_heading_rad;
            tc->yaw_valid = 1;
            tc->yaw_target_latched = 1;  /* ICD v1.5: absolute heading latched immediately */
            tc->yaw_ttl_s = (cmd->yaw_ttl_s > 0.0) ? cmd->yaw_ttl_s : cs->default_yaw_ttl_s;
            tc->yaw_issue_time_s = -1.0;
            tc->yaw_basis_sim_time_s = cmd->command_basis_sim_time_s; /* W-13 */
            tc->yaw_state = FCR_CH_ACCEPTED;
            tc->command_status_flags &= ~FCR_CMD_FLAG_INVALID_SEQ;
            g_dbg_yaw_accept++;
        } else if (cmd->yaw_seq == tc->yaw_seq_rx) {
            /* 重复 seq — 幂等，不续命 */
            if (tc->yaw_state == FCR_CH_IDLE || tc->yaw_state == FCR_CH_EXPIRED) {
                tc->yaw_state = FCR_CH_ACCEPTED;
                tc->yaw_issue_time_s = -1.0;
            }
        } else {
            /* 乱序 seq：拒绝 */
            tc->command_status_flags |= FCR_CMD_FLAG_INVALID_SEQ;
        }
    }

    /* ---- induction 通道 (S0105-S0107) ---- */
    if (cmd->induction_valid) {
        if (cmd->induction_seq > tc->ind_seq_rx) {
            tc->ind_seq_rx = cmd->induction_seq;
            tc->induction_ref = cmd->induction_ref;
            tc->ind_valid = 1;
            tc->ind_ttl_s = (cmd->induction_ttl_s > 0.0) ? cmd->induction_ttl_s
                                                         : cs->default_induction_ttl_s;
            tc->ind_issue_time_s = -1.0;
            tc->ind_basis_sim_time_s = cmd->command_basis_sim_time_s; /* W-13 */
            tc->ind_state = FCR_CH_ACCEPTED;
            tc->command_status_flags &= ~FCR_CMD_FLAG_INVALID_SEQ;
            g_dbg_ind_accept++;
        } else if (cmd->induction_seq == tc->ind_seq_rx) {
            /* 重复 seq — 幂等，不续命 */
            if (tc->ind_state == FCR_CH_IDLE || tc->ind_state == FCR_CH_EXPIRED) {
                tc->ind_state = FCR_CH_ACCEPTED;
                tc->ind_issue_time_s = -1.0;
            }
        } else {
            tc->command_status_flags |= FCR_CMD_FLAG_INVALID_SEQ;
        }
    }
}

int fcr_command_store_accept_frame(FcrCommandStore *cs, const FarmCommandFrame *frame)
{
    uint32_t i;
    if (!cs || !frame) return -1;
    if (frame->protocol_version != FCR_PROTOCOL_VERSION) return -2;
    for (i = 0; i < frame->n_turbines && i < cs->n_turbines; ++i) {
        accept_turbine(cs, &frame->turbines[i], (int32_t)(i + 1));
    }
    return 0;
}

/* 单台机状态机推进（每高速步调用） */
void fcr_command_store_refresh_one(FcrCommandStore *cs, int32_t turbine_id,
                                   double sim_time_s, double heading_now_rad,
                                   uint64_t low_step)
{
    FcrTurbineCommandState *tc;
    if (!cs) return;
    tc = fcr_command_store_turbine(cs, turbine_id);
    if (!tc) return;

    /* ---- control_enable 检查 (S0104) ---- */
    if (!tc->control_enable) {
        /* 外部场控禁用：清除所有通道，回退 ROSCO */
        tc->yaw_valid = 0;
        tc->ind_valid = 0;
        tc->yaw_state = FCR_CH_IDLE;
        tc->ind_state = FCR_CH_IDLE;
        tc->command_status_flags |= FCR_CMD_FLAG_CONTROL_DISABLED;
        return;
    }
    tc->command_status_flags &= ~FCR_CMD_FLAG_CONTROL_DISABLED;

    /* ---- W-13: per-channel stale（各通道按自身命令依据时刻独立评估，
     * 不再共享单一 command_basis_sim_time_s——偏航 60s 锁存与诱导 1s 更新
     * 互不串扰；stale 只置标志不直接禁用，保留各通道 TTL 语义） ---- */
    if (tc->yaw_basis_sim_time_s > 0.0 &&
        (sim_time_s - tc->yaw_basis_sim_time_s) > cs->stale_threshold_s) {
        tc->command_status_flags |= FCR_CMD_FLAG_YAW_STALE;
    } else {
        tc->command_status_flags &= ~FCR_CMD_FLAG_YAW_STALE;
    }
    if (tc->ind_basis_sim_time_s > 0.0 &&
        (sim_time_s - tc->ind_basis_sim_time_s) > cs->stale_threshold_s) {
        tc->command_status_flags |= FCR_CMD_FLAG_INDUCTION_STALE;
    } else {
        tc->command_status_flags &= ~FCR_CMD_FLAG_INDUCTION_STALE;
    }

    /* ---- yaw 通道状态机 ---- */
    {
        FcrChannelState st = tc->yaw_state;
        /* ACCEPTED → SCHEDULED（检查 requested_effective_sim_time_s） */
        if (st == FCR_CH_ACCEPTED) {
            if (tc->requested_effective_sim_time_s <= 0.0 ||
                sim_time_s >= tc->requested_effective_sim_time_s) {
                st = FCR_CH_SCHEDULED;
            }
        }
        /* SCHEDULED → ACTIVE（起算 TTL） */
        if (st == FCR_CH_SCHEDULED) {
            tc->yaw_valid = 1;
            tc->yaw_seq_applied = tc->yaw_seq_rx;   /* S0902: 生效序号回读 */
            if (tc->yaw_issue_time_s < 0.0 && tc->yaw_ttl_s > 0.0) {
                tc->yaw_issue_time_s = sim_time_s;
            }
            st = FCR_CH_ACTIVE;
        }
        /* ACTIVE：TTL 检查 */
        if (st == FCR_CH_ACTIVE) {
            if (tc->yaw_valid && tc->yaw_ttl_s > 0.0 &&
                (sim_time_s - tc->yaw_issue_time_s) > tc->yaw_ttl_s) {
                tc->yaw_valid = 0;
                tc->yaw_target_latched = 0;
                st = FCR_CH_EXPIRED;
                tc->command_status_flags |= FCR_CMD_FLAG_YAW_TTL_EXPIRED;
            } else {
                tc->command_status_flags &= ~FCR_CMD_FLAG_YAW_TTL_EXPIRED;
            }
        }
        /* EXPIRED → IDLE */
        if (st == FCR_CH_EXPIRED) {
            st = FCR_CH_IDLE;
        }
        tc->yaw_state = st;
    }

    /* ---- induction 通道状态机（同上） ---- */
    {
        FcrChannelState st = tc->ind_state;
        if (st == FCR_CH_ACCEPTED) {
            if (tc->requested_effective_sim_time_s <= 0.0 ||
                sim_time_s >= tc->requested_effective_sim_time_s) {
                st = FCR_CH_SCHEDULED;
            }
        }
        if (st == FCR_CH_SCHEDULED) {
            tc->ind_valid = 1;
            tc->ind_seq_applied = tc->ind_seq_rx;   /* S0901: 生效序号回读 */
            if (tc->ind_issue_time_s < 0.0 && tc->ind_ttl_s > 0.0) {
                tc->ind_issue_time_s = sim_time_s;
            }
            st = FCR_CH_ACTIVE;
        }
        if (st == FCR_CH_ACTIVE) {
            if (tc->ind_valid && tc->ind_ttl_s > 0.0 &&
                (sim_time_s - tc->ind_issue_time_s) > tc->ind_ttl_s) {
                tc->ind_valid = 0;
                st = FCR_CH_EXPIRED;
                tc->command_status_flags |= FCR_CMD_FLAG_INDUCTION_TTL_EXPIRED;
            } else {
                tc->command_status_flags &= ~FCR_CMD_FLAG_INDUCTION_TTL_EXPIRED;
            }
        }
        if (st == FCR_CH_EXPIRED) {
            st = FCR_CH_IDLE;
        }
        tc->ind_state = st;
    }
}

void fcr_command_store_refresh(FcrCommandStore *cs, double sim_time_s,
                               const double *heading_now_rad,
                               uint64_t low_step)
{
    uint32_t i;
    if (!cs) return;
    for (i = 0; i < cs->n_turbines; ++i) {
        double h = (heading_now_rad) ? heading_now_rad[i] : 0.0;
        fcr_command_store_refresh_one(cs, (int32_t)(i + 1), sim_time_s, h, low_step);
    }
}

void fcr_command_store_get_setpoint(const FcrCommandStore *cs, int32_t turbine_id,
                                    FcrRoscoExternalSetpoint *sp)
{
    const FcrTurbineCommandState *tc;
    if (!cs || !sp) return;
    tc = fcr_command_store_turbine((FcrCommandStore *)cs, turbine_id);
    if (!tc) return;

    memset(sp, 0, sizeof(*sp));
    sp->valid = 1;
    sp->farm_control_enable = tc->control_enable;
    sp->command_status_flags = tc->command_status_flags;

    /* yaw 通道 (S0801-S0802) */
    if (tc->yaw_valid && tc->yaw_state == FCR_CH_ACTIVE) {
        sp->external_yaw_enable = 1;
        sp->yaw_seq = tc->yaw_seq_applied;
        sp->yaw_target_heading_rad = tc->yaw_target_heading_rad;
        sp->command_status_flags |= FCR_CMD_FLAG_YAW_VALID;
    }

    /* induction 通道 (S0803-S0807) */
    if (tc->ind_valid && tc->ind_state == FCR_CH_ACTIVE) {
        sp->induction_enable = 1;
        sp->induction_seq = tc->ind_seq_applied;
        sp->command_status_flags |= FCR_CMD_FLAG_INDUCTION_VALID;
    }
}

void fcr_command_store_debug_counters(uint32_t *yaw_accept, uint32_t *ind_accept,
                                      uint32_t *yaw_latch, uint32_t *ind_latch)
{
    if (yaw_accept) *yaw_accept = g_dbg_yaw_accept;
    if (ind_accept) *ind_accept = g_dbg_ind_accept;
    if (yaw_latch) *yaw_latch = g_dbg_yaw_latch;
    if (ind_latch) *ind_latch = g_dbg_ind_latch;
}
