/*
 * command_store.c — 命令存储与通道独立维护（Phase 1）
 *
 * 关键规则（对应 PROTOCOL_DECISIONS.md）：
 *   1. yaw / induction 通道完全独立，互不覆盖互不推导；
 *   2. 新 seq 才更新生效值；重复 seq 幂等；乱序 seq（<= 已应用）拒绝；
 *   3. TTL 到期自动失效（yaw 回退使能清除；induction 回退使能清除）；
 *   4. 命令超龄（source_time 距当前 sim_time 超过阈值）置 stale 标志。
 *   5. yaw 绝对目标只在新 seq 生效时用最新机舱方位锁存一次。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>

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

/* 处理单条 turbine 命令（按通道独立）                                 */
static void accept_turbine(FcrCommandStore *cs, const FcrTurbineCommand *cmd,
                           int32_t turbine_id)
{
    FcrTurbineCommandState *tc = fcr_command_store_turbine(cs, turbine_id);
    if (!tc) return;

    /* ---- yaw 通道 ---- */
    if (cmd->yaw_valid) {
        if (cmd->yaw_seq > tc->yaw_seq_rx) {
            /* 新 seq：更新待生效值（目标在 fcr_command_store_refresh 锁存）*/
            tc->yaw_seq_rx = cmd->yaw_seq;
            tc->yaw_delta_rad = cmd->yaw_delta_rad;
            tc->yaw_valid = 1;
            tc->yaw_target_latched = 0;      /* 等待锁存 */
            tc->yaw_effective_low_step = cmd->yaw_effective_low_step;
            tc->yaw_ttl_s = (cmd->yaw_ttl_s > 0.0) ? cmd->yaw_ttl_s : cs->default_yaw_ttl_s;
            tc->yaw_issue_time_s = cmd->source_time_s;
            tc->source_time_s = cmd->source_time_s;
            tc->command_status_flags &= ~FCR_CMD_FLAG_INVALID_SEQ;
        } else if (cmd->yaw_seq == tc->yaw_seq_rx) {
            /* 重复 seq：幂等，仅刷新 ttl/source_time */
            tc->yaw_ttl_s = (cmd->yaw_ttl_s > 0.0) ? cmd->yaw_ttl_s : cs->default_yaw_ttl_s;
            tc->source_time_s = cmd->source_time_s;
            tc->yaw_issue_time_s = cmd->source_time_s;
        } else {
            /* 乱序 seq：拒绝，置诊断位 */
            tc->command_status_flags |= FCR_CMD_FLAG_INVALID_SEQ;
        }
    }

    /* ---- induction 通道 ---- */
    if (cmd->induction_valid) {
        if (cmd->induction_seq > tc->ind_seq_rx) {
            tc->ind_seq_rx = cmd->induction_seq;
            tc->induction_ref = cmd->induction_ref;
            tc->ind_valid = 1;
            tc->ind_effective_low_step = cmd->induction_effective_low_step;
            tc->ind_ttl_s = (cmd->induction_ttl_s > 0.0) ? cmd->induction_ttl_s
                                                         : cs->default_induction_ttl_s;
            tc->ind_issue_time_s = cmd->source_time_s;
            tc->source_time_s = cmd->source_time_s;
            tc->command_status_flags &= ~FCR_CMD_FLAG_INVALID_SEQ;
        } else if (cmd->induction_seq == tc->ind_seq_rx) {
            tc->ind_ttl_s = (cmd->induction_ttl_s > 0.0) ? cmd->induction_ttl_s
                                                         : cs->default_induction_ttl_s;
            tc->source_time_s = cmd->source_time_s;
            tc->ind_issue_time_s = cmd->source_time_s;
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

/* TTL / stale 检查 + yaw 目标锁存（每高速步调用）                     */
void fcr_command_store_refresh(FcrCommandStore *cs, double sim_time_s,
                               const double *heading_now_rad, uint64_t low_step)
{
    uint32_t i;
    if (!cs) return;
    for (i = 0; i < cs->n_turbines; ++i) {
        FcrTurbineCommandState *tc = &cs->t[i];

        /* ---- stale（命令来源时间过老）---- */
        if (tc->source_time_s > 0.0 &&
            (sim_time_s - tc->source_time_s) > cs->stale_threshold_s) {
            tc->command_status_flags |= FCR_CMD_FLAG_YAW_STALE |
                                        FCR_CMD_FLAG_INDUCTION_STALE;
        } else {
            tc->command_status_flags &= ~(FCR_CMD_FLAG_YAW_STALE |
                                          FCR_CMD_FLAG_INDUCTION_STALE);
        }

        /* ---- yaw TTL ---- */
        if (tc->yaw_valid && tc->yaw_ttl_s > 0.0 &&
            (sim_time_s - tc->yaw_issue_time_s) > tc->yaw_ttl_s) {
            tc->yaw_valid = 0;               /* 通道失效（回退 ROSCO） */
            tc->yaw_target_latched = 0;
            tc->command_status_flags |= FCR_CMD_FLAG_YAW_TTL_EXPIRED;
        } else {
            tc->command_status_flags &= ~FCR_CMD_FLAG_YAW_TTL_EXPIRED;
        }

        /* ---- induction TTL ---- */
        if (tc->ind_valid && tc->ind_ttl_s > 0.0 &&
            (sim_time_s - tc->ind_issue_time_s) > tc->ind_ttl_s) {
            tc->ind_valid = 0;
            tc->command_status_flags |= FCR_CMD_FLAG_INDUCTION_TTL_EXPIRED;
        } else {
            tc->command_status_flags &= ~FCR_CMD_FLAG_INDUCTION_TTL_EXPIRED;
        }

        /* ---- yaw 目标锁存：新 seq 且机舱方位已知，只执行一次 ----   */
        if (tc->yaw_valid && !tc->yaw_target_latched) {
            if (heading_now_rad && fcr_is_finite(heading_now_rad[i])) {
                tc->yaw_target_heading =
                    fcr_compute_yaw_target(heading_now_rad[i], tc->yaw_delta_rad);
                tc->yaw_target_latched = 1;
                tc->yaw_seq_applied = tc->yaw_seq_rx;
                if (low_step > 0) tc->yaw_effective_low_step = (int64_t)low_step;
            }
            /* 方位未知：保持未锁存，等待下一高速步 */
        }
    }
}

void fcr_command_store_get_setpoint(const FcrCommandStore *cs, int32_t turbine_id,
                                    FcrRoscoExternalSetpoint *sp)
{
    const FcrTurbineCommandState *tc;
    memset(sp, 0, sizeof(*sp));
    if (!cs || !sp) return;
    tc = (turbine_id >= 1 && turbine_id <= (int32_t)cs->n_turbines)
             ? &cs->t[turbine_id - 1] : NULL;
    if (!tc) return;

    sp->command_status_flags = tc->command_status_flags;
    if (tc->yaw_valid) sp->command_status_flags |= FCR_CMD_FLAG_YAW_VALID;
    if (tc->ind_valid) sp->command_status_flags |= FCR_CMD_FLAG_INDUCTION_VALID;

    /* 外部场控总使能 = 任一通道有效 */
    if (tc->yaw_valid || tc->ind_valid) {
        sp->farm_control_enable = 1;
        sp->valid = 1;
    }

    /* ---- yaw ---- */
    if (tc->yaw_valid && tc->yaw_target_latched) {
        sp->yaw_enable = 1;
        sp->yaw_seq = tc->yaw_seq_applied;
        sp->yaw_target_heading_rad = tc->yaw_target_heading;
    }

    /* ---- induction（Phase 1 透传有效性；Phase 6 由 InductionSupervisor
     *      覆盖 ct_ref/power_ratio/speed_ref_ratio/torque_limit_ratio/
     *      min_pitch_rad 的实际映射） ---- */
    if (tc->ind_valid) {
        sp->induction_enable = 1;
        sp->induction_seq = tc->ind_seq_applied ? tc->ind_seq_applied : tc->ind_seq_rx;
        sp->power_ratio = 1.0;          /* 默认额定（disabled 语义） */
        sp->speed_ref_ratio = 1.0;
        sp->torque_limit_ratio = 1.0;
        sp->ct_ref = 0.0;
        sp->min_pitch_rad = 0.0;
    }

    if (sp->valid) sp->command_status_flags |= FCR_CMD_FLAG_VALID;
}
