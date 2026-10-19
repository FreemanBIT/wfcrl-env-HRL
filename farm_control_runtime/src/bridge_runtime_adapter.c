/*
 * bridge_runtime_adapter.c — Bridge core ↔ runtime 适配实现（DR-3/DR-4 接入）
 * 本文件在 src/ 下以访问内部类型（FcrTurbineCommandState / FcrRoscoRawState）。
 */
#include "bridge_runtime_adapter.h"
#include <string.h>

/* 命令状态 → bridge 命令（诱导通道；yaw 暂不接管） */
void fcr_bridge_adapter_cmd_from_state(const FcrTurbineCommandState *tc,
                                       uint32_t induction_mode,
                                       uint32_t alloc_w_pitch,
                                       uint32_t alloc_w_speed,
                                       uint32_t alloc_w_torque,
                                       FcrBridgeCmd *out) {
    memset(out, 0, sizeof(*out));
    if (!tc || !out) return;
    out->turbine_id = 1;  /* 调用方随后按机修正 */
    out->valid = 1;
    /* 诱导通道 */
    out->induction_mode = induction_mode;
    out->induction_valid = tc->ind_valid;
    out->induction_ref = tc->induction_ref;
    out->induction_seq = tc->ind_seq_rx;
    out->induction_ttl_s = tc->ind_ttl_s;
    /* 控制分配权重（千分比 → 归一化 [0,1]；调用方保证和=1000 或全 0） */
    out->alloc_w_pitch = alloc_w_pitch / 1000.0;
    out->alloc_w_speed = alloc_w_speed / 1000.0;
    out->alloc_w_torque = alloc_w_torque / 1000.0;
    out->source_time_s = tc->command_basis_sim_time_s;
    /* yaw 通道由 CommandStore/YawActionManager 处理（DR-5），bridge 不接管 */
}

/* ROSCO 快速状态 → bridge 测量 */
void fcr_bridge_adapter_meas_from_fast(const FcrRoscoRawState *fast,
                                       FcrBridgeMeas *out) {
    memset(out, 0, sizeof(*out));
    if (!fast || !out) return;
    out->turbine_id = 1;  /* 调用方随后按机修正 */
    out->valid = fast->valid;
    out->source_time_s = fast->source_time_s;
    out->rotor_speed_rad_s = fast->rotor_speed_rad_s;
    out->generator_speed_rad_s = fast->gen_speed_rad_s;
    out->rotor_thrust_n = 0.0;        /* fast 状态无推力（extra provider 提供） */
    out->blade_pitch_rad[0] = fast->blade_pitch_rad[0];
    out->blade_pitch_rad[1] = fast->blade_pitch_rad[1];
    out->blade_pitch_rad[2] = fast->blade_pitch_rad[2];
    out->nacelle_heading_rad = fast->nacelle_heading_rad;
    out->disk_wind_speed_mps = 0.0;   /* 由 WindMapper/AWAE 提供（后续接线） */
    out->ambient_upstream_wind_mps = fast->hub_wind_speed_mps; /* 近似上游参考 */
    out->disk_background_wind_mps = 0.0;
    out->disk_turbulence = 0.0;
    out->gen_power_w = fast->gen_power_w;
}

/* WIN-11: synthesize bridge measurement from full StateStore state */
void fcr_bridge_adapter_meas_from_state(const FcrRoscoRawState *fast,
                                        const FcrTurbineExtraState *extra,
                                        const FcrFlowState *flow,
                                        int32_t turbine_id,
                                        FcrBridgeMeas *out) {
    memset(out, 0, sizeof(*out));
    if (!fast || !out) return;
    out->turbine_id = turbine_id;
    out->valid = fast->valid;
    out->source_time_s = fast->source_time_s;
    out->rotor_speed_rad_s = fast->rotor_speed_rad_s;
    out->generator_speed_rad_s = fast->gen_speed_rad_s;
    /* WIN-11: thrust from extra provider (real measurement) */
    out->rotor_thrust_n = (extra && extra->valid) ? extra->rotor_aero_thrust_n : 0.0;
    /* WIN-11: disk wind from flow (AWAE), fallback to hub wind */
    if (flow && flow->valid && turbine_id <= (int32_t)flow->n_turbines) {
        out->disk_wind_speed_mps = flow->disk_disturbed_wind_mps[turbine_id - 1];
        out->disk_background_wind_mps = flow->disk_ambient_wind_mps[turbine_id - 1];
        out->disk_turbulence = flow->disk_ambient_ti[turbine_id - 1];
    } else {
        out->disk_wind_speed_mps = fast->hub_wind_speed_mps;
    }
    out->ambient_upstream_wind_mps = fast->hub_wind_speed_mps;
    memcpy(out->blade_pitch_rad, fast->blade_pitch_rad, sizeof(out->blade_pitch_rad));
    out->nacelle_heading_rad = fast->nacelle_heading_rad;
    out->gen_power_w = fast->gen_power_w;
}

/* ------------------------------------------------------------------ */
/* S07 Scheduler_to_Bridge（S0701-S0710）                              */
/* ------------------------------------------------------------------ */

void fcr_bridge_adapter_input_from_state(const FcrTurbineCommandState *tc,
                                         int32_t turbine_id,
                                         double bridge_step_sim_time_s,
                                         FcrBridgeInput *out)
{
    memset(out, 0, sizeof(*out));
    if (!tc || !out) return;

    out->bridge_step_sim_time_s = bridge_step_sim_time_s;         /* S0701 */
    out->turbine_id = (uint32_t)turbine_id;                       /* S0703 */
    out->control_enable = tc->control_enable ? 1u : 0u;           /* S0704 */
    out->control_mode = tc->control_mode;                         /* S0709 */
    out->command_basis_sim_time_s = tc->command_basis_sim_time_s; /* S0710 */

    /* S0704=0：回退 ROSCO 原控制，不得下发任何通道有效标记 */
    if (tc->control_enable) {
        out->induction_ref = tc->induction_ref;                   /* S0705 */
        out->induction_seq = tc->ind_seq_rx;                      /* S0706 */
        out->yaw_target_heading_rad = tc->yaw_target_heading_rad; /* S0707 */
        out->yaw_seq = tc->yaw_seq_rx;                            /* S0708 */
        out->bridge_command_valid_flags |= FCR_BRIDGE_CMD_CONTROL_ENABLED;
        if (tc->yaw_valid) out->bridge_command_valid_flags |= FCR_BRIDGE_CMD_YAW_VALID;
        if (tc->ind_valid) out->bridge_command_valid_flags |= FCR_BRIDGE_CMD_INDUCTION_VALID;
        if (tc->control_mode == FCR_CONTROL_MODE_CLOSEDLOOP)
            out->bridge_command_valid_flags |= FCR_BRIDGE_CMD_CLOSEDLOOP_REQ;
    }
    if (out->bridge_command_valid_flags)
        out->bridge_command_valid_flags |= FCR_BRIDGE_CMD_VALID;
}

void fcr_bridge_adapter_cmd_from_input(const FcrBridgeInput *in,
                                       uint32_t induction_mode,
                                       uint32_t alloc_w_pitch,
                                       uint32_t alloc_w_speed,
                                       uint32_t alloc_w_torque,
                                       double induction_ttl_s,
                                       double yaw_ttl_s,
                                       FcrBridgeCmd *out)
{
    memset(out, 0, sizeof(*out));
    if (!in || !out) return;

    out->turbine_id = (int32_t)in->turbine_id;                    /* S0703 */
    out->valid = (in->bridge_command_valid_flags & FCR_BRIDGE_CMD_VALID) ? 1u : 0u;
    out->induction_mode = induction_mode;                          /* DR-3 */

    /* 诱导通道（S0705/S0706） */
    out->induction_valid =
        (in->bridge_command_valid_flags & FCR_BRIDGE_CMD_INDUCTION_VALID) ? 1u : 0u;
    out->induction_ref = in->induction_ref;
    out->induction_seq = in->induction_seq;
    out->induction_ttl_s = induction_ttl_s;

    /* 偏航通道（S0707/S0708）—— bridge 只读不接管（DR-5 由 CommandStore 输出） */
    out->yaw_valid = (in->bridge_command_valid_flags & FCR_BRIDGE_CMD_YAW_VALID) ? 1u : 0u;
    out->yaw_target_heading_rad = in->yaw_target_heading_rad;
    out->yaw_seq = in->yaw_seq;
    out->yaw_ttl_s = yaw_ttl_s;

    /* 控制分配（千分比 → 归一化 [0,1]） */
    out->alloc_w_pitch = alloc_w_pitch / 1000.0;
    out->alloc_w_speed = alloc_w_speed / 1000.0;
    out->alloc_w_torque = alloc_w_torque / 1000.0;

    /* S0710：命令依据的仿真时刻（不得用墙钟替换） */
    out->source_time_s = in->command_basis_sim_time_s;
}

/* ------------------------------------------------------------------ */
/* S06 TurbineWindEstimate_to_Bridge（S0601-S0604）                    */
/* ------------------------------------------------------------------ */

int fcr_bridge_adapter_meas_apply_wind_estimate(FcrBridgeMeas *meas,
                                                const FcrTurbineWindEstimate *est,
                                                double sim_time_s,
                                                double stale_threshold_s)
{
    if (!meas || !est) return -1;

    /* S0603/S0604 始终回填（诊断可见），但仅在质量 OK 时才采用数值 */
    meas->wind_source = est->wind_source;
    meas->wind_quality_flags = est->wind_quality_flags;

    if (!(est->wind_quality_flags & FCR_WIND_QUALITY_OK)) return -1;

    /* S0602：估计所对应的仿真时间；超龄则不采用并置 STALE */
    if (stale_threshold_s > 0.0 &&
        (sim_time_s - est->estimate_sim_time_s) > stale_threshold_s) {
        meas->wind_quality_flags |= FCR_WIND_QUALITY_STALE;
        return -1;
    }

    meas->ambient_upstream_wind_mps = est->estimated_wind_mps;     /* S0601 */
    return 0;
}

/* 调 bridge core step，诱导结果合并进基础 setpoint（yaw 保留 base） */
int fcr_bridge_adapter_apply(FcrBridgeCore *core, int32_t turbine_id,
                             double sim_time_s,
                             const FcrBridgeCmd *cmd,
                             const FcrBridgeMeas *meas,
                             FcrRoscoExternalSetpoint *base) {
    FcrRoscoExternalSetpoint bs;
    if (!core || !cmd || !meas || !base) return -1;
    if (fcr_bridge_core_set_command(core, cmd) != 0) return -1;
    if (fcr_bridge_core_set_meas(core, meas) != 0) return -1;
    memset(&bs, 0, sizeof(bs));
    if (fcr_bridge_core_step(core, turbine_id, sim_time_s, &bs) != 0) return -1;
    /* 合并：诱导部分用 bridge 输出；yaw 部分保留 base（CommandStore） */
    base->induction_enable = bs.induction_enable;
    base->induction_seq = bs.induction_seq;
    base->ct_ref = bs.ct_ref;
    base->r_pitch = bs.r_pitch;
    base->r_speed = bs.r_speed;
    base->r_torque = bs.r_torque;
    base->min_pitch_ref_rad = bs.min_pitch_ref_rad;
    base->collective_pitch_offset_rad = bs.collective_pitch_offset_rad;
    base->command_status_flags |= bs.command_status_flags;
    return 0;
}
