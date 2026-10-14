/*
 * bridge_runtime_adapter.c — Bridge core ↔ runtime 适配实现（DR-3/DR-4 接入）
 * 本文件在 src/ 下以访问内部类型（FcrTurbineCommandState / FcrRoscoFastState）。
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
    out->source_time_s = tc->source_time_s;
    /* yaw 通道由 CommandStore/YawActionManager 处理（DR-5），bridge 不接管 */
}

/* ROSCO 快速状态 → bridge 测量 */
void fcr_bridge_adapter_meas_from_fast(const FcrRoscoFastState *fast,
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
    base->power_ratio = bs.power_ratio;
    base->speed_ref_ratio = bs.speed_ref_ratio;
    base->torque_limit_ratio = bs.torque_limit_ratio;
    base->min_pitch_rad = bs.min_pitch_rad;
    base->command_status_flags |= bs.command_status_flags;
    return 0;
}
