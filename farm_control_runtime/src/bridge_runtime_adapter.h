/*
 * bridge_runtime_adapter.h — Bridge core ↔ runtime 适配（DR-3/DR-4 接入）
 *
 * 把 runtime 的 CommandStore/StateStore 状态转换为 fcr_bridge_core 的
 * 输入（FcrBridgeCmd / FcrBridgeMeas），并调 fcr_bridge_core_step 生成
 * setpoint，再与基础 setpoint（yaw 通道保留 CommandStore/YawActionManager
 * 的 DR-5 输出）合并。本文件在 src/ 下以访问内部类型 FcrTurbineCommandState。
 *
 * 设计（最小侵入 + 可回退）：
 *   - bridge_enable=0：完全走 legacy InductionSupervisor 路径（默认兼容）
 *   - bridge_enable=1：诱导 setpoint 由 bridge core 生成（Mode 0/1/2/3），
 *     yaw 通道仍由 CommandStore/YawActionManager 生成（已实现 DR-5 绝对目标）
 */
#ifndef FCR_BRIDGE_RUNTIME_ADAPTER_H
#define FCR_BRIDGE_RUNTIME_ADAPTER_H

#include "fcr_bridge_core.h"
#include "fcr_state_types.h"
#include "fcr_internal.h"   /* FcrTurbineCommandState */

#ifdef __cplusplus
extern "C" {
#endif

/* FcrTurbineCommandState（runtime 命令状态）→ FcrBridgeCmd（bridge 输入） */
void fcr_bridge_adapter_cmd_from_state(const FcrTurbineCommandState *tc,
                                       uint32_t induction_mode,
                                       uint32_t alloc_w_pitch,
                                       uint32_t alloc_w_speed,
                                       uint32_t alloc_w_torque,
                                       FcrBridgeCmd *out);

/* FcrRoscoRawState（ROSCO 快速状态）→ FcrBridgeMeas（bridge 测量输入） */
void fcr_bridge_adapter_meas_from_fast(const FcrRoscoRawState *fast,
                                       FcrBridgeMeas *out);

/* WIN-11: full measurement synthesis from StateStore (fast + extra + flow) */
void fcr_bridge_adapter_meas_from_state(const FcrRoscoRawState *fast,
                                        const FcrTurbineExtraState *extra,
                                        const FcrFlowState *flow,
                                        int32_t turbine_id,
                                        FcrBridgeMeas *out);

/* ------------------------------------------------------------------ */
/* S07 Scheduler_to_Bridge（S0701-S0710）—— Bridge 的唯一输入路径      */
/* ------------------------------------------------------------------ */

/* runtime 命令状态 → ICD v1.5 FcrBridgeInput。
 * control_enable=0 时不下发任何通道有效标记（S0704 回退语义）。 */
void fcr_bridge_adapter_input_from_state(const FcrTurbineCommandState *tc,
                                         int32_t turbine_id,
                                         double bridge_step_sim_time_s,
                                         FcrBridgeInput *out);

/* FcrBridgeInput → bridge core 内部命令。
 * TTL 不在 S07 信号组内（属 runtime 内部语义），故由调用方从
 * CommandStore 显式传入；0.0 表示不设 TTL（由 CommandStore 单独保证失效）。 */
void fcr_bridge_adapter_cmd_from_input(const FcrBridgeInput *in,
                                       uint32_t induction_mode,
                                       uint32_t alloc_w_pitch,
                                       uint32_t alloc_w_speed,
                                       uint32_t alloc_w_torque,
                                       double induction_ttl_s,
                                       double yaw_ttl_s,
                                       FcrBridgeCmd *out);

/* ------------------------------------------------------------------ */
/* S06 TurbineWindEstimate_to_Bridge（S0601-S0604）                    */
/* ------------------------------------------------------------------ */

/* 把风速估计注入 bridge 测量。
 * 返回 0 = 已采用（S0601 覆盖 ambient_upstream_wind_mps）；
 * 返回 -1 = 未采用（质量标志未置 OK，或估计超龄 → 置 STALE 标志）。
 * 无论采用与否，S0603/S0604 都会回填到 meas 供诊断。 */
int fcr_bridge_adapter_meas_apply_wind_estimate(FcrBridgeMeas *meas,
                                                const FcrTurbineWindEstimate *est,
                                                double sim_time_s,
                                                double stale_threshold_s);

/* 调 bridge core step 并把诱导结果合并进基础 setpoint（yaw 保留 base）。 */
int fcr_bridge_adapter_apply(FcrBridgeCore *core, int32_t turbine_id,
                             double sim_time_s,
                             const FcrBridgeCmd *cmd,
                             const FcrBridgeMeas *meas,
                             FcrRoscoExternalSetpoint *base);

#ifdef __cplusplus
}
#endif

#endif /* FCR_BRIDGE_RUNTIME_ADAPTER_H */
