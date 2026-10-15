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

/* FcrRoscoFastState（ROSCO 快速状态）→ FcrBridgeMeas（bridge 测量输入） */
void fcr_bridge_adapter_meas_from_fast(const FcrRoscoFastState *fast,
                                       FcrBridgeMeas *out);

/* WIN-11: full measurement synthesis from StateStore (fast + extra + flow) */
void fcr_bridge_adapter_meas_from_state(const FcrRoscoFastState *fast,
                                        const FcrTurbineExtraState *extra,
                                        const FcrFlowState *flow,
                                        int32_t turbine_id,
                                        FcrBridgeMeas *out);

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
