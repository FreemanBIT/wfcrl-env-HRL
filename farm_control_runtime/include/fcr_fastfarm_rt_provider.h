/*
 * fcr_fastfarm_rt_provider.h — FAST.Farm RT Provider 入口（Phase 1 ABI / Phase 10 ICD 冻结）
 *
 * 责任方：FAST.Farm RT 团队按本头文件实现 RT 侧 Provider，
 *         我方（wfcrl-env-HRL）实现在线/离线验证与 ICD 语义冻结。
 *
 * RT Provider 必须复现 Offline Provider 已验证的数据语义（单位/坐标/
 * source time/seq），不得重新定义一套同名变量。逐标量语义见
 * docs/farm_control/FASTFARM_RT_PROVIDER_ICD.md（Phase 10 冻结，
 * 以 fastfarm_rt_interface_variables.csv 为基线）。
 *
 * 实现要点（RT 团队）：
 *   - 在 FAST.Farm RT scheduler 的 10 ms 高速循环 / 1 s 低速循环中调用
 *     下方 push 函数（与 runtime 共享内存，不得跨实时线程互锁）；
 *   - fcr_publish_rt_clock：每 10 ms 或 1 s 事件；
 *   - fcr_publish_turbine_extra_state：OpenFAST/AeroDyn 等模块计算后；
 *   - fcr_publish_flow_state：AWAE 快照完成后（1 s）。
 *   - 缺值必须置 valid=0 / NaN 语义按 ICD，禁止发送伪造占位。
 */
#ifndef FCR_FASTFARM_RT_PROVIDER_H
#define FCR_FASTFARM_RT_PROVIDER_H

#include "fcr_runtime.h"
#include "fcr_state_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 以下函数是 fcr_provider_api.h 中 runtime 侧 API 的 RT 实现契约：
 * 函数签名与 fcr_provider_api.h 完全一致（Phase 1 冻结），
 * 语义逐标量冻结见 ICD（Phase 10）。
 */

/* 实时时钟/元数据：sim_time（s）、fast_step（10 ms 步序）、
 * low_step（1 s 步序）、rt_health_flags（FCR_RT_HEALTH_*）            */
int fcr_publish_rt_clock(FcrRuntime *rt,
                         double sim_time_s,
                         uint64_t fast_step,
                         uint64_t low_step,
                         uint32_t rt_health_flags);

/* 单机 OpenFAST 额外状态（rotor_thrust/shaft_torque/tower_base loads/
 * Ct/Cp/load points），详见 FcrTurbineExtraState                        */
int fcr_publish_turbine_extra_state(FcrRuntime *rt, const FcrTurbineExtraState *st);

/* AWAE 流场快照（ambient/disturbed U/V/W at observation points、
 * point TI、disk ambient/disturbed wind、disk ambient TI）             */
int fcr_publish_flow_state(FcrRuntime *rt, const FcrFlowState *flow);

#ifdef __cplusplus
}
#endif

#endif /* FCR_FASTFARM_RT_PROVIDER_H */
