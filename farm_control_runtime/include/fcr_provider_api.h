/*
 * fcr_provider_api.h — Provider API（Phase 1 冻结）
 *
 * 统一 push 型 Provider 接口。Offline Provider（我方）与未来 RT Provider
 * （RT 团队按 fcr_fastfarm_rt_provider.h 实现）都必须调用同一组函数，
 * 保证数据语义一致。
 *
 * 调用线程约束：Provider 在各自数据就绪线程调用；runtime 内部对状态槽
 * 采用原子替换/锁保护，保证 10 ms 与 1 s 来源不互相阻塞。
 */
#ifndef FCR_PROVIDER_API_H
#define FCR_PROVIDER_API_H

#include "fcr_state_types.h"
#include "fcr_runtime.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 发布实时时钟/元数据（scheduler 或高速主循环，每 10 ms 可更新）        */
int fcr_publish_rt_clock(FcrRuntime *rt,
                         double sim_time_s,
                         uint64_t fast_step,
                         uint64_t low_step,
                         uint32_t rt_health_flags);

/* 发布单机 OpenFAST 额外状态（优选 ~10 ms）                            */
int fcr_publish_turbine_extra_state(FcrRuntime *rt, const FcrTurbineExtraState *st);

/* 发布 AWAE 流场快照（~1 s；两个快照之间 runtime 做 ZOH）              */
int fcr_publish_flow_state(FcrRuntime *rt, const FcrFlowState *flow);

#ifdef __cplusplus
}
#endif

#endif /* FCR_PROVIDER_API_H */
