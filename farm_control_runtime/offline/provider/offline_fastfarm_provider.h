/*
 * offline_fastfarm_provider.h — Offline FAST.Farm Provider（Phase 4）
 *
 * 在 DISCON DLL 内每 10 ms 从 Bladed DLL avrSWAP 记录读取 OpenFAST 真实量
 * 并通过 fcr_provider_api.h 发布（与 RT Provider 相同的 push 语义）。
 */
#ifndef OFFLINE_FASTFARM_PROVIDER_H
#define OFFLINE_FASTFARM_PROVIDER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化（读取环境变量 FCR_CFG_DT_LOW / FCR_CFG_DT_HIGH；幂等） */
int fcr_offline_provider_init(void);

/* 每 ~10 ms（DISCON 调用）发布：turbine extra 状态 + rt clock */
int fcr_offline_provider_publish_fast(int32_t turbine_id, double sim_time_s,
                                      const float *avrSWAP);

double fcr_offline_provider_dt_low(void);

#ifdef __cplusplus
}
#endif

#endif /* OFFLINE_FASTFARM_PROVIDER_H */
