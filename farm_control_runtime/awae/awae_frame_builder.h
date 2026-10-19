/*
 * awae_frame_builder.h — ICD v1.5 GG-04/05 目录与观测帧构造（供 GG-12/13 发布）
 *
 * 背景（用户指示，ICD v1.5 任务计划 §1.4）：Windows/FAST.Farm 离线环境**无法**
 * 产生真实 AWAE 量测。因此：
 *   - 保留全部字段与端口契约，**不得**因缺数据而删字段；
 *   - 目录（S0401-S0403）由 Provider/配置**注入**静态测点坐标；
 *   - 观测（S0501-S0507）由 Provider 注入的 `FcrFlowState` 组装。
 *
 * 本模块只做"组装"，不做任何控制算法，也不决定发布通道
 * （发布由 `fcr_transport_publish_awae()` 按 Transport 能力完成）。
 */
#ifndef FCR_AWAE_FRAME_BUILDER_H
#define FCR_AWAE_FRAME_BUILDER_H

#include "fcr_awae_types.h"
#include "fcr_state_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 静态测点目录（GG-04：S0401 point_id / S0402 position_xyz / S0403 version） */
typedef struct {
    uint32_t valid;                                          /* 目录已注入 */
    uint32_t n_points;                                       /* 有效点数 */
    uint32_t catalog_version;                                /* S0403 */
    uint32_t point_id[FCR_MAX_OBSERVATION_POINTS];            /* S0401 */
    double   position_xyz_m[FCR_MAX_OBSERVATION_POINTS][3];   /* S0402 */
} FcrAWAECatalog;

/* 重置/初始化目录（n_points 会被截断到 FCR_MAX_OBSERVATION_POINTS） */
void fcr_awae_catalog_reset(FcrAWAECatalog *cat, uint32_t n_points,
                            uint32_t catalog_version);

/* 设置单个测点。index 越界返回 -2；cat 为空返回 -1。 */
int fcr_awae_catalog_set_point(FcrAWAECatalog *cat, uint32_t index,
                               uint32_t point_id, double x, double y, double z);

/* 目录 → FcrAWAECatalogFrame（GG-13：S1301-S1303）。
 * 返回写入的点数；cat/out 为空返回 -1。目录未注入时返回 0（帧合法但空）。 */
int fcr_awae_build_catalog_frame(const FcrAWAECatalog *cat, uint64_t frame_seq,
                                 FcrAWAECatalogFrame *out);

/* 流场（+目录点 ID）→ FcrAWAEObservationFrame（GG-12：S1201-S1207）。
 * - point_id 取自目录；目录缺失时退化为 index+1（保证字段非空）。
 * - quality_flags = (flow->valid ? FCR_AWAE_QUALITY_OK : 0) | extra_quality_flags。
 *   离线合成数据请传 FCR_AWAE_QUALITY_SIMULATED，使"非真实 AWAE"可被追溯。
 * 返回写入的观测点数；flow/out 为空返回 -1。 */
int fcr_awae_build_observation_frame(const FcrFlowState *flow,
                                     const FcrAWAECatalog *cat,
                                     uint16_t extra_quality_flags,
                                     FcrAWAEObservationFrame *out);

#ifdef __cplusplus
}
#endif

#endif /* FCR_AWAE_FRAME_BUILDER_H */
