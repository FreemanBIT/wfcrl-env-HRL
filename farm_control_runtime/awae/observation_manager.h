/*
 * observation_manager.h — AWAE Observation Manager（M2，DR-7）
 *
 * 职责（与风机状态**分离**，DR-7）：
 *   - 点位注册：配置驱动（静态坐标，ICD）；初期 256 点、验证 1000 点容量。
 *   - 时间同步：帧带 source_time_s / frame_seq，供 age 与丢帧检测。
 *   - 数据缓存：环形缓冲（ICD 建议 ring buffer），最新帧 + 最近 N 帧。
 *
 * 与 Wind Mapper 的关系：本模块只管理"点位 + 原始帧"，不负责观察点→盘面
 * 风速的映射（wind_mapper 负责）。
 */
#ifndef FCR_OBSERVATION_MANAGER_H
#define FCR_OBSERVATION_MANAGER_H

#include "awae_point_frame.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FcrObservationManager FcrObservationManager;

/* 创建（capacity = 环形缓冲帧数；max_points 默认 FCR_AWAE_MAX_POINTS） */
FcrObservationManager *fcr_observation_manager_create(uint32_t capacity,
                                                      uint32_t max_points);

void fcr_observation_manager_destroy(FcrObservationManager *mgr);

/* 注册/更新一个静态观察点（配置驱动，Slow 路径） */
int fcr_observation_manager_register_point(FcrObservationManager *mgr,
                                           const FcrAwaePoint *pt);

/* 喂入一帧（AWAE 1s；写入环形缓冲） */
int fcr_observation_manager_push_frame(FcrObservationManager *mgr,
                                       const FcrAwaePointFrame *frame);

/* 取最新帧（含 source_time / seq） */
int fcr_observation_manager_get_latest(FcrObservationManager *mgr,
                                       FcrAwaePointFrame *out);

/* 按 point_id 查最新单点 */
int fcr_observation_manager_get_point(FcrObservationManager *mgr,
                                      uint32_t point_id, FcrAwaePoint *out);

/* 当前注册点数 / 最新帧 age（s）*/
uint32_t fcr_observation_manager_count(FcrObservationManager *mgr);
double   fcr_observation_manager_latest_age(FcrObservationManager *mgr,
                                            double sim_time_s);

#ifdef __cplusplus
}
#endif

#endif /* FCR_OBSERVATION_MANAGER_H */
