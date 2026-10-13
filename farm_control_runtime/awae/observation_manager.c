/*
 * observation_manager.c — AWAE Observation Manager 实现（M2，DR-7）
 * 环形缓冲 + 点位注册 + 时间同步。纯内存，无通信依赖。
 */
#include "observation_manager.h"
#include <stdlib.h>
#include <string.h>

struct FcrObservationManager {
    uint32_t capacity;          /* 环形缓冲帧数 */
    uint32_t max_points;        /* 最大观察点数 */
    uint32_t n_registered;      /* 已注册点位 */
    uint32_t head;              /* 最新帧索引 */
    uint32_t count;             /* 缓冲内帧数 */
    uint64_t last_frame_seq;
    double   last_frame_time;
    FcrAwaePointFrame *ring;    /* 环形缓冲 */
    FcrAwaePoint *points;       /* 注册点位（静态配置快照） */
    uint32_t *point_ids;
};

FcrObservationManager *fcr_observation_manager_create(uint32_t capacity,
                                                      uint32_t max_points) {
    FcrObservationManager *m;
    if (capacity == 0) capacity = 8;
    if (max_points == 0 || max_points > FCR_AWAE_MAX_POINTS) max_points = FCR_AWAE_MAX_POINTS;
    m = (FcrObservationManager *)calloc(1, sizeof(FcrObservationManager));
    if (!m) return NULL;
    m->capacity = capacity;
    m->max_points = max_points;
    m->ring = (FcrAwaePointFrame *)calloc(capacity, sizeof(FcrAwaePointFrame));
    m->points = (FcrAwaePoint *)calloc(max_points, sizeof(FcrAwaePoint));
    m->point_ids = (uint32_t *)calloc(max_points, sizeof(uint32_t));
    if (!m->ring || !m->points || !m->point_ids) {
        fcr_observation_manager_destroy(m);
        return NULL;
    }
    return m;
}

void fcr_observation_manager_destroy(FcrObservationManager *mgr) {
    if (!mgr) return;
    free(mgr->ring);
    free(mgr->points);
    free(mgr->point_ids);
    free(mgr);
}

int fcr_observation_manager_register_point(FcrObservationManager *mgr,
                                           const FcrAwaePoint *pt) {
    uint32_t i;
    if (!mgr || !pt) return -1;
    if (mgr->n_registered >= mgr->max_points) return -2;
    /* 去重：同 point_id 覆盖 */
    for (i = 0; i < mgr->n_registered; ++i) {
        if (mgr->point_ids[i] == pt->point_id) {
            mgr->points[i] = *pt;
            return 0;
        }
    }
    mgr->points[mgr->n_registered] = *pt;
    mgr->point_ids[mgr->n_registered] = pt->point_id;
    mgr->n_registered++;
    return 0;
}

int fcr_observation_manager_push_frame(FcrObservationManager *mgr,
                                       const FcrAwaePointFrame *frame) {
    FcrAwaePointFrame *slot;
    if (!mgr || !frame) return -1;
    if (frame->n_points > mgr->max_points) return -2;
    mgr->head = (mgr->head + 1) % mgr->capacity;
    slot = &mgr->ring[mgr->head];
    *slot = *frame;
    if (mgr->count < mgr->capacity) mgr->count++;
    mgr->last_frame_seq = frame->frame_seq;
    mgr->last_frame_time = frame->source_time_s;
    return 0;
}

int fcr_observation_manager_get_latest(FcrObservationManager *mgr,
                                       FcrAwaePointFrame *out) {
    if (!mgr || !out) return -1;
    if (mgr->count == 0) return -3;
    *out = mgr->ring[mgr->head];
    return 0;
}

int fcr_observation_manager_get_point(FcrObservationManager *mgr,
                                      uint32_t point_id, FcrAwaePoint *out) {
    uint32_t i;
    FcrAwaePointFrame *f;
    if (!mgr || !out) return -1;
    if (mgr->count == 0) return -3;
    f = &mgr->ring[mgr->head];
    for (i = 0; i < f->n_points; ++i) {
        if (f->points[i].point_id == point_id) {
            *out = f->points[i];
            return 0;
        }
    }
    return -4;
}

uint32_t fcr_observation_manager_count(FcrObservationManager *mgr) {
    return mgr ? mgr->n_registered : 0;
}

double fcr_observation_manager_latest_age(FcrObservationManager *mgr,
                                          double sim_time_s) {
    if (!mgr || mgr->count == 0) return -1.0;
    return sim_time_s - mgr->last_frame_time;
}
