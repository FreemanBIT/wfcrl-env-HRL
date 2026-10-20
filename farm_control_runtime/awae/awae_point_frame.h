/*
 * awae_point_frame.h — AWAE Observation Point Frame（M2，DR-7）
 *
 * ICD v1.1（AWAE 行）：point_id / position_xyz / wind_x/y/z / wind_speed /
 * wind_direction / turbulence_intensity。
 *
 * DR-7 语义分层：
 *   ambient_upstream  : 上游参考风（观察点映射，未受本机扰动）
 *   disk_background   : 轮盘背景风速（含他机尾流、不含本机诱导）
 *   disk_disturbed    : 轮盘扰动风速（盘面实际，含本机诱导）
 *   observation point : AWAE 原始观察点
 * 本帧承载 AWAE 原始观察点数据（观察点独立于 turbine_id，ICD 注明）。
 */
#ifndef FCR_AWAE_POINT_FRAME_H
#define FCR_AWAE_POINT_FRAME_H

#include "fcr_protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* W-09：容量统一到线协议常量（原 1024 与 FcrFlowState 64 点不一致；
 * 本 legacy v1.1 帧承载的观察点数不得超过 FCR_MAX_OBSERVATION_POINTS）。 */
#define FCR_AWAE_MAX_POINTS FCR_MAX_OBSERVATION_POINTS
#define FCR_AWAE_POINT_NAME_MAX 32

typedef struct {
    uint32_t  point_id;             /* 观察点 ID（独立于 turbine_id，ICD） */
    double    x, y, z;              /* 坐标（m，静态配置） */
    double    wind_x, wind_y, wind_z; /* 风矢量分量（m/s，1s 更新） */
    double    wind_speed;           /* 风矢量幅值（m/s） */
    double    wind_direction;       /* 风向（rad） */
    double    turbulence_intensity; /* 局部湍流强度（-） */
    double    source_time_s;        /* 帧时间 */
    uint32_t  valid;                /* 本条有效 */
} FcrAwaePoint;

typedef struct {
    uint32_t  frame_seq;            /* 帧序号（单调递增） */
    double    source_time_s;        /* 帧时间 */
    uint32_t  n_points;             /* 实际点数 */
    FcrAwaePoint points[FCR_AWAE_MAX_POINTS];
} FcrAwaePointFrame;

#ifdef __cplusplus
}
#endif

#endif /* FCR_AWAE_POINT_FRAME_H */
