/*
 * fcr_awae_types.h — ICD v1.5 AWAE 接口组类型定义
 *
 * 对应信号组:
 *   S0401-S0403  AWAE_Catalog
 *   S0501-S0507  AWAE_Observation  
 *   S1201-S1207  Scheduler_to_RCP_AWAE_Observation
 *   S1301-S1303  Scheduler_to_RCP_AWAE_Catalog
 *
 * Windows 离线仿真环境下 AWAE 数据由外部 Provider 注入。
 */
#ifndef FCR_AWAE_TYPES_H
#define FCR_AWAE_TYPES_H

#include "fcr_protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/* AWAE_Catalog (S0401-S0403)                                        */
/* ================================================================== */

typedef struct {
    uint32_t  point_id;                    /* S0401: 虚拟测风点唯一编号 */
    double    point_position_xyz_m[3];     /* S0402: 测风点全场坐标XYZ */
    uint32_t  point_catalog_version;       /* S0403: 测点目录版本 */
} FcrAWAECatalogEntry;

typedef struct {
    uint32_t              protocol_version;
    uint64_t              frame_seq;
    uint32_t              n_points;
    FcrAWAECatalogEntry   entries[FCR_MAX_OBSERVATION_POINTS];
} FcrAWAECatalogFrame;

/* ================================================================== */
/* AWAE_Observation (S0501-S0507)                                    */
/* ================================================================== */

typedef struct {
    uint64_t  observation_frame_seq;       /* S0501: AWAE空间观测帧序号 */
    double    observation_sample_sim_time_s; /* S0502: AWAE观测仿真时刻 */
    uint32_t  observation_point_count;     /* S0503: 本帧测风点数 */
    uint32_t  point_id;                    /* S0504: 测风点编号 */
    float     wind_uvw_mps[3];             /* S0505: 测风点全场XYZ风速矢量 */
    uint16_t  observation_quality_flags;   /* S0506: 测点观测质量 */
    float     turbulence_intensity;        /* S0507: 测点湍流强度 */
} FcrAWAEObservation;

/* 单帧可包含多个测风点观测 */
typedef struct {
    uint32_t              protocol_version;
    uint64_t              frame_seq;
    uint32_t              n_observations;
    FcrAWAEObservation    observations[FCR_MAX_OBSERVATION_POINTS];
} FcrAWAEObservationFrame;

/* ================================================================== */
/* TurbineWindEstimate_to_Bridge (S0601-S0604)                       */
/* ================================================================== */

typedef struct {
    int32_t   turbine_id;
    double    estimated_wind_mps;          /* S0601: 机组等效来流风速估计值 */
    double    estimate_sim_time_s;         /* S0602: 来流风速估计所对应的仿真时间 */
    uint16_t  wind_source;                 /* S0603: 0=unknown,1=ROSCO_WE,2=OpenFAST,3=AWAE */
    uint16_t  wind_quality_flags;          /* S0604: 风速估计质量标志 */
} FcrTurbineWindEstimate;

/* wind_source 枚举 */
#define FCR_WIND_SOURCE_UNKNOWN     0u
#define FCR_WIND_SOURCE_ROSCO_WE    1u
#define FCR_WIND_SOURCE_OPENFAST    2u
#define FCR_WIND_SOURCE_AWAE        3u

/* wind_quality_flags 位定义 */
#define FCR_WIND_QUALITY_OK             (1u << 0)  /* 估计有效 */
#define FCR_WIND_QUALITY_STALE          (1u << 1)  /* 数据超龄 */
#define FCR_WIND_QUALITY_INTERPOLATED   (1u << 2)  /* 插值数据 */
#define FCR_WIND_QUALITY_LOW_CONFIDENCE (1u << 3)  /* 低置信度 */

#ifdef __cplusplus
}
#endif

#endif /* FCR_AWAE_TYPES_H */
