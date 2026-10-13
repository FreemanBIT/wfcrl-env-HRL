/*
 * wind_mapper.h — Wind Mapper（M2，DR-7）
 *
 * 职责：AWAE 观察点 → 风轮盘面等效风速/TI。
 * 严格区分四类风速语义（DR-7）：
 *   ambient_upstream  : 上游参考风（取本机上游参考观察点/自由来流）
 *   disk_background   : 轮盘背景风速（含他机尾流、不含本机诱导）
 *   disk_disturbed    : 轮盘扰动风速（盘面实际）
 *
 * 映射算法（V1 确定性）：
 *   - disk_equiv_wind = 盘面观察点的加权平均（按观察点与轮盘中心的距离/权重）
 *   - ambient_upstream = 上游参考观察点风速（无扰动，最接近自由来流）
 *   - TI 取盘面观察点 TI 的加权平均
 * 后续可替换为更精细的盘面加权/插值（与供应商 GPU 高分辨率场对齐验证）。
 */
#ifndef FCR_WIND_MAPPER_H
#define FCR_WIND_MAPPER_H

#include "awae_point_frame.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double rotor_radius_m;         /* 轮盘半径（NREL 5MW = 63） */
    double rotor_center_x, rotor_center_y, rotor_center_z; /* 轮盘中心（全局坐标） */
    double upstream_offset_m;      /* 上游参考观察点距轮盘中心的 x 距离（取最上游） */
    double max_point_dist_m;       /* 参与盘面平均的观察点最大距离（默认 rotor_radius） */
    uint32_t v1_weight_mode;       /* 0=距离倒数加权；1=均匀平均 */
} FcrWindMapperConfig;

typedef struct {
    double ambient_upstream_wind_mps;  /* 上游参考风（DR-7） */
    double disk_background_wind_mps;   /* 轮盘背景风（DR-7） */
    double disk_disturbed_wind_mps;    /* 轮盘扰动风（DR-7） */
    double disk_turbulence;            /* 盘面等效 TI */
    uint32_t valid;
} FcrMappedWind;

typedef struct FcrWindMapper FcrWindMapper;

FcrWindMapper *fcr_wind_mapper_create(const FcrWindMapperConfig *cfg);
void fcr_wind_mapper_destroy(FcrWindMapper *m);

/* 映射一帧观察点 → 盘面等效风（含四类语义）。turbine_id 用于定位轮盘中心。 */
int fcr_wind_mapper_map(FcrWindMapper *m, int32_t turbine_id,
                        const FcrAwaePointFrame *frame, FcrMappedWind *out);

/* 从盘面等效风推导 lambda（Mode2 用）：lambda = om*R / disk_background */
double fcr_wind_mapper_lambda(const FcrMappedWind *w, double rotor_speed_rad_s,
                              double rotor_radius_m);

#ifdef __cplusplus
}
#endif

#endif /* FCR_WIND_MAPPER_H */
