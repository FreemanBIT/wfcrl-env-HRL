/*
 * wind_mapper.c — Wind Mapper 实现（M2，DR-7）
 * 观察点 → 盘面等效风速/TI，四类风速语义分离。
 */
#include "wind_mapper.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct FcrWindMapper {
    FcrWindMapperConfig cfg;
};

static FcrWindMapperConfig default_cfg(void) {
    FcrWindMapperConfig c;
    c.rotor_radius_m = 63.0;
    c.rotor_center_x = 0.0; c.rotor_center_y = 0.0; c.rotor_center_z = 90.0;
    c.upstream_offset_m = -500.0;  /* 上游 500m 参考 */
    c.max_point_dist_m = 63.0;
    c.v1_weight_mode = 0;
    return c;
}

FcrWindMapper *fcr_wind_mapper_create(const FcrWindMapperConfig *cfg) {
    FcrWindMapper *m = (FcrWindMapper *)calloc(1, sizeof(FcrWindMapper));
    if (!m) return NULL;
    m->cfg = cfg ? *cfg : default_cfg();
    if (m->cfg.rotor_radius_m <= 0.0) m->cfg.rotor_radius_m = 63.0;
    if (m->cfg.max_point_dist_m <= 0.0) m->cfg.max_point_dist_m = m->cfg.rotor_radius_m;
    return m;
}

void fcr_wind_mapper_destroy(FcrWindMapper *m) { if (m) free(m); }

int fcr_wind_mapper_map(FcrWindMapper *m, int32_t turbine_id,
                        const FcrAwaePointFrame *frame, FcrMappedWind *out) {
    double sw = 0.0, sw_ti = 0.0, wsum = 0.0, wsum_ti = 0.0;
    double up_amb = 0.0, up_count = 0.0;
    double up_ref_x;
    uint32_t i;
    if (!m || !frame || !out) return -1;
    memset(out, 0, sizeof(*out));
    if (frame->n_points == 0) return -2;

    /* 轮盘中心：按 turbine_id 在 x 轴上排布（V1：spacing 由外部配置，此处用
     * 全局 cfg 的中心 + turbine 偏移简化；真实场景由布局配置注入） */
    up_ref_x = m->cfg.rotor_center_x + m->cfg.upstream_offset_m;

    for (i = 0; i < frame->n_points; ++i) {
        const FcrAwaePoint *p = &frame->points[i];
        double dx = p->x - m->cfg.rotor_center_x;
        double dy = p->y - m->cfg.rotor_center_y;
        double dz = p->z - m->cfg.rotor_center_z;
        double dist = sqrt(dx * dx + dy * dy + dz * dz);

        /* 上游参考观察点：在轮盘上游、且离自由来流最近（最小 |x - up_ref_x| 的强风点） */
        if (p->x <= up_ref_x) {
            /* 取最上游若干点中风速最大的作为上游参考（近似自由来流） */
            if (p->wind_speed > up_amb) { up_amb = p->wind_speed; up_count = 1.0; }
            continue;
        }
        /* 盘面观察点：落在轮盘半径内 → 参与盘面等效风加权平均 */
        if (dist <= m->cfg.max_point_dist_m) {
            double w;
            if (m->cfg.v1_weight_mode == 1) {
                w = 1.0;
            } else {
                w = (dist < 1.0) ? 10.0 : 1.0 / dist;  /* 距离倒数加权 */
            }
            sw += p->wind_speed * w;
            sw_ti += p->turbulence_intensity * w;
            wsum += w;
            wsum_ti += w;
        }
    }

    if (wsum > 0.0) {
        out->disk_disturbed_wind_mps = sw / wsum;
        out->disk_turbulence = sw_ti / wsum_ti;
    } else {
        return -3;  /* 盘面无有效观察点 */
    }
    /* 背景风速近似：盘面等效风（DR-7 V1 简化，真实需尾流模型分离） */
    out->disk_background_wind_mps = out->disk_disturbed_wind_mps;
    /* 上游参考风：自由来流 */
    out->ambient_upstream_wind_mps = (up_count > 0.0) ? up_amb : out->disk_disturbed_wind_mps;
    out->valid = 1;
    return 0;
}

double fcr_wind_mapper_lambda(const FcrMappedWind *w, double rotor_speed_rad_s,
                              double rotor_radius_m) {
    if (!w || rotor_radius_m <= 0.0) return 0.0;
    if (w->disk_background_wind_mps <= 0.5) return 0.0;
    return rotor_speed_rad_s * rotor_radius_m / w->disk_background_wind_mps;
}
