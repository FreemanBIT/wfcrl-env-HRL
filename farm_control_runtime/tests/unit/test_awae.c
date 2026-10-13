/*
 * test_awae.c — AWAE Observation Manager + Wind Mapper 单元测试（M2，DR-7）
 */
#include "test_framework.h"
#include "observation_manager.h"
#include "wind_mapper.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Observation Manager：注册/环形缓冲/age ---- */
static void test_om_register_and_ring(void) {
    FcrObservationManager *m = fcr_observation_manager_create(4, 1024);
    FcrAwaePoint pt;
    FcrAwaePointFrame fr, out;
    int i;
    ASSERT(m != NULL);
    /* 注册 256 点（配置驱动） */
    for (i = 0; i < 256; ++i) {
        memset(&pt, 0, sizeof(pt));
        pt.point_id = (uint32_t)i + 1;
        pt.x = 0.0; pt.y = 0.0; pt.z = 90.0;
        pt.wind_speed = 8.0;
        pt.valid = 1;
        ASSERT(fcr_observation_manager_register_point(m, &pt) == 0);
    }
    ASSERT(fcr_observation_manager_count(m) == 256);
    /* 喂 6 帧（capacity=4 → 环形覆盖） */
    for (i = 0; i < 6; ++i) {
        memset(&fr, 0, sizeof(fr));
        fr.frame_seq = (uint32_t)i + 1;
        fr.source_time_s = (double)(i + 1);
        fr.n_points = 256;
        ASSERT(fcr_observation_manager_push_frame(m, &fr) == 0);
    }
    /* 最新帧应为第 6 帧 */
    ASSERT(fcr_observation_manager_get_latest(m, &out) == 0);
    ASSERT(out.frame_seq == 6);
    ASSERT(fcr_observation_manager_latest_age(m, 10.0) == 4.0);
    /* 按 point_id 查（最新帧内应有点）—— 测试帧 points 为空，应返回 -4 */
    ASSERT(fcr_observation_manager_get_point(m, 1, &pt) == -4);
    fcr_observation_manager_destroy(m);
}

/* ---- Wind Mapper：256 点盘面等效风 + DR-7 上游参考 ---- */
static void test_wind_mapper_256(void) {
    FcrWindMapperConfig cfg;
    FcrWindMapper *m;
    FcrAwaePointFrame fr;
    FcrMappedWind out;
    int i;
    memset(&cfg, 0, sizeof(cfg));
    cfg.rotor_radius_m = 63.0;
    cfg.rotor_center_x = 0.0; cfg.rotor_center_y = 0.0; cfg.rotor_center_z = 90.0;
    cfg.upstream_offset_m = -500.0;
    cfg.max_point_dist_m = 63.0;
    cfg.v1_weight_mode = 1;  /* 均匀平均，便于精确断言 */
    m = fcr_wind_mapper_create(&cfg);
    ASSERT(m != NULL);

    memset(&fr, 0, sizeof(fr));
    fr.frame_seq = 1; fr.source_time_s = 1.0;
    /* 256 点：200 点在盘面内（dist<=63，风速 8），56 点上游参考（风速 10，x=-600）*/
    fr.n_points = 256;
    for (i = 0; i < 256; ++i) {
        FcrAwaePoint *p = &fr.points[i];
        p->point_id = (uint32_t)i + 1;
        if (i < 56) {
            p->x = -600.0; p->y = 0.0; p->z = 90.0; p->wind_speed = 10.0;
            p->turbulence_intensity = 0.05;
        } else {
            /* 盘面内：沿 y 均匀分布在半径 63 内 */
            double yy = -60.0 + (i - 56) * (120.0 / 199.0);
            p->x = 0.0; p->y = yy; p->z = 90.0; p->wind_speed = 8.0;
            p->turbulence_intensity = 0.10;
        }
        p->valid = 1;
    }
    ASSERT(fcr_wind_mapper_map(m, 1, &fr, &out) == 0);
    /* 盘面等效风 = 8.0（均匀平均）；上游参考 = 10.0 */
    ASSERT_NEAR(out.disk_disturbed_wind_mps, 8.0, 1e-9);
    ASSERT_NEAR(out.ambient_upstream_wind_mps, 10.0, 1e-9);
    ASSERT_NEAR(out.disk_turbulence, 0.10, 1e-9);
    fcr_wind_mapper_destroy(m);
}

/* ---- Wind Mapper：1000 点容量 ---- */
static void test_wind_mapper_1000(void) {
    FcrWindMapperConfig cfg;
    FcrWindMapper *m;
    FcrAwaePointFrame fr;
    FcrMappedWind out;
    int i;
    memset(&cfg, 0, sizeof(cfg));
    cfg.rotor_radius_m = 63.0; cfg.rotor_center_x = 0; cfg.rotor_center_y = 0;
    cfg.rotor_center_z = 90.0; cfg.upstream_offset_m = -500.0;
    cfg.max_point_dist_m = 63.0; cfg.v1_weight_mode = 1;
    m = fcr_wind_mapper_create(&cfg);
    ASSERT(m != NULL);
    memset(&fr, 0, sizeof(fr));
    fr.frame_seq = 1; fr.source_time_s = 1.0;
    fr.n_points = 1000;  /* 1000 点容量（DR-7） */
    for (i = 0; i < 1000; ++i) {
        FcrAwaePoint *p = &fr.points[i];
        p->point_id = (uint32_t)i + 1;
        p->x = 0.0; p->y = -62.0 + (i % 200) * (124.0 / 199.0);
        p->z = 90.0; p->wind_speed = 8.0; p->turbulence_intensity = 0.10;
        p->valid = 1;
    }
    ASSERT(fcr_wind_mapper_map(m, 1, &fr, &out) == 0);
    ASSERT_NEAR(out.disk_disturbed_wind_mps, 8.0, 1e-6);
    fcr_wind_mapper_destroy(m);
}

/* ---- Wind Mapper：lambda（Mode2 用）---- */
static void test_wind_mapper_lambda(void) {
    FcrMappedWind w;
    memset(&w, 0, sizeof(w));
    w.disk_background_wind_mps = 8.0;
    double lam = fcr_wind_mapper_lambda(&w, 1.0, 63.0);
    ASSERT_NEAR(lam, 63.0 * 1.0 / 8.0, 1e-9);
    w.disk_background_wind_mps = 0.0;
    ASSERT(fcr_wind_mapper_lambda(&w, 1.0, 63.0) == 0.0);
}

void test_awae_register(void)
{
    RUN_TEST("om_register_and_ring", test_om_register_and_ring);
    RUN_TEST("wind_mapper_256", test_wind_mapper_256);
    RUN_TEST("wind_mapper_1000", test_wind_mapper_1000);
    RUN_TEST("wind_mapper_lambda", test_wind_mapper_lambda);
}
