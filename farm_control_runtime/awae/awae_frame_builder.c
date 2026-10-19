/*
 * awae_frame_builder.c — ICD v1.5 GG-04/05 帧组装实现
 *
 * 纯函数式：不分配、不 I/O、不持锁；可在任意任务上下文调用。
 */
#include "awae_frame_builder.h"
#include "fcr_protocol.h"
#include <string.h>

void fcr_awae_catalog_reset(FcrAWAECatalog *cat, uint32_t n_points,
                            uint32_t catalog_version)
{
    if (!cat) return;
    memset(cat, 0, sizeof(*cat));
    if (n_points > FCR_MAX_OBSERVATION_POINTS) n_points = FCR_MAX_OBSERVATION_POINTS;
    cat->n_points = n_points;
    cat->catalog_version = catalog_version;
    cat->valid = 1;
}

int fcr_awae_catalog_set_point(FcrAWAECatalog *cat, uint32_t index,
                               uint32_t point_id, double x, double y, double z)
{
    if (!cat) return -1;
    if (index >= FCR_MAX_OBSERVATION_POINTS) return -2;
    cat->point_id[index] = point_id;
    cat->position_xyz_m[index][0] = x;
    cat->position_xyz_m[index][1] = y;
    cat->position_xyz_m[index][2] = z;
    if (index >= cat->n_points) cat->n_points = index + 1;
    return 0;
}

int fcr_awae_build_catalog_frame(const FcrAWAECatalog *cat, uint64_t frame_seq,
                                 FcrAWAECatalogFrame *out)
{
    uint32_t i, n;

    if (!cat || !out) return -1;
    memset(out, 0, sizeof(*out));
    out->protocol_version = FCR_PROTOCOL_VERSION;
    out->frame_seq = frame_seq;

    /* 目录未注入时仍上送合法空帧（不删字段、不伪造点） */
    n = cat->valid ? cat->n_points : 0u;
    if (n > FCR_MAX_OBSERVATION_POINTS) n = FCR_MAX_OBSERVATION_POINTS;

    for (i = 0; i < n; ++i) {
        FcrAWAECatalogEntry *e = &out->entries[i];
        e->point_id = cat->point_id[i];                              /* S0401 */
        e->point_position_xyz_m[0] = cat->position_xyz_m[i][0];      /* S0402 */
        e->point_position_xyz_m[1] = cat->position_xyz_m[i][1];
        e->point_position_xyz_m[2] = cat->position_xyz_m[i][2];
        e->point_catalog_version = cat->catalog_version;             /* S0403 */
    }
    out->n_points = n;
    return (int)n;
}

int fcr_awae_build_observation_frame(const FcrFlowState *flow,
                                     const FcrAWAECatalog *cat,
                                     uint16_t extra_quality_flags,
                                     FcrAWAEObservationFrame *out)
{
    uint32_t i, n;
    uint16_t q;

    if (!flow || !out) return -1;
    memset(out, 0, sizeof(*out));
    out->protocol_version = FCR_PROTOCOL_VERSION;
    out->frame_seq = flow->flow_seq;                    /* S1201 <- S0501 */

    n = flow->n_points;
    if (n > FCR_MAX_OBSERVATION_POINTS) n = FCR_MAX_OBSERVATION_POINTS;

    q = (uint16_t)((flow->valid ? FCR_AWAE_QUALITY_OK : 0u) | extra_quality_flags);

    for (i = 0; i < n; ++i) {
        FcrAWAEObservation *o = &out->observations[i];
        o->observation_frame_seq = flow->flow_seq;                    /* S0501 */
        o->observation_sample_sim_time_s = flow->flow_source_time_s;  /* S0502 */
        o->observation_point_count = n;                               /* S0503 */
        o->point_id = (cat && cat->valid && i < cat->n_points)
                          ? cat->point_id[i]                          /* S0504 */
                          : (i + 1u);
        o->wind_uvw_mps[0] = (float)flow->ambient_u[i];               /* S0505 */
        o->wind_uvw_mps[1] = (float)flow->ambient_v[i];
        o->wind_uvw_mps[2] = (float)flow->ambient_w[i];
        o->observation_quality_flags = q;                             /* S0506 */
        o->turbulence_intensity = (float)flow->point_ti[i];           /* S0507 */
    }
    out->n_observations = n;
    return (int)n;
}
