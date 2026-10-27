/*
 * state_store.c — 多速率状态存储（ICD v1.5）
 *
 * 保存各来源的最新完整快照（latest-complete-snapshot 语义）：
 *   - ROSCO raw state      ~10 ms（每机槽位，S0301-S0303 + 扩展）
 *   - OF extra state       优选 ~10 ms（每机槽位，S0208-S0210 + 扩展）
 *   - AWAE flow state      ~1 s（ZOH 到下一次快照）
 *   - rt clock             sim_time / fast_step / low_step / health
 *
 * 不做插值、不伪造时间戳。线程安全：publish 采用"写后置有效标志 +
 * 消费者读快照"策略（单写者单读者场景，Volatile 屏障足够；
 * 多写者场景由宿主保证 Provider 线程互斥）。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>

FcrStateStore *fcr_state_store_create(uint32_t n_turbines)
{
    FcrStateStore *ss;
    if (n_turbines == 0 || n_turbines > FCR_MAX_TURBINES) return NULL;
    ss = (FcrStateStore *)calloc(1, sizeof(FcrStateStore));
    if (!ss) return NULL;
    ss->n_turbines = n_turbines;
    return ss;
}

void fcr_state_store_destroy(FcrStateStore *ss)
{
    if (ss) free(ss);
}

int fcr_state_store_set_clock(FcrStateStore *ss, double sim_time_s,
                              uint64_t fast_step, uint64_t low_step,
                              uint32_t rt_health_flags)
{
    if (!ss) return -1;
    ss->sim_time_s = sim_time_s;
    ss->fast_step = fast_step;
    ss->low_step = low_step;
    ss->rt_health_flags = rt_health_flags;
    return 0;
}

int fcr_state_store_set_fast(FcrStateStore *ss, int32_t turbine_id,
                             const FcrRoscoRawState *st)
{
    int32_t id;
    if (!ss || !st) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)ss->n_turbines) return -2;
    id = turbine_id - 1;
    ss->fast[id] = *st;
    /* S02/S03 分离输入（fastfarmRT：OF 与 ROSCO 是两个模块）：
     * 若该机已有有效 S02，则 OF 拥有的物理量测优先回填，
     * 避免 ROSCO 侧结构里未知/为 0 的字段把 OF 测点清零。
     * Windows 路径不输入 S02（of.valid=0），行为完全不变。 */
    if (ss->of[id].valid) {
        const FcrOpenFASTRawState *o = &ss->of[id];
        ss->fast[id].rotor_speed_rad_s = o->rotor_speed_rad_s;
        ss->fast[id].gen_speed_rad_s = o->generator_speed_rad_s;
        ss->fast[id].gen_power_w = o->generator_electrical_power_w;
        ss->fast[id].blade_pitch_rad[0] = o->blade_pitch_actual_rad[0];
        ss->fast[id].blade_pitch_rad[1] = o->blade_pitch_actual_rad[1];
        ss->fast[id].blade_pitch_rad[2] = o->blade_pitch_actual_rad[2];
        ss->fast[id].nacelle_heading_rad = o->nacelle_heading_actual_rad;
    }
    return 0;
}

int fcr_state_store_set_extra(FcrStateStore *ss, const FcrTurbineExtraState *st)
{
    if (!ss || !st) return -1;
    if (st->turbine_id < 1 || st->turbine_id > (int32_t)ss->n_turbines) return -2;
    ss->extra[st->turbine_id - 1] = *st;
    return 0;
}

/* S02：OpenFAST 原始状态写入 + 字段级合并。
 * 归属约定：OF 拥有物理量测（转速/功率/桨距/方位/推力/轴矩/塔基弯矩）；
 *           ROSCO 拥有命令、状态字、seq；extra 的 Ct/Cp/载荷测点由 Provider 另给。
 * 因此这里只写 OF 拥有的字段，绝不覆盖 ROSCO 侧 valid/source_seq/命令字段。 */
int fcr_state_store_set_openfast(FcrStateStore *ss, const FcrOpenFASTRawState *st)
{
    int32_t id;
    FcrRoscoRawState *f;
    FcrTurbineExtraState *e;
    if (!ss || !st) return -1;
    if (st->turbine_id < 1 || st->turbine_id > (int32_t)ss->n_turbines) return -2;
    id = st->turbine_id - 1;
    ss->of[id] = *st;
    if (!st->valid) return 0;

    f = &ss->fast[id];
    /* fast 槽尚未由 ROSCO 建立时，用 OF 的采样时间/序号初始化 */
    if (!f->valid) {
        f->turbine_id = st->turbine_id;
        f->valid = 1;
        f->source_time_s = st->of_sample_sim_time_s;
        f->source_seq = st->source_seq;
    }
    f->rotor_speed_rad_s = st->rotor_speed_rad_s;
    f->gen_speed_rad_s = st->generator_speed_rad_s;
    f->gen_power_w = st->generator_electrical_power_w;
    f->blade_pitch_rad[0] = st->blade_pitch_actual_rad[0];
    f->blade_pitch_rad[1] = st->blade_pitch_actual_rad[1];
    f->blade_pitch_rad[2] = st->blade_pitch_actual_rad[2];
    f->nacelle_heading_rad = st->nacelle_heading_actual_rad;

    e = &ss->extra[id];
    if (!e->valid) {
        e->turbine_id = st->turbine_id;
        e->valid = 1;
        e->source_time_s = st->of_sample_sim_time_s;
        e->source_seq = st->source_seq;
    }
    e->rotor_aero_thrust_n = st->rotor_aero_thrust_n;
    e->shaft_torque_nm = st->shaft_torque_nm;
    e->tower_base_fa_moment_nm = st->tower_base_fa_moment_nm;
    return 0;
}

int fcr_state_store_get_openfast(const FcrStateStore *ss, int32_t turbine_id,
                                 FcrOpenFASTRawState *out)
{
    if (!ss || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)ss->n_turbines) return -2;
    *out = ss->of[turbine_id - 1];
    return 0;
}

int fcr_state_store_set_flow(FcrStateStore *ss, const FcrFlowState *flow)
{
    if (!ss || !flow) return -1;
    ss->flow = *flow;   /* ZOH：保持到下一次 AWAE 快照 */
    return 0;
}

int fcr_state_store_get_fast(const FcrStateStore *ss, int32_t turbine_id,
                             FcrRoscoRawState *out)
{
    if (!ss || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)ss->n_turbines) return -2;
    *out = ss->fast[turbine_id - 1];
    return 0;
}

const FcrFlowState *fcr_state_store_get_flow(const FcrStateStore *ss)
{
    return ss ? &ss->flow : NULL;
}

int fcr_state_store_get_extra(const FcrStateStore *ss, int32_t turbine_id,
                              FcrTurbineExtraState *out)
{
    if (!ss || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)ss->n_turbines) return -2;
    *out = ss->extra[turbine_id - 1];
    return 0;
}

/* ---- S06 TurbineWindEstimate_to_Bridge ---------------------------- */

int fcr_state_store_set_wind_estimate(FcrStateStore *ss,
                                      const FcrTurbineWindEstimate *est)
{
    if (!ss || !est) return -1;
    if (est->turbine_id < 1 || est->turbine_id > (int32_t)ss->n_turbines) return -2;
    ss->wind_est[est->turbine_id - 1] = *est;   /* ZOH：保持到下一次估计 */
    return 0;
}

int fcr_state_store_get_wind_estimate(const FcrStateStore *ss, int32_t turbine_id,
                                      FcrTurbineWindEstimate *out)
{
    if (!ss || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)ss->n_turbines) return -2;
    *out = ss->wind_est[turbine_id - 1];
    return 0;
}

/* ---- ICD v1.5 GG-04/GG-05：AWAE 目录与质量位（离线注入） ---------- */

int fcr_state_store_set_awae_catalog(FcrStateStore *ss, const FcrAWAECatalog *cat)
{
    if (!ss || !cat) return -1;
    ss->awae_cat = *cat;
    return 0;
}

int fcr_state_store_set_awae_quality(FcrStateStore *ss, uint16_t extra_quality_flags)
{
    if (!ss) return -1;
    ss->awae_extra_quality_flags = extra_quality_flags;
    return 0;
}
