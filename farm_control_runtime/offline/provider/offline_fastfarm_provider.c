/*
 * offline_fastfarm_provider.c — Offline FAST.Farm Provider（Phase 4）
 *
 * 离线闭环中，FAST.Farm 以预编译二进制运行，DISCON DLL 是进程内唯一探针。
 * 本 Provider 在每个 10 ms 步（DISCON 调用）从 Bladed DLL avrSWAP 记录
 * 读取 OpenFAST/ServoDyn 侧真实量，并调用统一 Provider API 发布：
 *
 *   record 109 -> shaft_torque_nm      (LSSTipMxa/LSShftMxs, Nm)
 *   record 110 -> rotor_aero_thrust_n  (LSShftFxa, N; GL x 分量; ICD v1.5 S0208)
 *   record 111/112 -> load_point 0 Fy/Fz (LSShftFys/Fzs, N)
 *   record 77/78  -> load_point 1 My/Mz (YawBrMyn/Mzn, Nm, GL)
 *
 * 语义边界（与 RT Provider ICD 对齐）：
 *   - tower_base_fa/ss 弯矩与 rotor_ct/cp 在 DLL 接口不可得；
 *     tower_base 置 valid=0；ci/cp 由 runtime 侧按 ICD 派生规则计算
 *     （ct = 2*T/(rho*A*V^2)，V 取 disk disturbed wind；见 Phase 5/6）。
 *   - 不得读取 .out/.outb 文件给闭环提供状态（红线）。
 *
 * 低速步号推算：low_step = round(sim_time / DT_low)，DT_low 由环境变量
 * FCR_CFG_DT_LOW 提供（离线 harness 启动 FAST.Farm 时设置），缺省 1.0 s。
 */
#include "fcr_internal.h"
#include "fcr_provider_api.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

extern FcrRuntime *fcr_rosco_api_runtime(void);

/* WIN-13：合成 AWAE 流场生成。定义在本文件末尾，但 publish_flow_snapshot
 * （ICD v1.5 GG-04/05 合成测点注入）会先用到，故此处前置声明，
 * 避免隐式声明（C99 起非法）。 */
int fcr_offline_provider_generate_synthetic_flow(FcrFlowState *flow,
                                                  uint32_t n_points,
                                                  double sim_time_s,
                                                  uint32_t n_turbines,
                                                  uint32_t seed);

static double g_dt_low = 1.0;
static double g_dt_high = 0.01;
static double g_ambient_mps = 0.0;   /* 0 = 未配置（用全场最大 hub 估算） */
static int g_init = 0;

/* 每机最新 hub 扰动风速（record 27 = HorWindV），用于 1 s 流场快照 */
static double g_hub_wind[FCR_MAX_TURBINES];
static uint32_t g_hub_valid[FCR_MAX_TURBINES];
static double g_last_flow_time = -1.0;
static uint64_t g_flow_seq = 0;
/* ICD v1.5 GG-04/GG-05：合成 AWAE 测点数（0 = 关闭；env FCR_CFG_AWAE_SYNTH_POINTS） */
static uint32_t g_awae_synth_points = 0;
/* 合成测点目录版本号（S0403）；真实 AWAE 目录版本由 AWAE 侧提供 */
#define FCR_AWAE_CATALOG_VERSION_SYNTH 0u

int fcr_offline_provider_init(void)
{
    const char *s;
    if (g_init) return 0;
    s = getenv("FCR_CFG_DT_LOW");
    if (s && atof(s) > 0.0) g_dt_low = atof(s);
    s = getenv("FCR_CFG_DT_HIGH");
    if (s && atof(s) > 0.0) g_dt_high = atof(s);
    s = getenv("FCR_CFG_AMBIENT_MPS");
    if (s && atof(s) > 0.0) g_ambient_mps = atof(s);
    /* ICD v1.5 GG-04/GG-05：合成 AWAE 测点数（默认 0 = 关闭，见 publish_flow_snapshot） */
    s = getenv("FCR_CFG_AWAE_SYNTH_POINTS");
    if (s && atoi(s) > 0) {
        g_awae_synth_points = (uint32_t)atoi(s);
        if (g_awae_synth_points > FCR_MAX_OBSERVATION_POINTS)
            g_awae_synth_points = FCR_MAX_OBSERVATION_POINTS;
    }
    g_init = 1;
    return 0;
}

/* 1 s 边界发布 AWAE 语义流场快照（离线近似，见 OFFLINE_PROVIDER.md） */
static void publish_flow_snapshot(FcrRuntime *rt, double sim_time_s)
{
    FcrFlowState fl;
    uint32_t i;
    double ambient = g_ambient_mps;

    memset(&fl, 0, sizeof(fl));
    if (ambient <= 0.0) {
        /* 近似：全场最大 hub 风 ≈ 最少受尾流影响的机组 */
        for (i = 0; i < FCR_MAX_TURBINES; ++i) {
            if (g_hub_valid[i] && g_hub_wind[i] > ambient) ambient = g_hub_wind[i];
        }
    }
    for (i = 0; i < FCR_MAX_TURBINES; ++i) {
        fl.disk_disturbed_wind_mps[i] = g_hub_valid[i] ? g_hub_wind[i] : 0.0;
        fl.disk_ambient_wind_mps[i] = ambient;
    }
    fl.valid = 1;
    fl.flow_source_time_s = sim_time_s;
    fl.flow_seq = ++g_flow_seq;
    fl.n_turbines = FCR_MAX_TURBINES;
    fl.n_points = 0;   /* 观察点（U/V/W 网格）离线不可得：RT Provider 提供 */

    /* ICD v1.5 GG-04/GG-05：FAST.Farm 离线**无法**产生真实 AWAE 量测。
     * 按用户指示，由 Provider 注入**合成**测点（明确标记 SIMULATED），
     * 以便验证 S05→S12/S13 发布链路。
     * ⚠ 默认关闭（FCR_CFG_AWAE_SYNTH_POINTS 未设/为 0）：注入观察点会改变
     *   WindMapper 的映射结果，从而扰动既有 300 s 回归基线，故必须显式开启。
     *   只覆盖观察点数组，**不动** disk_* 盘面量（那些来自真实 hub 风）。 */
    if (g_awae_synth_points > 0) {
        FcrFlowState syn;
        FcrAWAECatalog cat;
        uint32_t np = g_awae_synth_points;
        if (np > FCR_MAX_OBSERVATION_POINTS) np = FCR_MAX_OBSERVATION_POINTS;
        if (fcr_offline_provider_generate_synthetic_flow(&syn, np, sim_time_s,
                                                        FCR_MAX_TURBINES, 1u) == 0) {
            for (i = 0; i < np; ++i) {
                fl.ambient_u[i] = syn.ambient_u[i];
                fl.ambient_v[i] = syn.ambient_v[i];
                fl.ambient_w[i] = syn.ambient_w[i];
                fl.disturbed_u[i] = syn.disturbed_u[i];
                fl.disturbed_v[i] = syn.disturbed_v[i];
                fl.disturbed_w[i] = syn.disturbed_w[i];
                fl.point_ti[i] = syn.point_ti[i];
            }
            fl.n_points = np;
        }
        /* 静态测点目录（S0401-S0403）：合成点均匀布置在半径 300 m 圆上 */
        fcr_awae_catalog_reset(&cat, np, FCR_AWAE_CATALOG_VERSION_SYNTH);
        for (i = 0; i < np; ++i) {
            double ang = 2.0 * 3.14159265358979323846 * (double)i / (double)np;
            fcr_awae_catalog_set_point(&cat, i, i + 1u,
                                       300.0 * cos(ang), 300.0 * sin(ang), 90.0);
        }
        fcr_state_store_set_awae_catalog(rt->ss, &cat);
        fcr_state_store_set_awae_quality(rt->ss, FCR_AWAE_QUALITY_SIMULATED);
    }

    fcr_publish_flow_state(rt, &fl);

    /* 1 s 低速步驱动：聚合（SignalStatistics 窗口闭合）+ watchdog +
     * Transport 发布 FarmStateFrame（Phase 7 ZMQ 状态链）。 */
    extern int fcr_rosco_step_low(double sim_time_s);
    fcr_rosco_step_low(sim_time_s);
}

/* 每 10 ms：发布 OpenFAST 额外快速状态（turbine extra） */
int fcr_offline_provider_publish_fast(int32_t turbine_id, double sim_time_s,
                                      const float *avrSWAP)
{
    FcrRuntime *rt = fcr_rosco_api_runtime();
    FcrTurbineExtraState st;
    uint64_t fast_step;
    if (!rt || !avrSWAP) return -1;
    fcr_offline_provider_init();

    memset(&st, 0, sizeof(st));
    st.turbine_id = turbine_id;
    st.source_time_s = sim_time_s;
    fast_step = (uint64_t)floor(sim_time_s / g_dt_high);
    st.source_seq = fast_step;
    st.valid = 1;

    /* record 109 = LSSTipMxa/LSShftMxs（主轴扭矩，Nm）；110 = LSShftFxa（GL x 力，对风 ≈ 推力） */
    st.shaft_torque_nm = (double)avrSWAP[109 - 1];
    st.rotor_aero_thrust_n = (double)avrSWAP[110 - 1];

    /* 记录 hub 扰动风（1 s 流场快照用） */
    g_hub_wind[turbine_id - 1] = (double)avrSWAP[27 - 1];
    g_hub_valid[turbine_id - 1] = (avrSWAP[27 - 1] > 0.0f) ? 1u : 0u;

    /* load point 0: 轮毂（低速轴）力，GL 坐标 */
    st.load_point_f[0][0] = (double)avrSWAP[110 - 1];  /* Fx */
    st.load_point_f[1][0] = (double)avrSWAP[111 - 1];  /* Fy */
    st.load_point_f[2][0] = (double)avrSWAP[112 - 1];  /* Fz */
    /* load point 1: 偏航轴承弯矩，GL 坐标（My/Mz；Mx 接口未提供） */
    st.load_point_f[4][1] = (double)avrSWAP[77 - 1];   /* My */
    st.load_point_f[5][1] = (double)avrSWAP[78 - 1];   /* Mz */

    /* tower_base 弯矩：DLL 接口不可得 -> valid=0（保持 0/NaN 语义） */
    st.tower_base_fa_moment_nm = 0.0 / 0.0;
    st.tower_base_ss_moment_nm = 0.0 / 0.0;

    /* 同时发布时钟（fast_step 由时间推算，全场一致） */
    fcr_publish_rt_clock(rt, sim_time_s, fast_step,
                         (uint64_t)floor(sim_time_s / g_dt_low), FCR_RT_HEALTH_OK);

    /* 1 s 边界：发布流场快照（AWAE 语义；ZOH 由 state store 维持）。
     * 各 WT 进程都发布状态帧（各自槽位）；Python 侧按 turbine_id 合并。 */
    if (sim_time_s - g_last_flow_time >= g_dt_low - 1e-9) {
        publish_flow_snapshot(rt, sim_time_s);
        g_last_flow_time = sim_time_s;
    }

    return fcr_publish_turbine_extra_state(rt, &st);
}

/* 供测试/工具：获取配置 */
double fcr_offline_provider_dt_low(void) { return g_dt_low; }

/* WIN-13: generate synthetic AWAE observation points for offline validation.
 * Produces deterministic flow field data clearly marked as SIMULATED.
 * Used to validate ObservationManager + WindMapper + StateAggregator chain
 * without requiring actual GPU/AWAE access. */
int fcr_offline_provider_generate_synthetic_flow(FcrFlowState *flow,
                                                  uint32_t n_points,
                                                  double sim_time_s,
                                                  uint32_t n_turbines,
                                                  uint32_t seed)
{
    uint32_t i, t;
    if (!flow || n_points == 0 || n_points > FCR_MAX_OBSERVATION_POINTS) return -1;
    memset(flow, 0, sizeof(*flow));
    flow->valid = 1;
    flow->flow_source_time_s = sim_time_s;
    flow->flow_seq = (uint64_t)(sim_time_s / g_dt_low);
    flow->n_points = n_points;
    flow->n_turbines = n_turbines;

    /* Deterministic pseudo-random wind field (SIMULATED — not real AWAE) */
    for (i = 0; i < n_points; ++i) {
        double angle = 2.0 * 3.14159265358979323846 * (double)i / (double)n_points;
        uint32_t h = i * 2654435761u + seed * 40503u;
        double jitter = (double)(h % 1000) / 1000.0;  /* 0..1 */
        flow->ambient_u[i] = 8.0 + 2.0 * sin(angle) + 0.5 * jitter;
        flow->ambient_v[i] = 0.5 * cos(angle);
        flow->ambient_w[i] = 0.0;
        flow->disturbed_u[i] = flow->ambient_u[i] * 0.85;  /* wake deficit */
        flow->disturbed_v[i] = flow->ambient_v[i];
        flow->disturbed_w[i] = 0.0;
        flow->point_ti[i] = 0.08 + 0.02 * sin(2.0 * angle);
    }

    /* Disk-averaged wind per turbine (simplified uniform) */
    for (t = 0; t < n_turbines && t < FCR_MAX_TURBINES; ++t) {
        flow->disk_ambient_wind_mps[t] = 8.0;
        flow->disk_disturbed_wind_mps[t] = 6.8;
        flow->disk_ambient_ti[t] = 0.08;
    }
    return 0;
}