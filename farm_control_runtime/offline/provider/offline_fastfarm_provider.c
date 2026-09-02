/*
 * offline_fastfarm_provider.c — Offline FAST.Farm Provider（Phase 4）
 *
 * 离线闭环中，FAST.Farm 以预编译二进制运行，DISCON DLL 是进程内唯一探针。
 * 本 Provider 在每个 10 ms 步（DISCON 调用）从 Bladed DLL avrSWAP 记录
 * 读取 OpenFAST/ServoDyn 侧真实量，并调用统一 Provider API 发布：
 *
 *   record 109 -> shaft_torque_nm      (LSSTipMxa/LSShftMxs, Nm)
 *   record 110 -> rotor_thrust_n       (LSShftFxa, N; GL x 分量)
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

static double g_dt_low = 1.0;
static double g_dt_high = 0.01;
static double g_ambient_mps = 0.0;   /* 0 = 未配置（用全场最大 hub 估算） */
static int g_init = 0;

/* 每机最新 hub 扰动风速（record 27 = HorWindV），用于 1 s 流场快照 */
static double g_hub_wind[FCR_MAX_TURBINES];
static uint32_t g_hub_valid[FCR_MAX_TURBINES];
static double g_last_flow_time = -1.0;
static uint64_t g_flow_seq = 0;

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
    fcr_publish_flow_state(rt, &fl);
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
    st.rotor_thrust_n = (double)avrSWAP[110 - 1];

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

    /* 1 s 边界：发布流场快照（AWAE 语义；ZOH 由 state store 维持） */
    if (sim_time_s - g_last_flow_time >= g_dt_low - 1e-9) {
        publish_flow_snapshot(rt, sim_time_s);
        g_last_flow_time = sim_time_s;
    }

    return fcr_publish_turbine_extra_state(rt, &st);
}

/* 供测试/工具：获取配置 */
double fcr_offline_provider_dt_low(void) { return g_dt_low; }