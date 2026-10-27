/*
 * rosco_api.c — runtime ↔ ROSCO C/Fortran ABI 的宿主实现（Phase 2）
 *
 * 本文件编译进 DISCON_WT1.dll（与 ROSCO 同进程同模块），维护 DLL 内
 * 全场 runtime 单例：
 *   - fcr_rosco_read_setpoint : ROSCO 10 ms 线程读取（无网络/文件 I/O）
 *   - fcr_rosco_publish_state : ROSCO 10 ms 发布快速状态
 *   - fcr_rosco_step_high     : DISCON 每 10 ms 驱动（命令 refresh + setpoint 生成）
 *   - fcr_rosco_inject_command_frame : 离线 harness / Transport 注入命令帧
 *
 * 线程模型：单进程多线程（OpenMP FAST.Farm）时，DLL 数据段共享，所有
 * 入口经互斥锁保护；单进程单机模式锁开销可忽略。
 */
#include "fcr_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* WIN-14: lock hold time statistics */
static double g_lock_hold_max_ms = 0.0;
static double g_lock_hold_total_ms = 0.0;
static uint64_t g_lock_count = 0;
static uint64_t g_queue_depth_max = 0;
/* ICD v1.5 S0901-S0904: Bridge 确认（seq 回读 + 状态字 + ct_ref） */
static FcrRoscoAck g_ack[FCR_MAX_TURBINES];

#ifdef _WIN32
#include <windows.h>
static CRITICAL_SECTION g_lock;
static int g_lock_init = 0;
#define FCR_LOCK_INIT() do { InitializeCriticalSection(&g_lock); g_lock_init = 1; } while (0)
#define FCR_LOCK_FINI() do { if (g_lock_init) DeleteCriticalSection(&g_lock); g_lock_init = 0; } while (0)

/* WIN-14: instrumented lock with hold-time tracking */
static LARGE_INTEGER g_lock_freq;
static double g_lock_t0_ms;
static void fcr_lock_enter(void) {
    EnterCriticalSection(&g_lock);
    if (g_lock_freq.QuadPart == 0) QueryPerformanceFrequency(&g_lock_freq);
    LARGE_INTEGER now; QueryPerformanceCounter(&now);
    g_lock_t0_ms = (double)now.QuadPart * 1000.0 / (double)g_lock_freq.QuadPart;
    g_lock_count++;
}
static void fcr_lock_exit(void) {
    LARGE_INTEGER now; QueryPerformanceCounter(&now);
    double hold = (double)now.QuadPart * 1000.0 / (double)g_lock_freq.QuadPart - g_lock_t0_ms;
    g_lock_hold_total_ms += hold;
    if (hold > g_lock_hold_max_ms) g_lock_hold_max_ms = hold;
    LeaveCriticalSection(&g_lock);
}
#define FCR_LOCK() fcr_lock_enter()
#define FCR_UNLOCK() fcr_lock_exit()
#else
#include <pthread.h>
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
#define FCR_LOCK_INIT() ((void)0)
#define FCR_LOCK_FINI() ((void)0)
#define FCR_LOCK() pthread_mutex_lock(&g_lock)
#define FCR_UNLOCK() pthread_mutex_unlock(&g_lock)
#endif

static FcrRuntime *g_rt = NULL;
static int g_transport_done = 0; /* shm/zmq 挂载只执行一次（多机并发保护） */

/* 弱符号：无 C++ 链接时（单元测试）自动解析为 NULL stub */
#if defined(__GNUC__)
extern int fcr_zmq_transport_hook(FcrRuntime *rt, int cmd_port, int state_port)
    __attribute__((weak));
#endif

/* ------------------------------------------------------------------ */
/* 生命周期（由 DISCON 首次调用 / 离线 harness 调用）                 */
/* ------------------------------------------------------------------ */

int fcr_rosco_api_init(uint32_t n_turbines_max, double dt_high_s)
{
    FcrRuntimeConfig cfg;
    if (g_rt) return 0;                    /* 已初始化 */
    memset(&cfg, 0, sizeof(cfg));
    cfg.n_turbines = (n_turbines_max > 0 && n_turbines_max <= FCR_MAX_TURBINES)
                         ? n_turbines_max : FCR_MAX_TURBINES;
    cfg.dt_high_s = (dt_high_s > 0.0) ? dt_high_s : 0.01;
    cfg.dt_low_s = 1.0;
    cfg.default_yaw_ttl_s = 60.0;
    cfg.default_induction_ttl_s = 5.0;  /* > 1 个状态帧周期（DT_low=3s） */
    cfg.stale_threshold_s = 5.0;
    cfg.flow_stale_threshold_s = 3.0;
    cfg.transport = NULL;
    /* DR-3/DR-4：bridge 接入开关（env 可覆盖，供离线/RT 验证） */
    {
        const char *be = getenv("FCR_BRIDGE_ENABLE");
        const char *bm = getenv("FCR_BRIDGE_INDUCTION_MODE");
        cfg.bridge_enable = (be && atoi(be) > 0) ? 1u : 0u;
        cfg.bridge_induction_mode = (bm && atoi(bm) >= 0 && atoi(bm) <= 3)
                                    ? (uint32_t)atoi(bm) : 1u;
        if (cfg.bridge_enable) {
            const char *wp = getenv("FCR_BRIDGE_ALLOC_W_PITCH");
            const char *ws = getenv("FCR_BRIDGE_ALLOC_W_SPEED");
            const char *wt = getenv("FCR_BRIDGE_ALLOC_W_TORQUE");
            uint32_t p = wp ? (uint32_t)atoi(wp) : 0u;
            uint32_t s = ws ? (uint32_t)atoi(ws) : 0u;
            uint32_t t = wt ? (uint32_t)atoi(wt) : 0u;
            uint32_t sum = p + s + t;
            if (sum == 0) { p = 1000; }   /* 默认纯 r_pitch */
            cfg.bridge_alloc_w_pitch = p;
            cfg.bridge_alloc_w_speed = s;
            cfg.bridge_alloc_w_torque = t;
        }
        /* W-10: 诱导映射参数经 env 显式注入（YAML 未加载；C 权威，env 覆盖） */
        {
            const char *am = getenv("FCR_BRIDGE_IND_A_MAX");
            const char *an = getenv("FCR_BRIDGE_IND_A_MIN");
            double v;
            if (am && (v = atof(am)) > 0.0 && v < 1.0) cfg.bridge_ind_a_max = v;
            if (an && (v = atof(an)) >= 0.0 && v < 1.0) cfg.bridge_ind_a_min = v;
        }
    }
    FCR_LOCK_INIT();
    FCR_LOCK();
    if (!g_rt) g_rt = fcr_runtime_create(&cfg);
    if (g_rt && !g_transport_done) {
        /* 离线命令注入源（Phase 3 预实现；Phase 7 由 ZeroMQ Transport 替换）。
         * 只执行一次：FAST.Farm 多台机/多线程首调并发安全。 */
        extern int fcr_offline_shm_init(void);
        fcr_offline_shm_init();
        {
            const char *cp = getenv("FCR_ZMQ_CMD_PORT");
            const char *sp = getenv("FCR_ZMQ_STATE_PORT");
            if (cp || sp) {
                extern int fcr_zmq_transport_hook(FcrRuntime *rt, int cmd_port, int state_port);
                int cport = cp ? atoi(cp) : 0;
                int sport = sp ? atoi(sp) : 0;
                /* WSL/Linux 构建可不编入 ZMQ transport（弱符号解析为 NULL）：
                 * 未链接时跳过，避免空指针调用。RT 平台由调度器负责传输。 */
#if defined(__GNUC__)
                if (fcr_zmq_transport_hook)
#endif
                    fcr_zmq_transport_hook(g_rt, cport, sport);
            }
        }
        g_transport_done = 1;
    }
    FCR_UNLOCK();
    return g_rt ? 0 : -1;
}

int fcr_rosco_api_register_turbine(int32_t turbine_id)
{
    if (!g_rt) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)FCR_MAX_TURBINES) return -2;
    FCR_LOCK();
    if (g_rt->cfg.n_turbines < (uint32_t)turbine_id)
        g_rt->cfg.n_turbines = (uint32_t)turbine_id;
    if (g_rt->ss && g_rt->ss->n_turbines < (uint32_t)turbine_id)
        g_rt->ss->n_turbines = (uint32_t)turbine_id;
    if (g_rt->cs && g_rt->cs->n_turbines < (uint32_t)turbine_id)
        g_rt->cs->n_turbines = (uint32_t)turbine_id;
    FCR_UNLOCK();
    return 0;
}

/* 诊断（Phase 8 定位用；交付可保留）：直接读 setpoint 槽的 yaw target */
double fcr_rosco_debug_yaw_target(int32_t turbine_id)
{
    if (!g_rt) return -999.0;
    if (turbine_id < 1 || turbine_id > (int32_t)FCR_MAX_TURBINES) return -999.0;
    return g_rt->setpoint[turbine_id - 1].yaw_target_heading_rad;
}

int fcr_rosco_api_shutdown(void)
{
    extern int fcr_offline_shm_fini(void);
    fcr_offline_shm_fini();
    FCR_LOCK();
    if (g_rt) { fcr_runtime_destroy(g_rt); g_rt = NULL; }
    FCR_UNLOCK();
    FCR_LOCK_FINI();
    return 0;
}

/* RT 平台要求 §3.3：run → reset → run（不重启进程）。
 * 保留 runtime 配置与 g_ack 之外的诊断统计；命令/状态/setpoint/锁存全部归零。
 * 注意：ROSCO 侧的标称配置副本（nom_ok 等）**不得**在此清除——那是真实标称值，
 * 清除后会把当前已降载的配置误当标称。 */
int fcr_rosco_api_reset(void)
{
    int rc;
    if (!g_rt) return -1;
    FCR_LOCK();
    rc = fcr_runtime_reset(g_rt);
    if (rc == 0) {
        uint32_t i;
        for (i = 0; i < FCR_MAX_TURBINES; ++i) memset(&g_ack[i], 0, sizeof(g_ack[i]));
        g_queue_depth_max = 0;
        g_lock_hold_max_ms = 0.0;
        g_lock_hold_total_ms = 0.0;
        g_lock_count = 0;
    }
    FCR_UNLOCK();
    return rc;
}

FcrRuntime *fcr_rosco_api_runtime(void)   /* 供离线 harness 使用 */
{
    return g_rt;
}

/* ------------------------------------------------------------------ */
/* ROSCO 10 ms 入口                                                   */
/* ------------------------------------------------------------------ */

int fcr_rosco_read_setpoint(int32_t turbine_id, FcrRoscoExternalSetpoint *sp)
{
    if (!g_rt || !sp) return -1;
    FCR_LOCK();
    if (turbine_id >= 1 && turbine_id <= (int32_t)FCR_MAX_TURBINES)
        *sp = g_rt->setpoint[turbine_id - 1];
    else
        memset(sp, 0, sizeof(*sp));
    FCR_UNLOCK();
    return 0;
}

int fcr_rosco_publish_state(int32_t turbine_id, const FcrRoscoRawState *st)
{
    if (!g_rt || !st) return -1;
    FCR_LOCK();
    fcr_state_store_set_fast(g_rt->ss, turbine_id, st);
    fcr_runtime_feed_fast(g_rt, turbine_id, st);   /* 统计样本喂入 */
    FCR_UNLOCK();
    return 0;
}

int fcr_rosco_publish_ack(int32_t turbine_id, const FcrRoscoAck *ack)
{
    if (!g_rt || !ack) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)FCR_MAX_TURBINES) return -2;
    FCR_LOCK();
    g_ack[turbine_id - 1] = *ack;
    FCR_UNLOCK();
    return 0;
}

/* ICD v1.5: 查询已采纳 seq（供状态帧 S1013/S1014 填充） */
int fcr_rosco_get_ack(int32_t turbine_id, FcrRoscoAck *out)
{
    if (!out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)FCR_MAX_TURBINES) return -2;
    FCR_LOCK();
    *out = g_ack[turbine_id - 1];
    FCR_UNLOCK();
    return 0;
}

/* 单机 10 ms 主步进：transport 拉取 + 命令 refresh（含 yaw 目标锁存）+ setpoint */
int fcr_rosco_step_high(int32_t turbine_id, double sim_time_s)
{
    double heading;
    FcrRoscoRawState st;
    uint64_t qdepth = 0;
    if (!g_rt) return -1;
    FCR_LOCK();
    /* Transport 命令帧拉取（Phase 7：ZeroMQ 命令通道，每 10 ms 轮询） */
    if (g_rt->cfg.transport) {
        FarmCommandFrame frame;
        while (fcr_transport_poll(g_rt->cfg.transport, &frame) == 1) {
            fcr_command_store_accept_frame(g_rt->cs, &frame);
            qdepth++;
        }
        /* WIN-14: track max queue depth per step */
        if (qdepth > g_queue_depth_max) g_queue_depth_max = qdepth;
    }
    /* 取本机最新机舱方位（来自上一次 publish_state）                  */
    memset(&st, 0, sizeof(st));
    fcr_state_store_get_fast(g_rt->ss, turbine_id, &st);
    heading = st.valid ? st.nacelle_heading_rad : (0.0 / 0.0);
    fcr_command_store_refresh_one(g_rt->cs, turbine_id, sim_time_s, heading,
                                  g_rt->ss->low_step);
    fcr_runtime_update_setpoint_one(g_rt, turbine_id);
    FCR_UNLOCK();
    return 0;
}

/* WIN-14: diagnostics — lock hold statistics */
void fcr_rosco_lock_stats(double *max_hold_ms, double *avg_hold_ms,
                          uint64_t *lock_count, uint64_t *queue_depth_max)
{
    if (max_hold_ms) *max_hold_ms = g_lock_hold_max_ms;
    if (avg_hold_ms) *avg_hold_ms = g_lock_count > 0 ? g_lock_hold_total_ms / (double)g_lock_count : 0.0;
    if (lock_count) *lock_count = g_lock_count;
    if (queue_depth_max) *queue_depth_max = g_queue_depth_max;
}
void fcr_rosco_lock_stats_reset(void)
{
    g_lock_hold_max_ms = 0.0;
    g_lock_hold_total_ms = 0.0;
    g_lock_count = 0;
    g_queue_depth_max = 0;
}

/* 命令帧注入（离线 harness / Transport 线程；非 10 ms 路径）          */
int fcr_rosco_inject_command_frame(const FarmCommandFrame *frame)
{
    int rc;
    if (!g_rt || !frame) return -1;
    FCR_LOCK();
    rc = fcr_command_store_accept_frame(g_rt->cs, frame);
    FCR_UNLOCK();
    return rc;
}

/* 低速步（1 s）：聚合 + watchdog + 可选 transport publish             */
int fcr_rosco_step_low(double sim_time_s)
{
    int rc;
    if (!g_rt) return -1;
    FCR_LOCK();
    rc = fcr_runtime_step_low(g_rt, sim_time_s);
    FCR_UNLOCK();
    return rc;
}