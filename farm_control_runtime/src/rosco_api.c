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

#ifdef _WIN32
#include <windows.h>
static CRITICAL_SECTION g_lock;
static int g_lock_init = 0;
#define FCR_LOCK_INIT() do { InitializeCriticalSection(&g_lock); g_lock_init = 1; } while (0)
#define FCR_LOCK_FINI() do { if (g_lock_init) DeleteCriticalSection(&g_lock); g_lock_init = 0; } while (0)
#define FCR_LOCK() EnterCriticalSection(&g_lock)
#define FCR_UNLOCK() LeaveCriticalSection(&g_lock)
#else
#include <pthread.h>
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
#define FCR_LOCK_INIT() ((void)0)
#define FCR_LOCK_FINI() ((void)0)
#define FCR_LOCK() pthread_mutex_lock(&g_lock)
#define FCR_UNLOCK() pthread_mutex_unlock(&g_lock)
#endif

static FcrRuntime *g_rt = NULL;

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
    cfg.default_induction_ttl_s = 1.0;
    cfg.stale_threshold_s = 5.0;
    cfg.flow_stale_threshold_s = 3.0;
    cfg.transport = NULL;
    FCR_LOCK_INIT();
    FCR_LOCK();
    if (!g_rt) g_rt = fcr_runtime_create(&cfg);
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

int fcr_rosco_api_shutdown(void)
{
    FCR_LOCK();
    if (g_rt) { fcr_runtime_destroy(g_rt); g_rt = NULL; }
    FCR_UNLOCK();
    FCR_LOCK_FINI();
    return 0;
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

int fcr_rosco_publish_state(int32_t turbine_id, const FcrRoscoFastState *st)
{
    if (!g_rt || !st) return -1;
    FCR_LOCK();
    fcr_state_store_set_fast(g_rt->ss, turbine_id, st);
    FCR_UNLOCK();
    return 0;
}

/* 单机 10 ms 主步进：命令 refresh（含 yaw 目标锁存）+ setpoint 生成   */
int fcr_rosco_step_high(int32_t turbine_id, double sim_time_s)
{
    double heading;
    FcrRoscoFastState st;
    if (!g_rt) return -1;
    FCR_LOCK();
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
