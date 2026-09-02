/*
 * fcr_runtime.h — farm_control_runtime 核心 API（Phase 1 冻结）
 *
 * runtime 是唯一中间层：接收 Transport 命令帧 → CommandStore 按通道独立
 * 维护 → YawActionManager/InductionSupervisor 生成 ROSCO setpoint →
 * ROSCO 10 ms 读取；同时收集 Provider 发布的状态 → StateAggregator
 * 多速率聚合 → Transport publish_state(FarmStateFrame) ~1 s。
 *
 * 组件（句柄式，全部 C ABI）：
 *   CommandStore / StateStore / StateAggregator / YawActionManager /
 *   InductionSupervisor / SignalStatistics / Watchdog / RoscoApi
 */
#ifndef FCR_RUNTIME_H
#define FCR_RUNTIME_H

#include "fcr_protocol.h"
#include "fcr_state_types.h"
#include "fcr_transport.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FcrRuntime            FcrRuntime;
typedef struct FcrCommandStore       FcrCommandStore;
typedef struct FcrStateStore         FcrStateStore;
typedef struct FcrStateAggregator    FcrStateAggregator;
typedef struct FcrYawActionManager   FcrYawActionManager;
typedef struct FcrInductionSupervisor FcrInductionSupervisor;
typedef struct FcrSignalStatistics   FcrSignalStatistics;
typedef struct FcrWatchdog           FcrWatchdog;
typedef struct FcrRoscoApi           FcrRoscoApi;

/* ------------------------------------------------------------------ */
/* 运行配置                                                            */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t  n_turbines;              /* 场内机组数（≤ FCR_MAX_TURBINES） */
    double    dt_high_s;               /* 高速周期（默认 0.01）             */
    double    dt_low_s;                /* 低速周期（默认 1.0）              */
    double    default_yaw_ttl_s;       /* 默认偏航 TTL（0 = 无限）          */
    double    default_induction_ttl_s; /* 默认诱导 TTL（0 = 无限）          */
    double    stale_threshold_s;       /* 命令超龄阈值（默认 5.0）          */
    double    flow_stale_threshold_s;  /* 流场超龄阈值（默认 3.0）          */
    double    yaw_rate_limit_rad_s;    /* 偏航速率限值（供监督用，rad/s）   */
    FcrTransport *transport;           /* 挂载的 Transport（可空）          */
} FcrRuntimeConfig;

/* ------------------------------------------------------------------ */
/* 生命周期                                                            */
/* ------------------------------------------------------------------ */

/* 创建 runtime（内部按 config 创建全部组件并互连）                     */
FcrRuntime *fcr_runtime_create(const FcrRuntimeConfig *cfg);
/* 销毁 runtime，释放全部资源                                          */
int fcr_runtime_destroy(FcrRuntime *rt);

/* ------------------------------------------------------------------ */
/* 步进（由宿主线程驱动）                                              */
/* ------------------------------------------------------------------ */

/* 高速步（~10 ms）：1) 从 Transport 拉取命令帧入 CommandStore           *
 * 2) 更新 yaw/induction 生效目标（新 seq 才重算）                      *
 * 3) 刷新 ROSCO setpoint 供 ROSCO 10 ms 读取                           */
int fcr_runtime_step_high(FcrRuntime *rt, double sim_time_s);

/* 低速步（~1 s）：1) 状态聚合（多速率快照 + 1 s 统计）                 *
 * 2) Watchdog 检查 3) 通过 Transport 发布 FarmStateFrame              */
int fcr_runtime_step_low(FcrRuntime *rt, double sim_time_s);

/* ------------------------------------------------------------------ */
/* 组件访问（供 Provider/Transport/测试使用）                          */
/* ------------------------------------------------------------------ */
FcrCommandStore        *fcr_runtime_command_store(FcrRuntime *rt);
FcrStateStore          *fcr_runtime_state_store(FcrRuntime *rt);
FcrStateAggregator     *fcr_runtime_aggregator(FcrRuntime *rt);
FcrYawActionManager    *fcr_runtime_yaw_manager(FcrRuntime *rt);
FcrInductionSupervisor *fcr_runtime_induction_supervisor(FcrRuntime *rt);
FcrSignalStatistics    *fcr_runtime_signal_statistics(FcrRuntime *rt);
FcrWatchdog            *fcr_runtime_watchdog(FcrRuntime *rt);
FcrRoscoApi            *fcr_runtime_rosco_api(FcrRuntime *rt);

/* ------------------------------------------------------------------ */
/* ROCO API（runtime 内部实现；Fortran 侧经 ISO_C_BINDING 调用）        */
/* 见 fcr_rosco_api.h                                                  */
/* ------------------------------------------------------------------ */

/* 每台机查询：初始化时把 turbine 槽位绑定（n_turbines 一致）           */
int fcr_runtime_bind_turbines(FcrRuntime *rt, uint32_t n_turbines);

#ifdef __cplusplus
}
#endif

#endif /* FCR_RUNTIME_H */
