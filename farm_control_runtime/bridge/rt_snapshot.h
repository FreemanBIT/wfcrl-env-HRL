/*
 * rt_snapshot.h — 调度核三任务无锁快照层（M3，DR-4/DR-6）
 *
 * 三任务（Fast 10ms / Slow 1s / Mapping 1s）之间的无锁交接协议：
 *
 *   Slow Task（1s）          Fast Task（10ms）
 *   ─────────────            ─────────────
 *   cmd_snapshot_write() →   cmd_snapshot_read()  命令下行
 *   state_snapshot_read() ←  state_snapshot_write() 状态上行
 *   flow_snapshot_write() →  flow_snapshot_read()  流场下行（Mapping→Fast）
 *
 * 无锁正确性（DR-4）：
 *   - 单写者/单读者（Slow 写命令，Fast 读；Fast 写状态，Slow 读）
 *   - seq 递增 + 对齐槽（每槽 cache-line 对齐，≤64B 拷贝无撕裂）
 *   - 10ms 路径（read）只读、无锁、无分配、无等待（DR-6：不等 AWAE/WD/通信）
 *
 * 本层不依赖任何通信/存储；vendor 调度核只需在对应任务回调里调用
 * snapshot_write/read 即可完成接入（DR-6：不改九步数值算法，只挂钩子）。
 */
#ifndef FCR_RT_SNAPSHOT_H
#define FCR_RT_SNAPSHOT_H

#include "fcr_bridge_core.h"   /* FcrBridgeCmd / FcrBridgeMeas */
#include "wind_mapper.h"  /* FcrMappedWind */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 快照槽（cache-line 对齐，64B 对齐保证单缓存行） */
#if defined(_MSC_VER)
#define FCR_CACHELINE_ALIGN __declspec(align(64))
#else
#define FCR_CACHELINE_ALIGN __attribute__((aligned(64)))
#endif

typedef struct {
    FCR_CACHELINE_ALIGN volatile uint64_t seq;
    FcrBridgeCmd cmd;
} FcrCmdSlot;

typedef struct {
    FCR_CACHELINE_ALIGN volatile uint64_t seq;
    FcrBridgeMeas meas;
} FcrMeasSlot;

typedef struct {
    FCR_CACHELINE_ALIGN volatile uint64_t seq;
    FcrMappedWind wind;
    double source_time_s;
} FcrFlowSlot;

typedef struct FcrRtSnapshot FcrRtSnapshot;

/* 创建（n_turbines；编译期分配静态槽） */
FcrRtSnapshot *fcr_rt_snapshot_create(uint32_t n_turbines);
void fcr_rt_snapshot_destroy(FcrRtSnapshot *s);

/* ---- Slow Task 写命令（每机一槽）---- */
int fcr_rt_snapshot_cmd_write(FcrRtSnapshot *s, int32_t turbine_id,
                              const FcrBridgeCmd *cmd);

/* ---- Fast Task 读命令（10ms，无锁）---- */
int fcr_rt_snapshot_cmd_read(FcrRtSnapshot *s, int32_t turbine_id,
                             FcrBridgeCmd *out);

/* ---- Fast Task 写状态（10ms）---- */
int fcr_rt_snapshot_meas_write(FcrRtSnapshot *s, int32_t turbine_id,
                               const FcrBridgeMeas *meas);

/* ---- Slow Task 读状态（1s）---- */
int fcr_rt_snapshot_meas_read(FcrRtSnapshot *s, int32_t turbine_id,
                              FcrBridgeMeas *out);

/* ---- Mapping Task 写流场（1s，WindMapper 输出）---- */
int fcr_rt_snapshot_flow_write(FcrRtSnapshot *s, int32_t turbine_id,
                               const FcrMappedWind *w, double source_time_s);

/* ---- Fast Task 读流场（10ms，ZOH）---- */
int fcr_rt_snapshot_flow_read(FcrRtSnapshot *s, int32_t turbine_id,
                              FcrMappedWind *out, double *source_time_s);

#ifdef __cplusplus
}
#endif

#endif /* FCR_RT_SNAPSHOT_H */
