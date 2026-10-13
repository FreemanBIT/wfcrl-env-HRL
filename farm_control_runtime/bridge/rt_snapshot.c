/*
 * rt_snapshot.c — 调度核三任务无锁快照层实现（M3，DR-4/DR-6）
 *
 * 无锁协议：单写者/单读者 + seq 递增 + 对齐槽。
 *   cmd/flow：Slow（写）→ Fast（读）
 *   meas：     Fast（写）→ Slow（读）
 * Fast 10ms 路径只调用 read（无锁、无分配、无等待）；write 由对应任务调用。
 * 注意：seq 用 release/acquire 语义（volatile + 编译屏障），确保数据先于 seq 可见。
 */
#include "rt_snapshot.h"
#include <stdlib.h>
#include <string.h>

struct FcrRtSnapshot {
    uint32_t n_turbines;
    FcrCmdSlot  *cmd_slots;
    FcrMeasSlot *meas_slots;
    FcrFlowSlot *flow_slots;
};

/* 编译期内存屏障（GCC/Clang/MSVC） */
#if defined(_MSC_VER)
#include <intrin.h>
#define FCR_MB() _ReadWriteBarrier()
#else
#define FCR_MB() __asm__ __volatile__("" ::: "memory")
#endif

FcrRtSnapshot *fcr_rt_snapshot_create(uint32_t n_turbines) {
    FcrRtSnapshot *s;
    if (n_turbines == 0 || n_turbines > FCR_BRIDGE_MAX_TURBINES) return NULL;
    s = (FcrRtSnapshot *)calloc(1, sizeof(FcrRtSnapshot));
    if (!s) return NULL;
    s->n_turbines = n_turbines;
    s->cmd_slots = (FcrCmdSlot *)calloc(n_turbines, sizeof(FcrCmdSlot));
    s->meas_slots = (FcrMeasSlot *)calloc(n_turbines, sizeof(FcrMeasSlot));
    s->flow_slots = (FcrFlowSlot *)calloc(n_turbines, sizeof(FcrFlowSlot));
    if (!s->cmd_slots || !s->meas_slots || !s->flow_slots) {
        fcr_rt_snapshot_destroy(s);
        return NULL;
    }
    return s;
}

void fcr_rt_snapshot_destroy(FcrRtSnapshot *s) {
    if (!s) return;
    free(s->cmd_slots);
    free(s->meas_slots);
    free(s->flow_slots);
    free(s);
}

/* ---- Slow 写命令 → Fast 读（单写单读无锁）---- */
int fcr_rt_snapshot_cmd_write(FcrRtSnapshot *s, int32_t turbine_id,
                              const FcrBridgeCmd *cmd) {
    FcrCmdSlot *slot;
    if (!s || !cmd) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)s->n_turbines) return -2;
    slot = &s->cmd_slots[turbine_id - 1];
    slot->cmd = *cmd;        /* 先写数据 */
    FCR_MB();
    slot->seq++;             /* 再发布 seq（release） */
    return 0;
}

int fcr_rt_snapshot_cmd_read(FcrRtSnapshot *s, int32_t turbine_id,
                             FcrBridgeCmd *out) {
    FcrCmdSlot *slot;
    uint64_t seq;
    if (!s || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)s->n_turbines) return -2;
    slot = &s->cmd_slots[turbine_id - 1];
    seq = slot->seq;
    FCR_MB();
    if (seq == 0) return -3;   /* 无命令 */
    *out = slot->cmd;          /* 数据在 seq 发布前已写（release） */
    return 0;
}

/* ---- Fast 写状态 → Slow 读 ---- */
int fcr_rt_snapshot_meas_write(FcrRtSnapshot *s, int32_t turbine_id,
                               const FcrBridgeMeas *meas) {
    FcrMeasSlot *slot;
    if (!s || !meas) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)s->n_turbines) return -2;
    slot = &s->meas_slots[turbine_id - 1];
    slot->meas = *meas;
    FCR_MB();
    slot->seq++;
    return 0;
}

int fcr_rt_snapshot_meas_read(FcrRtSnapshot *s, int32_t turbine_id,
                              FcrBridgeMeas *out) {
    FcrMeasSlot *slot;
    uint64_t seq;
    if (!s || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)s->n_turbines) return -2;
    slot = &s->meas_slots[turbine_id - 1];
    seq = slot->seq;
    FCR_MB();
    if (seq == 0) return -3;
    *out = slot->meas;
    return 0;
}

/* ---- Mapping 写流场 → Fast 读（ZOH）---- */
int fcr_rt_snapshot_flow_write(FcrRtSnapshot *s, int32_t turbine_id,
                               const FcrMappedWind *w, double source_time_s) {
    FcrFlowSlot *slot;
    if (!s || !w) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)s->n_turbines) return -2;
    slot = &s->flow_slots[turbine_id - 1];
    slot->wind = *w;
    slot->source_time_s = source_time_s;
    FCR_MB();
    slot->seq++;
    return 0;
}

int fcr_rt_snapshot_flow_read(FcrRtSnapshot *s, int32_t turbine_id,
                              FcrMappedWind *out, double *source_time_s) {
    FcrFlowSlot *slot;
    uint64_t seq;
    if (!s || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)s->n_turbines) return -2;
    slot = &s->flow_slots[turbine_id - 1];
    seq = slot->seq;
    FCR_MB();
    if (seq == 0) return -3;
    *out = slot->wind;
    if (source_time_s) *source_time_s = slot->source_time_s;
    return 0;
}
