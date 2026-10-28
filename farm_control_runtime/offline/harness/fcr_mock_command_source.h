/*
 * fcr_mock_command_source.h — 场控命令源（替代 Modbus / RCP 的测试用输入）
 *
 * 用途：WSL 阶段不接入 Modbus 与 RCP 时，向调度器提供确定性的 S01 场控命令帧，
 *       以便对 OF / WD / AWAE / ROSCO 等**其他模块**做功能测试。
 *
 * 形态：事件表驱动。每个事件给出"从某时刻起、对某台机（或全场）生效的控制量"；
 *       每个 1 s（SCADA 边界）由调度器调用 fcr_mock_source_tick()，
 *       本模块合成一帧并注入 fcr_rt_set_command()。
 *
 * 与正式链路的关系：正式链路由 Modbus/ZMQ 传输 + RCP 产生 S01；
 *       本模块只替换"产生 S01 的那一端"，模块其余行为与正式链路完全一致
 *       （命令仍走 CommandStore 的 seq/TTL/stale/control_enable 状态机）。
 *
 * 线程：单线程调用（与场控模块一致）。
 */
#ifndef FCR_MOCK_COMMAND_SOURCE_H
#define FCR_MOCK_COMMAND_SOURCE_H

#include "fcr_protocol.h"
#include "fcr_state_types.h"
#include "fcr_rt.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 单个计划事件 */
typedef struct {
    double   t_s;                     /* 生效仿真时刻 [s]（t_low >= t_s 起生效） */
    int32_t  turbine_id;              /* 1..N；0 = 全场所有机组 */
    uint32_t control_enable;          /* 1 = 允许外部场控；0 = 回退 ROSCO */
    uint32_t control_mode;            /* FCR_CONTROL_MODE_*（0/1/2/3） */
    double   yaw_target_heading_rad;  /* 绝对机舱目标方位 [rad]；NAN = 不改偏航通道 */
    double   induction_ref;           /* 目标轴向诱导因子 a；NAN = 不改诱导通道 */
    double   ttl_s;                   /* 通道 TTL [s]；0 = 用模块默认（偏航 60 / 诱导 5） */
    double   duration_s;              /* 持续时间 [s]；0 = 持续到下一次同通道事件 */
} FcrMockEvent;

typedef struct FcrMockSource FcrMockSource;

/* 创建/销毁（n_turbines 与 FcrRtConfig.n_turbines 一致） */
FcrMockSource *fcr_mock_source_create(uint32_t n_turbines);
void           fcr_mock_source_destroy(FcrMockSource *src);
/* 清空事件表与内部状态（保留 n_turbines） */
void           fcr_mock_source_reset(FcrMockSource *src);

/* 追加事件（返回事件下标，失败 -1）。事件按 t_s 升序更高效，但非必需。 */
int  fcr_mock_source_add_event(FcrMockSource *src, const FcrMockEvent *ev);
/* 从 CSV 批量加载：追加到现有事件表。
 * 列：t_s,turbine_id,control_enable,control_mode,yaw_target_heading_rad,induction_ref,ttl_s,duration_s
 * 首行可为表头（以字母开头则跳过）；空值/NAN 表示"不改变"。返回加载条数，失败 -1。 */
int  fcr_mock_source_load_csv(FcrMockSource *src, const char *path);

/* 每个 SCADA 边界（1 s）调用：合成当前生效帧并注入 h。
 * 无需场控模块时（例如只测 OF/WD/AWAE），h 可传 NULL——此时仅按时间推进内部状态，
 * 可用 fcr_mock_source_frame() 取出合成结果供其他模块使用。 */
int fcr_mock_source_tick(FcrMockSource *src, FcrRtHandle *h, double t_low);

/* 读取最近一次合成的命令帧（供自测/日志/其他模块使用） */
int fcr_mock_source_frame(const FcrMockSource *src, FarmCommandFrame *out);
/* 已消费的最后一个 SCADA 时刻与帧序号（便于断言） */
double   fcr_mock_source_last_time_s(const FcrMockSource *src);
uint64_t fcr_mock_source_frame_seq(const FcrMockSource *src);

#ifdef __cplusplus
}
#endif

#endif /* FCR_MOCK_COMMAND_SOURCE_H */
