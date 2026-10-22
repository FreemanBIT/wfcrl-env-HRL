/*
 * fcr_rt.h — 场控交互模块的实时平台（调度器）接入接口
 *
 * 依据《风电控制策略实时验证平台子模块开发、接入与测试要求》：
 *   §2.2 模块定位 / §3 生命周期 / §5 时间接口 / §6 状态与错误 / §6.1 降级
 *
 * 约定（与平台方 2026-10-20 确认）：
 *   - 调度器周期 10 ms，本模块每 10 ms 被调用一次（fcr_rt_step）；
 *   - DT_SCADA = 1 s：对外（RCP）数据交换按 1 s 边界进行；
 *   - 时间上下文由调度器以只读指针传入（Q2 方案 A）；
 *   - 状态以"同步返回值 + 详情结构"回传（Q3 方案 A）；
 *   - 传输（ZMQ/Modbus）由调度器负责，本模块只提供接口数据（不持有 socket）；
 *   - 输出在下一个 DT_Low 生效（S0112 语义）；
 *   - 降级（D0~D4）由本模块自行判定并上报。
 *
 * 红线：本接口不分配/不阻塞/不做 I/O；fcr_rt_step 内不得动态申请内存。
 */
#ifndef FCR_RT_H
#define FCR_RT_H

#include "fcr_protocol.h"
#include "fcr_state_types.h"
#include "fcr_awae_types.h"
#include "fcr_rosco_api.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/* 1. 调度器时间上下文（要求 §5.1；只读传入）                          */
/* ================================================================== */
typedef struct {
    int32_t n_low;        /* DT_Low 步号，0 起 —— 唯一权威 */
    double  t_low;        /* = n_low * DT_Low [s] */
    double  dt_low;       /* 全局步进 [s]（实际 1.0） */
    int32_t n_fast_low;   /* 本 DT_Low 内已推进的 10 ms 子步数 */
    double  dt_high;      /* 高分辨风时间片间隔 [s]（0.01） */
    int32_t n_high_low;   /* 本 DT_Low 内风时间片数 */
} FcrRtTimeCtx;

/* ================================================================== */
/* 2. 模块状态与错误（要求 §6）                                        */
/* ================================================================== */
typedef enum {
    FCR_RT_OK = 0,           /* 正常 */
    FCR_RT_WARNING,          /* 可继续，但需记录 */
    FCR_RT_STALE_DATA,       /* 数据超龄（命令/流场）→ D1 */
    FCR_RT_INVALID_INPUT,    /* 非法输入（NaN/越界/时间回退）→ 拒绝 */
    FCR_RT_NOT_READY,        /* 未初始化/已停止 */
    FCR_RT_TIMEOUT,          /* 命令 TTL 到期 → D3 退出外部控制 */
    FCR_RT_NUMERICAL_ERROR,  /* 数值异常（状态量 NaN）→ D4 */
    FCR_RT_COMM_ERROR,       /* 通信异常（传输归调度器，保留用于上报） */
    FCR_RT_FATAL             /* 内部不变量破坏 → D4 */
} FcrRtStatus;

/* ErrorSource 子模块编号（要求 §6 "判断异常来自哪个模块"） */
#define FCR_RT_SRC_NONE        0u
#define FCR_RT_SRC_TIME        1u   /* 时间上下文 */
#define FCR_RT_SRC_COMMAND     2u   /* 场控命令 */
#define FCR_RT_SRC_STATE       3u   /* 平台状态 */
#define FCR_RT_SRC_BRIDGE      4u   /* 诱导桥接 */
#define FCR_RT_SRC_TRANSPORT   5u   /* 传输（保留编号） */
#define FCR_RT_SRC_INTERNAL    6u   /* 内部不变量 */

typedef struct {
    FcrRtStatus status;          /* 结构化状态 */
    int32_t     error_code;      /* 模块内细分码（0 = 无） */
    uint16_t    error_source;    /* FCR_RT_SRC_* */
    char        message[64];     /* 人类可读说明（可为空） */
    double      timestamp_s;     /* 发生时刻（仿真时间） */
} FcrRtStatusInfo;

/* ================================================================== */
/* 3. 配置（要求 §3 CONFIGURE；不依赖环境变量）                        */
/* ================================================================== */
typedef struct {
    uint32_t n_turbines;              /* 1..FCR_MAX_TURBINES */
    double   dt_high_s;               /* 0.01 */
    double   dt_low_s;                /* 1.0 */
    double   dt_scada_s;              /* 场控交换周期，默认 1.0；须与 dt_low 可整除 */
    double   default_yaw_ttl_s;       /* 偏航 TTL（最大保持时间），默认 60 */
    double   default_induction_ttl_s; /* 诱导 TTL，默认 5 */
    double   stale_threshold_s;       /* 命令超龄阈值，默认 5 */
    double   flow_stale_threshold_s;  /* 流场超龄阈值，默认 3 */
    uint32_t bridge_enable;           /* 1 = bridge core 生成诱导 setpoint */
    uint32_t bridge_induction_mode;   /* 0/1/2/3 */
    uint32_t bridge_alloc_w_pitch;    /* 千分比 */
    uint32_t bridge_alloc_w_speed;
    uint32_t bridge_alloc_w_torque;
    double   bridge_ind_a_max;        /* 0 = core 默认 1/3 */
    double   bridge_ind_a_min;        /* 0 = core 默认 0.05 */
} FcrRtConfig;

typedef struct FcrRtHandle FcrRtHandle;

/* ================================================================== */
/* 4. 生命周期（要求 §3；语义：create→configure→init→step→reset→stop→destroy） */
/* ================================================================== */
FcrRtHandle *fcr_rt_create(void);                                   /* CREATE */
int  fcr_rt_configure(FcrRtHandle *h, const FcrRtConfig *cfg);      /* CONFIGURE */
int  fcr_rt_init(FcrRtHandle *h);                                   /* INITIALIZE */
int  fcr_rt_reset(FcrRtHandle *h);                                  /* RESET */
int  fcr_rt_stop(FcrRtHandle *h);                                   /* STOP */
void fcr_rt_destroy(FcrRtHandle *h);                                /* DESTROY */

/* ================================================================== */
/* 5. 输入（由调度器搬运；本模块不持有 socket）                        */
/* ================================================================== */
/* 场控命令（S01 帧）：调度器在 1 s 边界送入；模块在下一个 10 ms 步落地 */
int fcr_rt_set_command(FcrRtHandle *h, const FarmCommandFrame *frame);
/* 平台状态：S02/S03 快速状态（每机）、S02 额外量、AWAE 流场、S06 风估计 */
int fcr_rt_set_state(FcrRtHandle *h, int32_t turbine_id, const FcrRoscoRawState *st);
int fcr_rt_set_extra(FcrRtHandle *h, const FcrTurbineExtraState *st);
int fcr_rt_set_flow(FcrRtHandle *h, const FcrFlowState *flow);
int fcr_rt_set_wind_estimate(FcrRtHandle *h, const FcrTurbineWindEstimate *est);
/* AWAE 静态测点目录（S04 注入；离线/外部提供） */
int fcr_rt_set_awae_catalog(FcrRtHandle *h, const FcrAWAECatalogFrame *cat);

/* ================================================================== */
/* 6. 步进（每 10 ms 调用一次；要求 §3.2/§5.4）                        */
/* ================================================================== */
/* 返回值为快速判断；out_status（可空）返回详情。
 * 不推进/不修改调度器时间；不阻塞；不分配内存。 */
FcrRtStatus fcr_rt_step(FcrRtHandle *h, const FcrRtTimeCtx *t,
                        FcrRtStatusInfo *out_status);

/* ================================================================== */
/* 7. 输出（调度器在 1 s 边界取走）                                    */
/* ================================================================== */
/* S10 状态帧：仅当自上次取走后产生过新的 1 s 帧时返回 1，否则 0 */
int fcr_rt_get_output(FcrRtHandle *h, FarmStateFrame *out);
/* S08 ROSCO setpoint（10 ms 级，供单机控制读取；RT 下由调度器搬运） */
int fcr_rt_get_setpoint(FcrRtHandle *h, int32_t turbine_id,
                        FcrRoscoExternalSetpoint *out);
/* S12/S13 AWAE 二跳帧（1 s 级） */
int fcr_rt_get_awae(FcrRtHandle *h, FcrAWAEObservationFrame *obs,
                    FcrAWAECatalogFrame *cat);

/* 状态查询（诊断/测试） */
int fcr_rt_get_status(const FcrRtHandle *h, FcrRtStatusInfo *out);

/* 状态码转字符串（日志/测试用） */
const char *fcr_rt_status_str(FcrRtStatus s);

#ifdef __cplusplus
}
#endif

#endif /* FCR_RT_H */
