/*
 * fcr_rosco_api.h — runtime ↔ ROSCO C/Fortran ABI（Phase 1 冻结）
 *
 * 边界规则（红线）：
 *   - ROSCO 10 ms 控制路径内只允许调用 fcr_rosco_read_setpoint 与
 *     fcr_rosco_publish_state（共享内存读写，无 socket/文件）。
 *   - setpoint 由 runtime 在命令生效/1 s ZOH 时更新，ROSCO 侧只读。
 *   - 角度一律使用 OpenFAST 内部约定（CCW+，0=+X）；CW+ → CCW 的
 *     转换已封装在 runtime（angle_convention.c），RT 侧不得再次反号。
 */
#ifndef FCR_ROSCO_API_H
#define FCR_ROSCO_API_H

#include "fcr_protocol.h"
#include "fcr_state_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 外部设定点（runtime → ROSCO，10 ms 读取）                           */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t  valid;                  /* 本组设定点有效                     */
    uint32_t  farm_control_enable;    /* 外部场控总使能（0 = 完全回退）     */

    /* ---- 偏航通道 ---- */
    uint32_t  yaw_enable;             /* 外部偏航目标使能                   */
    uint64_t  yaw_seq;                /* 当前生效动作序号                   */
    double    yaw_target_heading_rad; /* 绝对机舱目标方位（锁存，OpenFAST 内部约定） */

    /* ---- 诱导通道 ---- */
    uint32_t  induction_enable;       /* 诱导控制使能                       */
    uint64_t  induction_seq;          /* 当前生效动作序号                   */
    double    ct_ref;                 /* 目标推力系数（无量纲）             */
    double    power_ratio;            /* 功率参考比例（1.0 = 额定）         */
    double    speed_ref_ratio;        /* 转速参考比例（1.0 = 额定）         */
    double    torque_limit_ratio;     /* 转矩限值比例（1.0 = 额定）         */
    double    min_pitch_rad;          /* 最小桨距约束（rad）                */

    uint32_t  command_status_flags;   /* FCR_CMD_FLAG_*                     */
} FcrRoscoExternalSetpoint;

typedef struct FcrRoscoApi FcrRoscoApi;

/* ------------------------------------------------------------------ */
/* C ABI 入口（runtime 实现；Fortran 通过 ISO_C_BINDING 绑定）          */
/* 返回 0 成功；<0 出错。turbine_id 从 1 开始。                         */
/* ------------------------------------------------------------------ */

/* ROSCO 每 ~10 ms 调用：读取最新外部设定点                              */
int fcr_rosco_read_setpoint(int32_t turbine_id, FcrRoscoExternalSetpoint *sp);

/* ROSCO 每 ~10 ms 调用：发布快速状态                                   */
int fcr_rosco_publish_state(int32_t turbine_id, const FcrRoscoFastState *st);

#ifdef __cplusplus
}
#endif

#endif /* FCR_ROSCO_API_H */
