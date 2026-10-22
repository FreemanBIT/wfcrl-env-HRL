/*
 * fcr_rosco_api.h — runtime ↔ ROSCO C/Fortran ABI（ICD v1.5）
 *
 * 对应信号组:
 *   S0801-S0807  Bridge_to_ROSCO
 *   S0901-S0904  Bridge_Ack_to_Scheduler
 *
 * 边界规则（红线）：
 *   - ROSCO 10 ms 控制路径内只允许调用 fcr_rosco_read_setpoint 与
 *     fcr_rosco_publish_state（共享内存读写，无 socket/文件）。
 *   - setpoint 由 runtime 在命令生效/1 s ZOH 时更新，ROSCO 侧只读。
 *   - 角度一律使用 OpenFAST 内部约定（CCW+，0=+X）。
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
/* FcrRoscoExternalSetpoint — Bridge_to_ROSCO (S0801-S0807)            */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t  valid;                       /* 本组设定点有效 */
    uint32_t  farm_control_enable;         /* S0104/S0704: 外部场控总使能 */

    /* ---- 偏航通道 (S0801-S0802) ---- */
    uint32_t  external_yaw_enable;         /* S0801: 外部偏航使能 */
    uint64_t  yaw_seq;                     /* S0708: 当前生效偏航序号 */
    double    yaw_target_heading_rad;      /* S0802: 绝对机舱目标方位 */

    /* ---- 诱导通道 (S0803-S0807) ---- */
    uint32_t  induction_enable;            /* S0107: 诱导控制使能 */
    uint64_t  induction_seq;               /* S0706: 当前生效诱导序号 */
    double    ct_ref;                      /* S0904: 目标推力系数 */
    double    r_pitch;                     /* S0803: PRC桨距降载比例 */
    double    r_speed;                     /* S0804: PRC转速参考比例 */
    double    r_torque;                    /* S0805: PRC转矩参考/额定比例 */
    double    min_pitch_ref_rad;           /* S0806: 额外最小桨距约束 */
    double    collective_pitch_offset_rad; /* S0807: 集体桨距修正量 */

    uint32_t  command_status_flags;        /* FCR_CMD_FLAG_* */

    /* S1101: Future_CT_Feedback —— 仅保留接口（本期不实现闭环，恒为 0）。
     * 追加在末尾：既有字段偏移不变（ABI 尺寸 104 → 112，check_abi/test_abi
     * 同步锁定）；供 RT 侧将来闭环 Ct 跟踪误差回传。 */
    double    ct_tracking_error;
} FcrRoscoExternalSetpoint;

/* ------------------------------------------------------------------ */
/* FcrRoscoAck — Bridge_Ack_to_Scheduler (S0901-S0904)                 */
/* ------------------------------------------------------------------ */
typedef struct {
    uint64_t  induction_seq_applied;       /* S0901 */
    uint64_t  yaw_seq_applied;             /* S0902 */
    uint32_t  bridge_status_flags;         /* S0903 */
    double    ct_ref;                      /* S0904 */
} FcrRoscoAck;

typedef struct FcrRoscoApi FcrRoscoApi;

/* ------------------------------------------------------------------ */
/* C ABI 入口（runtime 实现；Fortran 通过 ISO_C_BINDING 绑定）          */
/* ------------------------------------------------------------------ */

/* ROSCO 每 ~10 ms 调用：读取最新外部设定点 */
int fcr_rosco_read_setpoint(int32_t turbine_id, FcrRoscoExternalSetpoint *sp);

/* ROSCO 每 ~10 ms 调用：发布快速状态 */
int fcr_rosco_publish_state(int32_t turbine_id, const FcrRoscoRawState *st);

/* ROSCO 每 ~10 ms 调用：发布确认信息 */
int fcr_rosco_publish_ack(int32_t turbine_id, const FcrRoscoAck *ack);

/* 读取最新 Bridge 确认（S09，GG-09 回读路径）。
 * 注：本原型此前缺失，调用方依赖隐式声明（C99 起非法），现补上。 */
int fcr_rosco_get_ack(int32_t turbine_id, FcrRoscoAck *out);

/* RT 平台要求 §3.3：run → reset → run（不重启进程）。
 * 清空 runtime 命令/状态/setpoint/锁存与 ack；保留配置与 ROSCO 标称副本。
 * 未初始化返回 -1；成功 0。 */
int fcr_rosco_api_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* FCR_ROSCO_API_H */
