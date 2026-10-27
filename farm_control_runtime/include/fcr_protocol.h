/*
 * fcr_protocol.h — farm_control_runtime 公共协议定义（ICD v1.5）
 *
 * 本文件是 Farm Controller ↔ runtime ↔ ROSCO/Provider 之间所有帧与标志的
 * 唯一权威定义。所有字段类型/名称/单位冻结后不得随意更改；
 * ABI 尺寸与对齐由 tests/unit/test_abi.c 用 _Static_assert 锁定。
 *
 * 单位约定（全链路 SI）：
 *   角度: rad       时间: s        力: N        力矩: N·m
 *   功率: W         转速: rad/s    风速: m/s     系数: 无量纲
 */
#ifndef FCR_PROTOCOL_H
#define FCR_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 版本与容量常量                                                     */
/* ------------------------------------------------------------------ */

#define FCR_PROTOCOL_VERSION_V1    1u   /* 旧版本（兼容保留）            */
#define FCR_PROTOCOL_VERSION_V2    2u   /* ICD v1.5 版本                 */
#define FCR_PROTOCOL_VERSION       FCR_PROTOCOL_VERSION_V2  /* 当前版本 */

/* ------------------------------------------------------------------ */
/* 容量版本化（W-09 代码审查）：                                       */
/*   线协议容量（FCR_MAX_TURBINES / FCR_MAX_OBSERVATION_POINTS）冻结于   */
/*   当前 ABI（帧尺寸不变，见 tests/unit/test_abi.c + tools/check_abi.py）。*/
/*   为支持未来容量升级而不破坏旧 Windows 二进制，容量常量按版本命名     */
/*   （*_V1 = 当前 ABI 值），FCR_MAX_* 始终指向当前版本。帧的            */
/*   protocol_version 字段决定接收方应按哪套容量布局解析。               */
/*   所有内部消费者（bridge、legacy ObservationManager/WindMapper）必须   */
/*   引用这些常量，禁止自行放大/缩小（W-09 统一到线协议容量）。           */
/* ------------------------------------------------------------------ */
#define FCR_CAPACITY_VERSION            2u  /* 当前容量版本（V2：AWAE 1000 点） */
#define FCR_MAX_TURBINES_V1             64u /* 机组容量（V1 冻结 ABI）    */
#define FCR_MAX_OBSERVATION_POINTS_V1   64u /* AWAE 观察点容量（V1 冻结 ABI）*/
#define FCR_MAX_OBSERVATION_POINTS_V2   1000u /* AWAE 观察点容量（V2，fastfarmRT/GPU AWAE）*/
#define FCR_MAX_TURBINES                FCR_MAX_TURBINES_V1            /* 最大机组数（与 RT 侧 NT_MAX 一致） */
#define FCR_MAX_OBSERVATION_POINTS      FCR_MAX_OBSERVATION_POINTS_V2  /* 最大 AWAE 观察点数 */
#define FCR_MAX_LOAD_POINTS             16u /* 最大可配置载荷测点数      */

#define FCR_FAST_RATE_HZ          100   /* ROSCO/OF 快速态标称 10 ms     */
#define FCR_LOW_RATE_HZ             1   /* AWAE/状态帧标称 1 s            */

/* ------------------------------------------------------------------ */
/* command_status_flags — runtime 下发 ROSCO 的命令健康/降级标志       */
/* ------------------------------------------------------------------ */
#define FCR_CMD_FLAG_VALID              (1u << 0)  /* 整体命令有效    */
#define FCR_CMD_FLAG_YAW_VALID          (1u << 1)  /* 偏航通道有效    */
#define FCR_CMD_FLAG_INDUCTION_VALID    (1u << 2)  /* 诱导通道有效    */
#define FCR_CMD_FLAG_YAW_STALE          (1u << 3)  /* 偏航命令超龄    */
#define FCR_CMD_FLAG_INDUCTION_STALE    (1u << 4)  /* 诱导命令超龄    */
#define FCR_CMD_FLAG_YAW_TTL_EXPIRED    (1u << 5)  /* 偏航 TTL 到期   */
#define FCR_CMD_FLAG_INDUCTION_TTL_EXPIRED (1u << 6) /* 诱导 TTL 到期  */
#define FCR_CMD_FLAG_INVALID_SEQ        (1u << 7)  /* 检测到乱序 seq  */
#define FCR_CMD_FLAG_FALLBACK_ACTIVE    (1u << 8)  /* 处于 fallback   */
#define FCR_CMD_FLAG_CONTROL_DISABLED   (1u << 9)  /* control_enable=0 */
/* G-02（RT 平台要求 §4.1/§14）：命令值非法（NaN/Inf）被拒绝时置位。
 * 该通道不锁存、不生效；调用方可据此区分"未收到"与"收到但非法"。 */
#define FCR_CMD_FLAG_INVALID_VALUE      (1u << 10)

/* ------------------------------------------------------------------ */
/* rosco_controller_status_flags (S0301) — ROSCO 上报的状态/饱和/降级标志 */
/* ------------------------------------------------------------------ */
#define FCR_CTRL_FLAG_EXTERNAL_ENABLED      (1u << 0) /* 外部场控总使能 */
#define FCR_CTRL_FLAG_YAW_EXTERNAL          (1u << 1) /* 偏航外部目标生效 */
#define FCR_CTRL_FLAG_INDUCTION_EXTERNAL    (1u << 2) /* 诱导外部参考生效 */
#define FCR_CTRL_FLAG_YAW_TARGET_REACHED    (1u << 3) /* 偏航到达目标   */
#define FCR_CTRL_FLAG_YAW_RATE_SATURATED    (1u << 4) /* 偏航速率饱和   */
#define FCR_CTRL_FLAG_TORQUE_SATURATED      (1u << 5) /* 转矩限值饱和   */
#define FCR_CTRL_FLAG_PITCH_SATURATED       (1u << 6) /* 桨距限值饱和   */
#define FCR_CTRL_FLAG_CT_FALLBACK           (1u << 7) /* Ct 映射 fallback */

/* ------------------------------------------------------------------ */
/* rt_health_flags — 实时仿真侧健康标志（位定义由 ICD 冻结）            */
/* ------------------------------------------------------------------ */
#define FCR_RT_HEALTH_OK                (1u << 0)  /* 全部正常 */
#define FCR_RT_HEALTH_RT_OVERLOAD       (1u << 1)  /* 实时过载/抖动 */
#define FCR_RT_HEALTH_FLOW_STALE        (1u << 2)  /* 流场数据超龄 */
#define FCR_RT_HEALTH_FLOW_DROPPED      (1u << 3)  /* 流场丢帧 */
#define FCR_RT_HEALTH_FAST_DROPPED      (1u << 4)  /* 快速步丢步 */

/* 保留位：后续扩展使用 (1u << 16) 以上为供应商扩展区 */

#ifdef __cplusplus
}
#endif

#endif /* FCR_PROTOCOL_H */
