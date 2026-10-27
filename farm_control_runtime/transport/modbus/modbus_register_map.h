/*
 * modbus_register_map.h — Modbus TCP 寄存器映射（ICD v1.5 对齐）
 *
 * 寄存器均为 16 bit 保持寄存器（Holding Registers），字节序 big-endian
 * （Modbus 标准）。64 位量 = 4 寄存器；32 位 = 2；16 位 = 1。
 * 单精度浮点（IEEE754）双寄存器；双精度浮点四寄存器。
 *
 * 注意：ICD v1.5 明确**不定义 Modbus 地址**（§5.3 G-6）；本映射是本工程的
 * 内部实现选择，字段语义与 ICD 信号保持一致，但地址不构成接口约束。
 *
 * 地址段（DR-8 冻结 v2）：
 *   0x0000..0x07FF  命令区（RCP → runtime；每机 64 寄存器）
 *   0x0800..0x0FFF  时钟/健康区（runtime → RCP；含协议版本/完整性）
 *   0x1000..0x17FF  状态区·机组 1..N（runtime → RCP；每机 128 寄存器）
 *   0x2000..0x2FFF  AWAE 观察点帧缓冲（runtime → RCP；环形，每点 32 寄存器）
 */
#ifndef MODBUS_REGISTER_MAP_H
#define MODBUS_REGISTER_MAP_H

/* 容量常量（AWAE 点容量随 V1/V2 版本变化，寄存器区总长据此计算） */
#include "fcr_protocol.h"

/* ---- 命令区：每机 64 寄存器，从 0x0000 + (turbine_id-1)*64 ---- */
#define MB_CMD_BASE              0x0000
#define MB_CMD_PER_TURBINE       64

#define MB_CMD_YAW_TARGET_HEADING  0   /* S0108: f64 绝对机舱目标方位 (rad)  */
#define MB_CMD_YAW_SEQ             4   /* S0109: u64                          */
#define MB_CMD_YAW_VALID           8   /* S0110: u16 (1=valid)                */
#define MB_CMD_REQ_EFFECTIVE_TIME  9   /* S0112: f64 期望生效仿真时刻 (s)     */
#define MB_CMD_YAW_TTL_S          13   /* f64 (s)                             */

#define MB_CMD_IND_REF            17   /* S0105: f64 目标轴向诱导 a           */
#define MB_CMD_IND_SEQ            21   /* S0106: u64                          */
#define MB_CMD_IND_VALID          25   /* S0107: u16                          */
#define MB_CMD_CONTROL_ENABLE     26   /* S0104: u16 (1=允许外部场控)         */
#define MB_CMD_CONTROL_MODE       27   /* S0111: u16 (0基线/1PRC/2Ct/3闭环)   */
#define MB_CMD_IND_TTL_S          30   /* f64 (s)                             */

#define MB_CMD_COMMAND_BASIS_TIME 34   /* S0102: f64 命令依据的仿真时刻 (s)   */
#define MB_CMD_COMMAND_FRAME_SEQ  38   /* S0101: u64 命令帧序号               */
/* 42..63 保留（v1.4 的 commit_seq 已按 ICD v1.5 删除） */

/* ---- 时钟/健康区 ---- */
#define MB_CLK_STATE_SAMPLE_TIME  0x0800  /* S1002: f64 状态采样仿真时刻 */
#define MB_CLK_FAST_STEP          0x0804  /* u64 */
#define MB_CLK_LOW_STEP           0x0808  /* u64 */
#define MB_CLK_RT_HEALTH          0x080C  /* u16 */
#define MB_CLK_CMD_FRAME_SEQ_SLOT 0x080D  /* S0101 镜像 u64（新命令触发） */
#define MB_CLK_PROTOCOL_VERSION   0x0815  /* u16（DR-8：协议版本） */
#define MB_CLK_CRC16              0x0816  /* u16（时钟区 CRC16 完整性，DR-8） */

/* ---- 状态区质量/完整性（每机 128 寄存器内的保留尾部，DR-8）----
 * 修正（ICD v1.5 Phase B）：原布局把 u64 timestamp 放在偏移 125，会跨越
 * 本机 128 寄存器块边界（125..128）并与 CRC16/SEQ 槽重叠。ICD v1.5 §5.3 G-6
 * 明确"不产生 Modbus 地址约束"，故此处重新排布到互不重叠的 102..112。
 * 这些槽目前**尚未由 modbus_transport.c 实现**（DR-8 完整性待落地）。 */
#define MB_ST_DATA_VERSION       102  /* u16（数据版本） */
#define MB_ST_QUALITY_FLAGS      103  /* u16（bit0 valid/bit1 stale/bit2 fallback） */
#define MB_ST_TIMESTAMP_SLOT     104  /* u64（帧时间戳，104..107） */
#define MB_ST_SEQ_SLOT           108  /* u64（帧序号，108..111） */
#define MB_ST_CRC16              112  /* u16（本机状态区 CRC16，DR-8） */

/* ---- 状态区（只读保持寄存器）：每机 128 寄存器，0x1000 + (tid-1)*128 ---- */
#define MB_STATE_BASE            0x1000
#define MB_STATE_PER_TURBINE     128

#define MB_ST_TURBINE_ID         0   /* u16            */
#define MB_ST_VALID              1   /* u16            */
#define MB_ST_SOURCE_TIME        2   /* f64            */
#define MB_ST_FAST_STEP          6   /* u64            */
#define MB_ST_GEN_POWER_W        10  /* S1004: f64    */
#define MB_ST_GEN_POWER_MEAN     14  /* S1005: f64    */
#define MB_ST_GEN_POWER_RMS      18  /* f64（扩展，非 ICD 信号） */
#define MB_ST_ROTOR_AERO_THRUST  22  /* S1010: f64 */
/* 注：S0209 shaft_torque_nm 属于 OpenFAST RawState，不在 S10 状态帧内，
 * 因此本寄存器承载 S10 实际提供的塔基前后向弯矩（历史上被误标为 shaft_torque）。 */
#define MB_ST_TOWER_BASE_FA_MOM  26  /* f64 (N·m) 塔基前后向弯矩 */
#define MB_ST_NACELLE_HEADING    30  /* S1009: f64 (rad, 实际) */
#define MB_ST_NACELLE_VANE       34  /* f64 (rad)     */
#define MB_ST_HUB_WIND_MPS       38  /* f64           */
#define MB_ST_ROTOR_SPEED        42  /* f64 (rad/s)   */
#define MB_ST_GEN_SPEED          46  /* f64 (rad/s)   */
#define MB_ST_CT                 50  /* f64           */
#define MB_ST_CP                 54  /* f64           */
#define MB_ST_YAW_TARGET_HEADING 58  /* f64 (rad)     */
#define MB_ST_YAW_SEQ_APPLIED    62  /* S1013: u64    */
#define MB_ST_IND_SEQ_APPLIED    66  /* S1014: u64    */
#define MB_ST_CTRL_STATUS_FLAGS  70  /* u16 (扩展)    */
#define MB_ST_DISK_AMB_WIND      71  /* f64 (m/s)     */
#define MB_ST_DISK_DIST_WIND     75  /* f64 (m/s)     */
#define MB_ST_DISK_AMB_TI        79  /* f64           */
#define MB_ST_FLOW_SOURCE_TIME   83  /* f64 (s)       */
#define MB_ST_FLOW_SEQ           87  /* u64           */
#define MB_ST_EST_WIND_MPS       91  /* S1011: f64    */
#define MB_ST_CT_HAT             95  /* S1012: f64    */
#define MB_ST_WIND_SOURCE        99  /* S1016: u16    */
#define MB_ST_WIND_QUALITY      100  /* S1017: u16    */
#define MB_ST_TURBINE_STATUS    101  /* S1015: u16    */

/* ---- AWAE 观察点帧缓冲（ICD v1.5 GG-12/GG-13）：0x3000 起，每点 32 寄存器 ----
 * 修正（Phase C）：原置于 0x2000，与状态区 0x1000..0x2FFF（64 机 × 128）在
 * ≥33 机时**地址冲突**（0x2000 = 状态区第 33 机起）。ICD v1.5 §5.3 G-6 明确
 * 不约束 Modbus 地址，故移到状态区之后。寄存器区总长随之扩到 0x4000。
 * 本区当前由 modbus_transport.c 的 publish_awae 写入。 */
#define MB_AWAE_BASE             0x3000
#define MB_AWAE_PER_POINT        32
#define MB_AWAE_N_POINTS_SLOT    0x3000   /* S0503: u16（当前帧点数） */
#define MB_AWAE_FRAME_SEQ_SLOT   0x3001   /* S0501: u64（帧序号，0x3001..0x3004） */
#define MB_AWAE_SOURCE_TIME      0x3005   /* S0502: f64（采样仿真时间，0x3005..0x3008） */
#define MB_AWAE_POINT_0          0x3009   /* 点 0 起始（末点 0x3808） */
/* 单点字段（相对点起始的寄存器下标）。
 * ⚠ 修正（Phase C）：原布局把 f64 当作 2 寄存器，导致 3×f64 坐标跨到
 *   1..6 并与 wind_xyz(7) 重叠、且与 mb_put_double 的 4 寄存器写法不符。
 *   现按 f64 = 4 寄存器排布：1 + 12 + 12 + 4 + 1 + 1 + 1(保留) = 32。
 *   同时删除 wind_speed / wind_direction —— 二者非 ICD v1.5 信号
 *   （S0505 wind_uvw 已含矢量），保留会使单点超过 32 寄存器。 */
#define MB_AWAE_POINT_POINT_ID   0        /* S0504: u16（点 ID） */
#define MB_AWAE_POINT_POS_XYZ    1        /* S0402: 3×f64（坐标，1..12） */
#define MB_AWAE_POINT_WIND_XYZ   13       /* S0505: 3×f64（风矢量分量，13..24） */
#define MB_AWAE_POINT_TI         25       /* S0507: f64（湍流强度，25..28） */
#define MB_AWAE_POINT_QUALITY    29       /* S0506: u16（质量标志） */
#define MB_AWAE_POINT_CRC16      30       /* u16（单点完整性） */
/* 31: 保留（对齐 32 寄存器/点） */

/* ---- 寄存器区总长 ----
 * 随 AWAE 容量扩展：V2（1000 点）时 = 0x3009 + 1000×32 ≈ 44.3k 寄存器，
 * 仍低于 Modbus 单从站 65536 寄存器上限。modbus_transport_create 按此分配。 */
#define MB_AWAE_POINT_0_REG      MB_AWAE_POINT_0
#define MB_TOTAL_REGS            (MB_AWAE_POINT_0_REG + \
                                  (int)FCR_MAX_OBSERVATION_POINTS * MB_AWAE_PER_POINT)

#endif /* MODBUS_REGISTER_MAP_H */
