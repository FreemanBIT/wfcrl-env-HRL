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

/* ---- 状态区质量/完整性（每机 128 寄存器尾部，DR-8）---- */
#define MB_ST_DATA_VERSION       0x7B   /* u16（数据版本） */
#define MB_ST_QUALITY_FLAGS      0x7C   /* u16（bit0 valid/bit1 stale/bit2 fallback） */
#define MB_ST_TIMESTAMP_SLOT     0x7D   /* u64（帧时间戳） */
#define MB_ST_CRC16              0x7E   /* u16（本机状态区 CRC16，DR-8） */
#define MB_ST_SEQ_SLOT           0x7F   /* u64（帧序号） */

/* ---- 状态区（只读保持寄存器）：每机 128 寄存器，0x1000 + (tid-1)*128 ---- */
#define MB_STATE_BASE            0x1000
#define MB_STATE_PER_TURBINE     128

#define MB_ST_TURBINE_ID         0   /* u16            */
#define MB_ST_VALID              1   /* u16            */
#define MB_ST_SOURCE_TIME        2   /* f64            */
#define MB_ST_FAST_STEP          6   /* u64            */
#define MB_ST_GEN_POWER_W        10  /* S1004: f64    */
#define MB_ST_GEN_POWER_MEAN     14  /* S1005: f64    */
#define MB_ST_GEN_POWER_RMS      18  /* S1006: f64    */
#define MB_ST_ROTOR_AERO_THRUST  22  /* S1010: f64    */
#define MB_ST_SHAFT_TORQUE_NM    26  /* S0209: f64    */
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

/* ---- AWAE 观察点帧缓冲（DR-8）：0x2000 起，每点 32 寄存器 ---- */
#define MB_AWAE_BASE             0x2000
#define MB_AWAE_PER_POINT        32
#define MB_AWAE_N_POINTS_SLOT    0x2000   /* S0503: u16（当前帧点数） */
#define MB_AWAE_FRAME_SEQ_SLOT   0x2001   /* S0501: u64（帧序号） */
#define MB_AWAE_SOURCE_TIME      0x2005   /* S0502: f64（采样仿真时间） */
#define MB_AWAE_POINT_0          0x2009   /* 点 0 起始 */
#define MB_AWAE_POINT_POINT_ID   0        /* S0504: u16（点 ID） */
#define MB_AWAE_POINT_POS_XYZ    1        /* S0402: 3×f64（坐标） */
#define MB_AWAE_POINT_WIND_XYZ   7        /* S0505: 3×f64（风矢量分量） */
#define MB_AWAE_POINT_WIND_SPEED 13       /* f64（幅值） */
#define MB_AWAE_POINT_WIND_DIR   17       /* f64（风向 rad） */
#define MB_AWAE_POINT_TI         21       /* S0507: f64（湍流强度） */
#define MB_AWAE_POINT_QUALITY    25       /* S0506: u16（质量标志） */
#define MB_AWAE_POINT_CRC16      31       /* u16（单点完整性） */

#endif /* MODBUS_REGISTER_MAP_H */
