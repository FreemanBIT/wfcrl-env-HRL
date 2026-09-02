/*
 * modbus_register_map.h — Modbus TCP 寄存器映射（Phase 9 冻结）
 *
 * 寄存器均为 16 bit 保持寄存器（Holding Registers），字节序 big-endian
 * （Modbus 标准）。64 位量 = 4 寄存器；32 位 = 2；16 位 = 1。
 * 单精度浮点（IEEE754）双寄存器；双精度浮点四寄存器。
 *
 * 地址段：
 *   0x0000..0x07FF  命令区（RCP → runtime）
 *   0x0800..0x0FFF  时钟/健康区（runtime → RCP）
 *   0x1000..0x17FF  状态区·机组 1..N（runtime → RCP）
 */
#ifndef MODBUS_REGISTER_MAP_H
#define MODBUS_REGISTER_MAP_H

/* ---- 命令区：每机 64 寄存器，从 0x0000 + (turbine_id-1)*64 ---- */
#define MB_CMD_BASE              0x0000
#define MB_CMD_PER_TURBINE       64

#define MB_CMD_YAW_DELTA_RAD     0   /* f64 (rad, CW+)            */
#define MB_CMD_YAW_SEQ           4   /* u64                       */
#define MB_CMD_YAW_VALID         8   /* u16 (1=valid)             */
#define MB_CMD_YAW_EFFECTIVE_LOW 9   /* i64                       */
#define MB_CMD_YAW_TTL_S         13  /* f64 (s)                   */

#define MB_CMD_IND_REF           17  /* f64 (无量纲)              */
#define MB_CMD_IND_SEQ           21  /* u64                       */
#define MB_CMD_IND_VALID         25  /* u16                       */
#define MB_CMD_IND_EFFECTIVE_LOW 26  /* i64                       */
#define MB_CMD_IND_TTL_S         30  /* f64 (s)                   */

#define MB_CMD_SOURCE_TIME_S     34  /* f64 (s)                   */
#define MB_CMD_FRAME_SEQ         38  /* u64                       */
#define MB_CMD_COMMIT_SEQ        42  /* u64                       */

/* ---- 时钟/健康区 ---- */
#define MB_CLK_SIM_TIME_S        0x0800  /* f64 */
#define MB_CLK_FAST_STEP         0x0804  /* u64 */
#define MB_CLK_LOW_STEP          0x0808  /* u64 */
#define MB_CLK_RT_HEALTH         0x080C  /* u16 */
#define MB_CLK_FRAME_SEQ_SLOT    0x080D  /* frame_seq u64（时钟扩展） */
#define MB_CLK_COMMIT_SEQ_SLOT   0x0811  /* commit_seq u64 */

/* ---- 状态区（只读保持寄存器）：每机 128 寄存器，0x1000 + (tid-1)*128 ---- */
#define MB_STATE_BASE            0x1000
#define MB_STATE_PER_TURBINE     128

#define MB_ST_TURBINE_ID         0   /* u16            */
#define MB_ST_VALID              1   /* u16            */
#define MB_ST_SOURCE_TIME        2   /* f64            */
#define MB_ST_FAST_STEP          6   /* u64            */
#define MB_ST_GEN_POWER_W        10  /* f64            */
#define MB_ST_GEN_POWER_MEAN     14  /* f64            */
#define MB_ST_GEN_POWER_RMS      18  /* f64            */
#define MB_ST_ROTOR_THRUST_N     22  /* f64            */
#define MB_ST_SHAFT_TORQUE_NM    26  /* f64            */
#define MB_ST_NACELLE_HEADING    30  /* f64 (rad)      */
#define MB_ST_NACELLE_VANE       34  /* f64 (rad)      */
#define MB_ST_HUB_WIND_MPS       38  /* f64            */
#define MB_ST_ROTOR_SPEED        42  /* f64 (rad/s)    */
#define MB_ST_GEN_SPEED          46  /* f64 (rad/s)    */
#define MB_ST_CT                 50  /* f64            */
#define MB_ST_CP                 54  /* f64            */
#define MB_ST_YAW_TARGET_HEADING 58  /* f64 (rad)      */
#define MB_ST_YAW_SEQ_APPLIED    62  /* u64            */
#define MB_ST_IND_SEQ_APPLIED    66  /* u64            */
#define MB_ST_CTRL_STATUS_FLAGS  70  /* u16            */
#define MB_ST_DISK_AMB_WIND      71  /* f64 (m/s)      */
#define MB_ST_DISK_DIST_WIND     75  /* f64 (m/s)      */
#define MB_ST_DISK_AMB_TI        79  /* f64            */
#define MB_ST_FLOW_SOURCE_TIME   83  /* f64 (s)        */
#define MB_ST_FLOW_SEQ           87  /* u64            */

#endif /* MODBUS_REGISTER_MAP_H */