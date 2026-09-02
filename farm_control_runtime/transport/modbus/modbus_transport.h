/*
 * modbus_transport.h — Modbus TCP Transport（Phase 9）
 *
 * 实时部署：FAST.Farm RT 仿真机侧运行 runtime；RCP（Farm Controller +
 * Modbus Client）经以太网连接。本 Transport 实现 Modbus TCP Server：
 *   - 命令区（0x0000+）：RCP 写 → poll_command 读取并注入 CommandStore；
 *   - 状态区（0x1000+）/时钟区（0x0800）：publish_state 写入 → RCP 读。
 *
 * 实现自包含（无第三方依赖）：Modbus TCP ADU（MBAP + PDU）由本模块编解码，
 * 支持功能码：0x03 读保持寄存器、0x06 写单寄存器、0x10 写多寄存器。
 * 同一时刻仅接受一个客户端连接（RT 场景）。
 */
#ifndef MODBUS_TRANSPORT_H
#define MODBUS_TRANSPORT_H

#include "fcr_transport.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <windows.h>
#include <stdint.h>

/* 实现细节暴露（供单元测试访问寄存器区）；交付使用仅需 FcrTransport* */
struct FcrModbusTransport {
    FcrTransport base;
    SOCKET listen_sock;
    SOCKET client_sock;
    HANDLE thread;
    volatile LONG stop;
    uint32_t n_turbines;
    uint16_t *regs;   /* 4096 x 16bit 保持寄存器 */
    uint64_t frame_seq;
    uint64_t last_frame_seq;
    /* 命令缓存槽（与寄存器区对齐读取，见 modbus_register_map.h） */
    double yaw_delta[64]; uint64_t yaw_seq[64]; uint32_t yaw_valid[64];
    double ind_ref[64]; uint64_t ind_seq[64]; uint32_t ind_valid[64];
};

typedef struct FcrModbusTransport FcrModbusTransport;

/* 创建 Modbus TCP server：监听端口 port（默认 502），
 * n_turbines 决定命令/状态区有效范围。 */
FcrTransport *modbus_transport_create(int port, uint32_t n_turbines);
void modbus_transport_destroy(FcrTransport *t);

/* 寄存器编解码辅助（单元测试直接使用） */
void      mb_put_u16(uint16_t *regs, int idx, uint16_t v);
uint16_t  mb_get_u16(const uint16_t *regs, int idx);
void      mb_put_u32(uint16_t *regs, int idx, uint32_t v);
uint32_t  mb_get_u32(const uint16_t *regs, int idx);
void      mb_put_u64(uint16_t *regs, int idx, uint64_t v);
uint64_t  mb_get_u64(const uint16_t *regs, int idx);
void      mb_put_double(uint16_t *regs, int idx, double d);
double    mb_get_double(const uint16_t *regs, int idx);
void      mb_put_i64(uint16_t *regs, int idx, int64_t v);
int64_t   mb_get_i64(const uint16_t *regs, int idx);

/* 状态发布目标（runtime step_low 调 publish_state 时写入寄存器区） */
int modbus_transport_map_farm_state(FcrModbusTransport *mt, const FarmStateFrame *state);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_TRANSPORT_H */