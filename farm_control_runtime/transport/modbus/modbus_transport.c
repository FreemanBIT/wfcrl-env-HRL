/*
 * modbus_transport.c — Modbus TCP Transport 实现（Phase 9）
 *
 * 自包含实现（无第三方库）：
 *   - TCP server（单客户端），Modbus TCP ADU（MBAP 7B + PDU）；
 *   - 功能码：0x03（读保持寄存器）、0x06（写单寄存器）、0x10（写多寄存器）；
 *   - 寄存器区布局见 modbus_register_map.h；
 *   - poll_command：把命令区写值与上次提交帧 diff 后输出 FarmCommandFrame；
 *   - publish_state：把 FarmStateFrame 编码进状态/时钟寄存器区。
 *
 * 10 ms ROSCO 路径不触碰本模块（命令行/独立线程处理）。
 */
#include "modbus_transport.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <process.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "fcr_protocol.h"
#include "fcr_state_types.h"
#include "modbus_register_map.h"

#pragma comment(lib, "ws2_32.lib")

/* ---------- 寄存器区 ---------- */
/* 实现结构体见 modbus_transport.h（单一定义） */

/* ---------- 编解码辅助 ---------- */
void mb_put_u16(uint16_t *regs, int idx, uint16_t v) { regs[idx] = v; }
uint16_t mb_get_u16(const uint16_t *regs, int idx) { return regs[idx]; }
void mb_put_u32(uint16_t *regs, int idx, uint32_t v) {
    regs[idx] = (uint16_t)(v >> 16);
    regs[idx + 1] = (uint16_t)(v & 0xFFFF);
}
uint32_t mb_get_u32(const uint16_t *regs, int idx) {
    return ((uint32_t)regs[idx] << 16) | regs[idx + 1];
}
void mb_put_u64(uint16_t *regs, int idx, uint64_t v) {
    mb_put_u32(regs, idx, (uint32_t)(v >> 32));
    mb_put_u32(regs, idx + 2, (uint32_t)(v & 0xFFFFFFFF));
}
uint64_t mb_get_u64(const uint16_t *regs, int idx) {
    return ((uint64_t)mb_get_u32(regs, idx) << 32) | mb_get_u32(regs, idx + 2);
}
static uint16_t f32hi(float f) { uint32_t u; memcpy(&u, &f, 4); return (uint16_t)(u >> 16); }
static uint16_t f32lo(float f) { uint32_t u; memcpy(&u, &f, 4); return (uint16_t)(u & 0xFFFF); }
static float hiLoToF32(uint16_t hi, uint16_t lo) {
    uint32_t u = ((uint32_t)hi << 16) | lo;
    float f; memcpy(&f, &u, 4); return f;
}
void mb_put_double(uint16_t *regs, int idx, double d) {
    uint64_t u; memcpy(&u, &d, 8);
    mb_put_u64(regs, idx, u);
}
double mb_get_double(const uint16_t *regs, int idx) {
    uint64_t u = mb_get_u64(regs, idx);
    double d; memcpy(&d, &u, 8);
    return d;
}
void mb_put_i64(uint16_t *regs, int idx, int64_t v) { mb_put_u64(regs, idx, (uint64_t)v); }
int64_t mb_get_i64(const uint16_t *regs, int idx) { return (int64_t)mb_get_u64(regs, idx); }

/* ---------- 连接处理 ---------- */
static void handle_client(struct FcrModbusTransport *mt)
{
    unsigned char buf[1500];
    SOCKET c = mt->client_sock;
    while (!mt->stop) {
        int n = recv(c, (char *)buf, sizeof(buf), 0);
        if (n <= 0) {
            closesocket(c);
            mt->client_sock = INVALID_SOCKET;
            return;
        }
        /* 按 ADU 边界分割（可能一次含多请求：先处理单帧路径，循环） */
        int off = 0;
        while (off + 7 <= n) {
            /* MBAP: tid(2) pid(2) len(2) uid(1) | PDU */
            unsigned int pdu_len = ((unsigned int)buf[off + 4] << 8) | buf[off + 5];
            unsigned int total = 6 + pdu_len;      /* MBAP header without tid? MBAP=6 bytes + PDU */
            /* Modbus TCP MBAP is 7 bytes: tid2 pid2 len2 uid1; len counts uid+PDU */
            unsigned int adu_len = 7 + pdu_len - 1; /* uid included in len */
            (void)total;
            if (off + (int)adu_len > n) break;
            unsigned char fc = buf[off + 7];
            unsigned char resp[260];
            /* 请求地址与大端寄存器值已在网络字节序？Modbus regs ARE big-endian in PDU */
            unsigned int req_len = 0;
            switch (fc) {
            case 0x03: {
                unsigned int addr = ((unsigned int)buf[off + 8] << 8) | buf[off + 9];
                unsigned int cnt  = ((unsigned int)buf[off + 10] << 8) | buf[off + 11];
                if (cnt == 0 || cnt > 125 || addr + cnt > 0x3000) {
                    resp[0] = 0x83; resp[1] = 0x02; req_len = 2;
                    break;
                }
                resp[0] = 0x03;
                resp[1] = (unsigned char)(cnt * 2);
                for (unsigned int i = 0; i < cnt; i++) {
                    uint16_t v = mt->regs[addr + i];
                    resp[2 + 2 * i] = (unsigned char)(v >> 8);
                    resp[3 + 2 * i] = (unsigned char)(v & 0xFF);
                }
                req_len = 2 + cnt * 2;
                break;
            }
            case 0x06: {
                unsigned int addr = ((unsigned int)buf[off + 8] << 8) | buf[off + 9];
                uint16_t v = ((uint16_t)buf[off + 10] << 8) | buf[off + 11];
                if (addr + 1 > 0x3000) { resp[0] = 0x86; resp[1] = 0x02; req_len = 2; break; }
                mt->regs[addr] = v;
                resp[0] = 0x06; resp[1] = (unsigned char)(addr >> 8); resp[2] = (unsigned char)(addr & 0xFF);
                resp[3] = (unsigned char)(v >> 8); resp[4] = (unsigned char)(v & 0xFF);
                req_len = 5;
                break;
            }
            case 0x10: {
                unsigned int addr = ((unsigned int)buf[off + 8] << 8) | buf[off + 9];
                unsigned int cnt  = ((unsigned int)buf[off + 10] << 8) | buf[off + 11];
                unsigned int bc   = buf[off + 12];
                if (addr + cnt > 0x3000 || bc != cnt * 2) { resp[0] = 0x90; resp[1] = 0x02; req_len = 2; break; }
                for (unsigned int i = 0; i < cnt; i++) {
                    mt->regs[addr + i] = ((uint16_t)buf[off + 13 + 2 * i] << 8) | buf[off + 14 + 2 * i];
                }
                resp[0] = 0x10; resp[1] = (unsigned char)(addr >> 8); resp[2] = (unsigned char)(addr & 0xFF);
                resp[3] = (unsigned char)(cnt >> 8); resp[4] = (unsigned char)(cnt & 0xFF);
                req_len = 5;
                break;
            }
            default:
                resp[0] = fc | 0x80; resp[1] = 0x01; req_len = 2;
                break;
            }
            unsigned char pdu[300];
            memcpy(pdu, resp, req_len);
            unsigned char adu[7 + 300];
            adu[0] = buf[off]; adu[1] = buf[off + 1];       /* tid */
            adu[2] = buf[off + 2]; adu[3] = buf[off + 3];   /* pid */
            unsigned int len = 1 + req_len;                 /* uid + PDU */
            adu[4] = (unsigned char)(len >> 8); adu[5] = (unsigned char)(len & 0xFF);
            adu[6] = buf[off + 6];                          /* uid */
            memcpy(adu + 7, pdu, req_len);
            send(c, (const char *)adu, 7 + req_len, 0);
            off += (int)adu_len;
        }
        if (off > 0 && off < n) {
            /* 残余帧（半包）→ 丢弃并重同步（简化） */
        }
    }
}

static unsigned __stdcall server_thread_fn(void *arg)
{
    struct FcrModbusTransport *mt = (struct FcrModbusTransport *)arg;
    u_long mode = 1;
    /* 非阻塞 accept：destroy 时 stop 标记即可退出（阻塞 accept 无法被唤醒） */
    ioctlsocket(mt->listen_sock, FIONBIO, &mode);
    while (!mt->stop) {
        SOCKET a = accept(mt->listen_sock, NULL, NULL);
        if (a == INVALID_SOCKET) {
            if (WSAGetLastError() == WSAEWOULDBLOCK) { Sleep(10); continue; }
            continue;
        }
        if (mt->client_sock != INVALID_SOCKET) { closesocket(a); continue; }
        mt->client_sock = a;
        handle_client(mt);       /* 阻塞处理该客户端直至断开 */
        mt->client_sock = INVALID_SOCKET;
    }
    return 0;
}

/* ---------- Transport ops ---------- */
static int mb_poll_command(FcrTransport *self, FarmCommandFrame *cmd)
{
    struct FcrModbusTransport *mt = (struct FcrModbusTransport *)self->impl;
    uint32_t i;
    memset(cmd, 0, sizeof(*cmd));
    cmd->protocol_version = FCR_PROTOCOL_VERSION;
    cmd->n_turbines = mt->n_turbines;

    /* 从寄存器区读取命令（若 frame_seq 有更新） */
    uint64_t fs = mb_get_u64(mt->regs, MB_CLK_FRAME_SEQ_SLOT);
    if (fs == 0) return 0;   /* 无新命令（RCP 尚未写 frame_seq） */
    if (fs == mt->last_frame_seq) return 0;   /* 幂等 */

    for (i = 0; i < mt->n_turbines && i < FCR_MAX_TURBINES; i++) {
        int b = MB_CMD_BASE + (int)i * MB_CMD_PER_TURBINE;
        FcrTurbineCommand *tc = &cmd->turbines[i];
        tc->yaw_delta_rad = mb_get_double(mt->regs, b + MB_CMD_YAW_DELTA_RAD);
        tc->yaw_seq = mb_get_u64(mt->regs, b + MB_CMD_YAW_SEQ);
        tc->yaw_valid = mb_get_u16(mt->regs, b + MB_CMD_YAW_VALID);
        tc->yaw_effective_low_step = mb_get_i64(mt->regs, b + MB_CMD_YAW_EFFECTIVE_LOW);
        tc->yaw_ttl_s = mb_get_double(mt->regs, b + MB_CMD_YAW_TTL_S);
        tc->induction_ref = mb_get_double(mt->regs, b + MB_CMD_IND_REF);
        tc->induction_seq = mb_get_u64(mt->regs, b + MB_CMD_IND_SEQ);
        tc->induction_valid = mb_get_u16(mt->regs, b + MB_CMD_IND_VALID);
        tc->induction_effective_low_step = mb_get_i64(mt->regs, b + MB_CMD_IND_EFFECTIVE_LOW);
        tc->induction_ttl_s = mb_get_double(mt->regs, b + MB_CMD_IND_TTL_S);
        tc->source_time_s = mb_get_double(mt->regs, b + MB_CMD_SOURCE_TIME_S);
    }
    cmd->frame_seq = fs;
    cmd->commit_seq = mb_get_u64(mt->regs, MB_CLK_COMMIT_SEQ_SLOT);
    cmd->receive_time_s = 0.0;
    mt->last_frame_seq = fs;
    return 1;
}

static int mb_publish_state(FcrTransport *self, const FarmStateFrame *state)
{
    struct FcrModbusTransport *mt = (struct FcrModbusTransport *)self->impl;
    uint32_t i;
    mb_put_double(mt->regs, MB_CLK_SIM_TIME_S, state->sim_time_s);
    mb_put_u64(mt->regs, MB_CLK_FAST_STEP, state->fast_step);
    mb_put_u64(mt->regs, MB_CLK_LOW_STEP, state->low_step);
    mb_put_u16(mt->regs, MB_CLK_RT_HEALTH, (uint16_t)state->rt_health_flags);
    for (i = 0; i < mt->n_turbines && i < FCR_MAX_TURBINES; i++) {
        int b = MB_STATE_BASE + (int)i * MB_STATE_PER_TURBINE;
        const FcrFarmTurbineState *ts = &state->turbines[i];
        mb_put_u16(mt->regs, b + MB_ST_TURBINE_ID, (uint16_t)ts->turbine_id);
        mb_put_u16(mt->regs, b + MB_ST_VALID, (uint16_t)ts->valid);
        mb_put_double(mt->regs, b + MB_ST_SOURCE_TIME, ts->source_time_s);
        mb_put_double(mt->regs, b + MB_ST_GEN_POWER_W, ts->gen_power_w.instant);
        mb_put_double(mt->regs, b + MB_ST_GEN_POWER_MEAN, ts->gen_power_w.mean);
        mb_put_double(mt->regs, b + MB_ST_GEN_POWER_RMS, ts->gen_power_w.rms);
        mb_put_double(mt->regs, b + MB_ST_ROTOR_THRUST_N, ts->rotor_thrust_n.instant);
        mb_put_double(mt->regs, b + MB_ST_SHAFT_TORQUE_NM, ts->tower_base_fa_moment_nm.instant);
        mb_put_double(mt->regs, b + MB_ST_NACELLE_HEADING, ts->nacelle_heading_rad);
        mb_put_double(mt->regs, b + MB_ST_NACELLE_VANE, ts->nacelle_vane_rad);
        mb_put_double(mt->regs, b + MB_ST_HUB_WIND_MPS, ts->hub_wind_speed_mps);
        mb_put_double(mt->regs, b + MB_ST_ROTOR_SPEED, ts->rotor_speed_rad_s);
        mb_put_double(mt->regs, b + MB_ST_GEN_SPEED, ts->gen_speed_rad_s);
        mb_put_double(mt->regs, b + MB_ST_CT, ts->rotor_ct);
        mb_put_double(mt->regs, b + MB_ST_CP, ts->rotor_cp);
        mb_put_double(mt->regs, b + MB_ST_YAW_TARGET_HEADING, ts->yaw_target_heading_rad);
        mb_put_u64(mt->regs, b + MB_ST_YAW_SEQ_APPLIED, ts->yaw_seq_applied);
        mb_put_u64(mt->regs, b + MB_ST_IND_SEQ_APPLIED, ts->induction_seq_applied);
        mb_put_u16(mt->regs, b + MB_ST_CTRL_STATUS_FLAGS, (uint16_t)ts->controller_status_flags);
        mb_put_double(mt->regs, b + MB_ST_DISK_AMB_WIND, ts->disk_ambient_wind_mps);
        mb_put_double(mt->regs, b + MB_ST_DISK_DIST_WIND, ts->disk_disturbed_wind_mps);
        mb_put_double(mt->regs, b + MB_ST_DISK_AMB_TI, ts->disk_ambient_ti);
        mb_put_double(mt->regs, b + MB_ST_FLOW_SOURCE_TIME, ts->flow_source_time_s);
        mb_put_u64(mt->regs, b + MB_ST_FLOW_SEQ, ts->flow_seq);
    }
    return 1;
}

static const FcrTransportOps mb_ops = { mb_poll_command, mb_publish_state };

/* WSA 生命周期引用计数（多实例安全） */
static int g_wsa_refcount = 0;
static int mb_wsa_start(void)
{
    WSADATA wsa;
    if (g_wsa_refcount == 0) {
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return -1;
    }
    g_wsa_refcount++;
    return 0;
}
static void mb_wsa_stop(void)
{
    if (g_wsa_refcount > 0) {
        g_wsa_refcount--;
        if (g_wsa_refcount == 0) WSACleanup();
    }
}



/* ---------- 生命周期 ---------- */
FcrTransport *modbus_transport_create(int port, uint32_t n_turbines)
{
    struct FcrModbusTransport *mt;
    struct sockaddr_in addr;
    if (mb_wsa_start() != 0) return NULL;
    mt = (struct FcrModbusTransport *)calloc(1, sizeof(*mt));
    if (!mt) return NULL;
    mt->base.ops = &mb_ops;
    mt->base.impl = mt;
    mt->n_turbines = n_turbines > FCR_MAX_TURBINES ? FCR_MAX_TURBINES : n_turbines;
    mt->listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    mt->client_sock = INVALID_SOCKET;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((unsigned short)port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(mt->listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(mt->listen_sock, 1) != 0) {
        closesocket(mt->listen_sock);
        free(mt);
        return NULL;
    }
    /* 寄存器区覆盖命令(0x0000+0x1000)+时钟(0x0800)+状态(0x1000..0x2FFF) */
    mt->regs = (uint16_t *)calloc(0x3000, sizeof(uint16_t));
    if (!mt->regs) { closesocket(mt->listen_sock); free(mt); return NULL; }
    mt->stop = 0;
    mt->last_frame_seq = 0;
#ifdef FCR_MODBUS_NO_THREAD
    mt->thread = NULL;   /* 单元测试模式：不启动监听线程（逻辑验证） */
#else
    mt->thread = (HANDLE)_beginthreadex(NULL, 0, server_thread_fn, mt, 0, NULL);
#endif
    return &mt->base;
}

void modbus_transport_destroy(FcrTransport *self)
{
    struct FcrModbusTransport *mt;
    if (!self) return;
    mt = (struct FcrModbusTransport *)self->impl;
    InterlockedExchange(&mt->stop, 1);
    /* 先关 listen 使 accept 返回，再等线程退出（避免 use-after-free） */
    closesocket(mt->listen_sock);
    if (mt->client_sock != INVALID_SOCKET) {
        shutdown(mt->client_sock, SD_BOTH);
        closesocket(mt->client_sock);
    }
    if (mt->thread) {
        WaitForSingleObject(mt->thread, 2000);
        CloseHandle(mt->thread);
    }
    free(mt->regs);
    free(mt);
    mb_wsa_stop();
}