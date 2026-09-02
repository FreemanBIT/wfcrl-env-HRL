/*
 * zmq_transport.cpp — ZeroMQ Transport 实现（Phase 7）
 *
 * 动态加载 libzmq（LoadLibrary + GetProcAddress），避免与 MSVC 静态库的
 * 链接差异；内部线程：
 *   - cmd_thread : SUB 收命令帧 -> 队列 -> poll_command
 *   - publish    : PUB 发送状态帧（runtime step_low 调用，非 10 ms 路径）
 *
 * 10 ms ROSCO 控制路径不触碰本模块的任何函数（红线）。
 */
#include "zmq_transport.h"

#include <windows.h>
#include <process.h>
#include <queue>
#include <mutex>
#include <cstring>
#include <cstdio>
#include <cstdarg>

#include "fcr_state_types.h"
#include "fcr_protocol.h"
#include "fcr_runtime.h"

/* ---------- libzmq 动态绑定 ---------- */
typedef void *(*zmq_ctx_new_fn)(void);
typedef int (*zmq_ctx_destroy_fn)(void *);
typedef void *(*zmq_socket_fn)(void *, int);
typedef int (*zmq_close_fn)(void *);
typedef int (*zmq_bind_fn)(void *, const char *);
typedef int (*zmq_connect_fn)(void *, const char *);
typedef int (*zmq_setsockopt_fn)(void *, int, const void *, size_t);
typedef int (*zmq_recv_fn)(void *, void *, int, int);
typedef int (*zmq_send_fn)(void *, const void *, size_t, int);
typedef int (*zmq_poll_fn)(void *, int, long);
typedef int (*zmq_errno_fn)(void);

struct ZmqApi {
    HMODULE mod = nullptr;
    zmq_ctx_new_fn ctx_new = nullptr;
    zmq_ctx_destroy_fn ctx_destroy = nullptr;
    zmq_socket_fn socket = nullptr;
    zmq_close_fn close = nullptr;
    zmq_bind_fn bind = nullptr;
    zmq_connect_fn connect = nullptr;
    zmq_setsockopt_fn setsockopt = nullptr;
    zmq_recv_fn recv = nullptr;
    zmq_send_fn send = nullptr;
    zmq_poll_fn poll = nullptr;
    zmq_errno_fn errno_fn = nullptr;
};

static FILE *g_diag = NULL;
static void diag(const char *fmt, ...) {
    if (!g_diag) {
        const char *f = getenv("FCR_DIAG_FILE");
        if (f && f[0]) g_diag = fopen(f, "a");
    }
    if (!g_diag) return;
    va_list ap; va_start(ap, fmt);
    vfprintf(g_diag, fmt, ap);
    va_end(ap);
    fflush(g_diag);
}

static ZmqApi g_zmq;
static bool g_zmq_loaded = false;
static bool g_zmq_failed = false;

static bool ensure_zmq()
{
    if (g_zmq_loaded) return true;
    if (g_zmq_failed) return false;
    const char *custom = getenv("FCR_LIBZMQ_PATH");
    const char *names[] = { custom && custom[0] ? custom : nullptr,
                            "libzmq.dll", "libzmq-v143-mt-4_3_5.dll" };
    for (const char *n : names) {
        if (!n) continue;
        HMODULE m = LoadLibraryA(n);
        if (!m) continue;
        g_zmq.mod = m;
        g_zmq.ctx_new = (zmq_ctx_new_fn)GetProcAddress(m, "zmq_ctx_new");
        g_zmq.ctx_destroy = (zmq_ctx_destroy_fn)GetProcAddress(m, "zmq_ctx_destroy");
        g_zmq.socket = (zmq_socket_fn)GetProcAddress(m, "zmq_socket");
        g_zmq.close = (zmq_close_fn)GetProcAddress(m, "zmq_close");
        g_zmq.bind = (zmq_bind_fn)GetProcAddress(m, "zmq_bind");
        g_zmq.connect = (zmq_connect_fn)GetProcAddress(m, "zmq_connect");
        g_zmq.setsockopt = (zmq_setsockopt_fn)GetProcAddress(m, "zmq_setsockopt");
        g_zmq.recv = (zmq_recv_fn)GetProcAddress(m, "zmq_recv");
        g_zmq.send = (zmq_send_fn)GetProcAddress(m, "zmq_send");
        g_zmq.poll = (zmq_poll_fn)GetProcAddress(m, "zmq_poll");
        g_zmq.errno_fn = (zmq_errno_fn)GetProcAddress(m, "zmq_errno");
        if (g_zmq.ctx_new && g_zmq.socket && g_zmq.send && g_zmq.recv) {
            g_zmq_loaded = true;
            return true;
        }
        FreeLibrary(m);
        g_zmq.mod = nullptr;
    }
    g_zmq_failed = true;
    fprintf(stderr, "[zmq] libzmq.dll load failed; set FCR_LIBZMQ_PATH\n");
    return false;
}

/* ---------- Transport 内部 ---------- */
enum { ZMQ_PUB = 1, ZMQ_SUB = 2, ZMQ_DONTWAIT = 1, ZMQ_SUBSCRIBE = 6, ZMQ_POLLIN = 1 };

struct ZmqTransportImpl {
    FcrTransport base;
    void *ctx = nullptr;
    void *cmd_sub = nullptr;   /* 命令 SUB（bind） */
    void *state_pub = nullptr; /* 状态 PUB（bind） */
    HANDLE cmd_thread = nullptr;
    volatile LONG stop = 0;
    std::mutex qlock;
    std::queue<FarmCommandFrame> cmdq;
    zmq_cmd_cb_fn cmd_cb = nullptr;
    void *cmd_user = nullptr;
    int state_port = 0;
    std::mutex pub_lock;
};

static unsigned __stdcall cmd_thread_fn(void *arg)
{
    ZmqTransportImpl *t = (ZmqTransportImpl *)arg;
    char buf[sizeof(FarmCommandFrame) + 16];
    while (!t->stop) {
        int rc = g_zmq.recv(t->cmd_sub, buf, (int)sizeof(buf), ZMQ_DONTWAIT);
        if (rc > 0) {
            FarmCommandFrame frame;
            std::memcpy(&frame, buf, sizeof(FarmCommandFrame));
            if (frame.protocol_version == FCR_PROTOCOL_VERSION) {
                if (t->cmd_cb) {
                    t->cmd_cb(&frame);
                } else {
                    std::lock_guard<std::mutex> lk(t->qlock);
                    t->cmdq.push(frame);
                    if (t->cmdq.size() > 256) t->cmdq.pop();
                }
            }
            continue;
        }
        Sleep(5);
    }
    return 0;
}

extern "C" {

static int zmq_poll_command(FcrTransport *self, FarmCommandFrame *cmd)
{
    ZmqTransportImpl *t = (ZmqTransportImpl *)self->impl;
    std::lock_guard<std::mutex> lk(t->qlock);
    if (t->cmdq.empty()) return 0;
    *cmd = t->cmdq.front();
    t->cmdq.pop();
    return 1;
}

static int zmq_publish_state(FcrTransport *self, const FarmStateFrame *state)
{
    ZmqTransportImpl *t = (ZmqTransportImpl *)self->impl;
    if (!t->state_pub) return 0;
    std::lock_guard<std::mutex> lk(t->pub_lock);
    g_zmq.send(t->state_pub, state, sizeof(FarmStateFrame), ZMQ_DONTWAIT);
    return 1;
}

static const FcrTransportOps g_ops = { zmq_poll_command, zmq_publish_state };

FcrTransport *zmq_transport_create(int cmd_port, int state_port)
{
    diag("[zmq] create entry cmd=%d state=%d\n", cmd_port, state_port);
    if (!ensure_zmq()) { diag("[zmq] ensure failed\n"); return nullptr; }
    diag("[zmq] ensure ok\n");
    ZmqTransportImpl *t = new ZmqTransportImpl();
    t->base.ops = &g_ops;
    t->base.impl = t;
    t->state_port = state_port;
    t->ctx = g_zmq.ctx_new();
    diag("[zmq] ctx_new %p\n", t->ctx);
    if (!t->ctx) { delete t; return nullptr; }
    if (cmd_port > 0) {
        t->cmd_sub = g_zmq.socket(t->ctx, ZMQ_SUB);
        diag("[zmq] cmd_sub %p\n", t->cmd_sub);
        if (!t->cmd_sub) { delete t; return nullptr; }
        g_zmq.setsockopt(t->cmd_sub, ZMQ_SUBSCRIBE, "", 0);
        char addr[64];
        snprintf(addr, sizeof(addr), "tcp://*:%d", cmd_port);
        int brc = g_zmq.bind(t->cmd_sub, addr);
        diag("[zmq] bind cmd %s -> %d (errno=%d)\n", addr, brc, g_zmq.errno_fn ? g_zmq.errno_fn() : -1);
        if (brc != 0) {
            delete t;
            return nullptr;
        }
    }
    if (state_port > 0) {
        t->state_pub = g_zmq.socket(t->ctx, ZMQ_PUB);
        if (!t->state_pub) { delete t; return nullptr; }
        char addr[64];
        snprintf(addr, sizeof(addr), "tcp://*:%d", state_port);
        if (g_zmq.bind(t->state_pub, addr) != 0) {
            fprintf(stderr, "[zmq] bind state %s failed\n", addr);
            delete t;
            return nullptr;
        }
    }
    if (t->cmd_sub) {
        t->cmd_thread = (HANDLE)_beginthreadex(nullptr, 0, cmd_thread_fn, t, 0, nullptr);
    }
    diag("[zmq] transport ready (cmd=%d state=%d)\n", cmd_port, state_port);
    return &t->base;
}

void zmq_transport_destroy(FcrTransport *self)
{
    if (!self) return;
    ZmqTransportImpl *t = (ZmqTransportImpl *)self->impl;
    InterlockedExchange(&t->stop, 1);
    if (t->cmd_thread) {
        WaitForSingleObject(t->cmd_thread, 1000);
        CloseHandle(t->cmd_thread);
    }
    if (t->cmd_sub) g_zmq.close(t->cmd_sub);
    if (t->state_pub) g_zmq.close(t->state_pub);
    if (t->ctx) g_zmq.ctx_destroy(t->ctx);
    delete t;
}

void zmq_transport_set_command_callback(FcrTransport *self, zmq_cmd_cb_fn cb, void *user)
{
    ZmqTransportImpl *t = (ZmqTransportImpl *)self->impl;
    t->cmd_cb = cb;
    t->cmd_user = user;
}

/* 状态查询（Fortran 侧诊断用）：0=未尝试 1=成功 -1=失败 */
int fcr_zmq_transport_status(void)
{
    if (!g_zmq_loaded) return (g_zmq_failed ? -1 : 0);
    return 1;
}

int fcr_zmq_transport_hook(FcrRuntime *rt, int cmd_port, int state_port)
{
    /* 挂载到 runtime（Phase 7 离线主闭环） */
    if (!rt) return -1;
    FcrTransport *t = zmq_transport_create(cmd_port, state_port);
    if (!t) return -2;
    /* runtime 内部在 step_high 通过 poll_command 拉取命令帧（队列模式），
     * step_low 通过 publish_state 发布状态帧。无需注入回调。 */
    fcr_runtime_set_transport(rt, t);
    return 0;
}

} /* extern "C" */