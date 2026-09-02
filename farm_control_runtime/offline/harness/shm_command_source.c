/*
 * shm_command_source.c — 进程间命令注入源（离线验证用；Windows）
 *
 * 原理：Python Farm Controller（或测试脚本）把 FarmCommandFrame 写入
 * 命名共享内存（CreateFileMapping），DLL 内一个**独立接收线程**（非 10 ms
 * 控制路径）轮询帧序号变化并注入 runtime（fcr_rosco_inject_command_frame）。
 *
 * 红线合规：10 ms ROSCO 控制路径内无任何 socket / 文件 I/O；
 * 本模块是 Phase 7 ZeroMQ Transport 的临时前置实现，Phase 7 将替换为
 * transport/zmq 的正式实现（同一注入点）。
 *
 * 共享内存名：FCR_CMD_V1；内容：FarmCommandFrame（二进制，见 fcr_state_types.h）。
 */
#include "fcr_internal.h"
#include <string.h>

#ifdef _WIN32
#include <windows.h>

#define FCR_SHM_NAME   L"Local\\FCR_CMD_V1"
#define FCR_SHM_SIZE   (sizeof(FarmCommandFrame))

static HANDLE  g_shm_map     = NULL;
static LPVOID  g_shm_view    = NULL;
static HANDLE  g_shm_thread  = NULL;
static volatile LONG g_shm_stop = 0;
static uint64_t g_last_frame_seq = 0;

static DWORD WINAPI shm_thread_fn(LPVOID param);

int fcr_shm_command_start(void)
{
    if (g_shm_thread) return 0;                 /* 已启动 */
    g_shm_map = CreateFileMappingW(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE,
                                   0, (DWORD)FCR_SHM_SIZE, FCR_SHM_NAME);
    if (!g_shm_map) {
        /* 可能因权限，试试打开已存在的 */
        g_shm_map = OpenFileMappingW(FILE_MAP_ALL_ACCESS, FALSE, FCR_SHM_NAME);
    }
    if (!g_shm_map) return -1;
    g_shm_view = MapViewOfFile(g_shm_map, FILE_MAP_ALL_ACCESS, 0, 0, FCR_SHM_SIZE);
    if (!g_shm_view) { CloseHandle(g_shm_map); g_shm_map = NULL; return -2; }
    g_last_frame_seq = ((volatile FarmCommandFrame *)g_shm_view)->frame_seq;
    g_shm_stop = 0;
    g_shm_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)shm_thread_fn, NULL, 0, NULL);
    if (!g_shm_thread) { UnmapViewOfFile(g_shm_view); CloseHandle(g_shm_map);
                         g_shm_view = NULL; g_shm_map = NULL; return -3; }
    return 0;
}

int fcr_shm_command_stop(void)
{
    if (!g_shm_thread) return 0;
    InterlockedExchange(&g_shm_stop, 1);
    WaitForSingleObject(g_shm_thread, 1000);
    CloseHandle(g_shm_thread); g_shm_thread = NULL;
    if (g_shm_view) { UnmapViewOfFile(g_shm_view); g_shm_view = NULL; }
    if (g_shm_map)  { CloseHandle(g_shm_map); g_shm_map = NULL; }
    return 0;
}

extern FcrRuntime *fcr_rosco_api_runtime(void);
extern int fcr_rosco_inject_command_frame(const FarmCommandFrame *frame);

static DWORD WINAPI shm_thread_fn(LPVOID param);

static FcrRuntime *shm_runtime(void)
{
    return fcr_rosco_api_runtime();
}

static DWORD WINAPI shm_thread_fn(LPVOID param)
{
    (void)param;
    while (!g_shm_stop) {
        volatile FarmCommandFrame *f = (volatile FarmCommandFrame *)g_shm_view;
        if (f && f->protocol_version == FCR_PROTOCOL_VERSION &&
            f->frame_seq != g_last_frame_seq) {
            g_last_frame_seq = f->frame_seq;
            if (shm_runtime()) {
                fcr_rosco_inject_command_frame((const FarmCommandFrame *)f);
            }
        }
        Sleep(10);
    }
    return 0;
}

#else  /* !_WIN32 */
/* Linux 参考实现（Phase 7 由 ZeroMQ 取代；此处仅桩） */
int fcr_shm_command_start(void) { return -1; }
int fcr_shm_command_stop(void)  { return 0; }
#endif

/* 生命周期钩子：由 fcr_rosco_api_init / shutdown 调用 */
int fcr_offline_shm_init(void)
{
#ifdef _WIN32
    return fcr_shm_command_start();
#else
    return -1;
#endif
}

int fcr_offline_shm_fini(void)
{
#ifdef _WIN32
    return fcr_shm_command_stop();
#else
    return 0;
#endif
}