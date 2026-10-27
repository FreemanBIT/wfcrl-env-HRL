/*
 * shm_command_source_stub.c — Linux/WSL 构建用的 no-op 实现
 *
 * 原实现（shm_command_source.c）依赖 Windows 共享内存 + CreateThread，
 * 仅用于离线 Windows 闭环的命令注入。RT/WSL 平台由**调度器负责传输**
 * （见 fcr_rt.h：本模块不持有 socket），不需要离线 SHM 注入源。
 *
 * 提供同名符号，使 rosco_api.c 的 fcr_offline_shm_init/fini 在 Linux 下可链接。
 */
#include <stddef.h>

int fcr_offline_shm_init(void)
{
    return 0;
}

int fcr_offline_shm_fini(void)
{
    return 0;
}
