/*
 * fcr_transport.h — Transport API（Phase 1 冻结）
 *
 * 统一 Transport 接口：ZeroMQ 与 Modbus 只实现 poll_command/publish_state，
 * 不得实现任何控制算法。Transport 可挂载到 runtime（fcr_runtime_create 的
 * config.transport），也可由宿主直接驱动。
 */
#ifndef FCR_TRANSPORT_H
#define FCR_TRANSPORT_H

#include "fcr_state_types.h"
#include "fcr_awae_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct FcrTransport FcrTransport;

/* poll_command：从 Transport 取一帧命令；无命令返回 0，有命令返回 1     */
typedef int (*fcr_poll_command_fn)(FcrTransport *self, FarmCommandFrame *cmd);
/* publish_state：把聚合状态帧交给 Transport 上送                        */
typedef int (*fcr_publish_state_fn)(FcrTransport *self, const FarmStateFrame *state);
/* publish_awae：把 AWAE 观测/目录帧上送 RCP（ICD v1.5 GG-12/GG-13）。
 * 返回 1 = 已上送；0 = 本 Transport 不承载 AWAE（调用方忽略，非错误）；<0 = 错误。
 * 追加在末尾：既有的位置初始化 { poll, publish } 会把本字段零初始化为 NULL，
 * 因此 fcr_transport_publish_awae() 对其为 NULL 时安全返回 0。 */
typedef int (*fcr_publish_awae_fn)(FcrTransport *self,
                                   const FcrAWAEObservationFrame *obs,
                                   const FcrAWAECatalogFrame *catalog);

typedef struct {
    fcr_poll_command_fn   poll_command;
    fcr_publish_state_fn  publish_state;
    fcr_publish_awae_fn   publish_awae;
} FcrTransportOps;

struct FcrTransport {
    const FcrTransportOps *ops;
    void *impl;                        /* 具体 Transport 实现数据          */
};

/* 便捷调用（空指针安全）                                                */
static inline int fcr_transport_poll(FcrTransport *t, FarmCommandFrame *cmd) {
    if (!t || !t->ops || !t->ops->poll_command) return 0;
    return t->ops->poll_command(t, cmd);
}
static inline int fcr_transport_publish(FcrTransport *t, const FarmStateFrame *s) {
    if (!t || !t->ops || !t->ops->publish_state) return 0;
    return t->ops->publish_state(t, s);
}
static inline int fcr_transport_publish_awae(FcrTransport *t,
                                             const FcrAWAEObservationFrame *obs,
                                             const FcrAWAECatalogFrame *cat) {
    if (!t || !t->ops || !t->ops->publish_awae) return 0;
    return t->ops->publish_awae(t, obs, cat);
}

#ifdef __cplusplus
}
#endif

#endif /* FCR_TRANSPORT_H */
