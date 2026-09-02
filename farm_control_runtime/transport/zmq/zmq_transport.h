/*
 * zmq_transport.h — ZeroMQ Transport（Phase 7）
 *
 * 拓扑（离线主闭环）：
 *   命令：Python Farm Controller (PUB)  -->  DLL SUB (bind tcp://*:CMD_PORT)
 *   状态：DLL PUB (bind tcp://*:STATE_PORT)  -->  Python (SUB)
 *
 * 帧 = FarmCommandFrame / FarmStateFrame 的原始二进制（ZMQ 消息即帧）。
 * 本模块只实现 Transport API（poll_command/publish_state），无控制算法。
 * 运行时动态加载 libzmq.dll（MSVC 预编译），离线 harness 设置
 * FCR_LIBZMQ_PATH 或确保 DLL 在搜索路径中。
 */
#ifndef ZMQ_TRANSPORT_H
#define ZMQ_TRANSPORT_H

#include "fcr_transport.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 创建 ZMQ transport：
 *   cmd_port   命令 SUB 绑定端口（0 = 禁用）
 *   state_port 状态 PUB 绑定端口（0 = 禁用）
 * 返回可挂载到 runtime 的 FcrTransport*（NULL = 失败）           */
FcrTransport *zmq_transport_create(int cmd_port, int state_port);

/* 销毁并停止内部线程 */
void zmq_transport_destroy(FcrTransport *t);

/* 注入回调：收到命令帧时调用（可空，默认注入 runtime） */
typedef void (*zmq_cmd_cb_fn)(const void *frame);
void zmq_transport_set_command_callback(FcrTransport *t, zmq_cmd_cb_fn cb, void *user);

#ifdef __cplusplus
}
#endif

#endif /* ZMQ_TRANSPORT_H */
