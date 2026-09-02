# ZMQ_PROTOCOL — 离线主闭环 ZeroMQ 协议（Phase 7 冻结）

## 1. 拓扑

```text
Python Farm Controller              FAST.Farm 进程（FCR 集成 DLL）
  FarmZmqClient                          farm_control_runtime
  ├─ PUB ── tcp://127.0.0.1:CMD_PORT --> SUB（命令线程）-> CommandStore
  └─ SUB <-- tcp://127.0.0.1:STATE_PORT -- PUB（step_low ~1 Hz 状态帧）
```

## 2. 帧格式

- 命令帧：`FarmCommandFrame`（fcr_state_types.h）原始二进制 5672 B；
- 状态帧：`FarmStateFrame` 原始二进制 23096 B；
- ZMQ 消息 = 一整帧（无拆分/无附加头）。

## 3. 端口与启动

- DLL 内 ZMQ transport 由环境变量启用（DISCON 初始化时读）：
  `FCR_ZMQ_CMD_PORT`（默认缺省禁用）、`FCR_ZMQ_STATE_PORT`；
- `libzmq.dll`（MSVC 预编译）随 harness 提供：见 `transport/zmq/bin/`；
  运行时通过 `FCR_LIBZMQ_PATH` 指定或放在 FAST.Farm 工作目录；
- Python 侧使用 pyzmq（无需单独安装 libzmq）。

## 4. 语义（对应 PROTOCOL_DECISIONS）

- 状态 ~1 Hz 聚合帧：10 ms 量的 1 s 统计 + 1 s 流场 ZOH + source time/seq；
- 命令帧异步到达；`yaw`/`induction` 通道独立 seq，重复 seq 幂等，乱序拒绝；
- 命令帧含 `frame_seq`（transport 级单调）；`commit_seq` 原子提交（Modbus 语义预留）；
- ROSCO 10 ms 控制路径不触碰 ZMQ（只读共享 setpoint）。

## 5. Python API（wfcrl/transport/farm_zmq_client.py）

```python
from wfcrl.transport import FarmZmqClient
client = FarmZmqClient(cmd_port=5556, state_port=5557)
client.start()
state_bytes = client.get_state()          # isinstance bytes / None
client.send_yaw_delta(1, 0.0873, seq=1)   # T1 +5° CW
client.send_induction(1, 0.3, seq=1)      # T1 a=0.3
client.stop()
```

## 6. 验收（Phase 7）

- 无 `controls.txt` 依赖（FCR 模式 DLL 不再读写）；
- 多机组命令正确（按 turbine_id 路由）；
- controller 断连不阻塞 FAST.Farm（PUB/SUB 无连接状态）；
- reconnect 后 seq 规则不变（独立通道单调）；
- yaw/induction 异步互不覆盖（通道独立）。
