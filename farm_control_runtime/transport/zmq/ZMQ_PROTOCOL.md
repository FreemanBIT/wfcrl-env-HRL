# ZMQ_PROTOCOL — 离线主闭环 ZeroMQ 协议（Phase 7 冻结）

## 1. 拓扑

```text
Python Farm Controller              FAST.Farm 进程（FCR 集成 DLL）
  FarmZmqClient                          farm_control_runtime
  ├─ PUB ── tcp://127.0.0.1:CMD_PORT --> SUB（命令线程）-> CommandStore
  └─ SUB <-- tcp://127.0.0.1:STATE_PORT -- PUB（step_low ~1 Hz 状态帧）
```

## 2. 帧格式

- 命令帧：`FarmCommandFrame`（fcr_state_types.h）原始二进制 5664 B；
- 状态帧：`FarmStateFrame` 原始二进制 25664 B；
- **AWAE 观测帧（W-08，GG-12）**：`FcrAWAEObservationFrame`（fcr_awae_types.h）3096 B；
- **AWAE 目录帧（W-08，GG-13）**：`FcrAWAECatalogFrame` 2584 B；
- ZMQ 消息 = 一整帧（无拆分/无附加头）；**帧类型由冻结尺寸区分**：
  `25664`=状态帧、`3096`=AWAE 观测、`2584`=AWAE 目录（尺寸在
  test_abi.c / check_abi.py / farm_protocol.py 冻结，任何漂移即 ABI 破坏）。

## 3. 端口与启动

- DLL 内 ZMQ transport 由环境变量启用（DISCON 初始化时读）：
  `FCR_ZMQ_CMD_PORT`（默认缺省禁用）、`FCR_ZMQ_STATE_PORT`；
- `libzmq.dll`（MSVC 预编译）随 harness 提供：见 `transport/zmq/bin/`；
  运行时通过 `FCR_LIBZMQ_PATH` 指定或放在 FAST.Farm 工作目录；
- Python 侧使用 pyzmq（无需单独安装 libzmq）。

## 4. 语义（对应 PROTOCOL_DECISIONS）

- 状态 ~1 Hz 聚合帧：10 ms 量的 1 s 统计 + 1 s 流场 ZOH + `state_sample_sim_time_s`(S1002) / `state_frame_seq`(S1001)；
- 命令帧异步到达；`yaw`/`induction` 通道独立 seq，重复 seq 幂等，乱序拒绝；
- 命令帧含 `command_frame_seq`(S0101，transport 级单调)；v1.4 的 `commit_seq` 已按 ICD v1.5 删除；
- 命令帧的 `command_basis_sim_time_s`(S0102) 必须是**仿真时刻**（G-4，禁墙钟），
  且需 > 0 才能通过 WIN-05 帧校验；`FarmZmqClient` 自动用最近状态帧的
  `state_sample_sim_time_s` 补齐；
- ROSCO 10 ms 控制路径不触碰 ZMQ（只读共享 setpoint）。
- **AWAE 二跳（W-08，GG-12/GG-13）**：`fcr_runtime_step_low` 每次低速步在状态帧
  之后发布 AWAE 观测帧 + 目录帧（同一 state PUB socket）。离线环境下 AWAE 数据由
  Provider 注入（`FCR_CFG_AWAE_SYNTH_POINTS>0`，带 `FCR_AWAE_QUALITY_SIMULATED`
  质量位）；目录未注入时仍发布合法空帧（不删字段、不伪造点，用户指示）。

## 5. Python API（wfcrl/transport/farm_zmq_client.py）

```python
from wfcrl.transport import FarmZmqClient
client = FarmZmqClient(cmd_port=5556, state_port=5557)
client.start()
state_bytes = client.get_state()          # isinstance bytes / None
client.send_yaw_delta(1, 0.0873, seq=1)   # T1 +5° CW
client.send_induction(1, 0.3, seq=1)      # T1 a=0.3
# W-08: AWAE 二跳（GG-12/GG-13）——接收端按冻结尺寸分派
obs_raw  = client.get_awae_observation()  # FcrAWAEObservationFrame 原始字节 / None
cat_raw  = client.get_awae_catalog()      # FcrAWAECatalogFrame 原始字节 / None
# 解析：wfcrl.transport.farm_protocol.awae_observation_frame_from_bytes(...)
client.stop()
```

## 6. 验收（Phase 7）

- 无 `controls.txt` 依赖（FCR 模式 DLL 不再读写）；
- 多机组命令正确（按 turbine_id 路由）；
- controller 断连不阻塞 FAST.Farm（PUB/SUB 无连接状态）；
- reconnect 后 seq 规则不变（独立通道单调）；
- yaw/induction 异步互不覆盖（通道独立）。
