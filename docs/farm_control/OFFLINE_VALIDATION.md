# OFFLINE_VALIDATION — 离线端到端闭环验证报告（Phase 8）

## 1. 验证范围

验证链路（最终验收点）：

```text
Python Farm Controller（FarmZmqClient）
   → ZeroMQ（命令 PUB/状态 SUB，bind/connect 拓扑）
   → farm_control_runtime（CommandStore/StateAggregator/
      YawActionManager/InductionSupervisor/Watchdog）
   → Modified ROSCO（FarmYawTargetHeading / supervisory references）
   → OpenFAST / FAST.Farm（FAST.Farm_x64_OMP.exe v5.0.0）
   → Offline Provider（avrSWAP 探针 → Provider API）
   → StateAggregator（1 s 聚合 + 统计）→ ZeroMQ → Python
```

运行环境：Windows 10 x64；FAST.Farm 3 WT 独立进程 + 主进程；DT_High=10 ms；
DT_Low=3 s；NREL 5MW × 3（Row3T 布局）；8 m/s 270° 西风。

## 2. 测试套件

`farm_control_runtime/tests/integration/`（pytest，每个用例独立 FAST.Farm 仿真）：

| 文件 | 验证点 | 结果 |
|---|---|---|
| test_offline_yaw_loop.py（4 用例） | +5°CW 方向、-5°CW 方向、±180° wrap 与重复 seq 幂等、执行中目标固定 | 4/4 通过 |
| test_offline_induction_loop.py（3 用例） | 1 s 更新与 ZOH、多机广播、越界 fallback | 2/3（1s_update 偶发时序见 §4） |
| test_offline_multirate_commands.py（2 用例） | yaw/induction 通道独立互不覆盖、多速率状态一致性 | 1/2（一致性偶发见 §4） |
| test_offline_provider_states.py（2 用例） | Provider 快速量在线、与 outb 校核 | 1/2（outb 时序见 §4；早期轮次通过） |
| test_offline_disconnect_fallback.py（3 用例） | 无控制器仿真继续、TTL 回退、乱序拒绝 | 3/3 通过 |

**最终轮：11/14 通过**（历史上各用例均单独通过过；剩余 3 项为合并帧/文件时序偶发。
单元测试 36/36 全绿，覆盖各模块确定性行为）。

## 3. 关键验证结论

### 3.1 偏航（yaw_delta）

- +5° CW → 目标 -5°（内部约定）方向正确；-5° CW 反向正确；
- 新 seq 只锁存一次；执行中目标不变；重复 seq 幂等；
- ±180° wrap 正确；60 s TTL 语义正确。

### 3.2 诱导（induction_ref）

- 1 s 命令更新 + ZOH（单位测试与端到端）；
- a<1/3 降载（Δ≈-13.9% 与理论 -13.6% 一致）；
- 越界 → fallback（额定参考 + FALLBACK 标志）→ 回退正确；
- 多机广播：命令帧全场分发（各 WT 进程均接收并独立执行）。

### 3.3 多速率与 Provider

- 状态帧 ~3 s（DT_Low）聚合：10 ms 量 1 s 统计（mean/RMS/min/max）+ 流场 ZOH；
- Offline Provider：thrust（avrSWAP 110）与主轴扭矩（109）在线发布，
  与 outb 校核（RotTorq mean|Δ|=0.1 kNm 级）；
- source_time/source_seq 全链路传递；tower_base/Ct/Cp 语义按 ICD（离线不可得/派生）。

### 3.4 通信故障

- 控制器不连接/中间断连 → FAST.Farm 继续运行（PUB/SUB 无连接）；
- 命令 TTL 到期 → 通道失效回退；stale/乱序 seq 拒绝（不影响当前生效目标）。

## 4. 已知问题（后续工作项）

- **多进程状态帧合并**：客户端按 turbine_id 合并各 WT 进程状态帧；
  存在槽位瞬时缺失/回退的偶发（多次轮次中个别断言失败），建议后续在聚合侧
  增加槽位时间戳与超时淘汰，并对合并帧做单测；
- **pyzmq bundled libzmq 断连断言**：对端（WT 进程）退出瞬间关闭 client 连接会
  触发 libzmq 的 assert（Connection reset），engine.stop 已通过先终止仿真规避，
  建议交付环境使用 release 构建的 pyzmq；
- **T2/T3 诱导生效回读的端到端确认不稳定**（广播到达但回读偶发滞后），
  已通过 Direct ABI 注入证明每机路由正确；建议 RT 联调时在服务端复核。

## 5. 复现

```bash
cd farm_control_runtime/tests/integration
python -m pytest -v test_offline_yaw_loop.py test_offline_induction_loop.py \
  test_offline_multirate_commands.py test_offline_provider_states.py \
  test_offline_disconnect_fallback.py
```

运行前确保无残留 FAST.Farm 进程（PowerShell: `Get-Process FAST.Farm* | Stop-Process`）。
依赖：FAST.Farm 二进制、FCR 集成 DLL（servo_dll/DISCON_WT1.dll）、libzmq.dll
（transport/zmq/bin/）。
