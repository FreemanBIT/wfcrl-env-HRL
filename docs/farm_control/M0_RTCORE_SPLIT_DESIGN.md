# M0-0.5 — 实时核心拆分设计（RT-core 边界，DR-4）

> 日期：2026-10-13　分支：dev/farm-control-runtime
> 决策：DR-4 —— 复用 Runtime 算法，拆分出不依赖通信的实时核心；
> 10ms 高优先级计算路径禁全场互斥锁 / Socket I/O / 动态内存分配 / 等待 GPU/Modbus。

## 1. 现状 10ms 路径的问题（需拆分）

当前 fcr_rosco_step_high（rosco_api.c:154-176）在 10ms 内做了三件事：

    FCR_LOCK();                                    // 全场互斥锁（多机共享，DR-4 违禁）
      if (g_rt->cfg.transport) {
        while (fcr_transport_poll(...) == 1)       // Socket I/O（ZMQ SUB，DR-4 违禁）
          fcr_command_store_accept_frame(...);
      }
      fcr_command_store_refresh_one(...);           // 纯内存，OK
      fcr_runtime_update_setpoint_one(...);         // 纯内存（含 supervisor），OK
    FCR_UNLOCK();

问题：10ms 路径与通信/锁耦合。离线（单进程 ZMQ 线程）可容忍；RT 实时机
（调度核 Fast Task，DR-6 禁等通信）不可容忍。

## 2. 拆分边界

    RT CORE（10ms，高优先级，纯内存、无锁、无分配、无 I/O）
      CommandStore 快照读取（seq 守卫）
      YawActionManager.update（偏航目标锁存）
      InductionSupervisor.update + apply（诱导映射）
      safety_manager.check（TTL/stale，只读）
      setpoint 槽写入（rt->setpoint[tid]，volatile/对齐）
          |
          | 无锁快照交接（双缓冲 + seq）
          v
    COMM ADAPTER（1s / Slow Task，可阻塞）
      Transport 拉取命令帧（ZMQ/Modbus）→ CommandStore
      AWAE 帧接收 → ObservationManager → WindMapper
      FarmStateFrame 聚合 + Transport publish

## 3. 无锁交接机制

命令下行（Slow → Fast）：
- Slow 把新命令写入 cmd_snapshot[tid]（per-turbine，cache-line 对齐，每机 64B 槽）；
- 写入后递增 cmd_seq[tid]（volatile / seq_cst store）；
- Fast 每次 10ms 读 cmd_seq[tid]，若变化则原子拷贝快照到本地，再调 supervisor；
- 无锁正确性：单写者（Slow）单读者（Fast），seq 递增序 + 对齐槽 = 无撕裂（≤64B 拷贝）。

状态上行（Fast → Slow）：
- Fast 把最新状态写 state_snapshot[tid] + state_seq；
- Slow 1s 聚合时读快照（seq 守卫），供 FarmStateFrame。

约束：
- 快照槽固定大小、编译期分配（static，禁 malloc）；
- 全程不取全场锁（per-slot 无锁协议）；
- 允许 Fast 与 Slow 各自单线程；多核部署时 Fast 独占一核（DR-6/DR-9）。

## 4. RT Core 的编译期确定性（10ms 红线逐条）

| 红线 | 设计保证 |
|---|---|
| 禁全场互斥锁 | 无锁快照协议（per-slot seq + 对齐槽）；无共享写者竞争 |
| 禁 Socket I/O | transport 只挂 COMM ADAPTER（1s），Fast 不触碰 transport 指针 |
| 禁动态内存分配 | 所有 RT Core 状态编译期分配（struct 内定长数组 FCR_MAX_TURBINES）|
| 禁等待 GPU/Modbus | Fast 不依赖任何 slow-path 结果；AWAE/WindMapper 输出经快照 ZOH 消费 |
| 禁等待 AWAE/WD/通信 | 快照读是"最新可用"语义，不阻塞等待新帧 |

## 5. 源文件映射（RT Core 抽取）

| 现有 | RT Core 内 | COMM ADAPTER 内 |
|---|---|---|
| CommandStore | 快照读取器（cmd_snapshot.h，新）| accept_frame / refresh / TTL 计时（慢）|
| YawActionManager | update（纯内存锁存）| — |
| InductionSupervisor | update + apply（纯内存映射）| — |
| Watchdog | 只读健康位检查 | 健康计算（慢）|
| StateAggregator | — | build_frame（1s）|
| Transport | — | ZMQ/Modbus 拉取 + publish |
| rosco_api | fcr_rosco_read_setpoint / publish_state | step_low 聚合 |

新建：bridge/rt_core.h/.c（快照协议 + 10ms 步进）+ bridge/comm_adapter.h/.c（慢路径）。

## 6. 验收（RT Core 单测，纳入门禁 G1）

1. 快照协议无锁正确性：竞争注入（Slow 高频写 vs Fast 高频读）无撕裂（TSAN / 竞态压力测试）；
2. 10ms 路径零系统调用：API 追踪验证无 socket/malloc/mutex；
3. 确定性：固定输入 → 固定 setpoint 输出（bit 级）；
4. 性能预算：单机 RT Core 步进 < 50us（10ms 预算的 0.5%）；
5. 与现 runtime 语义等价：同一命令序列下 RT Core 输出 == 现 fcr_runtime_update_setpoint_one 输出。

## 7. 结论

实时核心拆分 = 把"现 runtime 的 10ms 纯内存部分"（refresh + supervisor + setpoint）
用无锁快照协议从通信/锁路径中剥离，形成可单独编译、可确定性测试的 bridge/rt_core。
离线模式继续走现 transport 路径（rt_core 作为内部实现，输出必须与现 setpoint 一致，见 §6.5）。
