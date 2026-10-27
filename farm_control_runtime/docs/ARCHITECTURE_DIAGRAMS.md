# 场控模块架构图（Windows 运行态 / WSL 部署态）

> 配套：`WSL_DEPLOY_AND_TEST_GUIDE.md`、`WSL_READINESS.md`、`RT_DATA_AND_TIME_POLICY.md`
> 图片：`docs/figures/arch_windows_runtime.png|.svg`、`docs/figures/arch_wsl_deploy.png|.svg`

---

## 图 1  Windows 离线闭环运行架构

![Windows 运行架构](figures/arch_windows_runtime.png)

**要点**：FAST.Farm 每台风机**一个独立进程**；场控 runtime（C11）与 ROSCO（Fortran）
**静态链接进同一个 `DISCON_WT1.dll`**；Python RCP 通过 ZMQ 与该 DLL 内的 transport 通信。

```mermaid
flowchart TB
  subgraph PY["Python 侧（RCP / 测试驱动）"]
    C1["FarmZmqClient<br/>PUB :5556 / SUB :5557"]
    C2["farm_protocol.py<br/>ctypes 逐字节镜像"]
  end
  BUS["ZeroMQ 总线（libzmq.dll）"]
  subgraph FF["FAST.Farm_x64_OMP.exe × NT（每机一进程）"]
    subgraph P1["进程 #1（T1）"]
      OF1["OpenFAST 物理 + ServoDyn<br/>每 10 ms 调 DISCON()"]
      subgraph DLL1["DISCON_WT1.dll（FCR 集成）"]
        D1["DISCON.F90 钩子<br/>FCR_Init / FCR_Step /<br/>FCR_ApplyReferences / FCR_PublishState"]
        R1["farm_control_runtime（C11）<br/>CommandStore · StateStore ·<br/>Aggregator(1 s) · Bridge · AWAE"]
        S1["setpoint S08 → CntrPar → ROSCO 控制链"]
      end
      OF1 --> D1 --> R1 --> S1
    end
    P2["进程 #2…#NT<br/>各自一份 runtime 单例"]
  end
  PY -->|"① 命令帧 S01（5664 B）"| BUS
  BUS -->|"② 状态帧 S10（25664 B）+ AWAE S12/S13"| PY
  BUS <-->|"SUB 命令 / PUB 状态"| DLL1
```

**调用链（10 ms，进程内）**

```
ServoDyn → DISCON(avrSWAP,...)
  ├─ FCR_Init(tid) / FCR_ProviderInit()            （首次）
  ├─ FCR_Step(tid, t)
  │    └─ ZMQ 命令拉取 → CommandStore（seq/TTL/control_enable/偏航锁存）
  │       → 生成 setpoint（bridge core 或 InductionSupervisor）→ rt->setpoint[tid]
  ├─ FCR_ApplyReferences(CntrPar)                   → PRC/PS/VS 注入
  ├─ [ROSCO 官方控制链]（YawRateControl 内含外部偏航目标钩子）
  └─ FCR_PublishState / FCR_PublishExtra            → StateStore（1 s 后聚合为 S10）
```

**产物与验收**：`DISCON_WT1.dll`（2369.7 KB）、`libzmq.dll`、`check_abi.py`（82 字段偏移）、
`audit_signals.py`（88/88）、单元测试 98/98、T-21 300 s 回归 4379 kW（+0.3%）。

---

## 图 2  WSL 部署架构

![WSL 部署架构](figures/arch_wsl_deploy.png)

**要点**：WSL 内**没有** FAST.Farm 与 Windows DLL；场控模块以**静态库**形式被调度器
每 10 ms 调用；**传输由调度器负责**（模块不持有 socket）；FAST.Farm/ROSCO 的 Linux
集成属采购后 M4。

```mermaid
flowchart TB
  RCP["RCP / 上位场级控制器（可选，1 s 周期）"]
  COMM["通信层（调度器实现，DT_SCADA = 1 s）<br/>ZMQ / Modbus / 共享内存"]
  subgraph WSL["WSL 进程内（单线程调度器，10 ms 步进，唯一时间源）"]
    SRC["平台状态源<br/>FAST.Farm/OF/WD（采购后 M4）<br/>或 Mock / 文件注入（当前测试）"]
    subgraph MOD["场控模块 libfarm_control_runtime.a（C11）"]
      A["fcr_rt_adapter（10 ms 入口）<br/>step / set_state / set_command /<br/>get_output / get_setpoint"]
      B["CommandStore · StateStore ·<br/>Aggregator(1 s) · Bridge / InductionSupervisor"]
      C["AWAE builder · 生命周期<br/>create/configure/init/reset/stop/destroy<br/>FcrRtStatus + StatusInfo"]
    end
    CTRL["单机控制 / ROSCO<br/>（采购后 M4：Linux .so）"]
    T["测试与工具<br/>build_linux.sh · fcr_unit_tests 91/91<br/>rt_minimal_example · check_abi/gates/audit"]
  end
  RCP -->|"命令 S01"| COMM
  COMM -->|"set_command（1 s）"| A
  A -->|"get_output（S10）"| COMM
  COMM -->|"状态 S10"| RCP
  SRC -->|"S02/S03/S06/AWAE"| A
  A -->|"S08 setpoint"| CTRL
```

**调用序列**

```
create → configure → init
  每 10 ms : set_state(每机) → step(RTimeCtx) → 返回值/StatusInfo 判 D0~D4
  每 1 s  : set_command(RCP 命令) / get_output(状态帧)
reset（run→reset→run 可重复）→ stop → destroy
```

**WSL 中可执行**：`tools/build_linux.sh`（cmake 或 `--gcc` 兜底）、`fcr_unit_tests`（91/91）、
`examples/rt_minimal_example`、`check_abi.py` / `check_icd_gates.py` / `audit_signals.py`。

**WSL 中不含（M4）**：`DISCON_WT1.dll`（Windows PE）、ZMQ/Modbus 传输实现、FAST.Farm 闭环。

---

## 两图差异对照

| 维度 | Windows 运行态（图 1） | WSL 部署态（图 2） |
|---|---|---|
| 进程模型 | FAST.Farm 每机一进程 + Python 控制进程 | 单进程（调度器）+ 以后可拆 |
| 场控模块形态 | 静态链接进 `DISCON_WT1.dll` | 静态库 `libfarm_control_runtime.a` |
| 谁驱动 10 ms | ServoDyn 调 `DISCON()` | 调度器调 `fcr_rt_step()` |
| 时间来源 | FAST.Farm 仿真时间（`sim_time_s` 传入） | 调度器 `FcrRtTimeCtx`（唯一权威） |
| 传输 | DLL 内 ZMQ（SUB/PUB，libzmq.dll） | 调度器负责（模块不持有 socket） |
| 外部接口 | ZMQ 帧（S01/S10/S12/S13） | `set_command` / `get_output` 等函数调用 |
| 单机控制 | 同进程 ROSCO（Fortran） | 采购后 M4（Linux .so） |
| 物理模型 | FAST.Farm + OpenFAST | 采购后 M4；当前 Mock/注入 |
| 线程/锁 | OpenMP 多线程 + `CRITICAL_SECTION` | 单线程约定；RT 路径无锁 |
