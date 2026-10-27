# fastfarmRT 集成指南（WSL）

> 适用环境：WSL / Linux，具备 fastfarmRT（FAST.Farm 的 OF / WD / AWAE 可在 CPU / GPU 部署运行）
> 模块版本：RT ABI **v2**，AWAE 容量 **1000 点**（`FCR_CAPACITY_VERSION = 2`）
> 配套：`02_WSL_DEPLOY_AND_TEST_GUIDE.md`（部署与测试）、`03_RT_DATA_AND_TIME_POLICY.md`（策略）

---

## 1. 职责划分

| 角色 | fastfarmRT | 场控交互模块 |
|---|---|---|
| 物理模型（OF / WD / AWAE，CPU/GPU） | ✅ | ❌ |
| 时间推进 / 调度 | ✅ 唯一时间源 | 只读 `RTimeCtx` |
| 场级控制接口（命令/状态/桥接/降级） | — | ✅ |
| 与 RCP 通信 | **负责** | 只提供接口数据（无 socket） |
| 单机控制律（ROSCO） | 由平台决定 | 只产出 S08 setpoint |

**插入位置**：主循环中 AWAE 更新之后（主控/低速阶段）。场控模块每 10 ms 被调用一次，
SCADA 交换（每 1 s）由模块内部计数器在边界触发，与调度周期解耦。

---

## 2. 接口映射

| fastfarmRT 侧数据/时点 | 本模块 API | 信号组 | 频率 |
|---|---|---|---|
| 调度步（`n_low`/`t_low`/`dt_low`/`n_fast_low`） | `fcr_rt_step(h, &FcrRtTimeCtx, &status)` | — | 10 ms |
| OF 子步输出（转速/电功率/桨距/机舱方位/推力/轴矩/塔基弯矩） | `fcr_rt_set_openfast_state(h, &FcrOpenFASTRawState)` | S02 | 10 ms |
| 单机控制输出（状态字/转矩命令/桨距命令/已采纳 seq） | `fcr_rt_set_state(h, tid, &FcrRoscoRawState)` | S03 | 10 ms |
| 额外载荷（Ct/Cp/载荷测点，可缺省） | `fcr_rt_set_extra(h, &FcrTurbineExtraState)` | S02 扩展 | 10 ms |
| AWAE 流场 | `fcr_rt_set_flow(h, &FcrFlowState)` | S05 | 1 s（ZOH） |
| AWAE 静态测点目录 | `fcr_rt_set_awae_catalog(h, &FcrAWAECatalogFrame)` | S04 | 启动/变更 |
| 风估计（来源/质量） | `fcr_rt_set_wind_estimate(h, &FcrTurbineWindEstimate)` | S06 | 1 s |
| RCP 命令（调度器搬运） | `fcr_rt_set_command(h, &FarmCommandFrame)` | S01 | 1 s |
| 状态帧（→ RCP） | `fcr_rt_get_output(h, &FarmStateFrame)` | S10 | 1 s |
| AWAE 二跳帧 | `fcr_rt_get_awae(h, &obs, &cat)` | S12/S13 | 1 s |
| 单机控制设定（→ ROSCO） | `fcr_rt_get_setpoint(h, tid, &FcrRoscoExternalSetpoint)` | S08 | 每控制步 |

**字段级归属**：S02 提供物理量测（转速/电功率/桨距/机舱方位/推力/轴矩/塔基弯矩），
S03 提供命令/状态/seq。两者可同时调用，互不覆盖，**调用顺序无关**。

**主循环示例**

```c
FcrRtTimeCtx t = { .n_low = N, .t_low = N*DT_LOW, .dt_low = DT_LOW,
                   .n_fast_low = sub, .dt_high = DT_HIGH, .n_high_low = N_HIGH };
FcrRtStatusInfo st;

for (int i = 0; i < n_turbines; ++i)
    fcr_rt_set_openfast_state(h, &of_state[i]);        /* S02 */

if (scada_boundary) {
    fcr_rt_set_flow(h, &awae_flow);                    /* S05 */
    fcr_rt_set_command(h, &rcp_cmd);                   /* S01 */
}
for (int i = 0; i < n_turbines; ++i)
    fcr_rt_set_state(h, i + 1, &rosco_state[i]);        /* S03 */

fcr_rt_step(h, &t, &st);                               /* 10 ms 入口 */

if (scada_boundary) fcr_rt_get_output(h, &state_frame); /* S10 */
fcr_rt_get_setpoint(h, 1, &sp);                         /* S08 → ROSCO */
```

---

## 3. 运行时共享（调度器 ↔ ROSCO）

fastfarmRT 中 ROSCO 以静态库与 OpenFAST 同核运行，**必须与调度器共用同一个 runtime 实例**，
否则调度器算出的 S08 与 ROSCO 读到的不是同一份数据。模块已内建该共享：

```
fcr_rt_create → configure → init          （调度器，C 侧）
        │  init 内部注入全局单例
        ▼
   共享 runtime
        ├─ 调度器：fcr_rt_step / set_* / get_output / get_setpoint
        └─ ROSCO ：FCR_Init（幂等）/ FCR_ApplyReferences（读同一份 setpoint）
```

- `fcr_rt_init()` 后 `fcr_rosco_api_rt_mode() == 1`；
- RT 模式下 `fcr_rosco_step_high()` 为 no-op（调度器已步进，避免重复）；
- `fcr_rosco_read_setpoint()` / `fcr_rosco_publish_state()` 正常工作；
- `fcr_rt_destroy()` 先解除共享引用。

---

## 4. 容量与帧尺寸

| 对象 | 容量/尺寸 |
|---|---|
| 机组 | 64（`FCR_MAX_TURBINES`） |
| AWAE 观察点 | **1000**（`FCR_MAX_OBSERVATION_POINTS`） |
| `FcrFlowState` | 57568 B |
| `FcrAWAECatalogFrame` (S13) | 40024 B |
| `FcrAWAEObservationFrame` (S12) | 48024 B |
| `FarmStateFrame` (S10) | 25664 B |

收方按 `protocol_version` + `FCR_CAPACITY_VERSION`(=2) 选择布局。

---

## 5. ROSCO Fortran 静态库

```bash
sudo apt-get install -y gfortran
cd farm_control_runtime
bash tools/build_linux.sh          # C 运行时 → build-linux/libfarm_control_runtime.a
bash tools/build_rosco_linux.sh    # ROSCO     → build-linux/libfcr_rosco.a
```

源码在 `rosco/src/`（ROSCO v2.10.6 的控制器路径 9 个文件）+ `rosco/FarmControlCBindings.f90`
+ `rosco/FarmControlInterface.f90`；说明见 `rosco/README_ROSCO_SRC.md`。

编译选项与 Windows 构建一致：`-cpp -fdefault-real-8 -fdefault-double-8 -ffree-line-length-0 -fPIC`。

链接：

```bash
gfortran your_main.o build-linux/libfcr_rosco.a build-linux/libfarm_control_runtime.a -lpthread -lm
```

---

## 6. 里程碑与验证

| 里程碑 | 内容 | 预期证据 |
|---|---|---|
| **M1 模块** | `build_linux.sh` → 93/93；`rt_minimal_example` 冒烟 | ctest 输出、示例输出 |
| **M2 接口** | ABI / 门禁 / 审计三件套；S02/S03/S05/S08 映射联调；ROSCO `.a` 构建 | `check_abi.py` PASS、`libfcr_rosco.a` 生成 |
| **M3 调度/闭环** | fastfarmRT 跑 6T：10 ms 步进、1 s SCADA、命令生效、TTL→D3、状态帧上送、S08 被 ROSCO 采纳 | 与 Windows 基线对比、时序日志、`FcrRoscoAck` 回读 |
| **M4 RT** | 采购后：RT 框架、Jitter / Deadline / WorstCase | 实测报告 |

**M3 第一轮建议**（最小闭环，不依赖 RCP）：
1. fastfarmRT 跑 6T（270°、11 m/s，与 Windows 基线同工况）；
2. 不下发任何命令（纯 baseline），验证 `get_output` 的 1 s 帧与功率量级；
3. 与 Windows 基线对比整场平均功率（差异来源：GPU/CPU 数值差异与采样对齐）；
4. 再注入偏航/诱导命令，核对 NacYaw 与实际功率变化，以及 `FcrRoscoAck` 中已采纳 seq。

---

## 7. 取用清单

| 需要 | 路径 |
|---|---|
| 头文件 | `farm_control_runtime/include/`（入口 `fcr_rt.h`） |
| C 静态库 | `build-linux/libfarm_control_runtime.a` |
| Fortran 静态库 | `build-linux/libfcr_rosco.a`（源码 `rosco/src/`） |
| 最小示例 | `farm_control_runtime/examples/rt_minimal_example.c` |
| RT 测试 | `farm_control_runtime/tests/unit/test_rt_adapter.c` |
| 策略/时间/降级 | `03_RT_DATA_AND_TIME_POLICY.md` |
| 信号字典 | `docs/farm_control/inputs/FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv` |
| 不需要 | `wfcrl/.../DISCON_WT1.dll`（Windows 闭环用）、`transport/modbus`、`transport/zmq` |

---

## 8. 已知限制

| 项 | 说明 |
|---|---|
| 单线程 | 模块非线程安全；RT 路径无锁，须由调度器串行调用 |
| DT_Low | 当前按 1 s 配置；SCADA 边界由内部计数器独立驱动，DT_Low 变化不影响边界 |
| Linux 构建 | C 库与 ROSCO `.a` 构建脚本已提供，需在 WSL 主机首次实测确认 |
| GPU/CPU 数值差异 | 与 Windows 基线存在差异，首轮 baseline 对比后建立 WSL 基线 |
| Modbus 客户端 | 已实现 Server 侧；Client 属平台侧 |
