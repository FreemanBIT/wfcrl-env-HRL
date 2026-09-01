# BASELINE — 现有可运行版本冻结记录（Phase 0）

> 目的：记录本仓库切换到 `farm_control_runtime` 新架构之前的**可运行基线**，
> 后续不再要求新架构兼容 `controls.txt` 文件桥主链路。
> 对应 Git tag：`legacy-file-bridge-baseline`。

## 1. 基线组成

| 组件 | 位置 | 说明 |
|---|---|---|
| WFCRL Python 包 | `wfcrl/` | 仿真接口、配置、环境封装 |
| FAST.Farm 仿真器 | `wfcrl/simulators/fastfarm/bin/FAST.Farm_x64_OMP.exe` | OpenMP 版本，47.6 MB |
| ROSCO 控制 DLL | `wfcrl/simulators/fastfarm/servo_dll/DISCON_WT1.dll` | ROSCO v2.9 源码 + WFCRL 桥编译产物 |
| ROSCO 源码 | `wfcrl/simulators/fastfarm/src/*.f90` | Constants/Types/SysGnuWin/Filters/Functions/ControllerBlocks/ROSCO_Helpers/ReadSetParameters/ROSCO_IO/Controllers/ExtControl/ZeroMQInterface/DISCON |
| 文件桥（legacy） | `wfcrl/simulators/fastfarm/src/DISCON_bridge.f90` | 简单文件 I/O 桥，无 ROSCO |
| 闭环控制库 | `closed-loop-control/` | 方案 A/B/C 控制器、EnKF/MPC、评估与报告 |
| 输入数据 | `FarmInputs/` | TurbSim 风场（6/8/10 m/s × TI 0.05/0.10/0.15 × 2/4/6/8D） |
| 算例模板 | `Case File/DafengH1/`、`FarmInputs/layouts/` | 布局（turb3_row1/turb6_row2/dafeng_h1 等） |

## 2. Baseline 主链路：`controls.txt` 文件桥

### 2.1 指令下发（控制器 → FAST.Farm）

- 文件：运行目录下 `controls.txt`，DISCON 每步读取。
- 格式：

```text
step=<整数步号>
T1 mode=M yaw=<deg> pitch=<deg> power=<MW> minpitch=<deg>
...
END
```

- 步号门控：桥内 `applied_step` 记录已应用步号，仅当 `read_step > applied_step` 才应用。
- 5 种控制模式（`wfcrl.config.control.ControlInput`）：

| mode | 含义 | 下发字段 |
|---|---|---|
| 0 | 纯偏航（绝对目标角） | yaw= |
| 1 | 功率目标 + 最小变桨约束 | power= + minpitch= |
| 2 | 纯变桨绝对值 | pitch= |
| 3 | 变桨绝对值 + 偏航 | pitch= + yaw= |
| 4 | 功率 + 最小变桨 + 偏航 | power= + minpitch= + yaw= |

### 2.2 量测导出（FAST.Farm → 控制器）

- 文件：`measurements_T<i>.txt`，每机一份，每步覆写。
- 字段：

```text
step=<N> t=<s> genpwr=<kW> genspd=<rpm> gentq=<Nm> rotspd=<rpm>
wind_x=<HorWindV m/s>
blpitch=<deg> nacyaw=<deg>
mip1=<kNm> moop1=<kNm> mzb1=<kNm>
```

### 2.3 偏航执行

- 桥内以比例速率执行：`yaw_rate = (cmd_yaw_deg→rad − nac_yaw) × 0.2`，写入 `avrSWap(48)`。
- OpenFAST 坐标：NacYaw=0° 指 +X（东），逆时针为正；对风（270° 西风）时 NacYaw=0°。

### 2.4 降功率执行

- 优先级 `power > torque > pitch`：
  - `power=<MW>`：桥内功率闭环（前馈 `T_ff = P/ω` + PI 修正）；
  - `torque=<Nm>`：直接转矩覆盖；
  - `pitch=<deg>`：集体变桨覆盖。

## 3. ROSCO ZeroMQ 接口（备用通道，默认禁用）

`ZeroMQInterface.f90`（ROSCO v2.9 风格）：

- **17 维测量**（每 `n_DT_ZMQ` 步发送一次）：

| # | 量 | # | 量 |
|---|---|---|---|
| 1 | ZMQ_ID（机组号） | 10 | NacVane |
| 2 | iStatus | 11 | HorWindV |
| 3 | Time | 12 | rootMOOP(1) |
| 4 | VS_MechGenPwr | 13 | rootMOOP(2) |
| 5 | VS_GenPwr | 14 | rootMOOP(3) |
| 6 | GenSpeed | 15 | FA_Acc |
| 7 | RotSpeed | 16 | NacIMU_FA_Acc |
| 8 | GenTqMeas | 17 | Azimuth |
| 9 | NacHeading | | |

- **5 维设定点**（从 ZMQ 服务器接收）：`[torque_offset, yaw_offset, pitch_offset(3)]`，
  写入 `LocalVar%ZMQ_TorqueOffset / ZMQ_YawOffset / ZMQ_PitOffset(1:3)`。
- 编译宏 `ZMQ_CLIENT` 才有实际 ZeroMQ C 客户端调用；未定义时设置 `ErrVar%aviFAIL=-1`。
- 当前 `CntrPar%ZMQ_Mode = 0`（禁用）。

## 4. ContinuousFastFarmInterface（`wfcrl/engine/fastfarm_continuous.py`）

- 继承 `FastFarmInterface`（每步重启 FAST.Farm 的旧接口），一次启动、流场连续演化。
- 生命周期：`setup() → reset(wind) → start() → wait_step(controls) ×N → stop() → close()`。
- `setup()`：`create_ff_case()` 生成 `.fstf/.fst` 全套文件 → `create_dll()` 部署桥 DLL → 写初始 `controls.txt`。
- `wait_step()`：写 `controls.txt`（step 递增）→ 轮询 `measurements_T*.txt`（step 匹配且 `t ≥ step_idx×dt`）→ 解析为 `SimulationOutput`。
- 偏航语义：`nacyaw_from_misalignment / misalignment_from_nacyaw`（`wfcrl/engine/angle_utils.py`）在"对风失准角"与"OpenFAST 绝对 NacYaw"之间转换。

## 5. ROSCO 编译方式

- `src/compile_rosco.bat`（ROSCO + WFCRL 桥 → `servo_dll/DISCON_WT1.dll`）：

```bat
gfortran -shared -static -ffree-line-length-0 -static-libgcc -static-libgfortran -static ^
  -fdefault-real-8 -fdefault-double-8 -cpp -DIMPLICIT_DLLEXPORT -O2 ^
  Constants.f90 ROSCO_Types.f90 SysGnuWin.f90 Filters.f90 Functions.f90 ^
  ControllerBlocks.f90 ROSCO_Helpers.f90 ReadSetParameters.f90 ROSCO_IO.f90 ^
  Controllers.f90 ExtControl.f90 ZeroMQInterface.f90 DISCON.F90
```

- 编译器：TDM-GCC-64 gfortran 10.3.0（`D:\TDM-GCC-64\bin\gfortran.exe`，已在 PATH）。
- `src/_compile.py`：自动查找 gfortran 并编译 `DISCON.F90` → DLL。
- `src/compile.bat`：编译纯文件桥 `DISCON_bridge.f90`。
- 每台机 ServoDyn 输入中 `DLL_FileName` 指向同一 DLL，`DLL_InFile` 指向 `DISCON_T<i>.IN`（首行=机组 ID）。

## 6. 回归输出

短时仿真回归输出（无外部命令/最小命令场景）保存于：

```text
farm_control_runtime/tests/fixtures/legacy_baseline/
```

包括：`controls.txt` 样例、`measurements_T*.txt` 样例、短时 `full_output.csv`、仿真日志摘录。
复现命令见 `fixtures/legacy_baseline/README.md`。

## 7. 恢复与运行

- 基线 tag：`legacy-file-bridge-baseline`。
- 恢复：`git worktree add <dir> legacy-file-bridge-baseline` 或 `git checkout legacy-file-bridge-baseline`。
- 运行方式 A（桥 + 闭环库）：见 `closed-loop-control/INTEGRATION_GUIDE.md` §2。
- 运行方式 B（WFCRL 接口）：`python examples/example_FASTFarm.py --mode N --steps M --wind_speed S`。
