# ROSCO 源码包（供 WSL/Linux 构建 `.a`）

本目录提供 fastfarmRT 集成所需的 **ROSCO + 场控桥接 Fortran 源码**与构建方式。

---

## 1. 来源与修改

| 项 | 说明 |
|---|---|
| 基线 | ROSCO **v2.10.6**（NREL，Apache-2.0） |
| 来源 | 本工程 `wfcrl/simulators/fastfarm/src/`（Windows `DISCON_WT1.dll` 的构建源） |
| 本工程修改 | ① `Controllers.f90`：偏航目标外部钩子（`FCR_IsYawExternal` / `FCR_GetYawTargetHeadingDeg`）；② `ControllerBlocks.f90`：`PRC_R_Speed` 标定修正（3 行）；③ 版本号 v2.9.4 → v2.10.6 |
| 算法一致性 | 除上述偏航钩子外，控制律与上游一致 |

> `rosco/src/` 与 `wfcrl/simulators/fastfarm/src/` 的对应文件内容一致；
> 若上游有更新，两处需同步（Windows 侧以 `_compile.py` 为构建入口）。

## 2. 包含 / 排除

**包含（11 个源文件，`src/` 9 个 + 本目录 2 个）**

| 文件 | 作用 |
|---|---|
| `src/Constants.f90` | 物理常量 |
| `src/ROSCO_Types.f90` | 控制参数/局部变量/对象实例类型 |
| `src/SysSubs.f90` | 跨平台最小系统接口（`NewLine` / `PathSep`，`#ifdef _WIN32`） |
| `src/Functions.f90` | 数学/查表工具 |
| `src/Filters.f90` | 滤波器 |
| `src/ControllerBlocks.f90` | 控制器各功能块 |
| `src/ROSCO_Helpers.f90` | 参数读写/工具例程 |
| `src/ReadSetParameters.f90` | 控制参数与设定值读取 |
| `src/Controllers.f90` | **控制主入口**（`ROSCO_Init` / `ROSCO_Update`） |
| `FarmControlCBindings.f90` | FCR C ABI 的 Fortran 镜像 |
| `FarmControlInterface.f90` | FCR 钩子：`FCR_Init / FCR_Step / FCR_ApplyReferences / FCR_PublishState / FCR_Reset` |

**排除（Windows 专用或 RT 不需要）**

| 文件 | 原因 |
|---|---|
| `SysGnuWin.f90` | 定义**同名** `MODULE SysSubs` 且调用 Windows API（`LoadLibrary`）；Linux 用 `SysSubs.f90` |
| `DISCON.F90` | ServoDyn 的 `avrSWAP` 入口；fastfarmRT 直接调用控制器 |
| `ROSCO_IO.f90` | 文件 I/O 与日志；RT 路径不需要 |
| `ZeroMQInterface.f90` | Windows ZMQ 传输；RT 由调度器负责传输 |
| `ExtControl.f90` | 外部 DLL 控制器（Windows 动态加载） |

## 3. 构建（WSL）

```bash
sudo apt-get install -y gfortran          # 若未安装
cd farm_control_runtime

bash tools/build_linux.sh                 # 先构建 C 运行时 → build-linux/libfarm_control_runtime.a
bash tools/build_rosco_linux.sh           # 再构建 ROSCO → build-linux/libfcr_rosco.a
```

关键编译选项（与 Windows DLL 保持一致，勿改）：
`-cpp -fdefault-real-8 -fdefault-double-8 -ffree-line-length-0 -fPIC`

链接（两个归档一起交给链接器）：

```bash
gfortran your_main.o build-linux/libfcr_rosco.a build-linux/libfarm_control_runtime.a -lpthread -lm
```

## 4. 与场控模块的运行时共享（**重要**）

fastfarmRT 中调度器（C 侧，`fcr_rt_*`）与 ROSCO（Fortran 侧，`FCR_*`）**必须共用同一个 runtime 实例**，
否则调度器算出的 S08 setpoint 与 ROSCO 读到的不是同一份。

本工程已实现自动共享：

```
fcr_rt_create → configure → init        （C 侧，调度器）
        │  init 内部调用 fcr_rosco_api_adopt_runtime()
        ▼
   全局单例 g_rt（两者共享）
        ├─ 调度器：fcr_rt_step / set_* / get_output / get_setpoint
        └─ ROSCO ：FCR_Init（幂等）/ FCR_ApplyReferences（读同一份 setpoint）
```

- `fcr_rt_init()` 后，`fcr_rosco_api_rt_mode() == 1`；
- RT 模式下 `fcr_rosco_step_high()` **为 no-op**（调度器已步进，避免重复）；
- `fcr_rosco_read_setpoint()` 仍正常工作 → `FCR_ApplyReferences` 可用于每个控制步；
- `fcr_rt_destroy()` 会先解除单例引用，不会留下悬垂指针。

## 5. 建议的调用序列（fastfarmRT，OF 与 ROSCO 同核）

```c
/* 初始化（一次） */
h = fcr_rt_create(); fcr_rt_configure(h, &cfg); fcr_rt_init(h);
CALL FCR_Init(NUMTURBS, DT)          ! 动态库已共享，内部走 fcr_rosco_api_init（幂等）

/* 每个控制步（10 ms） */
fcr_rt_set_openfast_state(h, &of_state[i]);   /* S02：OF 物理量测 */
fcr_rt_set_state(h, i+1, &rosco_state[i]);    /* S03：ROSCO 输出（可选） */
fcr_rt_step(h, &t, &st);                      /* 调度器步进 */

CALL FCR_ApplyReferences(CntrPar)             ! 应用 S08 setpoint（读共享实例）
CALL ROSCO_Update(...)                        ! 官方控制律

/* 每 1 s */
fcr_rt_set_command(h, &rcp_cmd);   fcr_rt_get_output(h, &state_frame);
```

> 若 fastfarmRT 已直接调用 `ROSCO_Update`（绕过 `FCR_Step`），仍可用 `FCR_ApplyReferences`，
> 因为它只依赖共享实例中的 setpoint；`FCR_Step` 在 RT 模式下只会刷新本地 setpoint 副本。

## 6. 已知限制

- `SYS` 层：`SysSubs.f90` 只提供 `NewLine` / `PathSep`；若你要使用 `WrScr` 等，请补一个
  Linux 版 `SysSubs`（上游 `SysGnuLinux.f90`）。
- 本目录源码**未在 WSL 实测编译**（本机无 WSL/gfortran Linux 环境）；
  首次构建若报模块顺序错误，按错误提示调整 `tools/build_rosco_linux.sh` 中 `SOURCES` 的顺序。
