# M0-0.1 — ROSCO fork 自改盘点（v2.9.4 fork vs 上游 v2.9.4）

> 日期：2026-10-13　分支：dev/farm-control-runtime
> 上游参考：NREL/ROSCO tag v2.9.4（commit d6219569302fd4d8f2b8a3a0e4e72f6e9d10160e）
> 对比方式：git diff --no-index --ignore-all-space（fork 目录 wfcrl/simulators/fastfarm/src/）

## 1. 逐文件 diff 统计

| 文件 | 改动 | 分组 | 说明 |
|---|---|---|---|
| DISCON.F90 | +212/−3 | **A 场控集成** | FCR_FARM_CONTROL 宏：FCR_Step / FCR_ApplyReferences / FCR_PublishState / wfcrl 桥（核心钩子）|
| Controllers.f90 | +17/−1 | **A 场控集成** | 外部偏航目标钩子：FCR_IsYawExternal → FarmYawTargetHeading（替换目标来源，保留 deadband/状态机）|
| ROSCO_Types.f90 | +2/−1 | B 健壮性 | LocalVariables 增加 AlreadyInitialized 字段 |
| ControllerBlocks.f90 | +1 | B 健壮性 | WE_Vw 初始化守卫（避免除零）|
| ReadSetParameters.f90 | +13/−3 | B 健壮性 | ReadAvrSWAP 增加 ErrVar 参数 + AlreadyInitialized 双重加载守卫 |
| Functions.f90 | +3/−1 | B 健壮性 | AeroDynTorque WindSpeed=MAX(WE_Vw, EPSILON) 除零守卫 |
| ExtControl.f90 | −1 | B 健壮性 | 删除调试路径注释 |
| ROSCO_IO.f90 | +164/−160 | B 健壮性 | AlreadyInitialized 写入/读取 restart + Debug 通道 124→125 |
| SysGnuWin.f90 | fork-only | 构建 | Windows 构建辅助（上游无此文件）|
| ROSCO_Helpers.f90 / Constants.f90 | 仅行尾 | 无 | LF↔CRLF 差异，无逻辑改动 |

## 2. 分组含义（迁移策略）

### A 组 — 场控集成修改（v2.10.6 迁移时必须原样保留并适配）

- **DISCON.F90 FCR 宏段**：每 10ms 调用 FCR_Step（transport 拉取→CommandStore→setpoint 生成）+
  FCR_ApplyReferences（把 induction 参考写进 CntrPar：VS_RtPwr×power_ratio / VS_MaxTq / VS_RefSpd /
  PS_BldPitchMin）+ FCR_PublishState（发布 ROSCO 快速状态）。
- **Controllers.f90 偏航钩子**：FCR_IsYawExternal 时用 FCR_GetYawTargetHeadingDeg 替换 NacHeadingTarget，
  保留 ZMQ_Mode/wake-offset 逻辑分支。

迁移适配点（v2.10.6 检查项）：
1. FCR 宏段所在行号/上下文是否变化；
2. CntrPar/LocalVar 字段名是否变化（VS_RtPwr/PS_BldPitchMin/WE_Vw 等）；
3. yaw 控制函数是否重构（NacHeadingTarget 计算路径）。

### B 组 — 健壮性修复（v2.10.6 迁移时核对是否已被上游修复，避免重复/冲突）

| 修复 | 动机 | v2.10.6 需核对 |
|---|---|---|
| AlreadyInitialized 双重加载守卫 | 多机 FAST.Farm 场景防同一 DLL 被多实例重复加载 | 上游 v2.10 可能已有 LoadedDLL 机制 |
| WE_Vw / AeroDynTorque 除零守卫 | AMRWind 零风速除零防护 | 上游可能已修 |
| SysGnuWin.f90 | Windows 平台构建辅助 | 上游 Linux 构建用 sysGnuWin 替代？需确认 |

## 3. 结论

- 自改范围**收敛且可控**：真正的场控集成仅 DISCON.F90 + Controllers.f90 两处（A 组）；
  其余为健壮性修复（B 组）。
- v2.10.6 迁移 = 上游替换 12 个 .f90 + 重放 A 组两处改动 + 核对 B 组是否冗余。
- 多机实例隔离（M0-0.3）将验证 B 组 AlreadyInitialized 守卫在 v2.9.4 多实例下的行为，
  为迁移后的行为基线提供依据。

## 4. 复现命令

    git clone --depth 1 --branch v2.9.4 https://github.com/NREL/ROSCO <ref>
    git diff --no-index --ignore-all-space <ref>/rosco/controller/src/DISCON.F90 wfcrl/simulators/fastfarm/src/DISCON.F90
