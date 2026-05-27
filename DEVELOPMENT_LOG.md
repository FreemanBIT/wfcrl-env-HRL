# 开发记录

> 记录本分支相较于 [cibeah/WFCRL](https://github.com/cibeah/WFCRL) main 分支的全部变更。

## [v0.0.2] — 2026-05-27

### 1. 新增功能

#### ContinuousFastFarmInterface — 连续流场在线控制
- `wfcrl/interface.py` 新增类，一次启动 FAST.Farm，通过 DISCON bridge DLL 逐时间步交换控制/测量
- 流场持续演化（物理正确），区别于 FastFarmInterface 的每步重启
- 配套 `start()` / `wait_step(controls)` / `stop()` / `close()` 完整生命周期
- `_read_measurements_with_poll()` 轮询各风机测量文件，支持仿真时间同步

#### DISCON Bridge DLL（Fortran 桥接控制器）
- `wfcrl/simulators/fastfarm/src/DISCON_bridge.f90` — Fortran 源码
- 通过 `accINFILE` 读取各风机独立 `DISCON_T{i}.IN` 文件
- 每步写入 `measurements_T{id}.txt`（step、时间、功率、转速、弯矩等）
- 从 `controls.txt` 读取 yaw/pitch/torque 命令
- 偏航通过 `avrSWap(48)` yaw rate 比例控制生效
- 编译脚本 `wfcrl/simulators/fastfarm/src/compile.bat`

#### 统一配置系统（`wfcrl/simul_config.py`）
- `SimulationConfig` — 通用配置基类
- `FastFarmConfig` — FAST.Farm 专有配置
- `FlorisConfig` — FLORIS 专有配置
- 工厂方法 `create_fastfarm()` / `create_floris()` 自动参数映射

#### 统一数据结构（`wfcrl/config.py`）
- `WindConfig` — 支持 InflowWind.dat 全部 WindType (1-7)
- `ControlInput` — per-turbine 控制数组，`scalar()` 工厂方法
- `SimulationOutput` — 统一输出（time, power_mw, farm_power_mw, yaw_deg 等）
- `to_csv()` / `to_dataframe()` 持久化

#### FLORIS 接口重写
- 使用 FLORIS v4.6 API（`FlorisModel`）
- 统一 `setup()` / `reset(wind)` / `step(controls)` / `close()` 生命周期
- 输出 `SimulationOutput`

### 2. Bug 修复

| 问题 | 根因 | 修复 |
|------|------|------|
| 24 台风机功率全 0 | `create_dll()` 写共享 DISCON.IN（ID=1），所有 DLL 读同一 ID | 写 per-turbine `DISCON_T{i}.IN` |
| DISCON bridge 全部风机同一 ID | 硬编码 `'DISCON.IN'`，忽略 `accINFILE` | 使用 `accINFILE` 参数 |
| wait_step 只等 0.01s 就返回 | step 匹配即返回，不等 dt=3s 仿真时间 | 增加 `target_time = step_idx * dt` |
| 偏航命令不生效 | prev_yaw 仅记录，未写入 avrSWap(48) | `avrSWap(48) = (cmd_yaw - nac_yaw) * 0.2` |
| _add_outlist 重复标签 | 追加 OutList 时重写已有参数 | 仅追加通道列表 |
| Fortran 测量文件解析失败 | `*` 格式输出 `step=           0` | 显式格式 `'(A,I0)'` + Python 回退解析 |
| 示例脚本 None 崩溃 | 变量初始化/None 安全 | 预初始化 + None 保护 |

### 3. 升级

#### FAST.Farm v3.5.1 → v5.0.0
- Case.fstf 模板：RotorDiamRef、WrMooringVis、AMReX、WAT、5-参数 k_vAmb/k_vShr
- OutFmt: "ES10.3E2" → "G0"
- install_simulators.py / make_ff.sh 更新下载 URL

#### FLORIS v3 → v4.6
- FlorisModel 替代 tools.FlorisInterface
- fi.set() + fi.run() 替代 fi.calculate_wake()
- 属性 fi.floris.* → fi.core.*

### 4. 文档与示例

| 文件 | 变更 |
|------|------|
| examples/example_FASTFarm.py | 新增（重命名自 example_continuous_control.py + 画图） |
| examples/example_floris.py | 完全重写，使用 FlorisInterface + CLI/控制逻辑 + 画图 |
| README.md | 全面重写：架构总览、示例表格、三类接口代码 |
| requirements.txt | 新增 matplotlib、seaborn、notebook |
| pyproject.toml | 版本 0.0.2，新增分类器 |
| docs/INTERFACE.md | 更新 OpenFAST v5.0.0 参考 |

### 5. 移除

- 删除旧示例：example_fastfarm.py、example_hycon_farm_control.py、run_dafeng_baseline.py、example_online_control.py
- MPI 依赖从必需降为可选（pip install -e ".[mpi]"）
