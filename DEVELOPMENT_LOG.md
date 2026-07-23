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

---

## [v0.1.0] — 2026-07-23

### 新增: WFCRL Dashboard — Web 可视化实验平台

基于 Streamlit 构建的风电场控制实验管理界面 (`app/WFCRL_Dashboard.py`)。

#### 风电场管理
- 内置 11 个布局的可视化浏览（6T/3T/HornsRev/DafengH1...）
- 自定义布局创建器（预设 + 坐标编辑器 + 实时散点图预览）

#### 湍流风生成
- Kaimal 谱湍流风生成器 (`app/wind_generator.py`)
- 自动根据风场布局计算网格尺寸（避免 FAST.Farm 盒子太窄报错）
- 调度风：多段风速/风向随时间线性变化，叠加湍流
- 支持 .bts 文件浏览和验证

#### 控制器管理
- 自定义控制器插件系统 (`app/controller_loader.py`)
- exec 沙箱加载，自动识别 WindFarmController 子类
- 在线创建/编辑/上传/删除控制器
- 一键 Mock 验证

#### 内置 FastFarmYawController
- `wfcrl/controllers/reference.py` 新增，专为 FAST.Farm 设计
- 自动识别上游风机，施加固定偏航角
- 注册为内置控制器 `fastfarm_yaw`

#### 实验运行
- 四步流程：选风场 → 选控制器 → 配风况 → 运行
- 支持稳态/湍流/调度风三种风类型
- Mock（秒级）/ FAST.Farm（分钟级）双后端

#### 历史实验
- 详情模式：功率/偏航/风速/转矩/转速 曲线 Tab
- 对比模式：叠加曲线 + 增益计算 + 一键完整报告

#### 尾流可视化
- 简化高斯尾流模型 2D 热力图
- 偏航尾流控制可视化（偏航箭头 + 尾流偏转）
- 自适应网格降采样（支持 HornsRev1 80 台风机）

#### Bug 修复
| 问题 | 根因 | 修复 |
|------|------|------|
| ASCII 编码崩溃 | `.decode('ascii')` + 中文路径 | 改为 `utf-8` |
| .bts Grid 太窄 | 硬编码 680m，FAST.Farm 需 ≥756m | `calc_grid_from_layout()` 自动计算 |
| .bts ID 不兼容 | 生成时 `ID=7`，FAST.Farm 需 `ID=8` | 生成时显式设为 8 |
| FARMINPUTS_DIR 硬编码 | 指向旧项目路径 `D:\HR_Project\...` | 改为相对路径 + 环境变量覆盖 |
| FAST.Farm 读不到 .bts | 中文路径 Fortran 不识别 | 自动复制到 ASCII 短路径 |

#### 文档
- `DASHBOARD_README.md` — 界面使用说明
- `CURRENT_STATUS_AND_USAGE.md` — 更新至 v0.1.0 状态
