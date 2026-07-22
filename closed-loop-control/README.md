# closed-loop-control

数据驱动的风电场闭环控制代码库，面向本工程的 **WFCRL / FAST.Farm** 仿真环境，场站为 **NREL 5MW × 6 机组（2 行 3 列，行/列间距均 4D = 504 m）**。

实现了开发方案（`风电场闭环控制开发方案.md`）中的三套方案与全部共用模块，并配套完整的测试 demo（含贪婪基线对比）。所有公式与推导文档（`风电场闭环控制方案-环节实现与公式推导`）逐条对应。

## 目录结构

```
closed-loop-control/
├── closedloop/                     # 主包
│   ├── types.py                    # 数据类型 TurbineMeas / Cmd / FlowEstimate
│   ├── config.py                   # 集中配置（路径/时序/种子）
│   ├── control_mode.py             # M9  5 种控制模式（对齐工程 setpoints/桥字段）
│   ├── bridge.py                   # M2  FAST.Farm 文件桥接（controls.txt / measurements_T*.txt）
│   ├── sensing.py                  # M3  感知与预处理（滤波、来流重构、场级风向）
│   ├── induction.py                # M4  诱导因子 ↔ 功率/转矩/桨距换算
│   ├── surrogate.py                # M5  FLORIS 代理封装（+ 内置解析高斯尾流回退）
│   ├── base_controller.py          # M6  控制基类 + 指令仲裁（限幅/死区/速率）
│   ├── evaluate.py                 # M7  评估指标 + 轨迹保存/加载（CSV/pickle）
│   ├── runner.py                   # M8  闭环编排 + Plant 抽象（FastFarm / Mock）
│   ├── wfcrl_plant.py              #     WFCRL FastFarmInterface 适配器
│   ├── case_config.py              # M1  2×3@4D 场站配置与算例生成
│   ├── wind_schedule.py            #     风况序列（定常/阶跃/爬坡）
│   ├── scheme_a/                   # 方案 A：FLORIS 再标定 + 稳态重优化
│   │   ├── calibration.py          #   A1 周期参数标定（WLS + SVD 截断）
│   │   ├── steady_opt.py           #   A2 稳态重优化（Serial-Refine / SLSQP，模式感知）
│   │   └── controller_a.py         #   A3 方案 A 控制器
│   ├── scheme_b/                   # 方案 B：FLORIDyn + EnKF + MPC（主方案）
│   │   ├── floridyn_model.py       #   B1 动态尾流（OP 平流 + 时延 + TWF）
│   │   ├── enkf.py                 #   B2 集合卡尔曼滤波（联合状态-参数估计）
│   │   ├── mpc.py                  #   B3 滚动时域 MPC（时域覆盖 tau_max）
│   │   └── controller_b.py         #   B5 方案 B 控制器
│   ├── scheme_c/                   # 方案 C：灰箱学习增强
│   │   ├── residual_model.py       #   C1/C3 灰箱残差 + GreyBoxSurrogate
│   │   ├── joint_estimator.py      #   C2 联合状态-参数-残差估计
│   │   ├── rl_policy.py            #   C4 RL 策略 + 安全投影
│   │   └── controller_c.py         #   C5 方案 C 控制器（灰箱 MPC / RL 两条路线）
│   └── demos/                      # 测试 Demo（三方案共用）
│       ├── greedy_baseline.py      #   D1 贪婪基线控制器
│       ├── run_demo.py             #   D2 Demo 运行器（CLI，保存 pkl/csv）
│       ├── compare_report.py       #   D3 对比评估与 HTML 报告（在线双跑）
│       ├── compare_files.py        #   D3 离线对比两个已保存轨迹
│       └── test_smoke.py           #   D4 冒烟测试（自动判定）
├── examples/                       # 可直接运行的示例脚本
│   ├── example_mock.py             #   无需 FAST.Farm，立即跑通 A/B/C vs greedy
│   ├── example_fastfarm_bridge.py  #   接真实 FAST.Farm（文件桥，推荐）
│   └── example_fastfarm_wfcrl.py   #   接真实 FAST.Farm（WFCRL 接口）
├── cases/farm_2x3_4D/              # 生成的算例（FLORIS yaml + DISCON_T*.IN + 布局）
├── configs/                        # 实验与调参配置示例
├── tests/                          # 模块单元测试（含离线运行器）
├── INTEGRATION_GUIDE.md            # ★ 集成与运行指南（务必先读）
├── pyproject.toml                  # 可安装包定义
├── requirements.txt
└── README.md
```

## 安装

```bash
pip install -r requirements.txt
```

- 必需：`numpy`, `scipy`。
- 推荐：`floris`（代理尾流模型）。**未安装时代码自动回退到内置解析高斯尾流**，全流程仍可运行与测试（量级为示意，趋势与符号正确）。
- 可选：`scikit-learn`（方案 C 的高斯过程残差；否则用岭回归残差）。

## 快速开始

### 1）跑一次 Demo 并与贪婪基线对比

```bash
# 方案 B，纯偏航模式，定常 8 m/s，生成 HTML 报告
python -m closedloop.demos.compare_report \
    --controller B --mode 1 --wind steady_8ms --duration 400 --out report.html
```

输出示例：
```
[B mode1 vs greedy]
  net farm-power gain : +25.57 %
  mean farm power     : 6107.5 kW (baseline 4864.4 kW)
  per-turbine P ratio : T1:0.80, T2:3.82, T3:16.48, T4:0.78, T5:4.55, T6:53.04
```

### 2）单独运行某控制器

```bash
python -m closedloop.demos.run_demo --controller A --mode 4 --wind step --duration 400 --out results/
```

### 3）冒烟测试（回归）

```bash
python -m closedloop.demos.test_smoke          # 独立运行
# 或在装有 pytest 的环境：
pytest closedloop/demos/test_smoke.py tests/ -v
# 无 pytest 环境可用离线 shim：
python -m tests._run_offline tests.test_units
```

### 4）真实 FAST.Farm 运行 + 离线对比

真实运行时，greedy 与各方案通常是**两次独立 FAST.Farm 仿真**。分别跑并保存轨迹，再离线对比：

```bash
# 分别跑（接真实 FAST.Farm 见 INTEGRATION_GUIDE.md）
python examples/example_fastfarm_bridge.py --run-dir RUN_DIR --controller greedy --steps 300
python examples/example_fastfarm_bridge.py --run-dir RUN_DIR --controller B --steps 300
# 或 Mock 下用 run_demo 保存 pkl/csv：
python -m closedloop.demos.run_demo --controller B --mode 1 --out results/

# 离线对比两个已保存轨迹，出报告
python -m closedloop.demos.compare_files \
    --ctrl results/trajectory_B_mode1_steady_8ms.csv \
    --baseline results/trajectory_greedy_mode1_steady_8ms.csv \
    --name B --out report_B.html
```

> **接真实 FAST.Farm 请先读 [`INTEGRATION_GUIDE.md`](INTEGRATION_GUIDE.md)** —— 含桥 DLL 编译、DISCON_T*.IN 配置、2×3@4D 布局坐标、时间步对齐等关键步骤。

## 5 种控制模式（M9）

对齐工程 `ZeroMQInterface.f90` 的 `setpoints(5)` 与 `DISCON_bridge.f90` 的命令字段。三方案控制器均支持：

| mode | 名称 | 优化变量 | 下发字段 |
|---|---|---|---|
| 1 | YAW | 偏航 γ | `yaw=` |
| 2 | TORQUE | 诱导 a→转矩 | `power=`（桥内闭环）|
| 3 | PITCH | 诱导 a→桨距 | `pitch=` |
| 4 | YAW_TORQUE | γ + a | `yaw=` + `power=` |
| 5 | YAW_PITCH | γ + a | `yaw=` + `pitch=` |

## 接入真实 FAST.Farm

默认使用 `MockPlant`（代理驱动，无需 FAST.Farm 即可跑通全链路与测试）。接真实 FAST.Farm：

```bash
python -m closedloop.demos.compare_report --controller B --mode 1 \
    --fastfarm /path/to/fastfarm/run --duration 600
```

- 你需另行启动 FAST.Farm，其运行目录即 `--fastfarm` 指向目录。
- 桥 DLL 需先用工程内 `src/compile.bat` / `_compile.py` 编好 `DISCON_WT<i>.dll`，并把 `cases/farm_2x3_4D/DISCON_T<i>.IN`（首行 = 机组 ID）放到各机 ServoDyn 的 `DLL_InFile`。
- 场级风向（扫描雷达通道）可通过在运行目录写一个方向文件并把路径传给 `FastFarmPlant(direction_file=...)` 提供；缺省时用参考风向。

## 与开发方案 / 公式推导的对应

- 模块编号（M1–M9, A1–A3, B1–B5, C1–C5, D1–D4）与《开发方案》一致。
- 关键公式编号（如 EnKF 2.11–2.20、MPC 2.21–2.27、标定 1.7–1.11）在各文件 docstring 中标注。

## 说明与边界

- 内置解析尾流在 4D 间距下尾流较深，Mock 下的增益偏大（~25%）；真实 FLORIS/FAST.Farm 下通常为个位数百分比。**代码机理与符号正确，量级以高保真为准**。
- 方案 C 的残差/RL 需要 A/B 阶段的运行数据训练；未提供训练数据时，灰箱 MPC 退化为 B（物理模型），RL 使用占位线性策略 + 安全投影。
- MPC 采用坐标精化 + 稳态收益目标（内部滚动覆盖 tau_max），在 6 机规模稳健且快速；如需更精细的时序整形可替换为 B 样条参数化 + 梯度法（见 `mpc.py` 注释）。
