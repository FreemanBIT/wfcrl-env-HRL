# 集成与运行指南（INTEGRATION GUIDE）

本指南说明如何把 `closed-loop-control` 接到你的 **WFCRL / FAST.Farm** 工程并真正跑起来。分三种运行方式，按可靠性从高到低：

1. **Mock（内置解析尾流）** — 无需 FAST.Farm，立即可跑，用于验证代码与流程。
2. **FAST.Farm + 文件桥**（推荐用于你的工程）— 对接工程自带的 `DISCON_bridge.f90` 文件协议。
3. **FAST.Farm + WFCRL 接口** — 复用 WFCRL 的 `FastFarmInterface`（MPI + ZeroMQ）。

---

## 0. 安装

```bash
cd closed-loop-control
pip install -r requirements.txt          # numpy, scipy 必需；floris 强烈推荐
# 可选：pip install -e .                  # 以包形式安装（见 pyproject.toml）
```

- **务必安装 FLORIS**（`pip install floris`）。未安装时代码回退到内置解析尾流，量级偏差大（Mock 下增益 ~25%，真实为个位数百分比）。FLORIS 装好后，`SurrogateModel(prefer_floris=True)` 会自动使用它。
- 可选 `scikit-learn`（方案 C 的高斯过程残差）。

---

## 1. Mock 运行（先跑通这个）

```bash
python examples/example_mock.py                       # A/B/C 对比 greedy
python -m closedloop.demos.test_smoke                 # 冒烟测试（8 项）
python -m tests._run_offline tests.test_units         # 单元测试（13 项，无需 pytest）
python -m closedloop.demos.compare_report --controller B --mode 1 --out report.html
```

看到三方案相对 greedy 正增益、测试全绿，即代码链路无误。

---

## 2. FAST.Farm + 文件桥（对接你的工程，推荐）

你的工程用自定义文件桥 `wfcrl/simulators/fastfarm/src/DISCON_bridge.f90`，通过
`controls.txt` / `measurements_T<i>.txt` 通信。本库的 `FarmBridge` 与 `FastFarmPlant`
就是按这个协议写的。

### 2.1 准备工作

1. **编译桥 DLL**：进入 `wfcrl/simulators/fastfarm/src`，用工程自带脚本编译：
   - Windows：`compile.bat`（生成 `DISCON_WT<i>.dll`）
   - 或 `python _compile.py`
2. **配置 6 机 ServoDyn**：每台机的 ServoDyn 输入里，`DLL_FileName` 指向编好的桥 DLL，
   `DLL_InFile` 指向该机的 `DISCON_T<i>.IN`（**首行 = 机组 ID**，桥据此区分机组）。
   本库可自动生成这些 IN 文件：
   ```bash
   python -c "from closedloop.case_config import default_case; \
   from pathlib import Path; default_case().write_discon_inputs(Path('YOUR_RUN_DIR'))"
   ```
   或运行示例时加 `--write-inputs`。
3. **布局改为 2×3 @ 4D**：把 FAST.Farm 的 `.fstf`（WT_X/WT_Y）与各机 OpenFAST 实例，
   按下列坐标布置（西风 270°，x 为顺风向）：
   ```
   行0: T1(0,0)      T2(504,0)      T3(1008,0)
   行1: T4(0,504)    T5(504,504)    T6(1008,504)     # 单位 m，D=126，4D=504
   ```
   FLORIS 侧用 `cases/farm_2x3_4D/floris_case.yaml`（已是这套坐标）。
4. **时间步对齐**：控制步长 `--dt` 必须等于 FAST.Farm 的 `DT_low`（本工程常见 2 s）。

### 2.2 运行

分两个进程：

- **进程 A**：像平常一样启动 FAST.Farm（在某个运行目录 `RUN_DIR` 下）。
- **进程 B**：启动闭环控制器，指向同一 `RUN_DIR`：
  ```bash
  python examples/example_fastfarm_bridge.py \
      --run-dir RUN_DIR --controller B --mode 1 \
      --steps 300 --dt 2.0 --write-inputs \
      --floris-yaml cases/farm_2x3_4D/floris_case.yaml
  ```

控制器会：读 `measurements_T<i>.txt` → 感知 → 优化 → 原子写 `controls.txt`（step 递增）。

> 桥每步读 `controls.txt`，仅当 `step` 增大才应用；偏航按比例速率执行；降功率优先用
> `power=`（桥内功率闭环 PI）。本库的 `Cmd.to_line()` 已按此格式输出。

### 2.3 与 greedy 对比

分别用 `--controller greedy` 和 `--controller B` 各跑一次（相同风况、相同时长），
对比平均场功率 / 能量。或用 `compare_report`（Mock 下自动双跑；真实环境需你手动双跑
后用 `Evaluator.gain_vs` 比较，见 `closedloop/evaluate.py`）。

---

## 3. FAST.Farm + WFCRL 接口（可选）

若你用 WFCRL 原生方式（MPI + ZeroMQ）启动 FAST.Farm：

```bash
# UNIX
python examples/example_fastfarm_wfcrl.py --controller B --mode 1 --steps 300
# Windows（WFCRL 需 mpiexec）
mpiexec -n 1 python examples/example_fastfarm_wfcrl.py --controller B --steps 300
```

`WFCRLPlant` 适配器会调用你安装的 `wfcrl.interface.FastFarmInterface` 的
测量/命令方法。**注意**：不同 WFCRL 版本方法名略有差异，若绑定报错，打开
`closedloop/wfcrl_plant.py` 的 `_bind()`，把两处方法引用改成你版本的实际名字
（4 行改动），或传入自定义 `measure_map`/`command_map`。

WFCRL 自带 `Turb6_Row2`（2 行 3 列）布局与 `cases.fastfarm_6t`，但其默认间距未必是 4D，
需按 §2.1 第 3 步调整坐标以匹配本工程设定。

---

## 4. 控制模式（5 种）

所有示例都可加 `--mode {1..5}`：

| mode | 含义 | 下发字段 |
|---|---|---|
| 1 | 纯偏航 | `yaw=` |
| 2 | 纯转矩降功率 | `power=` |
| 3 | 纯变桨降功率 | `pitch=` |
| 4 | 偏航+转矩 | `yaw=` + `power=` |
| 5 | 偏航+变桨 | `yaw=` + `pitch=` |

---

## 5. 常见问题

- **控制器一直等不到量测（TimeoutError）**：确认 FAST.Farm 在写 `measurements_T<i>.txt`
  到 `RUN_DIR`，且 6 台机的文件都在更新；确认桥 DLL 已正确挂到 ServoDyn。
- **偏航不动**：确认 `controls.txt` 里 `step` 在递增（本库保证）；确认桥 DLL 版本正确
  （偏航经 `avrSWap(48)` 比例速率执行）。
- **增益为负或异常**：先确认 FLORIS 已安装（`SurrogateModel.backend` 应打印 `floris`）；
  确认布局坐标与风向一致（西风 270° 时 x 为顺风向，T1/T4 为最上游）。
- **方案 C 无残差修正**：需先用 A/B 阶段落盘数据训练 `JointEstimator`（见
  `closedloop/scheme_c/joint_estimator.py`）；未训练时灰箱 MPC 自动退化为方案 B。
- **实时性不足（B/C）**：调小 `EnKFConfig.n_ensemble`、`MPCConfig.n_passes`，或增大
  `t_ctrl`（控制器重规划间隔）。见 `configs/scheme_b_tuning.yaml`。

---

## 6. 代码与文档对应

- 模块编号（M1–M9, A1–A3, B1–B5, C1–C5, D1–D4）↔《风电场闭环控制开发方案.md》
- 公式编号（EnKF 2.11–2.20、MPC 2.21–2.27、标定 1.7–1.11 等）在各文件 docstring 中标注
- 场站：NREL 5MW × 6（2×3 @ 4D）
