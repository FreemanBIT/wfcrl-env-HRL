# legacy_baseline — 文件桥基线返回输出（Phase 0）

本目录保存 `legacy-file-bridge-baseline` 的短时回归输出，用于验证后续
`farm_control_runtime` 新架构在**外部控制 disabled** 时与旧链路数值等价。

## 内容

| 文件 | 说明 |
|---|---|
| `full_output.csv` | 3 机 Row3T 布局、8 m/s、270° 西风、6 控制步（dt=3 s）的完整时序输出（含 `fastfarm_continuous.log`） |
| `Case.fstf` | 生成的 FAST.Farm 主输入（含 `DT_low`/风场引用等） |
| `FFTest_WT*.fst` | 各机组 OpenFAST 输入 |
| `NRELOffshrBsline5MW_Onshore_*_T*.dat` | 各机组 ElastoDyn/ServoDyn 输入 |
| `InflowWind.dat` | 入流风输入 |
| `measurements_T1.txt` | DISCON 桥写出的量测样例（最后一步） |
| `controls.txt` | 桥读入的指令样例（最后一步） |
| `DISCON_T1.IN` | 机组文件（首行 = 机组 ID） |

## 复现命令

```bash
cd <repo root>
.venv/Scripts/python.exe examples/example_FASTFarm.py --case Turb3_Row1 --steps 6 --mode 0 --wind_speed 8 --wind_direction 270
```

输出目录：`__simul__/fastfarm/continuous/Row3T_Mode0_<timestamp>/`。

## 回归比对（供 Phase 2/8 使用）

- 指标：逐控制步 `farm_power_mw`、每机功率 `power_mw`。
- 短时回归允许的数值差异：ROSCO 数值路径改动引起的 <1e-6 级舍入差异；
  不允许结构性差异（如控制模式改变、缺步、时间偏移）。
