# 实验对比与可视化分析 v1

分析工具读取由 `wfcrl-experiment` 生成的实验目录，统一计算发电性能、控制活动量、降额代价和约束统计。

## 命令行

```powershell
wfcrl-compare `
  __simul__/experiments/greedy-20260722-120000 `
  __simul__/experiments/yaw-20260722-120100 `
  --baseline greedy `
  --output __simul__/comparisons/greedy-vs-yaw
```

未安装命令行入口时：

```powershell
python -m wfcrl.analysis.cli EXPERIMENT_A EXPERIMENT_B `
  --baseline EXPERIMENT_A_NAME --output REPORT_DIRECTORY
```

## 指标

- `energy_mwh`：根据每个时间步的风场总功率积分；
- `mean_farm_power_mw`：按时间步长加权的平均总功率；
- `energy_gain_mwh / energy_gain_pct`：相对基线的能量变化；
- `mean_power_gain_pct`：相对基线的平均功率变化；
- `yaw_activity_deg`：各风机偏航指令总行程；
- `pitch_activity_deg`：各风机变桨指令总行程；
- `commanded_curtailment_mwh`：根据可用功率参考与降额指令估算的指令降额能量；
- `constraint_events`：发生过至少一个约束裁剪的控制步数；
- `wall_time_s`：实验墙钟运行时间。

`commanded_curtailment_mwh` 是控制指令代价，不等同于尾流耦合后的实际发电损失。如果实验没有记录可用功率参考，该指标为缺失值，不会进行猜测。

## 可比性保护

默认要求所有实验具有相同的：

- 风机数量与布局；
- 风速和风向；
- 控制时间步长；
- 仿真步数。

不满足时会拒绝计算相对增益。探索性分析可以传入 `--allow-incompatible`，但此时相对能量增益可能没有物理可比性。

## 输出

- `comparison.csv`：适合后续统计分析的汇总表；
- `comparison.json`：适合前端或自动化流水线读取；
- `metrics.png`：能量、相对增益、偏航活动量和指令降额量对比；
- `farm_power.png`：所有实验的风场总功率时间序列。

报告目录必须尚不存在，避免覆盖已有分析结果。
