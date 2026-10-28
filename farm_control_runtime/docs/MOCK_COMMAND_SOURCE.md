# 场控命令源（Modbus / RCP 的测试替代）

> 适用：WSL 阶段**不接入 Modbus**、也不接场级控制器（RCP）时，仍需向调度器提供
> **S01 场控命令**，以便对 OF / WD / AWAE / ROSCO 等其他模块做功能测试。
> 代码：`offline/harness/fcr_mock_command_source.{h,c}`　示例计划：`config/mock_commands_example.csv`

---

## 1. 定位

正式链路是 **RCP → Modbus/ZMQ → 场控模块（S01）**。
本模块只替换"产生 S01 的那一端"：

```
正式：  RCP ──Modbus/ZMQ──> [S01 帧] ──> 场控模块
测试：  计划表(CSV/事件) ──> [S01 帧] ──> 场控模块   ← 本模块
```

命令进入场控模块后，仍走 **CommandStore 的完整状态机**（seq / TTL / stale /
control_enable / requested_effective_time），与正式链路行为一致——因此它既能
"顶替 Modbus"，也能单独用于其他模块测试。

---

## 2. 两种使用方式

### 2.1 带场控模块（验证命令处理）

每个 SCADA 边界（1 s）调用一次，内部自动 `fcr_rt_set_command()`：

```c
FcrMockSource *src = fcr_mock_source_create(n_turbines);     /* 与 FcrRtConfig.n_turbines 一致 */
fcr_mock_source_load_csv(src, "config/mock_commands_example.csv");

for (k = 1; k <= n_steps; ++k) {
    if (k % 100 == 0)                              /* 1 s 边界 */
        fcr_mock_source_tick(src, h, (double)(k / 100));
    fcr_rt_step(h, &ctx, &st);
}
```

### 2.2 不带场控模块（只测 OF / WD / AWAE / ROSCO）

`fcr_mock_source_tick(src, NULL, t)` 只推进内部状态，随后取出合成帧自行使用：

```c
fcr_mock_source_tick(src, NULL, t_low);      /* h = NULL */
FarmCommandFrame fr;
fcr_mock_source_frame(src, &fr);              /* 直接读取 S01 内容 */
/* 例：只取某台机的诱导因子，或仅用于日志比对 */
double a = fr.turbines[3].induction_valid ? fr.turbines[3].induction_ref : 0.0;
```

---

## 3. 接口

| 函数 | 说明 |
|---|---|
| `fcr_mock_source_create(n_turbines)` | 创建（`1..64`） |
| `fcr_mock_source_destroy(src)` | 销毁 |
| `fcr_mock_source_reset(src)` | 清空事件与状态 |
| `fcr_mock_source_add_event(src, &ev)` | 追加事件，返回下标 |
| `fcr_mock_source_load_csv(src, path)` | 批量加载计划，返回条数 |
| `fcr_mock_source_tick(src, h, t_low)` | 每个 1 s 边界调用；`h` 可为 `NULL` |
| `fcr_mock_source_frame(src, &frame)` | 取最近一次合成的帧 |
| `fcr_mock_source_last_time_s(src)` / `fcr_mock_source_frame_seq(src)` | 断言/日志用 |

**事件字段**（`FcrMockEvent`）

| 字段 | 含义 |
|---|---|
| `t_s` | 生效仿真时刻 [s] |
| `turbine_id` | `1..N`；`0` = 全场 |
| `control_enable` | `1` 允许外部场控；`0` 回退 ROSCO 标称 |
| `control_mode` | `0` 基线 / `1` PRC / `2` Ct 前馈 / `3` 预留 |
| `yaw_target_heading_rad` | 绝对机舱目标方位 [rad]；`NAN` = 不改偏航通道 |
| `induction_ref` | 目标轴向诱导因子 `a`；`NAN` = 不改诱导通道 |
| `ttl_s` | 通道 TTL [s]；`0` = 模块默认（偏航 60 / 诱导 5） |
| `duration_s` | 事件持续时长 [s]；`0` = 持续到下一次同通道事件 |

---

## 4. 计划文件格式（CSV）

```csv
t_s,turbine_id,control_enable,control_mode,yaw_target_heading_rad,induction_ref,ttl_s,duration_s
0,0,1,1,,,0,0
200,3,1,1,0.1745,,0,0
200,4,1,1,0.1745,,0,0
300,4,1,1,,0.20,0,100
500,0,0,0,,,0,0
```

- 首行可为表头（自动跳过）；`#` 开头为注释；空值 = "不改变该通道"；
- 同一通道多次命中时，取 `t_s` 最大且仍有效的一条；
- 未命中任何事件的机组 → `control_enable = 0`（回退 ROSCO），偏航/诱导通道无效。

示例见 `config/mock_commands_example.csv`（6 机、200 s 偏航、300 s 诱导、500 s 回退）。

---

## 5. 与信号字典的对应

| S01 字段 | 本模块来源 |
|---|---|
| S0102 `command_basis_sim_time_s` | 每次 tick 取当前 `t_low`（保证不触发 stale） |
| S0104 `control_enable` | 事件 `control_enable` |
| S0105–S0107 诱导 | 事件 `induction_ref` / `induction_seq`(=帧序号) / `induction_valid` |
| S0108–S0110 偏航 | 事件 `yaw_target_heading_rad` / `yaw_seq`(=帧序号) / `yaw_valid` |
| S0111 `control_mode` | 事件 `control_mode` |
| S0112 `requested_effective_sim_time_s` | 固定 `0`（立即生效） |

---

## 6. 测试

单元测试：`tests/unit/test_mock_command_source.c`

| 用例 | 覆盖 |
|---|---|
| `mock_all_turbines` | 全场事件 → 所有机组同帧生效 |
| `mock_per_turbine_channels` | 单机作用域 + 偏航/诱导通道独立 |
| `mock_timing_and_duration` | 未到时刻不生效、`duration_s` 到期失效 |
| `mock_csv_load` | 表头跳过、空值语义 |
| `mock_feeds_runtime` | 注入命令经 CommandStore → setpoint 生效（1 s 后 `induction_enable=1`） |
