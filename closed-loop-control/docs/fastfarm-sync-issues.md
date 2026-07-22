# FAST.Farm 闭环控制验证中的同步问题总结

## 背景

在 `closed-loop-control` 项目中，通过 `ContinuousFastFarmInterface` 驱动 FAST.Farm 进行闭环控制验证。Python 控制器（Scheme A/B/C）与 FAST.Farm 子进程之间通过 `controls.txt` / `measurements_T*.txt` 文件交换控制指令与测量值。

测试中发现多个层面的数据同步问题，导致测量值丢失、仿真提前终止。

---

## 问题 1：WFCRL 测量文件读取竞争

**位置**: `wfcrl/engine/fastfarm_continuous.py` — `_read_measurements_with_poll()` 内部的 `_read_all()`

**现象**: 每步只能读取到 1-3 台机组的测量值，其余为 0。

**根因**: `_read_all()` 遍历 6 个 `measurements_T*.txt` 文件，只要**任意一台**匹配了正确的 step 号就立即返回非 None，轮询退出。6 台机组的 DISCON DLL 在不同 OpenFAST 线程中并行运行，写文件有微小 wall-clock 时差，导致每次轮询只能捕获到先写完的 1-3 个文件。

```python
# wfcrl/engine/fastfarm_continuous.py 第 336-374 行
def _read_all():
    results = {}
    for t_id in range(1, n + 1):
        # ... 读文件、检查 step、检查时间 ...
        if 匹配:
            results[t_id] = vals
    return results if results else None   # ← 有 1 台就返回！
```

**临时修复**: launcher 中增加 `_ensure_all_turbines_measured()`，在 `wait_step` 返回后额外轮询直到全部 N 台文件都有非零风速。

---

## 问题 2：DISCON 测量文件多 token 格式解析

**现象**: `_ensure_all_turbines_measured` 始终报告缺失机组。

**根因**: DISCON bridge 写入的测量文件使用了 Fortran 风格的格式化输出，值与键之间有不定数量的空格：

```
 genpwr=      787.88 genspd=    745.8667 gentq=    10685.57
 wind_x=      5.3035
```

简单按空格 `split()` 后 `wind_x=` 和 `5.3035` 被拆成两个 token，`wind_x=` 之后无值（空字符串），而 `5.3035` 不含 `=` 被跳过。需要实现 `last_label` 状态机来关联跨 token 的键值对。

**修复**: 实现与 WFCRL 原生解析器一致的 `last_label` 多 token 解析逻辑。

---

## 问题 3：控制器未收到真实测量值

**位置**: `example_fastfarm_launcher.py` 原第 170-171 行

**现象**: 控制器内部标定窗口永远为空、优化器在初始模型上重复决策、偏航指令在 0° 和 30° 之间跳变。

**根因**: 循环内部每步重新初始化 `meas={}`：

```python
for k in range(n_steps):
    flow = sensing.estimate({})   # 空数据
    cmds = ctrl.step(flow, {})    # 空字典！
    output = ff.wait_step(cin)    # FAST.Farm 真实数据
    meas = _output_to_meas(output) # 解析了但下一轮被覆盖
```

**修复**: 将 `meas` 和 `flow` 初始化移到循环外，每步用上一步 FAST.Farm 的真实输出传给控制器。

---

## 问题 4：Scheme B 计算耗时导致同步丢失

**现象**: Scheme B 跑到 step 30-34 时 `wait_step` 内部轮询超时，FAST.Farm 早已正常跑完退出，Python 侧抛出 `FastFarmAborted`。

**根因**: Scheme B 的 MPC 每步需要 FLORIDyn 滚动优化（6 台机组 × 坐标细化 × surrogate 多次调用），计算耗时 4-6 秒，远超 FAST.Farm 的控制步长 DT=2s。Python 逐渐落后：

```
FAST.Farm: t=0──2s──t=2──2s──t=4──2s──...──t=200→退出
               ↑             ↑
          写 controls   写 controls (但已间隔6s，FAST.Farm已用旧指令多跑了2步)
Python: [算cmd₁,4s]→写→读→[算cmd₂,4s]→写→...
                        ↑ 测量文件step已是cmd₂发出时的3倍，永远对不上
```

`wait_step` 内部轮询要求测量文件 step 精准匹配，找不到就阻塞 120s，最终检测到 FAST.Farm 进程已退出，抛出异常。

**临时修复**: launcher 中增加进程存活检查 + `try-except` 捕获 `FastFarmAborted`，优雅退出并保存部分轨迹。但治标不治本——只能收集 30 步左右数据。

---

## 核心架构矛盾

当前架构存在一个不可绕过的硬约束：

> **Continuous 模式下，Python 每步计算时间必须 ≤ DT，否则必丢失同步。**

| 方案 | 每步计算时间 | 是否满足 DT=2s |
|---|---|---|
| Scheme A (稳态优化) | 0.3-1s | ✓ |
| Scheme B (EnKF + MPC + FLORIDyn) | 4-6s | ✗ |
| Scheme C (灰盒 MPC 路线) | 同 B | ✗ |

**需求**: 流场必须连续演化（不能每步重启 FAST.Farm），且 Python 与 FAST.Farm 步步同步。

如需在 Continuous 模式下保证同步，可能的解决方向：

1. **增大 DT**：将 Python 控制下发间隔从 2s 增大到 6-8s，匹配 MPC 计算时间。DT_low 保持 0.05s 不变，只改 Python 侧 `write_controls` 频率。MPC 设计中有 `t_ctrl=20s` 的 re-plan 间隔，增大 DT 不会丢失控制信息。

2. **流水线预计算**：在当前 DT 仿真期间预计算下一个 DT 的指令。但约束是下个 DT 的指令依赖当前 DT 的测量值才能决定，所以本质上仍然是顺序依赖。

3. **减少 MPC 计算量**：缩小 FLORIDyn ensemble、用 analytical fallback 替代 FLORIS 做内部 surrogate 评估、减少坐标细化 passes。

4. **最简 workaround**：对 Scheme B 只用 MockPlant 做初步逻辑验证，FAST.Farm 验证暂时只用 Scheme A 和 Scheme C（rl_safety 路线）。

---

## 已做的临时修复汇总

| 文件 | 修复内容 |
|---|---|
| `example_fastfarm_launcher.py` | `meas`/`flow` 初始化移到循环外 |
| `example_fastfarm_launcher.py` | `_parse_discon_file()` 多 token 解析 |
| `example_fastfarm_launcher.py` | `_ensure_all_turbines_measured()` 补全 N 台 |
| `example_fastfarm_launcher.py` | 进程存活检查 + `FastFarmAborted` 捕获 |
| `example_fastfarm_launcher.py` | `timeout=5.0` → `0.5` |

这些修复解决了 Scheme A 的完整运行。Scheme B 因计算量问题仍最多收集 ~30 步数据。
