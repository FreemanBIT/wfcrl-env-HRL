# WFCRL Dashboard v0.2 更新说明

> 更新日期: 2026-07-27

---

## 新增功能

### 1. VTK 尾流可视化

FAST.Farm 仿真完成后，自动生成 VTK 格式的 disturbed wind 流场文件，可在历史实验中直接查看：

- **俯视图**：轮毂高度 (z=90m) 水平切片，显示风速分布
- **时间步滑条**：拖动查看不同时刻的尾流发展
- **真实流场**：基于 FAST.Farm 的低分辨率域计算结果，非简化模型
- **按需开启**：在运行实验时勾选「生成 VTK」才输出，避免不必要的大量文件

### 2. 偏航尾流控制（修复）

修复了 DISCON bridge DLL 偏航指令不生效的问题：

- **根因**：OpenFAST v5.0.0 中 ServoDyn 忽略 avrSWAP(48) 偏航速率指令
- **解决方案**：使用 `set_fixed_yaw()` 方法，锁定 YawDOF 并直接设置初始 NacYaw
- **效果**：偏航 -17° 时尾流中心线偏移约 45m（4D 间距 ~500m 处），VTK 热力图清晰可见
- **内置控制器**：`fastfarm_yaw` 自动使用此方法

### 3. 自定义控制器插件系统

支持用户上传自定义控制器，无需修改平台代码：

- **标准接口**：继承 `WindFarmController`，实现 `reset()` 和 `compute()`
- **多种上传方式**：上传 .py 文件 / 在线编辑 / 从模板创建
- **一键验证**：使用 Mock 后端快速测试控制器是否正常
- **自动注册**：上传后自动出现在控制器选择列表中

### 4. 湍流风生成

支持生成 FAST.Farm 兼容的 Kaimal 谱湍流风场：

- **稳态湍流**：固定风速/风向，可选择目标风场自动计算网格尺寸
- **调度风**：多段风速/风向随时间线性变化，叠加 Kaimal 谱湍流
- **风场自适应**：根据风场布局自动计算 .bts 文件的最小网格宽度（≥756m 满足 FAST.Farm 要求）
- **格式兼容**：ID=8，非周期全流场格式

### 5. 风电场配置管理

- **11 个内置布局**：6T/3T/HornsRev/DafengH1 等
- **自定义布局**：可视化编辑器 + 预设模板（单行/2行3列/3行3列）
- **实时预览**：散点图显示风机位置

---

## 界面改进

| 页面 | 改进内容 |
|------|---------|
| ▶️ 运行实验 | 新增风类型选择（稳态 / 湍流）、VTK 勾选框、backend 选择 |
| 📊 历史实验 | VTK 流场直接显示、参数 Tab（功率/偏航/风速）、稳态值表格 |
| 🎮 控制器管理 | 上传 / 在线编辑 / 快速测试 / 删除 |

---

## Bug 修复

| 问题 | 根因 | 修复 |
|------|------|------|
| ASCII 编码崩溃 | `.decode('ascii')` 处理含中文路径 | 全部改为 `utf-8` |
| .bts Grid 太窄 | FAST.Farm 要求宽 ≥756m，默认 680m | `calc_grid_from_layout()` 自动计算 |
| .bts ID 不兼容 | 生成时 ID=7，FAST.Farm 需要 ID=8 | 生成时显式设为 8 |
| 偏航不生效 | DISCON bridge avrSWAP(48) 被 ServoDyn 忽略 | 改用 `set_fixed_yaw()` 锁定 YawDOF |
| 中文路径问题 | FAST.Farm Fortran 不识别中文路径 | 自动复制到 ASCII 短路径 |
| 实验运行卡死 | `st.rerun()` 状态混乱 | 移除 rerun，改用自然状态更新 |
| 变量名错误 | `wind_dir` vs `wind_dir_val` | 统一变量名 |
| 缺少 metadata | 异常导致 metadata 未保存 | 增加 try/except 保护 |
| VTK 未显示 | f-string 引号嵌套错误 | 修复 key 表达式 |

---

## 技术栈

| 组件 | 版本/说明 |
|------|----------|
| Python | ≥ 3.11 |
| Streamlit | ≥ 1.28 |
| FAST.Farm | v5.0.0 (OpenMP) |
| PyVista | VTK 可视化 |
| openfast_toolbox | .bts / .outb 文件读写 |
| FLORIS | 可选，工程尾流模型 |

---

## 已知限制

1. **偏航仅支持固定角度**：`set_fixed_yaw()` 在 setup 阶段锁定偏航 DOF，不支持动态偏航调节。如需动态偏航需重写 DISCON bridge DLL。
2. **VTK 文件较大**：每步 ~1MB，20 步实验约产生 130 个文件 (~130MB)。建议仅在研究需要时开启。
3. **FAST.Farm 文件竞争**：偶发的 `controls.txt` 权限冲突可能导致仿真失败。增大 dt 或重试可缓解。
4. **自定义控制器不支持 FAST.Farm**：当前自定义控制器仅支持 Mock 和 FLORIS 后端。
5. **尾流可视化精度**：VTK 显示的是低分辨率域 (Grid4D) 的 disturbed wind 场，高分辨率细节请查看 .outb 文件。
