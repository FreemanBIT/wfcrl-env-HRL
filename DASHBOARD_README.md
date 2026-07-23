# WFCRL Dashboard — 风电场控制实验平台 Web 界面

基于 Streamlit 构建的可视化实验管理界面，支持 **Mock / FAST.Farm** 两种仿真后端，
**内置控制器 + 自定义控制器插件**，以及**稳态/湍流/调度风**三种风况。

---

## 快速开始

### 1. 安装依赖

```bash
pip install -e "."                    # 安装 wfcrl 基础包
pip install streamlit pandas pyyaml matplotlib openfast_toolbox
```

### 2. 启动界面

```powershell
cd wfcrl-env-HRL
streamlit run app/WFCRL_Dashboard.py
```

浏览器打开 `http://localhost:8501`

### 3. 运行第一个实验

1. **▶️ 运行实验** → 选风场 `6T` → 选控制器 `greedy` → 后端 `Mock` → 点运行
2. **📊 历史实验** → 查看结果曲线

---

## 功能模块

### 🏭 风电场管理
- 查看 11 个内置布局（6T/3T/HornsRev/DafengH1...）
- 创建自定义布局（可选预设：2行3列/单行/3行3列）
- 坐标编辑器 + 实时散点图预览

### 🌤️ 湍流风
| Tab | 功能 |
|-----|------|
| **生成湍流风** | Kaimal 谱湍流，选择风场自动计算网格尺寸 |
| **调度风** | 定义风速/风向随时间变化的调度表，自动插值生成 |
| **已有风文件** | 浏览 custom_winds/ 和模板中的 .bts 文件 |

### 🎮 控制器管理
- **内置控制器**: greedy / fixed_yaw / fixed_derating / fastfarm_yaw
- **自定义控制器**: 上传 .py 文件 → 自动识别 WindFarmController 子类
- **在线创建**: 浏览器中编写控制器代码 → 保存 → 立即使用
- **快速测试**: 用 Mock 后端一键验证控制器是否能正常运行

### ▶️ 运行实验
四步流程:
```
1️⃣ 选择风电场  →  2️⃣ 选择控制器  →  3️⃣ 配置风况  →  4️⃣ 🚀 运行
```

支持:
- **风类型**: 稳态 / 湍流 / 调度风
- **后端**: Mock（秒级）/ FAST.Farm（分钟级）
- **自定义控制器**可在 Mock 后端使用

### 📊 历史实验
- 浏览全部已完成实验（筛选/搜索）
- **详情模式**: 查看完整参数面板（功率/偏航/风速/转矩/转速曲线）
- **对比模式**: 选两个实验 → 叠加功率曲线 → 计算增益 → 一键生成完整报告

### 🗺️ 尾流可视化
- 简化高斯尾流模型热力图
- 支持偏航尾流控制的可视化（偏航箭头 + 尾流偏转）
- 各风机估算功率/风速/偏航角数据表

---

## 自定义控制器开发

### 接口定义

```python
from wfcrl.controllers import WindFarmController, ControlIntent

class MyController(WindFarmController):
    def reset(self, context):
        self.n = context.n_turbines
        # 实验开始时调用

    def compute(self, observation):
        """每步调用一次，返回控制指令"""
        # 输入 observation:
        #   time_s: float           当前时间
        #   power_mw: array[6]      各风机功率 (MW)
        #   wind_speed: array[6]    各风机风速 (m/s)
        #   yaw_misalignment_deg    当前偏航角

        return ControlIntent(
            yaw_misalignment_deg=np.zeros(self.n),
        )
```

### 上传到平台
1. **🎮 控制器管理** → **📤 上传控制器** → 选择 .py 文件
2. 或 **✏️ 创建/编辑** → 在线编写 → 保存
3. 去 **▶️ 运行实验** → 选择自定义控制器 → 运行

---

## 湍流风生成

### 稳态湍流
- 固定风速/风向，Kaimal 谱湍流
- 选择目标风场 → 自动计算网格尺寸

### 调度风
```
时间节点:
t=0s    8 m/s  270°  TI=0.15
t=60s   8 m/s  270°  TI=0.15
t=120s  10 m/s 260°  TI=0.12
t=180s  6 m/s  280°  TI=0.18
```
- 各节点间风速/风向线性变化
- 叠加 Kaimal 谱湍流
- 节点数、时间、风速、风向、TI 均可自定义

---

## 项目结构

```
wfcrl-env-HRL/
├── app/                          # Dashboard 应用
│   ├── WFCRL_Dashboard.py        # 主界面 (Streamlit)
│   ├── controller_loader.py      # 自定义控制器加载器
│   └── wind_generator.py         # 湍流风生成器
│
├── custom_controllers/           # 用户上传的控制器
│   └── controller_template.py    # 控制器模板（含4个示例）
│
├── custom_winds/                 # 生成的湍流风文件
│
├── wfcrl/                        # 核心仿真库
│   ├── controllers/
│   │   └── reference.py          # 内置控制器 (含 FastFarmYawController)
│   ├── engine/
│   │   ├── fastfarm_continuous.py
│   │   ├── fastfarm_step.py
│   │   └── _ff_case.py
│   └── simulators/fastfarm/
│       ├── bin/                  # FAST.Farm 可执行文件
│       └── servo_dll/            # DISCON bridge DLL
│
└── __simul__/                    # 实验输出目录
    ├── experiments/              # 实验数据
    └── comparisons/              # 对比报告
```

---

## 依赖

| 包 | 用途 | 安装 |
|----|------|------|
| streamlit >= 1.28 | Web 界面 | `pip install streamlit` |
| wfcrl (本项目) | 仿真引擎 | `pip install -e .` |
| numpy | 数值计算 | 随 wfcrl 安装 |
| pandas | 数据处理 | 随 wfcrl 安装 |
| matplotlib | 图表 | 随 wfcrl 安装 |
| pyyaml | YAML 配置 | 随 wfcrl 安装 |
| openfast_toolbox | .bts 读写 | `pip install openfast_toolbox` |
| FAST.Farm | 高保真仿真 | 需单独下载安装 |

---

## FAST.Farm 配置

如需使用 FAST.Farm 后端:

1. 下载 FAST.Farm v5.0 (OpenMP 版本)
2. 将 `FAST.Farm_OpenMP.exe` 放入 `wfcrl/simulators/fastfarm/bin/`
3. 编译 DISCON bridge DLL:
   ```bash
   cd wfcrl/simulators/fastfarm/src
   python _compile.py     # 需要 gfortran
   ```
4. 确认 `wfcrl/simulators/fastfarm/servo_dll/DISCON_WT1.dll` 存在

---

## 常见问题

**Q: 启动后看不到实验列表？**
A: 实验保存在 `__simul__/experiments/`，运行过一次后才会显示。

**Q: 自定义控制器上传后显示"加载失败"？**
A: 确保你的控制器类继承自 `WindFarmController`，并实现了 `compute()` 方法。参考模板文件。

**Q: FAST.Farm 运行时提示中文路径错误？**
A: 项目路径包含中文字符时，FAST.Farm 的 Fortran 运行时无法识别。系统会自动将 `.bts` 文件复制到短路径。

**Q: 湍流风生成很慢？**
A: 长时长（>600s）的高分辨率湍流风生成需要几分钟。可以减小网格尺寸或增加时间步长。
