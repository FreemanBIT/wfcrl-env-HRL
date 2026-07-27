"""
WFCRL 动态仿真实验平台 — Web Dashboard
=======================================
面向风电场尾流控制算法研究的可视化实验管理界面。

启动:  streamlit run app/WFCRL_Dashboard.py
"""

import streamlit as st
import pandas as pd
import numpy as np
import json
import glob
import os
import sys
import subprocess
import time
import yaml
import re
import threading
from pathlib import Path
from datetime import datetime
from queue import Queue, Empty

# 自定义控制器加载器
sys.path.insert(0, str(Path(__file__).resolve().parent))
from controller_loader import (
    get_custom_controllers, get_all_controllers, get_controller_by_name,
    remove_controller, save_controller,
)

# matplotlib：用于尾流热力图
import matplotlib
if matplotlib.get_backend() == "agg":
    pass
elif "DISPLAY" not in os.environ:
    try:
        matplotlib.use("Agg")
    except Exception:
        pass
import matplotlib.pyplot as plt

# VTK visualization
_HAVE_PYVISTA = False
try:
    import pyvista as pv
    _HAVE_PYVISTA = True
except Exception:
    pass

def render_vtk_screenshot(vtk_dir, time_step=-1, slice_height=90.0):
    if not _HAVE_PYVISTA:
        return None
    from pathlib import Path
    import numpy as np
    vtk_path = Path(vtk_dir)
    if not vtk_path.exists():
        alt = vtk_path.parent / "vtk_ff"
        if alt.exists():
            vtk_path = alt
        else:
            return None
    low_files = sorted(vtk_path.glob("Case.Low.Dis.*.vtk"))
    if not low_files:
        return None
    fpath = str(low_files[time_step]) if time_step < len(low_files) else str(low_files[-1])
    try:
        mesh = pv.read(fpath)
        vel = mesh["Velocity"]
        mesh["VelocityMagnitude"] = np.linalg.norm(vel, axis=1)
        plotter = pv.Plotter(off_screen=True, window_size=[1000, 500])
        sl = mesh.slice("z", origin=(0, 0, slice_height))
        plotter.add_mesh(sl, scalars="VelocityMagnitude", cmap="RdYlGn", show_edges=False, lighting=False)
        plotter.show_bounds(grid="front", location="outer", all_edges=True, xtitle="x (m)", ytitle="y (m)")
        plotter.view_xy()
        img = plotter.screenshot(return_img=True)
        plotter.close()
        return img
    except Exception:
        return None

from matplotlib.patches import Circle, Patch

# matplotlib 中文字体
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "Noto Sans SC", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

# ── 页面配置 ──────────────────────────────────────────────────────────────
st.set_page_config(
    page_title="WFCRL 风电场控制实验平台",
    page_icon="🌬️",
    layout="wide",
    initial_sidebar_state="expanded",
)

# ── 项目路径 ──────────────────────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parent.parent
EXPERIMENTS_DIR = PROJECT_ROOT / "__simul__" / "experiments"
COMPARISONS_DIR = PROJECT_ROOT / "__simul__" / "comparisons"
YAML_EXAMPLES = PROJECT_ROOT / "examples" / "experiments"
FARMINPUTS_DIR = PROJECT_ROOT / "FarmInputs"
CUSTOM_DIR = PROJECT_ROOT / "custom_controllers"

# ── 初始化 session state ──────────────────────────────────────────────────
for key in [
    "page", "experiment_running", "experiment_output",
    "experiment_log", "experiment_progress", "experiment_status",
    "wizard_step", "config_backend", "config_controller",
    "config_wind_speed", "config_wind_dir", "config_turbulence",
    "config_steps", "config_dt", "config_layout",
    "config_mode", "config_yaw", "config_derating",
    "controller_a_params", "controller_b_params", "controller_c_params",
]:
    if key not in st.session_state:
        if key == "page":
            st.session_state[key] = "wizard"
        elif key == "experiment_running":
            st.session_state[key] = False
        elif key == "experiment_progress":
            st.session_state[key] = 0
        elif key.startswith("config_"):
            st.session_state[key] = None
        elif key.endswith("_params"):
            st.session_state[key] = {}
        else:
            st.session_state[key] = None


# ── 工具函数 ──────────────────────────────────────────────────────────────

def find_experiments(limit=50):
    """扫描所有实验目录，按时间降序。"""
    if not EXPERIMENTS_DIR.exists():
        return []
    dirs = sorted(EXPERIMENTS_DIR.glob("*"), key=os.path.getmtime, reverse=True)
    results = []
    for d in dirs[:limit]:
        if not d.is_dir():
            continue
        meta_file = d / "metadata.json"
        meta = {}
        if meta_file.exists():
            try:
                meta = json.loads(meta_file.read_text(encoding="utf-8"))
            except Exception:
                pass
        results.append({
            "dir": d,
            "name": meta.get("name", d.name),
            "backend": meta.get("backend", "?"),
            "steps": meta.get("steps", "?"),
            "mean_power_mw": meta.get("mean_farm_power_mw"),
            "wall_time_s": meta.get("duration_wall_s"),
            "n_turbines": meta.get("n_turbines", "?"),
            "final_time_s": meta.get("final_time_s"),
            "mtime": datetime.fromtimestamp(d.stat().st_mtime),
            "meta": meta,
        })
    return results


def load_timeseries(exp_dir):
    csv_file = Path(exp_dir) / "timeseries.csv"
    if not csv_file.exists():
        return None
    try:
        return pd.read_csv(csv_file)
    except Exception:
        return None


def load_metadata(exp_dir):
    meta_file = Path(exp_dir) / "metadata.json"
    if not meta_file.exists():
        return {}
    try:
        return json.loads(meta_file.read_text(encoding="utf-8"))
    except Exception:
        return {}


def get_builtin_layouts():
    """获取内置布局列表。"""
    try:
        from wfcrl.config.layout import LayoutRegistry
        reg = LayoutRegistry.from_builtin()
        result = {}
        for name in reg.list_names():
            layout = reg.get(name)
            result[name] = {
                "n_turbines": layout.num_turbines,
                "x": list(layout.xcoords),
                "y": list(layout.ycoords),
            }
        return result
    except Exception:
        return {}


def compute_wake_field(xs, ys, wind_speed=8.0, wind_dir=270.0, yaw_angles=None):
    """
    计算2D尾流场用于热力图显示。
    使用简化高斯尾流模型，方向适配270°西风（+x方向）。
    对大布局自动降采样以保持性能。
    """
    if yaw_angles is None:
        yaw_angles = np.zeros(len(xs))

    n_turbines = len(xs)
    D = 126.0  # 转子直径 (m)
    kw = 0.075  # 尾流衰减系数
    Ct = 0.8  # 推力系数

    # 自适应网格：大布局降采样, 小布局精细
    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    if n_turbines > 20:
        grid_size = 120
        margin = 300
    elif n_turbines > 6:
        grid_size = 160
        margin = 250
    else:
        grid_size = 200
        margin = 200

    # 风向适配（处理非270°风向：旋转坐标）
    # 简单实现：对270°靠+ x方向，对其他风向做近似
    x_min, x_max = min(xs) - margin, max(xs) + margin * 2
    y_min, y_max = min(ys) - margin, max(ys) + margin
    # 确保y范围非零（单行布局）
    if abs(y_max - y_min) < 10:
        y_pad = margin
        y_min -= y_pad
        y_max += y_pad

    xx = np.linspace(x_min, x_max, grid_size)
    yy = np.linspace(y_min, y_max, grid_size)
    X, Y = np.meshgrid(xx, yy)

    U = np.ones_like(X) * wind_speed

    # 对每个风机，计算尾流影响
    for i in range(n_turbines):
        x0, y0 = xs[i], ys[i]
        yaw = np.deg2rad(yaw_angles[i])

        # 偏航诱导尾流偏转 (简化模型)
        deflection = np.sin(yaw) * 2.0 * D

        # 顺风方向（x正方向）
        dx = X - x0
        dy = Y - (y0 + deflection)

        mask = dx > 0
        if not np.any(mask):
            continue

        # 尾流半径随距离线性增长
        sigma = kw * np.abs(dx) + D / 4
        # 速度亏损 (基于推力系数)
        deficit = (1 - np.sqrt(1 - Ct)) * (D / (D + 2 * kw * np.abs(dx))) ** 2
        deficit = np.clip(deficit, 0, 0.8)

        # 高斯分布权重
        weight = np.exp(-0.5 * (dy / sigma) ** 2)
        U[mask] -= deficit[mask] * weight[mask] * wind_speed

    U = np.maximum(U, wind_speed * 0.15)
    return X, Y, U


def generate_yaml_from_form(params):
    """根据表单参数生成 YAML 配置。"""
    config = {
        "name": params.get("name", "experiment"),
        "simulator": {
            "backend": params.get("backend", "mock"),
            "dt": params.get("dt", 2.0),
        },
        "farm": {
            "turbine_type": params.get("turbine_type", "nrel_5MW"),
            "xcoords": params.get("xcoords", [0.0, 504.0, 1008.0]),
            "ycoords": params.get("ycoords", [0.0, 0.0, 0.0]),
        },
        "wind": {
            "speed": params.get("wind_speed", 8.0),
            "direction": params.get("wind_direction", 270.0),
            "turbulence_intensity": params.get("turbulence_intensity", 0.06),
        },
        "controller": {"type": params.get("controller", "greedy")},
        "steps": params.get("steps", 40),
        "output": {"root": str(Path("../../__simul__/experiments").as_posix()), "plot": True},
    }

    # 偏航控制
    if params.get("controller") == "fixed_yaw":
        config["controller"]["yaw_misalignment_deg"] = params.get("yaw_misalignment_deg", [0.0]*6)
        config.setdefault("constraints", {})["yaw_bounds_deg"] = [-25.0, 25.0]
        config["constraints"]["yaw_rate_deg_s"] = 0.3
        config["constraints"]["yaw_deadband_deg"] = 0.5

    # 降额控制
    elif params.get("controller") == "fixed_derating":
        config["controller"]["derating_ratio"] = params.get("derating_ratio", [1.0]*6)
        if params.get("yaw_misalignment_deg"):
            config["controller"]["yaw_misalignment_deg"] = params["yaw_misalignment_deg"]
        config["power_reference"] = {
            "type": "power_curve",
            "wind_speed_ms": [0.0, 3.0, 5.0, 8.0, 11.4, 25.0, 30.0],
            "power_mw": [0.0, 0.0, 0.35, 1.75, 5.0, 5.0, 0.0],
            "smoothing_alpha": 0.2,
        }
        config.setdefault("constraints", {})["yaw_rate_deg_s"] = 0.3
        config["constraints"]["derating_ratio_rate_s"] = 0.02

    # FLORIS 特有
    if params.get("backend") == "floris":
        config["simulator"]["dt"] = max(params.get("dt", 30.0), 10.0)

    # FAST.Farm 特有
    if params.get("backend") == "fastfarm":
        config["simulator"]["dt"] = params.get("dt", 3.0)
        config["simulator"]["options"] = {"enable_vtk": params.get("enable_vtk", False)}

    return yaml.safe_dump(config, allow_unicode=True, sort_keys=False, default_flow_style=False)


def run_yaml_experiment(yaml_content, name):
    """运行YAML实验并返回输出目录。"""
    queue_dir = PROJECT_ROOT / "__simul__" / "_queue"
    queue_dir.mkdir(parents=True, exist_ok=True)
    yaml_path = queue_dir / f"{name}.yaml"
    yaml_path.write_text(yaml_content, encoding="utf-8")

    proc = subprocess.run(
        ["python", "-m", "wfcrl.experiments.cli", str(yaml_path)],
        cwd=str(PROJECT_ROOT), capture_output=True, text=True, timeout=600,
    )

    if yaml_path.exists():
        yaml_path.unlink()

    return proc, proc.stdout.strip() if proc.returncode == 0 else None


# ══════════════════════════════════════════════════════════════════════════
# 侧边栏导航
# ══════════════════════════════════════════════════════════════════════════

def render_sidebar():
    with st.sidebar:
        st.title("🌬️ WFCRL")
        st.caption("风电场控制实验平台")

        # 实验状态指示器
        if st.session_state.get("experiment_running"):
            st.warning("🔄 实验运行中...", icon="⚡")
        else:
            exps = find_experiments(limit=1)
            if exps:
                st.success(f"✅ 就绪 ({exps[0]['mtime'].strftime('%H:%M')} 最后实验)", icon="✅")

        st.divider()

        # 导航
        pages = {
            "🏭 风电场管理": "farm_config",
            "🌤️ 湍流风": "wind_config",
            "🎮 控制器管理": "controller_mgr",
            "▶️ 运行实验": "run",
            "📊 历史实验": "history",
            "🗺️ 尾流可视化": "wake_viz",
        }

        for label, page_id in pages.items():
            btn_type = "primary" if st.session_state.get("page") == page_id else "secondary"
            if st.button(label, key=f"nav_{page_id}", use_container_width=True, type=btn_type):
                st.session_state["page"] = page_id
                st.rerun()

        st.divider()
        st.caption(f"📁 项目: {PROJECT_ROOT.name}")
        st.caption(f"📂 实验目录: __simul__/experiments/")

        # 一键环境检查
        with st.expander("🔍 环境检查", expanded=False):
            checks = []
            # wfcrl
            try:
                import wfcrl
                checks.append(("wfcrl 包", "✅"))
            except Exception:
                checks.append(("wfcrl 包", "❌"))
            # FLORIS
            try:
                import floris
                checks.append(("FLORIS", "✅"))
            except Exception:
                checks.append(("FLORIS", "❌ 回退解析尾流"))
            # FAST.Farm exe
            ff_exe = PROJECT_ROOT / "wfcrl" / "simulators" / "fastfarm" / "bin" / "FAST.Farm_OpenMP.exe"
            checks.append(("FAST.Farm", "✅" if ff_exe.exists() else "⚠️ 未找到"))
            # DISCON DLL
            dll = PROJECT_ROOT / "wfcrl" / "simulators" / "fastfarm" / "servo_dll" / "DISCON_WT1.dll"
            checks.append(("DISCON DLL", "✅" if dll.exists() else "⚠️ 未找到"))
            # Streamlit
            import streamlit
            checks.append((f"Streamlit {streamlit.__version__}", "✅"))

            for label, status in checks:
                st.markdown(f"{status} {label}")


# ══════════════════════════════════════════════════════════════════════════
# 页面 1：新用户向导
# ══════════════════════════════════════════════════════════════════════════

def page_wizard():
    st.title("🚀 新用户向导")
    st.markdown("跟随以下步骤，完成你的第一个风电场尾流控制实验。")

    # 进度条
    step = st.session_state.get("wizard_step", 1)
    progress = (step - 1) / 5
    st.progress(progress, text=f"步骤 {step}/6")

    if step == 1:
        st.subheader("📖 什么是 WFCRL？")
        st.markdown("""
        **WFCRL (Wind Farm Control RL & Simulation)** 是一个面向风电场尾流控制算法研究的仿真实验平台。

        ### 核心能力
        | 能力 | 说明 |
        |------|------|
        | **三种仿真后端** | Mock（毫秒级）→ FLORIS（秒级）→ FAST.Farm（分钟级） |
        | **三套控制方案** | 方案A(稳态重优化) / B(动态MPC) / C(学习增强) |
        | **统一实验管理** | YAML 配置 → 自动运行 → CSV+JSON+PNG 输出 |
        | **对比分析** | 多实验指标自动对比 |

        ### 典型工作流
        ```
        ① 选后端 → ② 配参数 → ③ 运行 → ④ 看结果 → ⑤ 对比
        ```
        """)
        if st.button("开始第一步 →", type="primary"):
            st.session_state["wizard_step"] = 2
            st.rerun()

    elif step == 2:
        st.subheader("🎯 选择仿真后端")
        st.markdown("后端决定了仿真的速度和精度。")

        col1, col2, col3 = st.columns(3)
        with col1:
            st.markdown("#### 🟢 Mock")
            st.markdown("**速度**: 毫秒级\n\n**精度**: 趋势正确\n\n**用途**: 代码调试、冒烟测试")
            if st.button("选 Mock", key="wiz_mock", type="primary"):
                st.session_state["config_backend"] = "mock"
                st.session_state["wizard_step"] = 3
                st.rerun()

        with col2:
            st.markdown("#### 🟡 FLORIS")
            st.markdown("**速度**: 秒级\n\n**精度**: 工程可用\n\n**用途**: 尾流优化、参数扫描")
            if st.button("选 FLORIS", key="wiz_floris", type="primary"):
                st.session_state["config_backend"] = "floris"
                st.session_state["wizard_step"] = 3
                st.rerun()

        with col3:
            st.markdown("#### 🔴 FAST.Farm")
            st.markdown("**速度**: 分钟级\n\n**精度**: 高保真\n\n**用途**: 最终验证")
            has_ff = (PROJECT_ROOT / "wfcrl/simulators/fastfarm/bin/FAST.Farm_OpenMP.exe").exists()
            if not has_ff:
                st.warning("⚠️ 未检测到 FAST.Farm 可执行文件")
            if st.button("选 FAST.Farm", key="wiz_fastfarm", type="primary", disabled=not has_ff):
                st.session_state["config_backend"] = "fastfarm"
                st.session_state["wizard_step"] = 3
                st.rerun()

    elif step == 3:
        st.subheader("🌤️ 配置风况")
        col1, col2 = st.columns(2)
        with col1:
            speed = st.slider("风速 (m/s)", 4.0, 16.0, 8.0, 0.5)
            direction = st.slider("风向 (°)", 180, 360, 270, 5)
        with col2:
            turbulence = st.slider("湍流强度", 0.02, 0.20, 0.06, 0.01)
            layout_name = st.selectbox("风场布局", ["3T", "6T", "HornsRev1", "DafengH1"],
                                       index=1)

        st.session_state["config_wind_speed"] = speed
        st.session_state["config_wind_dir"] = direction
        st.session_state["config_turbulence"] = turbulence
        st.session_state["config_layout"] = layout_name

        col1, col2 = st.columns([1, 5])
        with col1:
            if st.button("← 上一步"):
                st.session_state["wizard_step"] = 2
                st.rerun()
        with col2:
            if st.button("下一步 →", type="primary"):
                st.session_state["wizard_step"] = 4
                st.rerun()

    elif step == 4:
        st.subheader("🎮 选择控制器")
        backend = st.session_state.get("config_backend", "mock")

        col1, col2, col3 = st.columns(3)
        with col1:
            st.markdown("#### 📌 贪婪基线")
            st.markdown("无偏航、无降额，作为对比基准")
            if st.button("选贪婪", key="wiz_greedy", type="primary"):
                st.session_state["config_controller"] = "greedy"
                st.session_state["wizard_step"] = 5
                st.rerun()

        with col2:
            st.markdown("#### 🌀 偏航控制")
            st.markdown("通过主动偏航偏转尾流")
            if st.button("选偏航", key="wiz_yaw", type="primary"):
                st.session_state["config_controller"] = "fixed_yaw"
                st.session_state["config_mode"] = 1
                st.session_state["wizard_step"] = 5
                st.rerun()

        with col3:
            st.markdown("#### ⬇️ 降额控制")
            st.markdown("降低上游风机功率减少尾流")
            if st.button("选降额", key="wiz_derating", type="primary"):
                st.session_state["config_controller"] = "fixed_derating"
                st.session_state["config_mode"] = 2
                st.session_state["wizard_step"] = 5
                st.rerun()

        if backend in ("mock", "floris"):
            col1, col2 = st.columns(2)
            with col1:
                st.markdown("#### 🔬 方案A (稳态重优化)")
                st.markdown("FLORIS标定 + 稳态优化")
                if st.button("选方案A", key="wiz_a", type="secondary"):
                    st.session_state["config_controller"] = "a"
                    st.session_state["wizard_step"] = 5
                    st.rerun()
            with col2:
                st.markdown("#### 🧠 方案B (动态MPC)")
                st.markdown("FLORIDyn + EnKF + MPC")
                if st.button("选方案B", key="wiz_b", type="secondary"):
                    st.session_state["config_controller"] = "b"
                    st.session_state["wizard_step"] = 5
                    st.rerun()

    elif step == 5:
        st.subheader("⚙️ 运行参数")
        backend = st.session_state.get("config_backend", "mock")

        col1, col2 = st.columns(2)
        with col1:
            if backend == "mock":
                default_dt, default_steps = 2.0, 40
            elif backend == "floris":
                default_dt, default_steps = 30.0, 10
            else:
                default_dt, default_steps = 3.0, 20

            dt = st.number_input("时间步长 (s)", min_value=0.5, max_value=60.0,
                                 value=default_dt, step=0.5)
            steps = st.number_input("仿真步数", min_value=3, max_value=500,
                                    value=default_steps, step=1)

        with col2:
            duration = dt * steps
            st.metric("仿真总时长", f"{duration:.0f} 秒")
            st.metric("仿真后端", {"mock": "Mock 🟢", "floris": "FLORIS 🟡",
                                   "fastfarm": "FAST.Farm 🔴"}.get(backend, backend))

        st.session_state["config_dt"] = dt
        st.session_state["config_steps"] = steps

        col1, col2 = st.columns([1, 5])
        with col1:
            if st.button("← 上一步"):
                st.session_state["wizard_step"] = 4
                st.rerun()
        with col2:
            if st.button("🚀 运行实验！", type="primary"):
                st.session_state["wizard_step"] = 6
                st.rerun()

    elif step == 6:
        st.subheader("🚀 运行中...")
        backend = st.session_state.get("config_backend", "mock")
        controller = st.session_state.get("config_controller", "greedy")

        # 构建参数
        layouts = get_builtin_layouts()
        layout_name = st.session_state.get("config_layout", "6T")
        layout_info = layouts.get(layout_name, {"n_turbines": 6, "x": [0,504,1008,0,504,1008],
                                                 "y": [-252,-252,-252,252,252,252]})

        params = {
            "name": f"wizard-{controller}-{datetime.now().strftime('%H%M%S')}",
            "backend": backend,
            "dt": st.session_state.get("config_dt", 2.0),
            "steps": st.session_state.get("config_steps", 40),
            "wind_speed": st.session_state.get("config_wind_speed", 8.0),
            "wind_direction": st.session_state.get("config_wind_dir", 270.0),
            "turbulence_intensity": st.session_state.get("config_turbulence", 0.06),
            "controller": controller,
            "xcoords": layout_info.get("x", [0, 504, 1008]),
            "ycoords": layout_info.get("y", [0, 0, 0]),
            "turbine_type": "nrel_5MW",
        }

        if controller == "fixed_yaw":
            n = layout_info.get("n_turbines", 6)
            yaw = [10.0] + [0.0] * (n - 1)
            params["yaw_misalignment_deg"] = yaw
        elif controller == "fixed_derating":
            n = layout_info.get("n_turbines", 6)
            params["derating_ratio"] = [0.8] + [1.0] * (n - 1)
            params["yaw_misalignment_deg"] = [5.0] + [0.0] * (n - 1)

        yaml_content = generate_yaml_from_form(params)

        with st.status("🏗️ 正在运行实验...", expanded=True) as status:
            st.code(yaml_content, language="yaml")
            st.markdown("---")
            progress_bar = st.progress(0, text="初始化...")

            def update_progress(step, total):
                progress_bar.progress(min(step / total, 1.0),
                                      text=f"仿真步 {step}/{total}")

            try:
                # 写入YAML
                queue_dir = PROJECT_ROOT / "__simul__" / "_queue"
                queue_dir.mkdir(parents=True, exist_ok=True)
                yaml_path = queue_dir / f"{params['name']}.yaml"
                yaml_path.write_text(yaml_content, encoding="utf-8")

                start_time = time.time()
                proc = subprocess.run(
                    ["python", "-m", "wfcrl.experiments.cli", str(yaml_path)],
                    cwd=str(PROJECT_ROOT), capture_output=True, text=True,
                    timeout=600,
                )
                elapsed = time.time() - start_time

                if yaml_path.exists():
                    yaml_path.unlink()

                if proc.returncode == 0:
                    out_dir = proc.stdout.strip()
                    progress_bar.progress(1.0, text="✅ 完成！")
                    status.update(label="✅ 实验完成！", state="complete")

                    st.success(f"✅ 实验成功！耗时 {elapsed:.1f}s")
                    st.markdown(f"输出目录: `{out_dir}`")

                    # 显示结果预览
                    if out_dir and Path(out_dir).exists():
                        meta = load_metadata(out_dir)
                        if meta:
                            col1, col2, col3, col4 = st.columns(4)
                            col1.metric("平均功率", f"{meta.get('mean_farm_power_mw', 0):.3f} MW")
                            col2.metric("仿真时长", f"{meta.get('final_time_s', 0):.0f}s")
                            col3.metric("步数", meta.get("steps", 0))
                            col4.metric("耗时", f"{meta.get('duration_wall_s', 0):.1f}s")

                        df = load_timeseries(out_dir)
                        if df is not None and "farm_power_MW" in df.columns:
                            st.subheader("📈 场功率曲线")
                            st.line_chart(df.set_index("time_s")["farm_power_MW"])

                    st.session_state["experiment_output"] = out_dir

                else:
                    status.update(label="❌ 实验失败", state="error")
                    st.error(f"返回码: {proc.returncode}")
                    with st.expander("错误日志"):
                        st.code(proc.stderr[:2000] if proc.stderr else "无输出")

            except subprocess.TimeoutExpired:
                st.error("⏰ 实验超时 (>10分钟)")
            except Exception as e:
                st.error(f"❌ 错误: {e}")

        if st.button("🏠 回到首页", type="primary"):
            st.session_state["wizard_step"] = 1
            st.session_state["page"] = "results"
            st.rerun()


# ══════════════════════════════════════════════════════════════════════════
# 页面 2：快速实验
# ══════════════════════════════════════════════════════════════════════════

def page_quick_run():
    st.title("⚡ 快速实验")
    st.markdown("选择后端 → 配置参数 → 运行实验")

    # ── 后端选择 ──────────────────────────────────────────────────────
    st.subheader("1️⃣ 仿真后端")
    backend = st.radio(
        "选择仿真引擎",
        ["mock", "floris", "fastfarm"],
        format_func=lambda x: {
            "mock": "🟢 Mock（毫秒级，无安装依赖）",
            "floris": "🟡 FLORIS（秒级，需安装 floris）",
            "fastfarm": "🔴 FAST.Farm（分钟级，需编译好的二进制）",
        }[x],
        horizontal=True,
        index=0,
    )

    # ── 控制器选择 ────────────────────────────────────────────────────
    st.subheader("2️⃣ 控制器")

    # 内置控制器
    builtin_ctrl = {
        "greedy": "贪婪基线",
        "fixed_yaw": "固定偏航",
        "fixed_derating": "固定降额",
    }
    # 自定义控制器
    custom_ctrls = get_custom_controllers()
    custom_ctrl_names = {c["name"]: c["description"] for c in custom_ctrls if not c["error"]}

    controller_options = list(builtin_ctrl.keys()) + list(custom_ctrl_names.keys())
    ctrl_format = {**builtin_ctrl, **custom_ctrl_names}

    controller = st.selectbox(
        "控制方案",
        controller_options,
        format_func=lambda x: f"🧩 {x}" if x in custom_ctrl_names else f"📦 {ctrl_format.get(x, x)}",
        index=0,
    )

    is_custom_ctrl = controller in custom_ctrl_names

    c1, c2 = st.columns([1, 3])
    with c1:
        st.caption(f"{'🧩 自定义' if is_custom_ctrl else '📦 内置'}控制器")
    with c2:
        n_turbines = 6
        if controller == "fixed_yaw":
            st.markdown("**偏航角配置**")
            yaw_cols = st.columns(n_turbines)
            yaw_vals = []
            for i in range(n_turbines):
                val = yaw_cols[i].number_input(f"T{i+1} (°)", -25.0, 25.0,
                                               value=10.0 if i == 0 else 0.0,
                                               step=1.0, key=f"qyaw_{i}")
                yaw_vals.append(val)
            st.session_state["config_yaw"] = yaw_vals

        elif controller == "fixed_derating":
            st.markdown("**降额比例配置**")
            der_cols = st.columns(n_turbines)
            der_vals = []
            for i in range(n_turbines):
                val = der_cols[i].number_input(f"T{i+1}", 0.3, 1.0,
                                               value=0.8 if i == 0 else 1.0,
                                               step=0.05, key=f"qder_{i}")
                der_vals.append(val)
            st.session_state["config_derating"] = der_vals

        elif is_custom_ctrl:
            # 显示自定义控制器的简要信息
            for c in custom_ctrls:
                if c["name"] == controller:
                    st.markdown(f"**{c['description']}**")
                    with st.expander("查看源代码", expanded=False):
                        st.code(c["source"], language="python")
                    break

    # ── 风况配置 ──────────────────────────────────────────────────────
    st.subheader("3️⃣ 风况与布局")
    col1, col2, col3 = st.columns(3)
    with col1:
        wind_speed = st.slider("风速 (m/s)", 4.0, 16.0, 8.0, 0.5)
    with col2:
        wind_dir = st.slider("风向 (°)", 180, 360, 270, 5)
    with col3:
        turbulence = st.slider("湍流强度", 0.02, 0.20, 0.06, 0.01)

    layouts = get_builtin_layouts()
    layout_names = list(layouts.keys()) if isinstance(layouts, dict) else ["6T"]
    layout_choice = st.selectbox("风场布局", layout_names,
                                 index=layout_names.index("6T") if "6T" in layout_names else 0,
                                 format_func=lambda n: f"{n} ({layouts[n]['n_turbines']}台)" if isinstance(layouts, dict) and n in layouts else n)

    # ── 运行参数 ──────────────────────────────────────────────────────
    st.subheader("4️⃣ 运行参数")
    col1, col2 = st.columns(2)
    with col1:
        dt_default = {"mock": 2.0, "floris": 30.0, "fastfarm": 3.0}
        steps_default = {"mock": 40, "floris": 10, "fastfarm": 20}
        dt = st.number_input("时间步长 (s)", 0.5, 60.0, dt_default.get(backend, 2.0), 0.5)
    with col2:
        steps = st.number_input("步数", 3, 500, steps_default.get(backend, 40), 1)

    # ── 运行 ──────────────────────────────────────────────────────────
    st.subheader("5️⃣ 运行")

    run_disabled = st.session_state.get("experiment_running", False) or (
        is_custom_ctrl and backend not in ("mock", "floris")
    )
    if is_custom_ctrl and backend == "fastfarm":
        st.warning("⚠️ 自定义控制器当前仅支持 Mock 和 FLORIS 后端。FAST.Farm 请使用内置控制器。")

    if st.button("🚀 运行实验", type="primary", use_container_width=True,
                 disabled=run_disabled, key="qr_run_btn"):
        st.session_state["experiment_running"] = True

        layout_info = layouts.get(layout_choice, {"x": [0,504,1008], "y": [0,0,0],
                                                   "n_turbines": 6})
        n_t = layout_info.get("n_turbines", 6)

        with st.status("🏗️ 运行中...", expanded=True) as status:
            progress_bar = st.progress(0, text="请稍候...")

            try:
                start_t = time.time()

                if is_custom_ctrl:
                    # ── 自定义控制器：用 ControllerRunner 直接运行 ──
                    from wfcrl.engine import DeterministicMockSimulator, FlorisInterface
                    from wfcrl.config import SimulationConfig, FastFarmConfig, FlorisConfig
                    from wfcrl.controllers import ControllerRunner

                    if backend == "mock":
                        cfg = SimulationConfig(
                            case_name=f"custom-{controller}",
                            num_turbines=n_t,
                            xcoords=list(layout_info["x"]),
                            ycoords=list(layout_info["y"]),
                            dt=dt, max_iter=steps,
                            wind=WindConfig(speed=wind_speed, direction=wind_dir,
                                            turbulence_intensity=turbulence),
                        )
                        sim = DeterministicMockSimulator(cfg)
                    elif backend == "floris":
                        cfg = FlorisConfig(
                            case_name=f"custom-{controller}",
                            num_turbines=n_t,
                            xcoords=list(layout_info["x"]),
                            ycoords=list(layout_info["y"]),
                            dt=max(dt, 10.0), max_iter=steps,
                            wind=WindConfig(speed=wind_speed, direction=wind_dir),
                        )
                        sim = FlorisInterface(cfg)
                    else:
                        raise ValueError(f"不支持的后端: {backend}")

                    # 实例化自定义控制器
                    ctrl_class = None
                    for c in custom_ctrls:
                        if c["name"] == controller:
                            ctrl_class = c["class"]
                            break
                    if ctrl_class is None:
                        raise ValueError(f"未找到控制器: {controller}")
                    ctrl_instance = ctrl_class()

                    runner = ControllerRunner(sim, ctrl_instance)
                    result = runner.run(
                        steps, wind=WindConfig(speed=wind_speed, direction=wind_dir,
                                                turbulence_intensity=turbulence),
                        setup=True,
                    )
                    elapsed = time.time() - start_t
                    sim.close()

                    # 保存结果到实验目录
                    exp_name = f"custom-{controller}-{datetime.now().strftime('%H%M%S')}"
                    out_dir = EXPERIMENTS_DIR / exp_name
                    out_dir.mkdir(parents=True, exist_ok=True)
                    result.output.to_csv(out_dir / "timeseries.csv")
                    meta = {
                        "schema_version": "1.0", "name": exp_name,
                        "backend": "deterministic_mock" if backend == "mock" else "floris",
                        "steps": steps, "n_turbines": n_t,
                        "final_time_s": steps * dt,
                        "mean_farm_power_mw": float(np.mean(result.output.farm_power_mw)),
                        "constraints_enabled": False,
                        "duration_wall_s": elapsed,
                        "controller": controller,
                    }
                    json.dump(meta, open(out_dir / "metadata.json", "w", encoding="utf-8"),
                              indent=2, ensure_ascii=False)

                else:
                    # ── 内置控制器：用 YAML 方式运行 ──
                    params = {
                        "name": f"run-{controller}-{datetime.now().strftime('%H%M%S')}",
                        "backend": backend, "dt": dt, "steps": steps,
                        "wind_speed": wind_speed, "wind_direction": wind_dir_val,
                        "turbulence_intensity": turbulence,
                        "controller": controller,
                        "xcoords": list(layout_info["x"]),
                        "ycoords": list(layout_info["y"]),
                        "turbine_type": "nrel_5MW",
                        "enable_vtk": enable_vtk,
                    }
                    if controller == "fixed_yaw" and st.session_state.get("config_yaw"):
                        params["yaw_misalignment_deg"] = st.session_state["config_yaw"]
                    if controller == "fixed_derating":
                        if st.session_state.get("config_derating"):
                            params["derating_ratio"] = st.session_state["config_derating"]
                        if st.session_state.get("config_yaw"):
                            params["yaw_misalignment_deg"] = st.session_state["config_yaw"]

                    yaml_content = generate_yaml_from_form(params)
                    proc, out_dir = run_yaml_experiment(yaml_content, params["name"])
                    elapsed = time.time() - start_t

                    if proc.returncode != 0 or not out_dir:
                        raise RuntimeError(f"实验失败 (rc={proc.returncode})")

                progress_bar.progress(1.0, text="✅ 完成")
                status.update(label="✅ 实验完成", state="complete")
                st.success(f"✅ 完成！耗时 {elapsed:.1f}s")

                meta = load_metadata(out_dir)
                if meta:
                    c1, c2, c3, c4 = st.columns(4)
                    c1.metric("平均功率", f"{meta.get('mean_farm_power_mw', 0):.3f} MW")
                    c2.metric("仿真时间", f"{meta.get('final_time_s',0):.0f}s")
                    c3.metric("步数", meta.get("steps", 0))
                    c4.metric("耗时", f"{meta.get('duration_wall_s',0):.1f}s")

                df = load_timeseries(out_dir)
                if df is not None and "farm_power_MW" in df.columns:
                    st.line_chart(df.set_index("time_s")["farm_power_MW"])

                st.session_state["experiment_output"] = str(out_dir)

            except Exception as e:
                progress_bar.progress(1.0, text="❌ 失败")
                status.update(label="❌ 失败", state="error")
                st.error(f"{type(e).__name__}: {e}")
                import traceback
                st.code(traceback.format_exc())

        st.session_state["experiment_running"] = False


# ══════════════════════════════════════════════════════════════════════════
# ══════════════════════════════════════════════════════════════════════════
# 页面：湍流风配置
# ══════════════════════════════════════════════════════════════════════════

def page_wind_config():
    st.title("🌤️ 湍流风配置")
    st.markdown("""
    生成 FAST.Farm 兼容的湍流风场 (Kaimal 谱，.bts 格式)。
    生成的文件可在「▶️ 运行实验」中选择使用。
    """)

    tab1, tab2, tab3 = st.tabs(["🌬️ 生成湍流风", "⏱️ 调度风", "📋 已有风文件"])

    # ── Tab 1: 生成湍流风 ──────────────────────────────────────────
    with tab1:
        st.subheader("参数配置")

        col1, col2 = st.columns(2)
        with col1:
            ws = st.slider("平均风速 (m/s)", 4.0, 25.0, 8.0, 0.5, key="tw_ws")
            ti = st.slider("湍流强度", 0.05, 0.30, 0.15, 0.01, key="tw_ti",
                          help="IEC 湍流等级: A=0.16, B=0.14, C=0.12")
            wdir = st.number_input("风向 (°)", 0, 360, 270, 5, key="tw_wdir")

        with col2:
            duration = st.number_input("风场时长 (s)", 30, 3600, 600, 30, key="tw_dur",
                                       help="越长越精确，但文件越大。600s = ~6MB")
            dt = st.select_slider("时间步长 (s)", options=[0.05, 0.1, 0.2], value=0.1,
                                  key="tw_dt", help="FAST.Farm 通常用 0.05~0.1s")
            seed = st.number_input("随机种子", 0, 99999, 42, key="tw_seed",
                                   help="同一种子生成相同风场（可复现）")

        # 选择风场布局（自动计算网格尺寸）
        col_a, col_b = st.columns([1, 2])
        with col_a:
            st.markdown("**目标风场**")
            farm_names = _list_farm_names()
            sel_farm = st.selectbox("", farm_names if farm_names else ["无"],
                                    key="tw_farm", label_visibility="collapsed")
            lx, ly, n_t = _get_farm_coords(sel_farm) if farm_names else (None, None, 0)
        with col_b:
            # 自动计算网格尺寸
            if lx and ly:
                from wind_generator import calc_grid_from_layout
                gw, gh, hub_h = calc_grid_from_layout(list(lx), list(ly))
                st.info(f"风场: {sel_farm} ({n_t}台) | "
                        f"网格: {gw:.0f}m × {gh:.0f}m | 轮毂高: {hub_h:.0f}m")
            else:
                gw, gh, hub_h = 680.0, 340.0, 90.0
                st.info(f"默认网格: {gw:.0f}m × {gh:.0f}m (未选风场)")

        # 高级选项
        with st.expander("网格参数（通常不用改）"):
            col1, col2, col3 = st.columns(3)
            with col1:
                hub_h = st.number_input("轮毂高度 (m)", 50, 200, int(hub_h), 5, key="tw_hub")
            with col2:
                gh = st.number_input("网格高度 (m)", 200, 1000, int(gh), 10, key="tw_gh")
            with col3:
                gw = st.number_input("网格宽度 (m)", 200, 2000, int(gw), 10, key="tw_gw")

        name = st.text_input("文件名", f"U{ws:.0f}_TI{ti*100:.0f}", key="tw_name",
                            help="自动保存到 custom_winds/ 目录")

        # 生成按钮
        gen_disabled = st.session_state.get("wind_generating", False)
        if st.button("🚀 生成湍流风场", type="primary", use_container_width=True,
                     disabled=gen_disabled, key="tw_gen"):
            st.session_state["wind_generating"] = True
            try:
                from wind_generator import generate_turbulent_wind
                with st.spinner(f"正在生成 {duration:.0f}s 湍流风场..."):
                    kw = dict(wind_speed=ws, ti=ti, direction=wdir,
                              duration=duration, dt=dt, seed=seed, name=name)
                    if lx and ly:
                        kw["layout_x"] = list(lx)
                        kw["layout_y"] = list(ly)
                    else:
                        kw["grid_height"] = gh
                        kw["grid_width"] = gw
                        kw["hub_height"] = hub_h
                    bts_path = generate_turbulent_wind(**kw)
                st.success(f"✅ 生成完成!")
                st.metric("文件大小", f"{Path(bts_path).stat().st_size / 1024 / 1024:.1f} MB")

                # 验证
                from openfast_toolbox.io.turbsim_file import TurbSimFile
                ts = TurbSimFile(bts_path)
                u = ts['u']
                u_mean = u[0].mean()
                u_std = u[0].std()
                ti_actual = u_std / u_mean if u_mean > 0 else 0
                st.info(f"轮毂风速: {u_mean:.2f} m/s, 湍流强度: {ti_actual:.3f}")

            except Exception as e:
                st.error(f"❌ 生成失败: {e}")
                import traceback
                st.code(traceback.format_exc())
            finally:
                st.session_state["wind_generating"] = False

    # ── Tab 2: 调度风 ──────────────────────────────────────────
    with tab2:
        st.subheader("⏱️ 调度风 — 随时间变化的风速/风向")
        st.markdown("定义多个时间节点，风速和风向在各节点间线性变化。")

        col1, col2 = st.columns([2, 1])
        with col1:
            st.markdown("**风调度表** (每行一个时间节点)")
            # 默认 4 段
            default_schedule = pd.DataFrame({
                "时间 (s)": [0, 60, 120, 180],
                "风速 (m/s)": [8.0, 8.0, 10.0, 6.0],
                "风向 (°)": [270, 270, 260, 280],
                "TI": [0.15, 0.15, 0.12, 0.18],
            })
            sched_df = st.data_editor(default_schedule, use_container_width=True,
                                       hide_index=True, num_rows="dynamic",
                                       key="sched_editor")
        with col2:
            sched_dt = st.select_slider("时间步长 (s)", [0.05, 0.1, 0.2], 0.1, key="sched_dt")
            sched_seed = st.number_input("随机种子", 0, 99999, 42, key="sched_seed")
            sched_name = st.text_input("文件名", "schedule_wind", key="sched_name")

        # 选择风场（自动计算网格）
        farm_names_sched = _list_farm_names()
        sel_farm_sched = st.selectbox("目标风场（自动计算网格）", farm_names_sched if farm_names_sched else ["无"],
                                       key="sched_farm")
        lx_s, ly_s, nt_s = _get_farm_coords(sel_farm_sched) if farm_names_sched else (None, None, 0)

        # 预览曲线
        if len(sched_df) > 1:
            st.subheader("📈 风调度预览")
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
            ax1.plot(sched_df["时间 (s)"], sched_df["风速 (m/s)"], "o-", color="#1f77b4")
            ax1.set_ylabel("风速 (m/s)")
            ax1.grid(alpha=0.2)
            ax2.plot(sched_df["时间 (s)"], sched_df["风向 (°)"], "o-", color="#d35400")
            ax2.set_ylabel("风向 (°)")
            ax2.set_xlabel("时间 (s)")
            ax2.grid(alpha=0.2)
            fig.tight_layout()
            st.pyplot(fig)
            plt.close(fig)

        if st.button("🚀 生成调度风", type="primary", use_container_width=True,
                     key="sched_gen"):
            st.session_state["wind_generating"] = True
            try:
                from wind_generator import generate_turbulent_wind_schedule
                schedule = [
                    (row["时间 (s)"], row["风速 (m/s)"], row["风向 (°)"], row["TI"])
                    for _, row in sched_df.iterrows()
                ]
                kw = dict(schedule=schedule, dt=sched_dt, seed=sched_seed,
                          name=sched_name)
                if lx_s and ly_s:
                    kw["layout_x"] = list(lx_s)
                    kw["layout_y"] = list(ly_s)
                else:
                    kw["grid_width"] = 680
                    kw["grid_height"] = 340
                    kw["hub_height"] = 90

                total_time = max(row["时间 (s)"] for _, row in sched_df.iterrows())
                with st.spinner(f"生成调度风 {total_time:.0f}s..."):
                    bts_path = generate_turbulent_wind_schedule(**kw)
                st.success(f"✅ 生成完成!")

                from openfast_toolbox.io.turbsim_file import TurbSimFile
                ts = TurbSimFile(bts_path)
                u = ts['u']
                n_t = u.shape[1]
                n_y = u.shape[2]
                n_z = u.shape[3]
                st.metric("文件大小", f"{Path(bts_path).stat().st_size / 1024 / 1024:.1f} MB")
                st.info(f"时长: {n_t * ts['dt']:.0f}s | 网格: {n_y}x{n_z}")

            except Exception as e:
                st.error(f"❌ 生成失败: {e}")
                import traceback
                st.code(traceback.format_exc())
            finally:
                st.session_state["wind_generating"] = False

    # ── Tab 3: 已有风文件 ──────────────────────────────────────────
    with tab3:
        st.subheader("📋 已有湍流风文件")
        wind_dir = Path(PROJECT_ROOT / "custom_winds")
        wind_dir.mkdir(parents=True, exist_ok=True)

        bts_files = []
        for d in [wind_dir, Path(PROJECT_ROOT / "wfcrl/simulators/fastfarm/inputs/template/FarmInputs")]:
            if d.exists():
                for f in sorted(d.glob("*.bts")):
                    size_mb = f.stat().st_size / 1024 / 1024
                    bts_files.append({
                        "name": f.name,
                        "path": str(f),
                        "size": f"{size_mb:.1f} MB",
                        "location": "custom" if "custom_winds" in str(f) else "template",
                    })

        if bts_files:
            df = pd.DataFrame(bts_files)
            st.dataframe(df, use_container_width=True, hide_index=True)

            sel_file = st.selectbox("选择查看详情", [f["name"] for f in bts_files], key="tw_sel")
            for f in bts_files:
                if f["name"] == sel_file:
                    try:
                        from openfast_toolbox.io.turbsim_file import TurbSimFile
                        ts = TurbSimFile(f["path"])
                        u = ts['u']
                        st.json({
                            "风速均值": f"{u[0].mean():.2f} m/s",
                            "风速标准差": f"{u[0].std():.2f} m/s",
                            "湍流强度": f"{u[0].std() / u[0].mean():.3f}" if u[0].mean() > 0 else "N/A",
                            "网格": f"{ts['u'].shape[3]}x{ts['u'].shape[2]}",
                            "时长": f"{ts['u'].shape[1] * ts['dt']:.0f}s",
                            "步长": f"{ts['dt']}s",
                            "文件": f["path"],
                        })
                    except Exception as e:
                        st.error(f"读取失败: {e}")
                    break
        else:
            st.info("还没有湍流风文件。去「生成湍流风」Tab 生成一个。")

        st.caption("💡 模板目录中已有: 90m_08mps.bts, inflow_6/8/10/11.4ms.bts")


# ══════════════════════════════════════════════════════════════════════════
# 页面：控制器管理
# ══════════════════════════════════════════════════════════════════════════

def page_controller_mgr():
    st.title("🎮 控制器管理")
    st.markdown("""
    上传和管理自定义控制器。控制器是继承 `WindFarmController` 的 Python 类，
    实现 `compute(observation)` 方法返回控制指令即可。

    **输入** → `ControllerObservation`: time_s, power_mw, wind_speed, yaw, pitch
    **输出** ← `ControlIntent`: yaw_misalignment_deg, power_setpoint_mw, derating_ratio, pitch_deg
    """)

    tab1, tab2, tab3 = st.tabs(["📋 可用控制器", "📤 上传控制器", "✏️ 创建/编辑"])

    # ── Tab 1: 可用控制器列表 ──────────────────────────────────────
    with tab1:
        controllers = get_all_controllers()
        if not controllers:
            st.info("暂无控制器。去「上传控制器」Tab 上传一个。")
        else:
            st.caption(f"共 {len(controllers)} 个控制器")

            for ctrl in controllers:
                is_custom = ctrl["type"] == "custom"
                icon = "📦" if not is_custom else ("⚠️" if ctrl["error"] else "🧩")
                with st.expander(
                    f"{icon} {ctrl['name']}  ({ctrl['type']})",
                    expanded=is_custom and not ctrl["error"],
                ):
                    st.markdown(f"**描述**: {ctrl['description']}")
                    st.caption(f"来源: {'内置' if not is_custom else ctrl['filename']}")

                    if ctrl["error"]:
                        st.error(f"加载错误:")
                        st.code(ctrl["error"])

                    if is_custom and ctrl["source"]:
                        with st.expander("查看源代码"):
                            st.code(ctrl["source"], language="python")

                    # 删除按钮（仅自定义控制器）
                    if is_custom and ctrl["filename"]:
                        col1, col2 = st.columns([1, 5])
                        with col1:
                            if st.button(f"🗑️ 删除 {ctrl['name']}",
                                         key=f"del_{ctrl['name']}",
                                         type="secondary", use_container_width=True):
                                if remove_controller(ctrl["filename"]):
                                    st.success(f"已删除 {ctrl['filename']}")
                                    st.rerun()
                                else:
                                    st.error("删除失败")

    # ── Tab 2: 上传控制器 ──────────────────────────────────────────
    with tab2:
        st.subheader("📤 上传控制器文件 (.py)")
        st.markdown("上传你编写好的控制器 Python 文件。")

        uploaded_file = st.file_uploader(
            "选择 .py 文件",
            type=["py"],
            accept_multiple_files=False,
            key="ctrl_upload",
        )

        if uploaded_file is not None:
            content = uploaded_file.read().decode("utf-8")
            filename = uploaded_file.name

            # 预览
            with st.expander("预览文件内容", expanded=True):
                st.code(content, language="python")

            if st.button("📥 上传并注册", type="primary", use_container_width=True):
                err = save_controller(filename, content)
                if err:
                    st.error(f"保存失败: {err}")
                else:
                    st.success(f"✅ {filename} 已上传！")
                    # 验证是否能加载
                    sys.path.insert(0, str(CUSTOM_DIR))
                    try:
                        import importlib
                        spec = importlib.util.spec_from_file_location(
                            filename[:-3], str(CUSTOM_DIR / filename)
                        )
                        if spec and spec.loader:
                            mod = importlib.util.module_from_spec(spec)
                            spec.loader.exec_module(mod)
                            from wfcrl.controllers import WindFarmController
                            found = False
                            for name, obj in inspect.getmembers(mod, inspect.isclass):
                                if name != "WindFarmController" and issubclass(obj, WindFarmController):
                                    st.info(f"✅ 识别到控制器类: {name}")
                                    found = True
                            if not found:
                                st.warning("⚠️ 文件中未找到继承 WindFarmController 的类")
                    except Exception as e:
                        st.error(f"❌ 加载失败: {e}")
                    st.rerun()

        # 下载模板
        st.divider()
        st.subheader("📄 下载模板")
        template_path = CUSTOM_DIR / "controller_template.py"
        if template_path.exists():
            with open(template_path, "r", encoding="utf-8") as f:
                template_content = f.read()
            st.download_button(
                "📥 下载控制器模板",
                template_content,
                file_name="my_controller.py",
                mime="text/x-python",
                use_container_width=True,
            )
        st.markdown("""
        模板中包含 4 个示例控制器：
        - `GreedyController` — 贪婪基线
        - `FixedYawController` — 固定偏航
        - `WakeAwareYawController` — 尾流感知偏航
        - `DeratingController` — 降额控制
        """)

    # ── Tab 3: 在线创建/编辑 ──────────────────────────────────────
    with tab3:
        st.subheader("✏️ 在线创建控制器")
        st.markdown("直接在浏览器中编写控制器代码。")

        # 选择编辑已有或新建
        existing = get_custom_controllers()
        edit_options = ["--- 新建控制器 ---"] + [c["name"] for c in existing if not c["error"]]
        edit_choice = st.selectbox("加载已有控制器", edit_options, key="ctrl_edit_sel")

        default_code = f"""class MyController(WindFarmController):
    \"\"\"我的自定义控制器。

    输入 observation:
      - time_s: 当前时间 (s)
      - power_mw: 各风机功率 (MW)        shape=({6},)
      - wind_speed: 各风机风速 (m/s)      shape=({6},)
      - yaw_misalignment_deg: 偏航角 (°)   shape=({6},)

    返回 ControlIntent:
      - yaw_misalignment_deg: 偏航指令 (°)
      - 或其他控制通道
    \"\"\"

    def reset(self, context):
        self.context = context
        self.n = context.n_turbines
        self.step = 0

    def compute(self, observation):
        self.step += 1
        yaw = np.zeros(self.n)

        if observation is not None and observation.wind_speed is not None:
            # 示例：风速 > 10m/s 时偏航上游风机
            for i in range(self.n):
                if observation.wind_speed[i] > 10.0:
                    yaw[i] = -15.0

        return ControlIntent(yaw_misalignment_deg=yaw)
"""

        if edit_choice != "--- 新建控制器 ---":
            for c in existing:
                if c["name"] == edit_choice and c["source"]:
                    default_code = c["source"]
                    break

        edited_code = st.text_area("控制器代码", default_code, height=400)

        c1, c2 = st.columns([1, 3])
        with c1:
            new_name = st.text_input("文件名", "my_controller.py")
        with c2:
            if st.button("💾 保存控制器", type="primary", use_container_width=True):
                if not new_name.endswith(".py"):
                    new_name += ".py"
                err = save_controller(new_name, edited_code)
                if err:
                    st.error(f"保存失败: {err}")
                else:
                    st.success(f"✅ {new_name} 已保存！")
                    st.rerun()

    # ── 快速测试 ──────────────────────────────────────────────────
    st.divider()
    st.subheader("🧪 快速测试控制器")
    st.markdown("选择一个控制器，用 Mock 后端快速验证能否正常运行。")

    testable = [c for c in get_all_controllers() if c["type"] == "custom" and not c["error"]]
    if testable:
        test_sel = st.selectbox(
            "选择控制器",
            testable,
            format_func=lambda c: c["name"],
            key="ctrl_test_sel",
        )

        col1, col2 = st.columns([1, 3])
        with col1:
            test_steps = st.number_input("测试步数", 5, 50, 10, key="ctrl_test_steps")
        with col2:
            if st.button("▶️ 运行测试", type="primary", use_container_width=True,
                         key="ctrl_test_run"):
                try:
                    from wfcrl.engine import DeterministicMockSimulator
                    from wfcrl.config import SimulationConfig, WindConfig
                    from wfcrl.controllers import ControllerRunner

                    # 创建 Mock 仿真器
                    cfg = SimulationConfig(
                        case_name="ctrl-test",
                        num_turbines=6,
                        xcoords=[0, 504, 1008, 0, 504, 1008],
                        ycoords=[-252, -252, -252, 252, 252, 252],
                        dt=2.0,
                        max_iter=test_steps,
                        wind=WindConfig(speed=8.0, direction=270.0),
                    )
                    sim = DeterministicMockSimulator(cfg)

                    # 实例化控制器
                    ctrl_class = test_sel["class"]
                    ctrl_instance = ctrl_class() if ctrl_class else None

                    if ctrl_instance is None:
                        st.error("控制器实例化失败")
                    else:
                        runner = ControllerRunner(sim, ctrl_instance)
                        result = runner.run(
                            test_steps, wind=WindConfig(speed=8.0, direction=270.0),
                            setup=True,
                        )
                        pw = result.output.farm_power_mw
                        st.success(f"✅ 测试通过！{test_steps}步完成")
                        st.metric("平均场功率", f"{np.mean(pw):.3f} MW")

                        # 简单图表
                        chart = pd.DataFrame({
                            "time_s": result.output.time,
                            "farm_power_MW": pw,
                        })
                        st.line_chart(chart.set_index("time_s"), height=200)

                        sim.close()

                except Exception as e:
                    st.error(f"❌ 测试失败: {type(e).__name__}: {e}")
                    import traceback
                    st.code(traceback.format_exc())
    else:
        st.info("没有可测试的自定义控制器。先上传一个吧！")


# ══════════════════════════════════════════════════════════════════════════
# 页面 4：方案参数配置
# ══════════════════════════════════════════════════════════════════════════

def page_controller():
    st.title("🎮 控制器配置面板")
    st.markdown("在界面上调节 A/B/C 三套控制方案的参数，无需修改代码。")

    scheme = st.segmented_control(
        "选择方案",
        ["scheme_a", "scheme_b", "scheme_c"],
        format_func=lambda x: {"scheme_a": "📐 方案A — 稳态重优化",
                                "scheme_b": "🧠 方案B — 动态MPC (主方案)",
                                "scheme_c": "🤖 方案C — 学习增强"}[x],
        default="scheme_b",
    )

    if scheme == "scheme_a":
        st.subheader("📐 方案A — FLORIS 再标定 + 稳态重优化")
        st.markdown("周期性调用 FLORIS 标定 + 稳态优化引擎")

        col1, col2 = st.columns(2)
        with col1:
            cal_interval = st.number_input("标定间隔 (s)", 100, 3600, 600, 50,
                                           help="每隔多长时间重新标定一次FLORIS参数")
            opt_method = st.selectbox("优化方法", ["Serial-Refine", "SLSQP", "COBYLA"], index=0)

        with col2:
            opt_interval = st.number_input("优化间隔 (s)", 10, 600, 60, 10,
                                           help="每隔多长时间重新计算一次设定点")
            yaw_bounds = st.slider("偏航角范围 (°)", 5.0, 35.0, 25.0, 2.5,
                                   help="偏航优化的约束范围 ±")
            st.caption(f"偏航约束: ±{yaw_bounds:.0f}°")

        with st.expander("高级参数"):
            svd_truncate = st.slider("SVD 截断阈值", 0.01, 1.0, 0.1, 0.01,
                                     help="标定中SVD奇异值截断阈值")
            ka_range = st.slider("ka 初始范围", 0.01, 0.10, 0.05, 0.01,
                                 help="尾流膨胀系数ka的搜索范围")

        st.session_state["controller_a_params"] = {
            "cal_interval": cal_interval,
            "opt_interval": opt_interval,
            "opt_method": opt_method,
            "yaw_bounds": yaw_bounds,
            "svd_truncate": svd_truncate,
            "ka_range": ka_range,
        }

        if st.button("📋 导出配置", use_container_width=True):
            st.code(json.dumps(st.session_state["controller_a_params"], indent=2))

    elif scheme == "scheme_b":
        st.subheader("🧠 方案B — FLORIDyn + EnKF + MPC")
        st.markdown("动态尾流模型 + 集合卡尔曼滤波 + 滚动时域MPC")

        tab1, tab2, tab3 = st.tabs(["🎯 MPC 参数", "📊 EnKF 参数", "🌊 尾流模型"])

        with tab1:
            col1, col2 = st.columns(2)
            with col1:
                prediction_horizon = st.number_input("预测时域 (步)", 3, 30, 10, 1,
                                                     help="MPC向前看的步数")
                n_passes = st.slider("优化轮数", 1, 5, 2, 1,
                                     help="Serial-Refine轮数，越多越精确但越慢")
                yaw_refine = st.slider("偏航精细化等级", 1, 4, 2, 1,
                                       help="偏航搜索的精细化等级")
            with col2:
                dt = st.number_input("MPC 内部步长 (s)", 0.5, 10.0, 2.0, 0.5)
                tau_max = prediction_horizon * dt
                st.metric("覆盖时域", f"{tau_max:.0f}s")
                st.caption("MPC 内部滚动覆盖尾流传播时间尺度")

        with tab2:
            col1, col2 = st.columns(2)
            with col1:
                n_ensemble = st.number_input("集合数", 4, 200, 32, 4,
                                             help="EnKF集合大小")
                inflate = st.slider("膨胀因子", 1.0, 2.0, 1.05, 0.01,
                                    help="协方差膨胀因子")
            with col2:
                localization_radius = st.number_input("局地化半径 (m)", 100, 2000, 500, 50,
                                                       help="EnKF 局地化半径")
                st.caption("建议: 4D间距(504m)时 500m 为合理值")

        with tab3:
            col1, col2 = st.columns(2)
            with col1:
                advection_speed = st.slider("平流速度比", 0.5, 1.0, 0.8, 0.05,
                                            help="尾流传播速度 / 来流风速")
                wake_decay = st.slider("尾流衰减系数 kw", 0.02, 0.10, 0.05, 0.005)
            with col2:
                use_twf = st.checkbox("使用 TWF (时域尾流因子)", value=True,
                                      help="考虑尾流传播时延")
                st.caption("TWF 使尾流模型更接近 FAST.Farm 的动态行为")

        st.session_state["controller_b_params"] = {
            "prediction_horizon": prediction_horizon,
            "n_passes": n_passes,
            "yaw_refine_levels": yaw_refine,
            "mpc_dt": dt,
            "n_ensemble": n_ensemble,
            "inflate": inflate,
            "localization_radius": localization_radius,
            "advection_speed": advection_speed,
            "wake_decay": wake_decay,
            "use_twf": use_twf,
        }

        # 生成配置预览
        if st.button("📋 导出 B 配置 (YAML)", use_container_width=True):
            b_config = {
                "scheme": "B",
                "mpc": {
                    "dt": dt, "prediction_horizon": prediction_horizon,
                    "n_passes": n_passes, "yaw_refine_levels": yaw_refine,
                },
                "enkf": {
                    "n_ensemble": n_ensemble, "inflate": inflate,
                    "localization_radius": localization_radius,
                },
                "wake_model": {
                    "advection_speed": advection_speed,
                    "wake_decay": wake_decay, "use_twf": use_twf,
                },
            }
            st.code(yaml.safe_dump(b_config, default_flow_style=False), language="yaml")

    elif scheme == "scheme_c":
        st.subheader("🤖 方案C — 灰箱学习增强")
        st.markdown("在方案B基础上 + 灰箱残差修正 + RL策略")

        route = st.radio("运行路线",
                         ["rl_safety", "greybox_mpc"],
                         format_func=lambda x: {
                             "rl_safety": "🛡️ 安全RL（RL策略 + 物理安全投影）",
                             "greybox_mpc": "📦 灰箱MPC（残差修正 + MPC）",
                         }[x],
                         horizontal=True)

        col1, col2 = st.columns(2)
        with col1:
            residual_model = st.selectbox(
                "残差模型", ["gaussian_process", "ridge_regression", "neural_network"],
                format_func=lambda x: {"gaussian_process": "高斯过程 (GP)",
                                       "ridge_regression": "岭回归",
                                       "neural_network": "神经网络 (MLP)"}[x],
            )
            rl_gamma = st.slider("RL 折扣因子 γ", 0.8, 0.999, 0.95, 0.005)

        with col2:
            train_freq = st.number_input("残差训练频率 (步)", 50, 1000, 200, 50,
                                         help="每多少步重新训练残差模型")
            safe_margin = st.slider("安全裕度", 0.0, 0.5, 0.1, 0.05,
                                    help="RL动作的安全裕度倍数")

        st.session_state["controller_c_params"] = {
            "route": route,
            "residual_model": residual_model,
            "rl_gamma": rl_gamma,
            "train_freq": train_freq,
            "safe_margin": safe_margin,
        }

        if route == "greybox_mpc":
            with st.expander("灰箱MPC 说明"):
                st.markdown("""
                灰箱 MPC = **物理模型 (B方案) + 残差修正**。
                - 未提供训练数据时自动退化为纯物理模型（方案B）
                - 残差模型先用 A/B 阶段的离线数据训练
                - 在线运行时会持续更新残差
                """)
        else:
            with st.expander("安全RL 说明"):
                st.markdown("""
                **安全投影机制**：RL策略输出的动作经过物理约束投影后再下发。
                - 确保RL探索期间不会产生危险的偏航/降额指令
                - 线性占位策略保证初始阶段不劣于贪婪基线
                """)


# ══════════════════════════════════════════════════════════════════════════
# 页面 4：尾流可视化
# ══════════════════════════════════════════════════════════════════════════

def page_wake_viz():
    st.title("🗺️ 尾流可视化")
    st.markdown("显示风电场尾流热力图——颜色越绿风速越高，越红风速越低（尾流区）。")

    # 前置检查
    _HAS_MPL = True

    # ── 参数配置 ──────────────────────────────────────────────────────
    col1, col2, col3 = st.columns([1, 1, 1])
    with col1:
        viz_speed = st.slider("风速 (m/s)", 4.0, 16.0, 8.0, 0.5, key="wv_speed")
        viz_dir = st.slider("风向 (°)", 180, 360, 270, 5, key="wv_dir")

    with col2:
        layout_name = st.selectbox("风场布局",
                                   ["6T", "3T", "HornsRev1", "Ablaincourt"],
                                   index=0, key="wv_layout")
        # 获取布局坐标
        layouts = get_builtin_layouts()
        if isinstance(layouts, dict) and layout_name in layouts:
            info = layouts[layout_name]
            xs = np.array(info["x"])
            ys = np.array(info["y"])
            n = info["n_turbines"]
        else:
            xs = np.array([0, 504, 1008, 0, 504, 1008])
            ys = np.array([-252, -252, -252, 252, 252, 252])
            n = 6
        st.caption(f"{layout_name}: {n}台风机, x范围[{xs.min():.0f},{xs.max():.0f}], "
                   f"y范围[{ys.min():.0f},{ys.max():.0f}]")

    with col3:
        viz_mode = st.selectbox("控制模式",
                                ["greedy", "yaw_steering", "derating"],
                                format_func=lambda x: {
                                    "greedy": "贪婪基线",
                                    "yaw_steering": "偏航尾流控制",
                                    "derating": "降额控制",
                                }[x], key="wv_mode")
        if viz_mode == "yaw_steering":
            yaw_angle = st.slider("偏航角 (°)", -25.0, 25.0, -17.0, 1.0, key="wv_yaw")
        else:
            yaw_angle = 0.0

    # 设置偏航角
    yaw_angles = np.zeros(n)
    if viz_mode == "yaw_steering":
        min_x = xs.min()
        upstream = xs == min_x
        yaw_angles[upstream] = yaw_angle

    # ── 计算并显示尾流场 ──────────────────────────────────────────────
    st.subheader("🌊 尾流热力图")
    st.caption("颜色=风速, 圆点=风机位置, 橙色=主动偏航, 箭头=偏航方向")

    with st.spinner("正在计算尾流场..."):
        try:
            X, Y, U = compute_wake_field(xs, ys, viz_speed, viz_dir, yaw_angles)

            if not _HAS_MPL:
                st.warning("⚠️ matplotlib 不可用，无法渲染热力图")
            else:
                fig, ax = plt.subplots(figsize=(14, 7))

                # 尾流热力图（contourf）
                vmin = viz_speed * 0.2
                vmax = viz_speed * 1.05
                contour = ax.contourf(X, Y, U, levels=40, cmap="RdYlGn",
                                      vmin=vmin, vmax=vmax)
                cbar = plt.colorbar(contour, ax=ax, label="风速 (m/s)", shrink=0.7)
                cbar.ax.tick_params(labelsize=9)

                # 风机标注
                D_r = 63.0  # 转子半径 (m)
                scale = max(200, (xs.max() - xs.min()) * 0.06)  # 自适应标注大小
                label_offset = max(D_r + 10, scale * 0.15)

                for i in range(n):
                    is_yawing = abs(yaw_angles[i]) > 0.5
                    color = "#ff7f0e" if is_yawing else "#1f77b4"
                    edge = "#d35400" if is_yawing else "#2c3e50"
                    alpha_v = 0.95 if is_yawing else 0.85

                    # 偏航方向箭头
                    if is_yawing:
                        arr_len = scale * 0.4
                        dx_a = np.sin(np.deg2rad(yaw_angles[i])) * arr_len
                        dy_a = np.cos(np.deg2rad(yaw_angles[i])) * arr_len
                        ax.arrow(xs[i] - dx_a/2, ys[i] - dy_a/2, dx_a, dy_a,
                                 head_width=scale*0.08, head_length=scale*0.06,
                                 fc="#ff7f0e", ec="#ff7f0e", alpha=0.8,
                                 length_includes_head=True)

                    # 风机圆点
                    circle = Circle((xs[i], ys[i]), D_r, color=color, alpha=alpha_v,
                                    ec=edge, lw=1.5)
                    ax.add_patch(circle)
                    ax.annotate(f"T{i+1}", (xs[i], ys[i] + label_offset),
                                ha="center", fontsize=8 if n > 20 else 10,
                                fontweight="bold")

                # 风向指示
                mid_y = (ys.max() + ys.min()) / 2
                ax.annotate("", xy=(X.max() - 50, mid_y), xytext=(X.min() + 50, mid_y),
                            arrowprops=dict(arrowstyle="->", lw=3, color="#2980b9"))
                ax.text(X.min() + 80, mid_y + max(50, scale*0.2),
                        f"风向 {viz_dir}°", fontsize=11, color="#2980b9",
                        fontweight="bold")

                # 图例
                legend_elements = [
                    Patch(facecolor="#1f77b4", label="未偏航"),
                    Patch(facecolor="#ff7f0e", label="主动偏航"),
                ]
                ax.legend(handles=legend_elements, loc="upper right", fontsize=9,
                          framealpha=0.8)

                ax.set_xlabel("x — 顺风向 (m)", fontsize=10)
                ax.set_ylabel("y — 展向 (m)", fontsize=10)
                ax.set_title(f"{layout_name} — {n}台风机, {viz_speed}m/s {viz_dir}° ({viz_mode})",
                             fontsize=12)
                ax.set_aspect("equal")
                ax.tick_params(labelsize=8)
                fig.tight_layout()

                st.pyplot(fig)
                plt.close(fig)

        except Exception as e:
            st.error(f"❌ 尾流计算出错: {type(e).__name__}: {e}")
            st.info("建议: 切换到 6T 或 3T 布局测试")

    # ── 各风机状态表 ──────────────────────────────────────────────
    st.subheader("📊 各风机估算状态")
    try:
        # 安全地获取网格索引
        x_ptp = max(X.ptp(), 1.0)
        y_ptp = max(Y.ptp(), 1.0)
        cards = []
        for i in range(min(n, 50)):  # 最多显示50台
            xi = int(X.shape[1] / 2 + xs[i] / x_ptp * X.shape[1])
            yi = int(Y.shape[0] / 2 + ys[i] / y_ptp * Y.shape[0])
            xi = np.clip(xi, 0, X.shape[1] - 1)
            yi = np.clip(yi, 0, Y.shape[0] - 1)
            u_i = float(U[yi, xi])
            p_i = 0.5 * 1.225 * np.pi * (126/2)**2 * (u_i**3) * 0.45 / 1e6
            cards.append({
                "风机": f"T{i+1}", "x (m)": f"{xs[i]:.0f}", "y (m)": f"{ys[i]:.0f}",
                "风速": f"{u_i:.1f} m/s", "功率": f"{p_i:.2f} MW",
                "偏航": f"{yaw_angles[i]:.0f}°",
            })
        if cards:
            st.dataframe(pd.DataFrame(cards), use_container_width=True, hide_index=True)
        if n > 50:
            st.caption(f"* 仅显示前50台风机（共{n}台）")
    except Exception as e:
        st.caption(f"状态表计算跳过: {e}")

    # ── 与历史实验对比 ──────────────────────────────────────────────
    with st.expander("📈 与已完成的实验对比", expanded=False):
        exps = find_experiments(limit=5)
        if exps:
            exp_options = {f"{e['mtime'].strftime('%m-%d %H:%M')} — {e['name']} ({e['backend']})": e
                           for e in exps}
            sel = st.selectbox("选择一个实验", list(exp_options.keys()), index=None, key="wv_exp")
            if sel and sel in exp_options:
                df = load_timeseries(exp_options[sel]["dir"])
                if df is not None and "farm_power_MW" in df.columns:
                    st.line_chart(df.set_index("time_s")["farm_power_MW"],
                                  ylabel="Farm Power (MW)", height=200)


# ══════════════════════════════════════════════════════════════════════════
# 页面 5：实验浏览
# ══════════════════════════════════════════════════════════════════════════

def _plot_turbine_series(df, prefix, label, y_label, height=200):
    """绘制各风机参数曲线。prefix = T1_power_MW 中的 'power' 部分"""
    cols = [c for c in df.columns if c.startswith(f"T") and prefix in c]
    if not cols:
        return False
    n = len(cols)
    chart = pd.DataFrame({"time_s": df["time_s"]})
    for c in cols:
        tid = c.split("_")[0]  # "T1", "T2"
        chart[tid] = df[c]
    st.line_chart(chart.set_index("time_s"), y_label=y_label, height=height)
    return True


def _plot_all_turbines_separate(df, prefix, label, y_label):
    """每台风机一张小图。"""
    cols = sorted([c for c in df.columns if c.startswith("T") and prefix in c])
    if not cols:
        return False
    n = len(cols)
    # 最多6台并排，超过则折行
    per_row = min(n, 6)
    rows = (n + per_row - 1) // per_row
    fig, axes = plt.subplots(rows, per_row, figsize=(per_row * 3.5, rows * 2.5),
                              squeeze=False)
    for idx, col in enumerate(cols):
        r, c = idx // per_row, idx % per_row
        ax = axes[r][c]
        tid = col.split("_")[0]
        ax.plot(df["time_s"], df[col], linewidth=1.5)
        ax.set_title(tid, fontsize=10)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel(y_label, fontsize=8)
        ax.grid(alpha=0.25)
        ax.tick_params(labelsize=7)
    # 隐藏多余的子图
    for idx in range(n, rows * per_row):
        r, c = idx // per_row, idx % per_row
        axes[r][c].set_visible(False)
    fig.tight_layout()
    st.pyplot(fig)
    plt.close(fig)
    return True


def page_results():
    st.title("📊 实验浏览")
    st.markdown("浏览所有已完成实验的详细参数，包括各风机功率、偏航、风速、转矩、转速等。")

    exps = find_experiments(limit=100)
    if not exps:
        st.info("还没有实验。前往「快速实验」或「新用户向导」开始运行。")
        return

    # 过滤和搜索
    col1, col2, col3 = st.columns(3)
    with col1:
        filter_backend = st.selectbox("后端筛选", ["全部", "mock", "floris", "fastfarm_continuous"],
                                      key="res_filter_backend")
    with col2:
        search = st.text_input("搜索实验名称", "", key="res_search")
    with col3:
        sort_by = st.selectbox("排序", ["时间(最新)", "时间(最早)", "功率(高→低)", "功率(低→低)"],
                               key="res_sort")

    filtered = exps
    if filter_backend != "全部":
        filtered = [e for e in filtered if e["backend"] == filter_backend]
    if search:
        filtered = [e for e in filtered if search.lower() in e["name"].lower()]
    if sort_by == "时间(最新)":
        pass
    elif sort_by == "时间(最早)":
        filtered = list(reversed(filtered))
    elif sort_by == "功率(高→低)":
        filtered = sorted(filtered, key=lambda e: e["mean_power_mw"] or 0, reverse=True)
    elif sort_by == "功率(低→高)":
        filtered = sorted(filtered, key=lambda e: e["mean_power_mw"] or 0)

    st.caption(f"共 {len(filtered)} 个实验")

    # ── 实验列表 + 详细参数查看 ──────────────────────────────────────
    exp_labels = [f"{'🟢' if e['backend']=='deterministic_mock' else '🟡' if e['backend']=='floris' else '🔴'} "
                  f"{e['name']} | {e['mtime'].strftime('%m-%d %H:%M')} | "
                  f"{e['mean_power_mw']:.3f}MW" if e['mean_power_mw'] else e['name']
                  for e in filtered]

    sel_idx = st.selectbox("选择一个实验查看详细参数", range(len(filtered)),
                           format_func=lambda i: exp_labels[i], key="res_sel")
    exp = filtered[sel_idx]
    exp_dir = exp["dir"]

    # ── 基本信息 ──────────────────────────────────────────────────────
    meta = load_metadata(exp_dir)
    st.subheader(f"📁 {exp['name']}")

    c1, c2, c3, c4, c5 = st.columns(5)
    with c1:
        st.metric("后端", meta.get("backend", "?"))
    with c2:
        st.metric("风机数", meta.get("n_turbines", "?"))
    with c3:
        st.metric("步数", meta.get("steps", "?"))
    with c4:
        pw = meta.get("mean_farm_power_mw")
        st.metric("平均功率", f"{pw:.3f} MW" if pw else "?")
    with c5:
        wt = meta.get("duration_wall_s")
        st.metric("墙钟耗时", f"{wt:.1f}s" if wt else "?")

    # ── 详细参数 Tab ──────────────────────────────────────────────
    df = load_timeseries(exp_dir)
    if df is not None:
        # 检测可用参数组
        groups = []
        if any("power_MW" in c for c in df.columns):
            groups.append("⚡ 功率")
        if any("yaw_deg" in c for c in df.columns):
            groups.append("🔄 偏航")
        if any("wind_speed" in c for c in df.columns):
            groups.append("🌬️ 风速")
        if any("pitch_deg" in c for c in df.columns):
            groups.append("📐 变桨")
        if any("torque_nm" in c for c in df.columns):
            groups.append("🔧 转矩")
        if any("rotor_speed_rpm" in c for c in df.columns):
            groups.append("⚙️ 转速")
        groups.append("📋 原始数据")

        tab_titles = groups
        tabs = st.tabs(tab_titles)

        for ti, tab in enumerate(tabs):
            with tab:
                title = tab_titles[ti]

                if title == "⚡ 功率":
                    st.subheader("各风机功率")
                    # 总功率
                    if "farm_power_MW" in df.columns:
                        st.caption("场总功率")
                        st.line_chart(df.set_index("time_s")["farm_power_MW"],
                                      y_label="Farm Power (MW)", height=200)
                    # 各风机功率
                    st.caption("各风机功率")
                    _plot_turbine_series(df, "power_MW", "Power", "Power (MW)", height=250)

                elif title == "🔄 偏航":
                    st.subheader("各风机偏航角")
                    _plot_turbine_series(df, "yaw_deg", "Yaw", "Yaw (deg)", height=250)

                elif title == "🌬️ 风速":
                    st.subheader("各风机局部风速")
                    _plot_turbine_series(df, "wind_speed", "Wind", "Wind Speed (m/s)", height=250)

                elif title == "📐 变桨":
                    st.subheader("各风机变桨角")
                    _plot_turbine_series(df, "pitch_deg", "Pitch", "Pitch (deg)", height=250)

                elif title == "🔧 转矩":
                    st.subheader("各风机转矩")
                    _plot_turbine_series(df, "torque_nm", "Torque", "Torque (Nm)", height=250)

                elif title == "⚙️ 转速":
                    st.subheader("各风机转速")
                    _plot_turbine_series(df, "rotor_speed_rpm", "Rotor", "Rotor Speed (rpm)", height=250)

                elif title == "📋 原始数据":
                    st.subheader("timeseries.csv — 全部数据")
                    st.dataframe(df, use_container_width=True, hide_index=True)

                    # Controls CSV
                    ctrl_path = Path(exp_dir) / "controls.csv"
                    if ctrl_path.exists():
                        st.subheader("controls.csv — 控制指令追踪")
                        cdf = pd.read_csv(ctrl_path)
                        st.dataframe(cdf, use_container_width=True, hide_index=True)

                    # Overview PNG
                    png_path = Path(exp_dir) / "overview.png"
                    if png_path.exists():
                        st.subheader("overview.png — 概览图")
                        st.image(str(png_path), caption="Overview")

    else:
        st.warning("未找到 timeseries.csv 数据")

    # ── 文件列表 ──────────────────────────────────────────────────
    with st.expander("📂 实验文件列表"):
        files = sorted(Path(exp_dir).rglob("*"))
        for f in files:
            if f.is_file():
                rel = f.relative_to(exp_dir)
                st.caption(f"📄 {rel} ({f.stat().st_size/1024:.1f} KB)")


# ══════════════════════════════════════════════════════════════════════════
# 页面 6：YAML 编辑器
# ══════════════════════════════════════════════════════════════════════════

def page_yaml_editor():
    st.title("⚙️ YAML 配置编辑器")
    st.markdown("通过表单填写参数，自动生成 YAML 实验配置。")

    with st.form("yaml_form"):
        st.subheader("📋 基本配置")
        col1, col2 = st.columns(2)
        with col1:
            f_name = st.text_input("实验名称", "my-experiment")
            f_backend = st.selectbox("仿真后端", ["mock", "floris", "fastfarm"],
                                     format_func=lambda x: {
                                         "mock": "Mock", "floris": "FLORIS",
                                         "fastfarm": "FAST.Farm",
                                     }[x])
            f_controller = st.selectbox("控制器类型", ["greedy", "fixed_yaw", "fixed_derating"],
                                        format_func=lambda x: {
                                            "greedy": "贪婪基线",
                                            "fixed_yaw": "固定偏航",
                                            "fixed_derating": "固定降额",
                                        }[x])
        with col2:
            n_turb = st.number_input("风机数", 1, 100, 6)
            f_dt = st.number_input("时间步长 (s)", 0.5, 60.0,
                                   value={"mock": 2.0, "floris": 30.0,
                                          "fastfarm": 3.0}.get(f_backend, 2.0))
            f_steps = st.number_input("步数", 3, 1000, 40)

        st.subheader("🌤️ 风况")
        col1, col2, col3 = st.columns(3)
        with col1:
            f_wind_speed = st.number_input("风速 (m/s)", 2.0, 25.0, 8.0)
        with col2:
            f_wind_dir = st.number_input("风向 (°)", 0, 360, 270)
        with col3:
            f_ti = st.slider("湍流强度", 0.0, 0.3, 0.06, 0.01)

        st.subheader("🗺️ 布局坐标")
        layout_preset = st.selectbox("布局预设", ["自定义"] + list(get_builtin_layouts().keys() or ["6T"]))
        if layout_preset != "自定义":
            layouts = get_builtin_layouts()
            if isinstance(layouts, dict) and layout_preset in layouts:
                info = layouts[layout_preset]
                f_xcoords = info["x"]
                f_ycoords = info["y"]
                n_turb = info["n_turbines"]
                st.success(f"已加载 {layout_preset} ({n_turb}台风机)")

                # 显示坐标
                coord_df = pd.DataFrame({
                    "风机": [f"T{i+1}" for i in range(n_turb)],
                    "x (m)": f_xcoords,
                    "y (m)": f_ycoords,
                })
                st.dataframe(coord_df, use_container_width=True, hide_index=True)
        else:
            st.markdown("每行输入一个坐标，空格分隔")
            f_coords_text = st.text_area("坐标 (x y 每行一个)",
                                         "0 -252\n504 -252\n1008 -252\n0 252\n504 252\n1008 252",
                                         height=150)
            lines = [l.strip() for l in f_coords_text.strip().split("\n") if l.strip()]
            try:
                f_xcoords = [float(l.split()[0]) for l in lines]
                f_ycoords = [float(l.split()[1]) for l in lines]
                n_turb = len(lines)
            except Exception:
                f_xcoords = [0, 504, 1008]
                f_ycoords = [-252, -252, -252]

        st.subheader("🎮 控制器参数")
        if f_controller == "fixed_yaw":
            st.markdown("**偏航角 (度)**")
            yaw_inputs = st.columns(min(n_turb, 6))
            f_yaw = []
            for i in range(n_turb):
                col_idx = i % len(yaw_inputs)
                if i > 0 and i % len(yaw_inputs) == 0:
                    yaw_inputs = st.columns(min(n_turb - i, 6))
                val = yaw_inputs[i % len(yaw_inputs)].number_input(
                    f"T{i+1}", -25.0, 25.0, 10.0 if i == 0 else 0.0,
                    step=1.0, key=f"ye_yaw_{i}", format="%.1f")
                f_yaw.append(val)
        elif f_controller == "fixed_derating":
            st.markdown("**降额比例**")
            der_inputs = st.columns(min(n_turb, 6))
            f_der = []
            for i in range(n_turb):
                col_idx = i % len(der_inputs)
                if i > 0 and i % len(der_inputs) == 0:
                    der_inputs = st.columns(min(n_turb - i, 6))
                val = der_inputs[i % len(der_inputs)].number_input(
                    f"T{i+1}", 0.3, 1.0, 0.8 if i == 0 else 1.0,
                    step=0.05, key=f"ye_der_{i}", format="%.2f")
                f_der.append(val)

        # 提交按钮
        st.divider()
        submitted = st.form_submit_button("📄 生成 YAML", type="primary",
                                          use_container_width=True)

    # ── 生成 YAML ──────────────────────────────────────────────────────
    if submitted:
        params = {
            "name": f_name,
            "backend": f_backend,
            "dt": f_dt,
            "steps": f_steps,
            "wind_speed": f_wind_speed,
            "wind_direction": f_wind_dir,
            "turbulence_intensity": f_ti,
            "controller": f_controller,
            "xcoords": f_xcoords,
            "ycoords": f_ycoords,
            "turbine_type": "nrel_5MW",
        }
        if f_controller == "fixed_yaw" and f_yaw:
            params["yaw_misalignment_deg"] = f_yaw
        if f_controller == "fixed_derating" and f_der:
            params["derating_ratio"] = f_der

        yaml_out = generate_yaml_from_form(params)

        st.subheader("📄 生成的 YAML 配置")
        st.code(yaml_out, language="yaml")

        if st.button("🚀 运行此配置", type="primary", use_container_width=True):
            with st.spinner("运行中..."):
                proc, out_dir = run_yaml_experiment(yaml_out, f_name)
                if proc.returncode == 0 and out_dir:
                    st.success(f"✅ 完成！输出: {out_dir}")
                    df = load_timeseries(out_dir)
                    if df is not None and "farm_power_MW" in df.columns:
                        st.line_chart(df.set_index("time_s")["farm_power_MW"])
                else:
                    st.error(f"❌ 失败: {proc.stderr[:500]}")


# ══════════════════════════════════════════════════════════════════════════
# 页面 7：对比分析
# ══════════════════════════════════════════════════════════════════════════

def page_compare():
    st.title("🔬 对比分析")
    exps = find_experiments(limit=100)

    if len(exps) < 2:
        st.info("至少需要2个实验才能对比。")
        return

    # 选择实验
    exp_labels = {f"{e['mtime'].strftime('%m-%d %H:%M')} — {e['name']} ({e['backend']})": e
                  for e in exps}

    col1, col2 = st.columns(2)
    with col1:
        sel_a = st.selectbox("参照组 (A)", list(exp_labels.keys()), index=0)
    with col2:
        sel_b = st.selectbox("对照组 (B)", list(exp_labels.keys()),
                             index=min(1, len(exp_labels) - 1))

    exp_a = exp_labels[sel_a]
    exp_b = exp_labels[sel_b]
    meta_a = load_metadata(exp_a["dir"])
    meta_b = load_metadata(exp_b["dir"])

    # 指标对比
    st.subheader("📊 指标对比")

    def get_meta(m, key):
        v = m.get(key)
        if v is None:
            return "-"
        if isinstance(v, float):
            return round(v, 3)
        return v

    rows = []
    for label, key in [("后端", "backend"), ("风机数", "n_turbines"),
                        ("步数", "steps"), ("仿真时长(s)", "final_time_s"),
                        ("平均功率(MW)", "mean_farm_power_mw"),
                        ("约束事件", "constraint_events"),
                        ("墙钟耗时(s)", "duration_wall_s")]:
        va, vb = get_meta(meta_a, key), get_meta(meta_b, key)
        diff = ""
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
            pct = (vb / va - 1) * 100 if va != 0 else 0
            diff = f"{pct:+.1f}%"
        rows.append({"指标": label, "实验A": va, "实验B": vb, "差异": diff})

    st.table(pd.DataFrame(rows))

    # 功率曲线叠加
    df_a = load_timeseries(exp_a["dir"])
    df_b = load_timeseries(exp_b["dir"])

    if df_a is not None and df_b is not None:
        has_power = "farm_power_MW" in df_a.columns and "farm_power_MW" in df_b.columns
        if has_power:
            st.subheader("📈 场功率对比")
            # 对齐时间
            t_min = max(df_a["time_s"].min(), df_b["time_s"].min())
            t_max = min(df_a["time_s"].max(), df_b["time_s"].max())
            a_clip = df_a[(df_a["time_s"] >= t_min) & (df_a["time_s"] <= t_max)]
            b_clip = df_b[(df_b["time_s"] >= t_min) & (df_b["time_s"] <= t_max)]

            chart = pd.DataFrame({
                "time_s": a_clip["time_s"],
                f"A: {exp_a['name']}": a_clip["farm_power_MW"].values[:len(a_clip)],
                f"B: {exp_b['name']}": b_clip["farm_power_MW"].values[:len(a_clip)],
            })
            st.line_chart(chart.set_index("time_s"),
                          ylabel="Farm Power (MW)")

            # 增益统计
            pA = a_clip["farm_power_MW"].mean()
            pB = b_clip["farm_power_MW"].mean()
            gain = (pB / pA - 1) * 100

            c1, c2, c3 = st.columns(3)
            c1.metric("A 平均", f"{pA:.3f} MW")
            c2.metric("B 平均", f"{pB:.3f} MW")
            c3.metric("增益", f"{gain:+.2f}%",
                      delta_color="inverse" if gain < 0 else "normal")

    # 完整对比报告
    st.subheader("📋 完整对比报告")
    if st.button("生成完整对比报告 (wfcrl-compare)", use_container_width=True):
        out_dir = COMPARISONS_DIR / f"compare-{exp_a['name']}-vs-{exp_b['name']}"
        out_dir.mkdir(parents=True, exist_ok=True)
        with st.spinner("生成报告中..."):
            proc = subprocess.run(
                ["python", "-m", "wfcrl.analysis.cli",
                 str(exp_a["dir"]), str(exp_b["dir"]),
                 "--baseline", meta_a.get("name", exp_a["name"]),
                 "--output", str(out_dir), "--allow-incompatible"],
                cwd=str(PROJECT_ROOT), capture_output=True, text=True, timeout=120,
            )
        if proc.returncode == 0:
            st.success(f"✅ 报告已生成")
            comp_json = out_dir / "comparison.json"
            if comp_json.exists():
                try:
                    data = json.loads(comp_json.read_text(encoding="utf-8"))
                    st.json(data)
                except Exception:
                    pass
            for img_name in ["farm_power.png", "metrics.png"]:
                img_path = out_dir / img_name
                if img_path.exists():
                    st.image(str(img_path), caption=img_name)
        else:
            st.error(f"❌ 失败: {proc.stderr[:500]}")


# ══════════════════════════════════════════════════════════════════════════
# 页面：风电场管理
# ══════════════════════════════════════════════════════════════════════════

def _save_custom_layout(name, xcoords, ycoords, turbine_type="nrel_5MW"):
    """保存自定义布局到 custom_layouts/ 目录。"""
    import yaml
    layout_dir = PROJECT_ROOT / "custom_layouts"
    layout_dir.mkdir(parents=True, exist_ok=True)
    data = {
        "name": name,
        "num_turbines": len(xcoords),
        "xcoords": [float(x) for x in xcoords],
        "ycoords": [float(y) for y in ycoords],
        "turbine_type": turbine_type,
        "metadata": {"source": "user_defined", "created": datetime.now().isoformat()},
    }
    fpath = layout_dir / f"{name}.yaml"
    with open(fpath, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    return fpath


def _load_custom_layouts():
    """加载 custom_layouts/ 目录中的所有自定义布局。"""
    import yaml
    layout_dir = PROJECT_ROOT / "custom_layouts"
    if not layout_dir.exists():
        return []
    layouts = []
    for fpath in sorted(layout_dir.glob("*.yaml")):
        try:
            data = yaml.safe_load(fpath.read_text(encoding="utf-8"))
            if data and "xcoords" in data and "ycoords" in data:
                layouts.append(data)
        except Exception:
            pass
    return layouts


def page_farm_config():
    st.title("🏭 风电场配置管理")
    st.markdown("管理风电场布局——查看内置布局、创建自定义布局。")

    tab1, tab2, tab3 = st.tabs(["📋 所有布局", "✏️ 新建布局", "⚙️ 布局参数"])

    # ── Tab 1: 布局列表 ──────────────────────────────────────────
    with tab1:
        # 内置布局
        builtin = get_builtin_layouts()
        # 自定义布局
        custom = _load_custom_layouts()

        st.subheader("📦 内置布局")
        if isinstance(builtin, dict) and builtin:
            data = []
            for name, info in builtin.items():
                xs = info["x"]
                ys = info["y"]
                dx = np.diff(sorted(set(xs)))
                dy = np.diff(sorted(set(ys)))
                row_spacing = f"{dy[dy > 0].mean():.0f}m" if len(dy) > 0 and dy[dy > 0].size > 0 else "-"
                col_spacing = f"{dx[dx > 0].mean():.0f}m" if len(dx) > 0 and dx[dx > 0].size > 0 else "-"
                data.append({
                    "名称": name,
                    "风机数": info["n_turbines"],
                    "x范围": f"{min(xs):.0f}~{max(xs):.0f}m",
                    "y范围": f"{min(ys):.0f}~{max(ys):.0f}m",
                    "行间距": row_spacing,
                    "列间距": col_spacing,
                })
            st.dataframe(pd.DataFrame(data), use_container_width=True, hide_index=True)
        else:
            st.info("无内置布局数据")

        if custom:
            st.subheader("🧩 自定义布局")
            for layout in custom:
                with st.expander(f"{layout['name']} ({layout['num_turbines']}台)"):
                    st.json(layout)
                    if st.button(f"🗑️ 删除", key=f"del_layout_{layout['name']}"):
                        fpath = PROJECT_ROOT / "custom_layouts" / f"{layout['name']}.yaml"
                        if fpath.exists():
                            fpath.unlink()
                            st.rerun()

    # ── Tab 2: 新建布局 ──────────────────────────────────────────
    with tab2:
        st.subheader("✏️ 创建新布局")

        col1, col2 = st.columns(2)
        with col1:
            name = st.text_input("布局名称", "my_farm")
            n_turbines = st.number_input("风机数量", 1, 100, 6, key="fc_nt")
            turbine_type = st.selectbox("风机型号", ["nrel_5MW", "goldwind_8_5mw",
                                                      "iea_15MW", "DTU_10MW"], index=0)

        with col2:
            # 快速预设
            preset = st.selectbox("快速预设", ["自定义", "单行3台@4D", "2行3列@4D",
                                               "单行6台@4D", "3行3列@4D"],
                                  key="fc_preset")
            if preset == "单行3台@4D":
                n_turbines = 3
                xs = [0.0, 504.0, 1008.0]
                ys = [0.0, 0.0, 0.0]
            elif preset == "2行3列@4D":
                n_turbines = 6
                xs = [0.0, 504.0, 1008.0, 0.0, 504.0, 1008.0]
                ys = [-252.0, -252.0, -252.0, 252.0, 252.0, 252.0]
            elif preset == "单行6台@4D":
                n_turbines = 6
                xs = [0.0, 504.0, 1008.0, 1512.0, 2016.0, 2520.0]
                ys = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            elif preset == "3行3列@4D":
                n_turbines = 9
                xs = []
                ys = []
                spacing = 504
                for row in range(3):
                    for col in range(3):
                        xs.append(col * spacing)
                        ys.append((row - 1) * spacing)
            else:
                xs = [i * 504 for i in range(n_turbines)]
                ys = [0.0] * n_turbines

        # 坐标编辑
        st.markdown("**风机坐标**")
        coord_data = pd.DataFrame({
            "风机": [f"T{i+1}" for i in range(n_turbines)],
            "x (m)": xs[:n_turbines],
            "y (m)": ys[:n_turbines],
        })
        edited_coords = st.data_editor(coord_data, use_container_width=True,
                                        hide_index=True, num_rows="dynamic",
                                        key="fc_coords")

        # 预览
        if len(edited_coords) > 0:
            fig, ax = plt.subplots(figsize=(10, 5))
            ax.scatter(edited_coords["x (m)"], edited_coords["y (m)"],
                       s=200, c="#1f77b4", zorder=5)
            for i, row in edited_coords.iterrows():
                ax.annotate(f"T{i+1}", (row["x (m)"], row["y (m)"] + 15),
                            ha="center", fontsize=9)
            # 风向
            ax.annotate("", xy=(max(edited_coords["x (m)"]) + 50, edited_coords["y (m)"].mean()),
                        xytext=(min(edited_coords["x (m)"]) - 50, edited_coords["y (m)"].mean()),
                        arrowprops=dict(arrowstyle="->", lw=2, color="#2980b9"))
            ax.text(min(edited_coords["x (m)"]) - 30, edited_coords["y (m)"].mean() + 30,
                    "270 deg", fontsize=9, color="#2980b9")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_title(f"{name} — {len(edited_coords)}台风机")
            ax.grid(alpha=0.2)
            ax.set_aspect("equal")
            st.pyplot(fig)
            plt.close(fig)

        if st.button("💾 保存布局", type="primary", use_container_width=True):
            fpath = _save_custom_layout(
                name,
                edited_coords["x (m)"].tolist(),
                edited_coords["y (m)"].tolist(),
                turbine_type,
            )
            st.success(f"✅ 布局已保存: {fpath}")

    # ── Tab 3: 布局详情 ──────────────────────────────────────────
    with tab3:
        st.subheader("⚙️ 布局详情")
        all_names = []
        builtin = get_builtin_layouts()
        if isinstance(builtin, dict):
            all_names.extend(builtin.keys())
        custom = _load_custom_layouts()
        for c in custom:
            all_names.append(c["name"])

        sel = st.selectbox("选择布局查看详情", all_names if all_names else ["无"], key="fc_detail")

        info = None
        if isinstance(builtin, dict) and sel in builtin:
            info = builtin[sel]
            xs, ys = info["x"], info["y"]
            n = info["n_turbines"]
        else:
            for c in custom:
                if c["name"] == sel:
                    xs, ys = c["xcoords"], c["ycoords"]
                    n = c["num_turbines"]
                    break

        if info or (xs and ys):
            fig, ax = plt.subplots(figsize=(10, 5))
            ax.scatter(xs, ys, s=200, c="#1f77b4", zorder=5)
            for i in range(n):
                ax.annotate(f"T{i+1}", (xs[i], ys[i] + 15), ha="center", fontsize=9)
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_title(f"{sel} — {n}台风机")
            ax.grid(alpha=0.2)
            ax.set_aspect("equal")
            st.pyplot(fig)
            plt.close(fig)

            # 坐标表
            st.dataframe(pd.DataFrame({
                "风机": [f"T{i+1}" for i in range(n)],
                "x (m)": xs, "y (m)": ys,
            }), use_container_width=True, hide_index=True)


# ══════════════════════════════════════════════════════════════════════════
# 页面：运行实验
# ══════════════════════════════════════════════════════════════════════════

def _list_farm_names():
    """获取所有可用风场名称（内置 + 自定义）。"""
    names = []
    builtin = get_builtin_layouts()
    if isinstance(builtin, dict):
        names.extend(builtin.keys())
    for c in _load_custom_layouts():
        names.append(c["name"])
    return names


def _get_farm_coords(name):
    """按名称获取风场坐标。"""
    builtin = get_builtin_layouts()
    if isinstance(builtin, dict) and name in builtin:
        info = builtin[name]
        return info["x"], info["y"], info["n_turbines"]
    for c in _load_custom_layouts():
        if c["name"] == name:
            return c["xcoords"], c["ycoords"], c["num_turbines"]
    return None, None, 0


def page_run():
    st.title("▶️ 运行实验")
    st.markdown("选择风电场 → 选择控制器 → 配置参数 → 运行")

    # ── 步骤 1: 选择风电场 ──────────────────────────────────────
    st.subheader("1️⃣ 选择风电场")
    farm_names = _list_farm_names()
    if not farm_names:
        st.warning("⚠️ 没有可用风场。先去「🏭 风电场管理」创建一个。")
        return

    col1, col2 = st.columns([1, 2])
    with col1:
        farm_name = st.selectbox("风电场", farm_names, key="run_farm")
    with col2:
        xs, ys, n_t = _get_farm_coords(farm_name)
        if xs:
            st.caption(f"{farm_name}: {n_t}台风机, "
                       f"x=[{min(xs):.0f},{max(xs):.0f}], y=[{min(ys):.0f},{max(ys):.0f}]")
            # 小预览图
            fig, ax = plt.subplots(figsize=(4, 2))
            ax.scatter(xs, ys, s=50, c="#1f77b4")
            for i in range(min(n_t, 6)):
                ax.annotate(f"T{i+1}", (xs[i], ys[i] + 10), ha="center", fontsize=7)
            ax.set_aspect("equal")
            ax.axis("off")
            st.pyplot(fig)
            plt.close(fig)

    # ── 步骤 2: 选择控制器 ──────────────────────────────────────
    st.subheader("2️⃣ 选择控制器")
    controllers = get_all_controllers()
    if not controllers:
        st.warning("⚠️ 没有可用控制器。先去「🎮 控制器管理」上传一个。")
        return

    ctrl_options = {f"{'🧩' if c['type']=='custom' else '📦'} {c['name']}": c
                    for c in controllers if not c["error"]}
    ctrl_sel = st.selectbox("控制器", list(ctrl_options.keys()), key="run_ctrl")
    ctrl_info = ctrl_options[ctrl_sel]
    is_custom = ctrl_info["type"] == "custom"
    st.caption(ctrl_info["description"])

    # ── 步骤 3: 风况参数 ──────────────────────────────────────
    st.subheader("3️⃣ 风况与仿真参数")

    col_wind, col_sim = st.columns([1, 1])

    with col_wind:
        wind_type = st.radio("风类型", ["稳态 (Steady)", "湍流 (Turbulent)"],
                             horizontal=True, key="run_wind_type")

        if wind_type == "湍流 (Turbulent)":
            # 选择已生成的湍流风文件
            winds_dir = Path(PROJECT_ROOT / "custom_winds")
            winds_dir.mkdir(parents=True, exist_ok=True)
            bts_files = sorted(winds_dir.glob("*.bts"))
            template_bts = sorted(Path(
                PROJECT_ROOT / "wfcrl/simulators/fastfarm/inputs/template/FarmInputs"
            ).glob("*.bts"))
            all_bts = bts_files + template_bts

            if all_bts:
                bts_options = {f.name: str(f) for f in all_bts}
                sel_bts = st.selectbox("选择湍流风文件", list(bts_options.keys()),
                                       index=0, key="run_bts")
                wind_file_path = bts_options[sel_bts]
                st.caption(f"已选: {sel_bts}")

                # 从文件名提取风速信息
                import re
                speed_match = re.search(r'(\d+\.?\d*)m?s', sel_bts)
                if speed_match:
                    wind_speed = float(speed_match.group(1))
                else:
                    wind_speed = 8.0
                wind_dir_val = st.number_input("风向 (°)", 180, 360, 270, 5, key="run_wd_turb")
            else:
                st.warning("没有湍流风文件。先去「🌤️ 湍流风」生成一个。")
                wind_speed = 8.0
                wind_dir_val = 270.0
                wind_file_path = None
            turbulence = 0.15
        else:
            col_a, col_b = st.columns(2)
            with col_a:
                wind_speed = st.slider("风速 (m/s)", 4.0, 16.0, 8.0, 0.5, key="run_ws")
            with col_b:
                wind_dir_val = st.slider("风向 (°)", 180, 360, 270, 5, key="run_wd")
            turbulence = st.slider("湍流强度", 0.02, 0.20, 0.06, 0.01, key="run_ti")
            wind_file_path = None

    with col_sim:
        backend = st.selectbox("仿真后端",
                               ["mock", "fastfarm"],
                               format_func=lambda x: {"mock": "Mock (秒级)",
                                                      "fastfarm": "FAST.Farm (分钟级)"}[x],
                               key="run_be")
        if backend == "mock":
            dt = st.number_input("时间步长 (s)", 1.0, 5.0, 2.0, 0.5, key="run_dt")
            steps = st.number_input("步数", 5, 500, 40, key="run_steps")
        else:
            dt = st.number_input("时间步长 (s)", 1.0, 5.0, 3.0, 0.5, key="run_dt")
            steps = st.number_input("步数", 5, 200, 30, key="run_steps")
        st.caption(f"仿真时长: {dt * steps:.0f}s")
        enable_vtk = False
        if backend == "fastfarm":
            enable_vtk = st.checkbox("VTK flow field visualization", value=False, help="Generate VTK files for wake visualization in history")

    if is_custom and backend == "fastfarm":
        st.warning("⚠️ 自定义控制器当前仅支持 Mock 后端。选择内置控制器可使用 FAST.Farm。")

    # ── 步骤 4: 运行 ───────────────────────────────────────────
    st.subheader("4️⃣ 运行")

    run_disabled = st.session_state.get("experiment_running", False)
    run_disabled = run_disabled or (is_custom and backend == "fastfarm")

    if st.button("🚀 运行实验", type="primary", use_container_width=True,
                 disabled=run_disabled, key="run_btn"):
        st.session_state["experiment_running"] = True

        from wfcrl.config import SimulationConfig, WindConfig
        from wfcrl.engine import DeterministicMockSimulator

        with st.status("运行中...", expanded=True) as status:
            progress = st.progress(0, text="初始化...")
            try:
                start_t = time.time()

                # 构建仿真器
                # 构建风配置（支持稳态和湍流）
                if wind_type == "湍流 (Turbulent)" and wind_file_path:
                    # 复制 .bts 到短路径（FAST.Farm Fortran 不支持中文路径）
                    import shutil
                    bts_dst = os.path.join(os.environ.get("TMP", str(Path.home())),
                                           os.path.basename(wind_file_path))
                    if os.path.abspath(wind_file_path) != os.path.abspath(bts_dst):
                        shutil.copy2(wind_file_path, bts_dst)
                    else:
                        bts_dst = wind_file_path

                    # 设置 FARMINPUTS_DIR 环境变量
                    template_farm = str(PROJECT_ROOT / "wfcrl/simulators/fastfarm/inputs/template/FarmInputs")
                    os.environ["WFCRL_FARMINPUTS_DIR"] = template_farm

                    from wfcrl.config.types import WindType
                    wind_cfg = WindConfig(
                        wind_type=WindType.TURBSIM_BTS,
                        speed=wind_speed, direction=wind_dir_val,
                        turbulence_intensity=turbulence,
                        wind_file=bts_dst,
                    )
                else:
                    wind_cfg = WindConfig(
                        speed=wind_speed, direction=wind_dir_val,
                        turbulence_intensity=turbulence,
                    )

                if backend == "mock":
                    cfg = SimulationConfig(
                        case_name=f"run-{farm_name}-{ctrl_info['name']}",
                        num_turbines=n_t, xcoords=list(xs), ycoords=list(ys),
                        dt=dt, max_iter=steps,
                        wind=wind_cfg,
                    )
                    sim = DeterministicMockSimulator(cfg)
                else:
                    from wfcrl.engine.fastfarm_continuous import ContinuousFastFarmInterface
                    from wfcrl.config.simulator import FastFarmConfig
                    exp_name = f"{farm_name}-{ctrl_info["name"]}-{datetime.now().strftime("%H%M%S")}"
                    os.makedirs(str(EXPERIMENTS_DIR / exp_name), exist_ok=True)
                    cfg = FastFarmConfig(
                        case_name=f"ff-run-{farm_name}",
                        num_turbines=n_t, xcoords=list(xs), ycoords=list(ys),
                        dt=dt, max_iter=steps,
                        wind=wind_cfg,
                        output_dir=str(EXPERIMENTS_DIR / exp_name),
                        enable_vtk=enable_vtk,
                    )
                    sim = ContinuousFastFarmInterface(cfg)

                # 实例化控制器
                if is_custom:
                    ctrl_class = ctrl_info["class"]
                    ctrl_instance = ctrl_class()
                else:
                    # 内置控制器
                    ctrl_type = ctrl_info["name"]
                    if ctrl_type == "greedy":
                        from wfcrl.controllers import GreedyController
                        ctrl_instance = GreedyController()
                    elif ctrl_type == "fixed_yaw":
                        from wfcrl.controllers import FixedYawController
                        yaw = [10.0] + [0.0] * (n_t - 1)
                        ctrl_instance = FixedYawController(yaw)
                    elif ctrl_type == "fixed_derating":
                        from wfcrl.controllers import FixedDeratingController
                        dr = [0.8] + [1.0] * (n_t - 1)
                        ctrl_instance = FixedDeratingController(dr, [5.0] + [0.0] * (n_t - 1))
                    elif ctrl_type == "fastfarm_yaw":
                        from wfcrl.controllers import FastFarmYawController
                        ctrl_instance = FastFarmYawController()
                        # Use set_fixed_yaw to directly lock yaw (avoids DLL yaw control issues)
                        if hasattr(sim, "set_fixed_yaw"):
                            yaw_arr = np.array([-17.0] + [0.0]*(n_t-1), dtype=np.float64)
                            sim.set_fixed_yaw(yaw_arr)
                    else:
                        raise ValueError(f"未知控制器: {ctrl_type}")

                # 运行
                from wfcrl.controllers import ControllerRunner
                from wfcrl.engine.base import SimulatorInterface
                runner = ControllerRunner(sim, ctrl_instance)
                result = runner.run(
                    steps, wind=wind_cfg,
                    setup=True,
                )
                elapsed = time.time() - start_t
                sim.close()

                # 保存结果
                exp_name = f"{farm_name}-{ctrl_info['name']}-{datetime.now().strftime('%H%M%S')}"
                out_dir = EXPERIMENTS_DIR / exp_name
                out_dir.mkdir(parents=True, exist_ok=True)
                result.output.to_csv(out_dir / "timeseries.csv")
                meta = {
                    "schema_version": "1.0", "name": exp_name,
                    "backend": backend,
                    "farm": farm_name, "controller": ctrl_info["name"],
                    "steps": steps, "n_turbines": n_t,
                    "final_time_s": steps * dt,
                    "mean_farm_power_mw": float(np.mean(result.output.farm_power_mw)),
                    "duration_wall_s": elapsed,
                    "wind_speed": wind_speed, "wind_direction": wind_dir_val,
                }
                json.dump(meta, open(out_dir / "metadata.json", "w", encoding="utf-8"),
                          indent=2, ensure_ascii=False)
                # Copy VTK files if FAST.Farm was used
                import shutil
                for sfx in ["FarmInputs/vtk_ff", "simulator/FarmInputs/vtk_ff"]:
                    src = Path(str(sim.config.output_dir)) / sfx
                    if src.exists():
                        dst = out_dir / sfx
                        dst.parent.mkdir(parents=True, exist_ok=True)
                        if dst.exists():
                            shutil.rmtree(str(dst))
                        shutil.copytree(str(src), str(dst))
                        break

                progress.progress(1.0, text="完成")
                status.update(label="✅ 实验完成", state="complete")
                st.success(f"✅ 完成！耗时 {elapsed:.1f}s")
                st.metric("平均场功率", f"{np.mean(result.output.farm_power_mw):.3f} MW")

                # 显示曲线
                chart = pd.DataFrame({
                    "time_s": result.output.time,
                    "farm_power_MW": result.output.farm_power_mw,
                })
                st.line_chart(chart.set_index("time_s"), height=200)

            except Exception as e:
                progress.progress(1.0, text="失败")
                status.update(label="❌ 失败", state="error")
                st.error(f"{type(e).__name__}: {e}")
                import traceback
                st.code(traceback.format_exc())

        # 标记实验结束，移除 rerun 以免状态混乱
        st.session_state["experiment_running"] = False


# ══════════════════════════════════════════════════════════════════════════
# 页面：历史实验
# ══════════════════════════════════════════════════════════════════════════

def _show_experiment_detail(exp, df, meta):
    """显示单个实验的详细参数面板。"""
    # 从实验名推断缺失的配置
    exp_name = meta.get("name", exp["name"])
    be = meta.get("backend", "?")

    # 从实验名尝试提取风场和控制器（兼容旧实验）
    name_parts = exp_name.split("-")
    guess_farm = name_parts[0] if len(name_parts) > 0 else exp_name
    guess_ctrl = name_parts[1] if len(name_parts) > 1 else "?"

    farm = meta.get("farm", guess_farm)
    ctrl = meta.get("controller", guess_ctrl)
    ws = meta.get("wind_speed")
    wd = meta.get("wind_direction")

    # ── 配置信息卡 ──────────────────────────────────────────────
    st.subheader("📋 实验配置")
    cols = st.columns(6)
    cols[0].metric("后端", be)
    cols[1].metric("风场", farm)
    cols[2].metric("控制器", ctrl)
    cols[3].metric("风机", meta.get("n_turbines", "?"))
    cols[4].metric("风速", f"{ws} m/s" if ws else "?")
    cols[5].metric("风向", f"{wd}°" if wd else "?")

    cols2 = st.columns(4)
    cols2[0].metric("步数", meta.get("steps", "?"))
    cols2[1].metric("仿真时长", f"{meta.get('final_time_s', 0):.0f}s" if meta.get('final_time_s') else "?")
    pw = meta.get("mean_farm_power_mw")
    cols2[2].metric("平均功率", f"{pw:.3f} MW" if pw else "?")
    wt = meta.get("duration_wall_s")
    cols2[3].metric("墙钟耗时", f"{wt:.1f}s" if wt else "?")

    col_del1, col_del2 = st.columns([1, 5])
    with col_del1:
        btn_key = f"del_{exp['dir'].name}"
        if st.button("🗑️ 删除", type="secondary", key=btn_key):
            import shutil
            shutil.rmtree(str(exp["dir"]), ignore_errors=True)
            st.success(f"已删除: {exp['dir'].name}")
            st.rerun()

    if df is None:
        st.warning("无时序数据")
        return

    # ── VTK visualization ──
    for _vd in [Path(str(exp["dir"]))/"simulator"/"FarmInputs"/"vtk_ff",
                Path(str(exp["dir"]))/"FarmInputs"/"vtk_ff"]:
        if _vd.exists() and _HAVE_PYVISTA:
            _lf = sorted(_vd.glob("Case.Low.Dis.*.vtk"))
            if _lf:
                st.subheader("VTK FAST.Farm flow field")
                _si = st.slider("Time step", 0, len(_lf)-1, len(_lf)-1, key=f"vtk_{exp["dir"].name}")
                _img = render_vtk_screenshot(str(_vd), _si, 90.0)
                if _img is not None:
                    st.image(_img, use_container_width=True)
                st.divider()
            break
        elif _vd.exists():
            st.info("VTK files exist. Install: pip install pyvista")
            break

    # ── 参数Tab ──────────────────────────────────────────────────
    st.subheader("📈 详细参数")
    tab_names = []
    if any("power_MW" in c for c in df.columns):
        tab_names.append("⚡ 功率")
    if any("yaw_deg" in c for c in df.columns):
        tab_names.append("🔄 偏航")
    if any("wind_speed" in c for c in df.columns):
        tab_names.append("🌬️ 风速")
    if any("pitch_deg" in c for c in df.columns):
        tab_names.append("📐 变桨")
    if any("torque_nm" in c for c in df.columns):
        tab_names.append("🔧 转矩")
    if any("rotor_speed_rpm" in c for c in df.columns):
        tab_names.append("⚙️ 转速")
    tab_names.append("📋 数据表")

    tabs = st.tabs(tab_names)
    col_map = {
        "⚡ 功率": "power_MW",
        "🔄 偏航": "yaw_deg",
        "🌬️ 风速": "wind_speed",
        "📐 变桨": "pitch_deg",
        "🔧 转矩": "torque_nm",
        "⚙️ 转速": "rotor_speed_rpm",
    }

    for ti, tab in enumerate(tabs):
        with tab:
            title = tab_names[ti]
            if title == "📋 数据表":
                st.dataframe(df, use_container_width=True, hide_index=True)
                # Controls CSV
                ctrl_path = Path(exp["dir"]) / "controls.csv"
                if ctrl_path.exists():
                    st.subheader("控制指令追踪 (controls.csv)")
                    st.dataframe(pd.read_csv(ctrl_path), use_container_width=True, hide_index=True)
                # Overview PNG
                png_path = Path(exp["dir"]) / "overview.png"
                if png_path.exists():
                    st.image(str(png_path), caption="概览图")
            elif title in col_map:
                prefix = col_map[title]
                # 总览（场总功率）
                if prefix == "power_MW" and "farm_power_MW" in df.columns:
                    st.caption("场总功率")
                    st.line_chart(df.set_index("time_s")["farm_power_MW"], height=180)
                # 各风机曲线
                st.caption(f"各风机{title.replace('⚡','').replace('🔄','').replace('🌬️','').replace('📐','').replace('🔧','').replace('⚙️','').strip()}")
                t_cols = sorted([c for c in df.columns if c.startswith("T") and prefix in c])
                if t_cols:
                    chart = pd.DataFrame({"time_s": df["time_s"]})
                    for c in t_cols:
                        chart[c.split("_")[0]] = df[c]
                    st.line_chart(chart.set_index("time_s"), height=200)

    # ── 稳态值卡片 ──────────────────────────────────────────────
    st.subheader("🎯 稳态值 (最后一步)")
    n_turbs = meta.get("n_turbines", 0)
    if n_turbs and df is not None and len(df) > 1:
        last = df.iloc[-1]
        cards = []
        for i in range(1, n_turbs + 1):
            def _safe(v):
                return v if pd.notna(v) else 0
            p = _safe(last.get(f"T{i}_power_MW", 0))
            ws = _safe(last.get(f"T{i}_wind_speed", 0))
            yaw = _safe(last.get(f"T{i}_yaw_deg", 0))
            cards.append({"风机": f"T{i}", "功率(MW)": f"{p:.3f}",
                          "风速(m/s)": f"{ws:.2f}", "偏航(°)": f"{yaw:.1f}"})
        if cards:
            st.dataframe(pd.DataFrame(cards), use_container_width=True, hide_index=True)


def page_history():
    st.title("📊 历史实验")
    st.markdown("浏览、查看和对比已完成的实验。")

    exps = find_experiments(limit=200)
    if not exps:
        st.info("还没有实验。去「▶️ 运行实验」跑一个吧。")
        return

    # ── 过滤 + 选择 ──────────────────────────────────────────────
    col1, col2, col3 = st.columns([2, 2, 1])
    with col1:
        be_filter = st.selectbox("后端", ["全部", "mock", "fastfarm_continuous"], key="hist_be")
    with col2:
        search = st.text_input("搜索名称", "", key="hist_search")
    with col3:
        st.caption("")
        view_mode = st.selectbox("视图", ["列表", "详情"], key="hist_view")

    filtered = exps
    if be_filter != "全部":
        filtered = [e for e in filtered if e["backend"] == be_filter]
    if search:
        filtered = [e for e in filtered if search.lower() in e["name"].lower()]

    st.caption(f"共 {len(filtered)} 个实验")

    if view_mode == "详情" and filtered:
        # ── 详情模式：选一个实验看全部参数 ──────────────────────────
        exp_labels = {f"{'🟢' if 'mock' in str(e['backend']) else '🔴'} "
                      f"{e['name']} ({e['mtime'].strftime('%m-%d %H:%M')})": e
                      for e in filtered}
        sel = st.selectbox("选择实验查看详情", list(exp_labels.keys()), key="hist_detail_sel")
        if sel:
            exp = exp_labels[sel]
            meta = load_metadata(exp["dir"])
            df = load_timeseries(exp["dir"])
            _show_experiment_detail(exp, df, meta)

    else:
        # ── 列表模式：展开列表 ──────────────────────────────────────
        for i, exp in enumerate(filtered):
            with st.expander(
                f"{'🟢' if 'mock' in str(exp['backend']) else '🔴'} "
                f"{exp['name']} — {exp['mtime'].strftime('%m-%d %H:%M')}"
                + (f"  {exp['mean_power_mw']:.3f} MW" if isinstance(exp.get('mean_power_mw'), (int, float)) else ""),
                expanded=(exp.get('_just_completed', False))
            ):
                meta = load_metadata(exp["dir"])
                df = load_timeseries(exp["dir"])
                _show_experiment_detail(exp, df, meta)

    # ── 对比 ───────────────────────────────────────────────────
    st.divider()
    st.subheader("🔬 对比实验")
    if len(filtered) >= 2:
        exp_labels = {f"{e['mtime'].strftime('%H:%M')} {e['name']}": e for e in filtered}
        col1, col2 = st.columns(2)
        with col1:
            sel_a = st.selectbox("参照 (A)", list(exp_labels.keys()), index=0, key="cmp_a")
        with col2:
            sel_b = st.selectbox("对比 (B)", list(exp_labels.keys()),
                                 index=min(1, len(exp_labels)-1), key="cmp_b")

        if sel_a and sel_b and sel_a != sel_b:
            a, b = exp_labels[sel_a], exp_labels[sel_b]
            ma, mb = load_metadata(a["dir"]), load_metadata(b["dir"])
            da, db = load_timeseries(a["dir"]), load_timeseries(b["dir"])

            if da is not None and db is not None and "farm_power_MW" in da.columns and "farm_power_MW" in db.columns:
                pA, pB = da["farm_power_MW"].mean(), db["farm_power_MW"].mean()
                gain = (pB / pA - 1) * 100

                cA, cB, cG = st.columns(3)
                cA.metric("A 平均", f"{pA:.3f} MW", f"{ma.get('controller','?')} / {ma.get('farm','?')}")
                cB.metric("B 平均", f"{pB:.3f} MW", f"{mb.get('controller','?')} / {mb.get('farm','?')}")
                cG.metric("增益", f"{gain:+.2f}%",
                          delta_color="inverse" if gain < 0 else "normal")

                # 叠加曲线
                n = min(len(da), len(db))
                chart = pd.DataFrame({
                    "time_s": da["time_s"][:n],
                    f"A: {a['name']}": da["farm_power_MW"].values[:n],
                    f"B: {b['name']}": db["farm_power_MW"].values[:n],
                })
                st.line_chart(chart.set_index("time_s"), height=200)

            # 一键对比报告
            if st.button("📄 生成完整对比报告", use_container_width=True, key="cmp_report"):
                out_dir = COMPARISONS_DIR / f"cmp-{a['name']}-vs-{b['name']}"
                out_dir.mkdir(parents=True, exist_ok=True)
                proc = subprocess.run(
                    ["python", "-m", "wfcrl.analysis.cli",
                     str(a["dir"]), str(b["dir"]),
                     "--baseline", a["name"],
                     "--output", str(out_dir), "--allow-incompatible"],
                    capture_output=True, text=True, timeout=60,
                )
                if proc.returncode == 0:
                    st.success("报告已生成")
                    for img in ["farm_power.png", "metrics.png"]:
                        p = out_dir / img
                        if p.exists():
                            st.image(str(p))
                else:
                    st.error(proc.stderr[:300])
    else:
        st.info("至少需要2个实验才能对比")


# ══════════════════════════════════════════════════════════════════════════
# 主入口
# ══════════════════════════════════════════════════════════════════════════

def main():
    render_sidebar()

    page_map = {
        "farm_config": page_farm_config,
        "wind_config": page_wind_config,
        "controller_mgr": page_controller_mgr,
        "run": page_run,
        "history": page_history,
        "wake_viz": page_wake_viz,
    }

    current = st.session_state.get("page", "wizard")
    page_fn = page_map.get(current, page_wizard)
    page_fn()


if __name__ == "__main__":
    EXPERIMENTS_DIR.mkdir(parents=True, exist_ok=True)
    COMPARISONS_DIR.mkdir(parents=True, exist_ok=True)
    main()
