"""
简单偏航控制器 Demo — 专为 FAST.Farm 设计
==========================================
控制逻辑：上游风机（第1行）以固定偏航角偏转尾流，
          下游风机（第2行）保持零偏航。

可直接运行:  python custom_controllers/demo_yaw_controller.py
"""

import numpy as np
import time
import os
from pathlib import Path

# ── 控制器参数 ──────────────────────────────────────────────────────────
YAW_ANGLE = -17.0        # 偏航角 (°)，负值 = 尾流向下偏转
WIND_SPEED = 8.0         # 风速 (m/s)
WIND_DIR = 270.0         # 风向 (°)
DT = 3.0                 # 时间步长 (s) = FAST.Farm DT_low
N_STEPS = 30             # 仿真步数
LAYOUT = "6T"            # 布局

# 项目根目录
PROJECT_ROOT = Path(__file__).resolve().parent.parent


def build_and_run():
    """构建 FAST.Farm 案例并运行闭环控制。"""
    from wfcrl.config.simulator import FastFarmConfig
    from wfcrl.config.types import WindConfig, WindType
    from wfcrl.config.control import ControlInput
    from wfcrl.engine.fastfarm_continuous import ContinuousFastFarmInterface
    from wfcrl.config.layout import LayoutRegistry

    # ── 1. 获取布局 ──────────────────────────────────────────────────
    layouts = LayoutRegistry.from_builtin()
    layout = layouts.get(LAYOUT)
    n_turbines = layout.num_turbines
    print(f"布局: {LAYOUT} ({n_turbines}台风机)")
    print(f"坐标:")
    for i in range(n_turbines):
        print(f"  T{i+1}: x={layout.xcoords[i]:.0f}, y={layout.ycoords[i]:.0f}")

    # ── 2. 配置 FAST.Farm ────────────────────────────────────────────
    timestamp = int(time.time())
    output_dir = str(PROJECT_ROOT / "__simul__" / "demo_ctrl" / f"run_{timestamp}")

    config = FastFarmConfig(
        case_name=f"demo-yaw-ctrl",
        num_turbines=n_turbines,
        xcoords=list(layout.xcoords),
        ycoords=list(layout.ycoords),
        dt=DT,
        max_iter=N_STEPS,
        wind=WindConfig(wind_type=WindType.STEADY, speed=WIND_SPEED,
                        direction=WIND_DIR, turbulence_intensity=0.06),
        output_dir=output_dir,
    )

    # ── 3. 构建仿真器 ────────────────────────────────────────────────
    print(f"\n初始化 FAST.Farm 接口...")
    ff = ContinuousFastFarmInterface(config)
    ff.setup()
    ff.reset(config.wind)
    print(f"案例目录: {ff._farm_base}")

    # ── 4. 控制器逻辑 ────────────────────────────────────────────────
    # 判断上游风机：x坐标最小的那一行
    x_coords = np.array(layout.xcoords)
    min_x = x_coords.min()
    upstream_mask = x_coords == min_x  # 上游风机 = True
    print(f"\n控制器: 上游风机偏航 {YAW_ANGLE}°")
    print(f"  上游风机索引: {np.where(upstream_mask)[0].tolist()}")

    # ── 5. 闭环运行 ──────────────────────────────────────────────────
    print(f"\n启动 FAST.Farm ({N_STEPS} 步, {DT}s/步)...")
    ff.start()

    results = []
    aborted = False

    try:
        for step in range(1, N_STEPS + 1):
            # 检查 FAST.Farm 进程
            if ff._process is not None and ff._process.poll() is not None:
                print(f"\n⚠️ FAST.Farm 异常退出 (rc={ff._process.returncode})")
                aborted = True
                break

            # ── 控制器计算 ────────────────────────────────────────────
            yaw_cmd = np.zeros(n_turbines)
            yaw_cmd[upstream_mask] = YAW_ANGLE  # 上游偏航

            # 构造控制指令
            controls = ControlInput(
                mode=np.full(n_turbines, 0, dtype=np.int32),  # mode 0 = 纯偏航
                yaw=yaw_cmd.astype(np.float64),
            )

            # ── 发送指令并等待测量 ────────────────────────────────────
            try:
                output = ff.wait_step(controls)
            except Exception as e:
                print(f"\n⚠️ 通信丢失 (step {step}): {e}")
                aborted = True
                break

            # ── 记录结果 ──────────────────────────────────────────────
            farm_power = float(output.farm_power_mw[0]) if output.farm_power_mw is not None else 0
            results.append({
                "step": step,
                "time_s": step * DT,
                "farm_power_MW": farm_power,
            })

            if step % max(1, N_STEPS // 10) == 0:
                print(f"  step {step:3d}/{N_STEPS}  P_farm={farm_power:.1f} MW"
                      f"  yaw={yaw_cmd[0]:.0f}°")

    finally:
        ff.stop()
        ff.close()

    # ── 6. 结果分析 ──────────────────────────────────────────────────
    print(f"\n{'='*50}")
    print(f"  实验结果")
    print(f"{'='*50}")
    if results:
        powers = [r["farm_power_MW"] for r in results]
        # 去掉前几步瞬态
        steady = powers[len(powers)//2:]
        print(f"  总步数: {len(results)}")
        print(f"  平均场功率 (稳态): {np.mean(steady):.2f} MW")
        print(f"  场功率范围: {min(powers):.2f} ~ {max(powers):.2f} MW")
        print(f"  输出目录: {output_dir}")

        # 保存 CSV
        import csv
        csv_path = Path(output_dir) / "demo_results.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["step", "time_s", "farm_power_MW"])
            w.writeheader()
            w.writerows(results)
        print(f"  结果已保存: {csv_path}")

    print(f"{'='*50}")
    return results


if __name__ == "__main__":
    results = build_and_run()
