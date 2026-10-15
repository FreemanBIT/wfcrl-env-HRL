"""Example 1 —（新架构）一键启动 FAST.Farm（FCR ZeroMQ 集成 DLL）+ closedloop 控制器闭环。

Usage:
    python examples/example_fastfarm_launcher.py --duration 300 --controller A --mode 1 --wind_time_series inflow_08ms_TI10_s8D.bts
    python examples/example_fastfarm_launcher.py --duration 90 --controller greedy --step 3 --wind_time_series inflow_08ms_TI10_s8D.bts

等价于 example_fastfarm_bridge.py，但按 --duration 换算控制步数，并在结束后
输出功率时序与轨迹图。
"""
import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

def main():
    p = argparse.ArgumentParser(description='ZeroMQ 场控闭环（新架构，一键启动）')
    p.add_argument('--duration', type=float, default=300.0, help='仿真时长（秒）')
    p.add_argument('--controller', default='B', choices=['greedy', 'A', 'B', 'C', 'a', 'b', 'c'])
    p.add_argument('--mode', type=int, default=3, choices=[1, 2, 3],
                   help='1=YAW 仅偏航；2=INDUCTION 仅诱导；3=YAW_INDUCTION 双通道（默认）')
    p.add_argument('--dt', type=float, default=3.0, help='FAST.Farm DT_low（秒）')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--case', default='6T')
    p.add_argument('--wind_speed', type=float, default=8.0)
    p.add_argument('--wind_direction', type=float, default=270.0)
    p.add_argument('--floris-yaml', default='cases/farm_2x3_4D/floris_case.yaml')
    p.add_argument('--mpc_t_ctrl', type=float, default=None,
                   help='ControllerB 的 MPC 重规划周期（s），默认 20s')
    p.add_argument('--wind_time_series', default=None,
                   help='TurbSim .bts 文件名（相对 wfcrl FastFarm input 模板目录）。'
                        '稳态(WindType=1)在 FAST.Farm 5.0 有 Grid4D 时间维限制（~260s），'
                        '建议 300s 以上仿真使用 bts（如 uniform8_700s.bts 周期性均匀场）')
    p.add_argument('--plot', action='store_true', help='结束后绘制功率时序图')
    args = p.parse_args()

    steps = max(3, int(round(args.duration / args.dt)))
    from closedloop.zmq_pipeline import build_farm_case, launch_farm, wait_first_state
    from closedloop.surrogate import SurrogateModel
    from closedloop.sensing import Sensing
    from closedloop.base_controller import CommandArbiter
    from closedloop.runner import ClosedLoopRunner
    from closedloop.evaluate import Evaluator
    from closedloop.control_mode import ControlMode

    case = build_farm_case(args.case, args.wind_speed, args.wind_direction)
    out_dir = os.path.join(Path(__file__).resolve().parents[1], '__simul__', 'fastfarm',
                           'p8_zmq', '%s_%s_%d' % (case.name, args.controller, int(time.time())))

    surrogate = SurrogateModel(case.layout_x, case.layout_y,
                               case_yaml=args.floris_yaml, prefer_floris=True)
    print('Surrogate backend:', surrogate.backend)

    from closedloop.controller_factory import build_controller
    controller = build_controller(args.controller, args.mode, case, surrogate,
                                  args.dt, seed=args.seed, mpc_t_ctrl=args.mpc_t_ctrl)

    sensing = Sensing(case.n_turbines, dt=args.dt, upstream_ids=case.upstream_ids,
                      wind_direction_ref=case.wind_direction)

    engine, bridge, plant = launch_farm(case, args.dt, steps, out_dir,
                                         wind_time_series=args.wind_time_series)
    try:
        wait_first_state(engine, timeout_s=90.0)
        runner = ClosedLoopRunner(plant, controller, sensing, CommandArbiter(yaw_deadband=0.5))
        print('Launch OK: %s, %d turbines, %d steps @ dt=%.1fs' %
              (case.name, case.n_turbines, steps, args.dt))
        t0 = time.time()
        transient = min(30.0, 0.25 * args.duration)
        result = runner.run(steps, controller_name=args.controller, mode=args.mode,
                            warmup_steps=int(min(0.3 * args.duration, 30.0) / args.dt),
                            verbose=True)
        wall = time.time() - t0
        ev = Evaluator(transient_s=min(30.0, 0.25 * args.duration))
        traj = result.trajectory
        mean_p = ev.mean_farm_power(traj)
        print('Done in %.1f s wall. Mean farm power (post-transient): %.1f kW over %d steps' %
              (wall, mean_p, result.n_steps))
        csv_path = os.path.join(out_dir, 'trajectory.csv')
        import numpy as np
        np.savetxt(csv_path, np.column_stack((traj.t, traj.power)) if hasattr(traj, 't') else [],
                   header='t,P_t1,..', fmt='%.4f' if False else '%g')
        print('Trajectory saved: %s' % csv_path)
        if args.plot and hasattr(traj, 't') and len(traj.t) > 0:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(12, 4))
            for i in range(min(6, len(traj.power[0]) if traj.power else 0)):
                ax.plot(traj.t, [r[i] for r in traj.power], label='T%d' % (i + 1))
            ax.set_xlabel('t (s)'); ax.set_ylabel('P (kW)'); ax.legend(); ax.grid(alpha=0.3)
            png = os.path.join(out_dir, 'power.png')
            fig.savefig(png, dpi=130, bbox_inches='tight')
            print('Plot saved: %s' % png)
    finally:
        engine.stop()


if __name__ == '__main__':
    main()