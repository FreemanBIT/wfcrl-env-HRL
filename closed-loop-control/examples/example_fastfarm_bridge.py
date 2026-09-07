"""Example 2 (new architecture) — ZeroMQ 闭环：启动 FAST.Farm（FCR 集成 DLL）+ closedloop 控制器。

新架构（Phase 7+）：不再使用 controls.txt / measurements_T*.txt 文件桥；
命令经 ZeroMQ（yaw_delta / induction_ref），状态经 FarmStateFrame 回读。

Usage:
    python examples/example_fastfarm_bridge.py --controller B --mode 1 --steps 60 \
        --case 3T --wind_speed 8 --wind_direction 270
"""
import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from closedloop.case_config import default_case
from closedloop.surrogate import SurrogateModel
from closedloop.sensing import Sensing
from closedloop.base_controller import CommandArbiter
from closedloop.runner import ClosedLoopRunner
from closedloop.evaluate import Evaluator
from closedloop.control_mode import ControlMode
from closedloop.zmq_pipeline import build_farm_case, launch_farm, wait_first_state


def build_controller(name, mode, case, surrogate, dt, seed):
    m = ControlMode(mode)
    name = name.lower()
    if name == 'greedy':
        from closedloop.demos.greedy_baseline import GreedyController
        return GreedyController(case.n_turbines, dt)
    if name == 'a':
        from closedloop.scheme_a import ControllerA
        return ControllerA(m, surrogate, case.n_turbines)
    if name == 'b':
        from closedloop.scheme_b import ControllerB
        return ControllerB(m, surrogate, case.layout_x, case.layout_y,
                           case.n_turbines, seed=seed)
    if name == 'c':
        from closedloop.scheme_c import ControllerC
        return ControllerC(m, surrogate, case.layout_x, case.layout_y,
                           case.n_turbines, seed=seed)
    raise ValueError(name)


def main():
    p = argparse.ArgumentParser(description='ZeroMQ 场控闭环（新架构）')
    p.add_argument('--controller', default='B', choices=['greedy', 'A', 'B', 'C', 'a', 'b', 'c'])
    p.add_argument('--mode', type=int, default=1, choices=[1, 2, 3, 4, 5])
    p.add_argument('--steps', type=int, default=60, help='控制步数（= FAST.Farm 低速步数）')
    p.add_argument('--dt', type=float, default=3.0, help='FAST.Farm DT_low（秒）')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--case', default='3T', help='wfcrl 内置布局名（如 3T/6T/Turb3_Row1）')
    p.add_argument('--wind_speed', type=float, default=8.0)
    p.add_argument('--wind_direction', type=float, default=270.0)
    p.add_argument('--floris-yaml', default='cases/farm_2x3_4D/floris_case.yaml')
    p.add_argument('--cmd-port', type=int, default=5556)
    p.add_argument('--state-port', type=int, default=5557)
    args = p.parse_args()

    case = build_farm_case(args.case, args.wind_speed, args.wind_direction)
    out_dir = os.path.join(Path(__file__).resolve().parents[1], '__simul__', 'fastfarm',
                           'p8_zmq', '%s_ctrl%s_%d' % (case.name, args.controller, int(time.time())))

    surrogate = SurrogateModel(case.layout_x, case.layout_y,
                               case_yaml=args.floris_yaml, prefer_floris=True)
    print('Surrogate backend:', surrogate.backend)

    sensing = Sensing(case.n_turbines, dt=args.dt, upstream_ids=case.upstream_ids,
                      wind_direction_ref=case.wind_direction)
    controller = build_controller(args.controller, args.mode, case, surrogate,
                                  args.dt, args.seed)

    engine, bridge, plant = launch_farm(case, args.dt, args.steps, out_dir,
                                        cmd_port=args.cmd_port, state_port=args.state_port)
    try:
        wait_first_state(engine, timeout_s=90.0)
        runner = ClosedLoopRunner(plant, controller, sensing,
                                  CommandArbiter(yaw_deadband=0.5))
        print('Controller=%s mode=%d steps=%d dt=%.1fs (FAST.Farm pid %s)' %
              (args.controller, args.mode, args.steps, args.dt,
               engine._proc.pid if engine._proc else '?'))
        transient = min(30.0, 0.25 * args.steps * args.dt)
        result = runner.run(args.steps, controller_name=args.controller,
                            mode=args.mode, warmup_steps=int(min(0.3 * args.steps * args.dt, 30.0) / args.dt),
                            verbose=True)
        ev = Evaluator(transient_s=min(30.0, 0.25 * args.steps * args.dt))
        traj = result.trajectory
        print()
        print('Done. Mean farm power (post-transient): %.1f kW over %d steps' %
              (ev.mean_farm_power(traj), result.n_steps))
        # 保存轨迹 CSV
        csv_path = os.path.join(out_dir, 'trajectory.csv')
        with open(csv_path, 'w') as f:
            traj.to_csv(f) if hasattr(traj, 'to_csv') else None
        print('Trajectory: %s' % csv_path)
    finally:
        engine.stop()


if __name__ == '__main__':
    main()