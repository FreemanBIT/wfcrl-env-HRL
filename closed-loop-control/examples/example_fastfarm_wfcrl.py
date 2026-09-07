"""Example 3 —（新架构）通过 FastFarmZmqEngine（ZeroMQ）驱动 FAST.Farm 的 closedloop 闭环。

与 example_fastfarm_bridge.py 等价；保留旧脚本参数名（--fstf 不再适用，
布局由 --case 指定）。使用 wfcrl.transport（FarmZmqClient）-> 集成 DLL 内
farm_control_runtime -> ROSCO 的完整场控链路。
"""
import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

def main():
    p = argparse.ArgumentParser(description='ZeroMQ 场控闭环（新架构）')
    p.add_argument('--controller', default='B', choices=['greedy', 'A', 'B', 'C', 'a', 'b', 'c'])
    p.add_argument('--mode', type=int, default=1, choices=[1, 2, 3, 4, 5])
    p.add_argument('--steps', type=int, default=60)
    p.add_argument('--dt', type=float, default=3.0)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--case', default='3T')
    p.add_argument('--wind_speed', type=float, default=8.0)
    p.add_argument('--wind_direction', type=float, default=270.0)
    p.add_argument('--fstf', default=None, help='（兼容保留，忽略）')
    p.add_argument('--floris-yaml', default='cases/farm_2x3_4D/floris_case.yaml')
    args = p.parse_args()

    from closedloop.zmq_pipeline import build_farm_case, launch_farm, wait_first_state
    from closedloop.surrogate import SurrogateModel
    from closedloop.sensing import Sensing
    from closedloop.base_controller import CommandArbiter
    from closedloop.runner import ClosedLoopRunner
    from closedloop.evaluate import Evaluator
    from closedloop.control_mode import ControlMode

    case = build_farm_case(args.case, args.wind_speed, args.wind_direction)
    out_dir = os.path.join(Path(__file__).resolve().parents[1], '__simul__', 'fastfarm',
                           'p8_zmq', '%s_wfcrl_%d' % (case.name, int(time.time())))
    surrogate = SurrogateModel(case.layout_x, case.layout_y,
                               case_yaml=args.floris_yaml, prefer_floris=True)
    sensing = Sensing(case.n_turbines, dt=args.dt, upstream_ids=case.upstream_ids,
                      wind_direction_ref=case.wind_direction)

    m = ControlMode(args.mode)
    name = args.controller.lower()
    if name == 'greedy':
        from closedloop.demos.greedy_baseline import GreedyController
        controller = GreedyController(case.n_turbines, args.dt)
    elif name == 'a':
        from closedloop.scheme_a import ControllerA
        controller = ControllerA(m, surrogate, case.n_turbines)
    elif name == 'b':
        from closedloop.scheme_b import ControllerB
        controller = ControllerB(m, surrogate, case.layout_x, case.layout_y,
                                 case.n_turbines, seed=args.seed)
    elif name == 'c':
        from closedloop.scheme_c import ControllerC
        controller = ControllerC(m, surrogate, case.layout_x, case.layout_y,
                                 case.n_turbines, seed=args.seed)
    else:
        raise ValueError(args.controller)

    engine, bridge, plant = launch_farm(case, args.dt, args.steps, out_dir)
    try:
        wait_first_state(engine, timeout_s=90.0)
        runner = ClosedLoopRunner(plant, controller, sensing, CommandArbiter(yaw_deadband=0.5))
        print('Controller=%s mode=%d steps=%d dt=%.1fs' % (args.controller, args.mode,
                                                           args.steps, args.dt))
        result = runner.run(args.steps, controller_name=args.controller, mode=args.mode,
                            warmup_steps=int(30 / args.dt), verbose=True)
        ev = Evaluator(transient_s=30.0)
        traj = result.trajectory
        print('Done. Mean farm power: %.1f kW over %d steps' %
              (ev.mean_farm_power(traj), result.n_steps))
    finally:
        engine.stop()


if __name__ == '__main__':
    main()
