"""
FAST.Farm ZeroMQ 场控示例（新架构，3 模式双通道）
====================================================
通过 FastFarmZmqEngine + farm_control_runtime（集成 DLL 内）下发：

    mode 1  YAW           仅偏航（正弦扫描 yaw_delta 动作指令）
    mode 2  INDUCTION     仅诱导（下发诱导因子 a --a）
    mode 3  YAW_INDUCTION 偏航 + 诱导（双通道，默认）

通道语义：偏航=动作指令（CW+ 增量，deg）；诱导=无量纲 a（1/3 贪婪）。

用法：
    python examples/example_FASTFarm_zmq.py --mode 1 --steps 12 --wind_speed 8
    python examples/example_FASTFarm_zmq.py --mode 2 --a 0.20 --steps 12
    python examples/example_FASTFarm_zmq.py --mode 3 --a 0.20 --steps 12
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "closed-loop-control"))

import numpy as np

from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine
from wfcrl.config.layout import LayoutRegistry
from wfcrl.config import WindConfig
from wfcrl.config.simulator import FastFarmConfig


def main():
    p = argparse.ArgumentParser(description="FAST.Farm ZeroMQ 场控示例（3 模式）")
    p.add_argument("--case", default="6T")
    p.add_argument("--steps", type=int, default=12)
    p.add_argument("--wind_speed", type=float, default=8.0)
    p.add_argument("--wind_direction", type=float, default=270.0)
    p.add_argument("--mode", type=int, default=3, choices=[1, 2, 3])
    p.add_argument("--a", type=float, default=0.20, help="诱导因子（mode 2/3）")
    p.add_argument("--yaw_amp", type=float, default=15.0, help="偏航动作幅值 deg（mode 1/3）")
    p.add_argument("--yaw_period", type=int, default=6)
    p.add_argument("--wind_time_series", default=None,
                   help="TurbSim .bts（建议 300s+ 使用，如 inflow_08ms_TI10_s8D.bts）")
    p.add_argument("--cmd_port", type=int, default=5556)
    p.add_argument("--state_port", type=int, default=5557)
    args = p.parse_args()

    reg = LayoutRegistry.from_builtin()
    if args.case in reg:
        layout = reg.get(args.case)
    else:
        import re
        if (m := re.match(r"Turb(\d+)_Row\d+$", args.case)):
            layout = reg.get_row_layout(int(m.group(1)))
        else:
            print("Unknown case", args.case, reg.list_names())
            sys.exit(1)
    n = layout.num_turbines
    print("Layout: %s (%d turbines)" % (layout.name, n))

    from wfcrl.config.types import WindType
    if args.wind_time_series:
        wind = WindConfig(speed=args.wind_speed, direction=args.wind_direction,
                          wind_type=WindType.TURBSIM_BTS, wind_file=args.wind_time_series)
    else:
        wind = WindConfig(speed=args.wind_speed, direction=args.wind_direction)
    ts = time.time()
    out_dir = os.path.join(os.path.dirname(__file__), "..", "__simul__", "fastfarm",
                           "zmq", "%s_Mode%d_%d" % (layout.name, args.mode, ts))
    config = FastFarmConfig(case_name=layout.name, num_turbines=n,
                            xcoords=layout.xcoords, ycoords=layout.ycoords,
                            dt=3.0, max_iter=args.steps, wind=wind, output_dir=out_dir)

    engine = FastFarmZmqEngine(config, cmd_port=args.cmd_port, state_port=args.state_port)
    engine.setup()
    engine.start()

    yaw_seq = [0] * n
    ind_seq = [0] * n
    row_powers = []
    try:
        st = None
        t0 = time.time()
        while time.time() - t0 < 90:
            st = engine.get_state(timeout_s=5.0)
            if st and st.get("fast_step", 0) > 0:
                break
        if st is None:
            raise TimeoutError("no first state frame")

        from closedloop.types import Cmd
        from closedloop.bridge import ZmqFarmBridge
        br = ZmqFarmBridge(engine, n, wind_direction=args.wind_direction)
        br.get_state()

        for step in range(args.steps):
            cmds = {}
            for tid in range(1, n + 1):
                if args.mode in (1, 3):
                    offset = args.yaw_amp * math.sin(2 * math.pi * step / args.yaw_period)
                    offset = max(-25.0, min(25.0, offset))
                    cmds[tid] = Cmd(turbine_id=tid, yaw_delta_deg=offset)
                if args.mode in (2, 3):
                    if cmds.get(tid) is None:
                        cmds[tid] = Cmd(turbine_id=tid)
                    cmds[tid].induction_a = args.a
            br.write(step + 1, cmds)
            st = engine.get_state(timeout_s=10.0)
            if st is None:
                raise TimeoutError("state timeout at step %d" % step)
            turbs = st["turbines"]
            farm_kw = sum(t.get("gen_power_w", 0.0) for t in turbs) / 1e3
            row_powers.append(farm_kw)
            hdg = math.degrees(turbs[0].get("nacelle_heading_rad", 0.0))
            print("Step %3d/%d | farm P=%.1f kW | T1 hdg=%.1f deg" %
                  (step + 1, args.steps, farm_kw, hdg))
        print()
        print("Mean farm power: %.1f kW over %d steps" %
              (float(np.mean(row_powers)) if row_powers else 0.0, len(row_powers)))
        np.savetxt(os.path.join(out_dir, "farm_power_kw.csv"), np.asarray(row_powers),
                   fmt="%.2f", header="farm_power_kw")
        print("Output dir:", out_dir)
    finally:
        engine.stop()


if __name__ == "__main__":
    main()
