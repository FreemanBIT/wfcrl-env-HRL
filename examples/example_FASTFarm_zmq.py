"""
FAST.Farm ZeroMQ 场控示例（新架构）— 直接通道版
======================================================
与旧 5-mode 文件桥示例同源的模式，通过新架构通道下发（Step D 收敛为 3 模式前暂保留）：
    wfcrl.engine.fastfarm_zmq.FastFarmZmqEngine + farm_control_runtime（集成 DLL 内）

通道映射：
    Mode 0: 偏航 —— yaw_delta（绝对目标经方位差换算为 CW+ 增量）
    Mode 1: 功率目标 —— power_to_induction -> induction_ref（min_pitch 忽略并提示）
    Mode 2: 变桨绝对值 —— Cp 比近似 -> induction_ref（新架构无直连变桨通道）
    Mode 3: 变桨 + 偏航  同上两通道
    Mode 4: 功率 + 偏航  同上两通道

Usage:
    python examples/example_FASTFarm_zmq.py --mode 0 --steps 12 --wind_speed 8
    python examples/example_FASTFarm_zmq.py --mode 1 --power 3.0 --steps 12
"""
import argparse
import math
import os
import sys
import time
import re

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'closed-loop-control'))

import numpy as np

from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine
from wfcrl.config.layout import LayoutRegistry
from wfcrl.config import WindConfig
from wfcrl.config.simulator import FastFarmConfig

from closedloop.induction import power_to_induction, A_GREEDY


def _wrap_pi(x):
    while x > math.pi:
        x -= 2 * math.pi
    while x < -math.pi:
        x += 2 * math.pi
    return x


def _norm180(deg):
    return math.degrees(_wrap_pi(math.radians(deg)))


def main():
    p = argparse.ArgumentParser(description='FAST.Farm ZeroMQ 场控示例（新架构）')
    p.add_argument('--case', default='6T')
    p.add_argument('--steps', type=int, default=12)
    p.add_argument('--wind_speed', type=float, default=8.0)
    p.add_argument('--wind_direction', type=float, default=270.0)
    p.add_argument('--mode', type=int, default=0, choices=[0, 1, 2, 3, 4])
    p.add_argument('--power', type=float, default=3.0, help='功率目标 MW（mode 1/4）')
    p.add_argument('--min_pitch', type=float, default=0.0, help='（兼容保留；新架构忽略并提示）')
    p.add_argument('--pitch', type=float, default=5.0, help='变桨命令 deg（mode 2/3）')
    p.add_argument('--yaw_amp', type=float, default=15.0, help='偏航幅值 deg（mode 0/3/4）')
    p.add_argument('--yaw_period', type=int, default=6)
    p.add_argument('--cmd_port', type=int, default=5556)
    p.add_argument('--state_port', type=int, default=5557)
    args = p.parse_args()

    reg = LayoutRegistry.from_builtin()
    if args.case in reg:
        layout = reg.get(args.case)
    elif (m := re.match(r'Turb(\d+)_Row\d+$', args.case)):
        layout = reg.get_row_layout(int(m.group(1)))
    else:
        print('Unknown case %r. Available: %s' % (args.case, reg.list_names()))
        sys.exit(1)
    n = layout.num_turbines
    print('Layout: %s (%d turbines)' % (layout.name, n))


    wind = WindConfig(speed=args.wind_speed, direction=args.wind_direction)
    ts = time.time()
    out_dir = os.path.join(os.path.dirname(__file__), '..', '__simul__', 'fastfarm',
                           'zmq', '%s_Mode%d_%d' % (layout.name, args.mode, ts))
    config = FastFarmConfig(case_name=layout.name, num_turbines=n,
                            xcoords=layout.xcoords, ycoords=layout.ycoords,
                            dt=3.0, max_iter=args.steps, wind=wind, output_dir=out_dir)

    if args.min_pitch > 0 and args.mode in (1, 4):
        print('[warn] 新架构无独立 min_pitch 通道——以 induction（功率->a）执行；min_pitch 由 RT 侧 ROSCO PC_MinPit 承担')

    engine = FastFarmZmqEngine(config, cmd_port=args.cmd_port, state_port=args.state_port)
    engine.setup()
    engine.start()

    yaw_seq = [0] * n
    ind_seq = [0] * n
    optimal_yaw = (270 - args.wind_direction) % 360
    row_powers = []
    try:
        st = None
        t0 = time.time()
        while time.time() - t0 < 90:
            st = engine.get_state(timeout_s=5.0)
            if st and st.get('fast_step', 0) > 0:
                break
        if st is None:
            raise TimeoutError('no first state frame')

        for step in range(args.steps):
            st = engine.get_state(timeout_s=10.0)
            if st is None:
                raise TimeoutError('state timeout at step %d' % step)
            turbs = st['turbines']
            # --- 生成并发送命令 ---
            for tid in range(1, n + 1):
                t = turbs[tid - 1]
                hdg_rad = t.get('nacelle_heading_rad', 0.0)
                if args.mode in (0, 3, 4):
                    yaw_off = args.yaw_amp * math.sin(2 * math.pi * step / args.yaw_period)
                    yaw_off = max(-25.0, min(25.0, yaw_off))
                    target_deg = optimal_yaw + yaw_off
                    delta_deg = _norm180(target_deg - math.degrees(hdg_rad))
                    yaw_seq[tid - 1] += 1
                    engine.send_yaw_delta(tid, math.radians(delta_deg),
                                          seq=yaw_seq[tid - 1], ttl_s=60.0)
                if args.mode in (1, 4):
                    U = t.get('hub_wind_speed_mps', args.wind_speed) or args.wind_speed
                    a = power_to_induction(args.power * 1e6, U,
                                           yaw_deg=math.degrees(hdg_rad))
                    if a < A_GREEDY - 1e-3:
                        ind_seq[tid - 1] += 1
                        engine.send_induction(tid, a, seq=ind_seq[tid - 1], ttl_s=30.0)
                if args.mode in (2, 3):
                    pitch = args.pitch * (1.0 + 0.5 * math.sin(2 * math.pi * step / 10))
                    pitch = max(0.0, min(30.0, pitch))
                    ratio = max(0.0, min(1.0, 1.0 - pitch * 0.04))
                    # Cp 比 -> a（与 bridge 的近似一致）
                    from closedloop.induction import cp_from_induction
                    cp_t = ratio * (16.0 / 27.0)
                    lo, hi = 0.0, A_GREEDY
                    for _ in range(40):
                        mid = 0.5 * (lo + hi)
                        if cp_from_induction(mid) < cp_t:
                            lo = mid
                        else:
                            hi = mid
                    a_mid = 0.5 * (lo + hi)
                    ind_seq[tid - 1] += 1
                    engine.send_induction(tid, a_mid, seq=ind_seq[tid - 1], ttl_s=30.0)

            time.sleep(args.dt if False else 1.0)   # 等待状态更新（约 3 s 一帧）
            farm_kw = sum(t.get('gen_power_w', 0.0) for t in turbs) / 1e3
            row_powers.append(farm_kw)
            print('Step %3d/%d | farm P=%.1f kW | T1 hdg=%.1f deg' %
                  (step + 1, args.steps, farm_kw, math.degrees(turbs[0].get('nacelle_heading_rad', 0.0))))

        print()
        print('Mean farm power: %.1f kW over %d steps' %
              (float(np.mean(row_powers)) if row_powers else 0.0, len(row_powers)))
        np.savetxt(os.path.join(out_dir, 'farm_power_kw.csv'), np.asarray(row_powers),
                   fmt='%.2f', header='farm_power_kw')
        print('Output dir: %s' % out_dir)
    finally:
        engine.stop()


if __name__ == '__main__':
    main()