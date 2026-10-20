"""bridge.py — ZMQ 场控桥（ZmqFarmBridge）。
历史文件桥（FarmBridge）已移除（REFACTOR_PLAN 第 1 部分）。
"""
import math
import time
from typing import Optional

from .types import TurbineMeas, Cmd
from .induction import power_to_induction, A_GREEDY
from .runner import ZmqPlant


class ZmqFarmBridge:
    def __init__(self, engine, n_turbines: int, wind_direction: float = 270.0):
        self.engine = engine
        self.n = int(n_turbines)
        self.wind_direction = float(wind_direction)
        self._yaw_seq = [0] * self.n
        self._ind_seq = [0] * self.n

    # -- 状态读取 -------------------------------------------------------------
    def get_state(self, timeout_s: float = 15.0):
        st = self.engine.get_state(timeout_s=timeout_s)
        return st

    def read(self, step: int) -> dict:
        st = self.get_state()
        if st is None:
            raise TimeoutError('no state frame within timeout (bridged FAST.Farm)')
        return self.meas_from_state(st, self.n)

    def wait_step(self, step: int, timeout: float = 120.0, poll: float = 0.5) -> dict:
        deadline = time.time() + timeout
        while time.time() < deadline:
            st = self.get_state(timeout_s=poll)
            if st is not None and st.get('low_step', 0) >= step:
                meas = self.meas_from_state(st, self.n)
                if len(meas) >= self.n:
                    return meas
            # 仿真进程已退出
            proc = getattr(self.engine, '_proc', None)
            max_iter = getattr(getattr(self.engine, 'config', None), 'max_iter', 0)
            if proc is not None and proc.poll() is not None:
                last_low = st.get('low_step', 0) if st else 0
                done_ok = bool(max_iter) and last_low >= max_iter - 1
                if done_ok or (max_iter and step >= max_iter):
                    # 仿真正常到达终点（或 python 落后于仿真完成）：返回最近帧收尾
                    if st is not None:
                        meas = self.meas_from_state(st, self.n)
                        if len(meas) >= self.n:
                            return meas
                t_last = st.get('sim_time_s', 0.0) if st else 0.0
                raise RuntimeError(
                    'FAST.Farm 提前退出（最后仿真时间 t=%.1f s，low_step=%d，'
                    '目标 low_step=%d 未达到）——请检查 fastfarm_zmq.log'
                    % (t_last, last_low, step))
            time.sleep(poll)
        raise TimeoutError('timed out waiting for low_step >= %d' % step)

    def current_direction(self) -> float:
        return self.wind_direction

    def reset(self) -> None:
        self._yaw_seq = [0] * self.n
        self._ind_seq = [0] * self.n

    # -- 命令下发 -------------------------------------------------------------
    def write(self, step: int, cmds: dict) -> None:
        for tid, cmd in cmds.items():
            if not (1 <= tid <= self.n):
                continue
            i = tid - 1
            # 偏航：动作指令（CW+ 增量）直通；绝对目标为控制器内部表示，桥内换算
            if cmd.yaw_delta_deg is not None:
                self._send_yaw_delta(tid, i, cmd.yaw_delta_deg)
            elif cmd.yaw_deg is not None:
                self._send_yaw(tid, i, cmd.yaw_deg)
            # 诱导：induction_a 直通
            if cmd.induction_a is not None:
                self._send_induction(tid, i, cmd.induction_a)

    def _send_yaw_delta(self, tid: int, i: int, delta_deg: float) -> None:
        """偏航动作指令直通（CW+ 增量，deg -> rad）。SCADA/RCP 语义。"""
        self._yaw_seq[i] += 1
        self.engine.send_yaw_delta(tid, math.radians(delta_deg),
                                   seq=self._yaw_seq[i], ttl_s=60.0)

    def _send_induction(self, tid: int, i: int, a: float) -> None:
        """诱导因子直通（无量纲 a -> induction_ref）。"""
        if a is None or a >= A_GREEDY - 1e-3:
            return   # 满发（greedy）无需下令
        self._ind_seq[i] += 1
        self.engine.send_induction(tid, float(a), seq=self._ind_seq[i], ttl_s=30.0)

    def _send_yaw(self, tid: int, i: int, target_deg: float) -> None:
        """偏航绝对目标直通（ICD v1.5 S0108 正式通道，CCW+）。

        持有绝对机舱方位时走 send_yaw_target，不再做"绝对→增量→再换算绝对"
        的往返（旧实现经 send_yaw_delta 由客户端换算）。
        """
        self._yaw_seq[i] += 1
        self.engine.send_yaw_target(tid, math.radians(target_deg),
                                    seq=self._yaw_seq[i], ttl_s=60.0)

    # -- 状态转换 -------------------------------------------------------------

    @staticmethod
    def meas_from_state(state: dict, n_turbines: int) -> dict:
        tstep = int(state.get('low_step', -1))   # 控制步（low-resolution step）
        tsim = float(state.get('sim_time_s', 0.0))
        turbs = state.get('turbines', [])
        out = {}
        for tid in range(1, n_turbines + 1):
            if tid > len(turbs):
                continue
            t = turbs[tid - 1] or {}
            valid = bool(t.get('valid'))
            p_w = float(t.get('gen_power_w', 0.0) or 0.0)
            gen_rads = float(t.get('gen_speed_rad_s', 0.0) or 0.0)
            rot_rads = float(t.get('rotor_speed_rad_s', 0.0) or 0.0)
            pitch_deg = 0.0
            bp = t.get('blade_pitch_rad')
            if bp:
                pitch_deg = math.degrees(sum(bp) / max(1, len(bp)))
            out[tid] = TurbineMeas(
                turbine_id=tid,
                step=tstep,
                t=tsim,
                genpwr_kw=p_w / 1e3 if valid else 0.0,
                genspd_rpm=gen_rads * 30.0 / math.pi,
                gentq_nm=0.0,
                rotspd_rpm=rot_rads * 30.0 / math.pi,
                wind_x=float(t.get('hub_wind_speed_mps', 0.0) or 0.0),
                blpitch_deg=pitch_deg,
                nacyaw_deg=math.degrees(float(t.get('nacelle_heading_rad', 0.0) or 0.0)),
                mip1_knm=0.0,
                moop1_knm=float(t.get('rotor_thrust_n', 0.0) or 0.0) / 1e3,
                mzb1_knm=0.0,
            )
        return out


__all__ = ['ZmqFarmBridge']


class FastFarmZmqPlant(ZmqPlant):
    """新架构 Plant：直接包装 FastFarmZmqEngine（ZeroMQ 通道）。

    用法：
        from wfcrl.engine.fastfarm_zmq import FastFarmZmqEngine
        engine = FastFarmZmqEngine(config)
        engine.setup(); engine.start()
        plant = FastFarmZmqPlant(engine, n_turbines=6, wind_direction=270.0)
        runner = ClosedLoopRunner(plant, controller, sensing)
    """

    def __init__(self, engine, n_turbines: int, wind_direction: float = 270.0,
                 step_timeout: float = 120.0):
        bridge = ZmqFarmBridge(engine, n_turbines, wind_direction=wind_direction)
        super().__init__(bridge, step_timeout=step_timeout)
        self.engine = engine