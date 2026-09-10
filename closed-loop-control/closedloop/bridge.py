"""bridge.py — ZMQ 场控桥（ZmqFarmBridge）。
历史文件桥（FarmBridge）已移除（REFACTOR_PLAN 第 1 部分）。
"""
import math
import time
import warnings as _warnings
from typing import Optional

from .types import TurbineMeas, Cmd
from .induction import power_to_induction, A_GREEDY
from .runner import ZmqPlant




def _wrap_pi_rad(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def _norm180_deg(x: float) -> float:
    return math.degrees(_wrap_pi_rad(math.radians(x)))


def _induction_from_cp_ratio(ratio: float) -> float:
    from .induction import cp_from_induction
    ratio = max(0.0, min(1.0, ratio))
    cp_target = ratio * (16.0 / 27.0)
    lo, hi = 0.0, A_GREEDY
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if cp_from_induction(mid) < cp_target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


class ZmqFarmBridge:
    def __init__(self, engine, n_turbines: int, wind_direction: float = 270.0):
        self.engine = engine
        self.n = int(n_turbines)
        self.wind_direction = float(wind_direction)
        self._yaw_seq = [0] * self.n
        self._ind_seq = [0] * self.n
        self._last_state = None
        self._heading_cache = {}   # tid -> 上次有效机舱方位（deg）
        self._warned = set()

    # -- 状态读取 -------------------------------------------------------------
    def get_state(self, timeout_s: float = 15.0):
        st = self.engine.get_state(timeout_s=timeout_s)
        if st is not None:
            self._last_state = st
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
        self._warned = set()
        self._heading_cache = {}

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
            # 诱导：induction_a 直通；power/pitch 为兼容入口
            if cmd.induction_a is not None:
                self._send_induction(tid, i, cmd.induction_a)
            elif cmd.power_mw is not None:
                self._send_power(tid, i, cmd.power_mw)
            elif cmd.pitch_deg is not None:
                self._send_pitch(tid, i, cmd.pitch_deg)
            elif cmd.torque_nm is not None:
                self._warn_once(tid, 'torque',
                                '新架构（ZeroMQ）无发电机扭矩直接通道，已忽略 torque 命令')

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
        hdg = self._heading_deg(tid)
        if hdg is None:
            self._warn_once(tid, 'nohdg',
                            '尚无状态帧提供 T%d 机舱方位，本次 yaw 命令未发送' % tid)
            return
        delta_deg = _norm180_deg(target_deg - hdg)
        self._yaw_seq[i] += 1
        self.engine.send_yaw_delta(tid, math.radians(delta_deg),
                                   seq=self._yaw_seq[i], ttl_s=60.0)

    def _send_power(self, tid: int, i: int, power_mw: float) -> None:
        st = self._last_state
        U = 8.0
        yaw_deg = 0.0
        if st is not None and tid <= len(st.get('turbines', [])):
            t = st['turbines'][tid - 1] or {}
            if t.get('hub_wind_speed_mps'):
                U = t['hub_wind_speed_mps']
            yaw_deg = math.degrees(float(t.get('nacelle_heading_rad', 0.0) or 0.0))
        a = power_to_induction(max(power_mw, 0.0) * 1e6, max(U, 0.5), yaw_deg=yaw_deg)
        if a >= A_GREEDY - 1e-3:
            return   # 满发（greedy）无需下令
        self._ind_seq[i] += 1
        self.engine.send_induction(tid, a, seq=self._ind_seq[i], ttl_s=30.0)

    def _send_pitch(self, tid: int, i: int, pitch_deg: float) -> None:
        ratio = max(0.0, min(1.0, 1.0 - pitch_deg * 0.04))
        a = _induction_from_cp_ratio(ratio)
        self._ind_seq[i] += 1
        self.engine.send_induction(tid, a, seq=self._ind_seq[i], ttl_s=30.0)

    def _warn_once(self, tid: int, key: str, msg: str) -> None:
        k = (tid, key)
        if k not in self._warned:
            _warnings.warn(msg, UserWarning, stacklevel=3)
            self._warned.add(k)

    # -- 状态转换 -------------------------------------------------------------
    def _heading_deg(self, tid: int):
        st = self._last_state
        if st is not None and tid <= len(st.get('turbines', [])):
            t = st['turbines'][tid - 1] or {}
            if t.get('valid'):
                h = t.get('nacelle_heading_rad', float('nan'))
                if h == h and abs(h) <= 2.0 * math.pi:  # 合理性防御（含合并帧瞬时垃圾）
                    deg = math.degrees(h)
                    self._heading_cache[tid] = deg
                    return deg
        # 当前帧无效时回退到上次有效方位（合并帧瞬时缺失容忍）
        return self._heading_cache.get(tid)

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