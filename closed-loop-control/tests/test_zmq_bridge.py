"""test_zmq_bridge.py — ZmqFarmBridge 单元测试（偏航动作指令直通 / 换算 / 诱导直通）。"""
from __future__ import annotations

import math

from closedloop.bridge import ZmqFarmBridge
from closedloop.types import Cmd


class FakeEngine:
    """记录下发命令的假引擎（不启动 FAST.Farm）。"""
    def __init__(self, state=None):
        self.yaw_calls = []
        self.yaw_target_calls = []
        self.ind_calls = []
        self.state = state or self._state2()

    @staticmethod
    def _state2():
        return {
            'sim_time_s': 6.0, 'fast_step': 600, 'low_step': 2,
            'turbines': [_t(1, 1.0), _t(2, 0.0)],
        }

    def get_state(self, timeout_s=1.0):
        return self.state

    def send_yaw_delta(self, tid, delta_rad, seq=None, ttl_s=60.0):
        self.yaw_calls.append((tid, delta_rad, seq))

    def send_yaw_target(self, tid, target_rad, seq=None, ttl_s=60.0):
        self.yaw_target_calls.append((tid, target_rad, seq))

    def send_induction(self, tid, ref, seq=None, ttl_s=5.0):
        self.ind_calls.append((tid, ref, seq))

    _proc = None
    config = type('C', (), {'max_iter': 100})()


def _t(tid, heading):
    return {'turbine_id': tid, 'valid': 1, 'source_time_s': 6.0,
            'gen_power_w': 2.5e6, 'gen_speed_rad_s': 120.0, 'rotor_speed_rad_s': 1.2,
            'hub_wind_speed_mps': 8.0, 'nacelle_heading_rad': heading,
            'blade_pitch_rad': [0.0, 0.0, 0.0], 'rotor_thrust_n': 4.0e5}


def test_yaw_delta_direct_passthrough():
    eng = FakeEngine()
    br = ZmqFarmBridge(eng, 2)
    br.write(1, {1: Cmd(turbine_id=1, yaw_delta_deg=10.0)})
    assert len(eng.yaw_calls) == 1
    tid, delta_rad, seq = eng.yaw_calls[0]
    assert tid == 1 and seq == 1
    assert abs(delta_rad - math.radians(10.0)) < 1e-12


def test_yaw_deg_absolute_target_direct():
    """yaw_deg（绝对目标）→ send_yaw_target 直传（ICD v1.5 S0108 正式通道）。

    旧实现把绝对目标换算成增量再经 send_yaw_delta 回传；清理后直接走绝对目标，
    不再依赖机舱方位缓存做增量往返。
    """
    eng = FakeEngine()
    br = ZmqFarmBridge(eng, 2)
    br.write(1, {1: Cmd(turbine_id=1, yaw_deg=math.degrees(1.0 - 0.05))})
    assert len(eng.yaw_target_calls) == 1
    tid, target_rad, seq = eng.yaw_target_calls[0]
    assert tid == 1 and seq == 1
    # 绝对目标直传：target = 1.0 - 0.05 rad（无增量往返）
    assert abs(target_rad - (1.0 - 0.05)) < 1e-9
    # 增量路径仍走 send_yaw_delta（CW+ 动作语义，客户端内换算）
    eng2 = FakeEngine()
    br2 = ZmqFarmBridge(eng2, 2)
    br2.write(1, {1: Cmd(turbine_id=1, yaw_delta_deg=10.0)})
    assert len(eng2.yaw_calls) == 1
    assert abs(eng2.yaw_calls[0][1] - math.radians(10.0)) < 1e-12


def test_induction_direct():
    eng = FakeEngine()
    br = ZmqFarmBridge(eng, 2)
    br.write(1, {1: Cmd(turbine_id=1, induction_a=0.2)})
    assert len(eng.ind_calls) == 1
    assert abs(eng.ind_calls[0][1] - 0.2) < 1e-12
    assert eng.ind_calls[0][2] == 1


def test_greedy_induction_not_sent():
    eng = FakeEngine()
    br = ZmqFarmBridge(eng, 2)
    br.write(1, {1: Cmd(turbine_id=1, induction_a=1.0 / 3.0)})
    assert len(eng.ind_calls) == 0


def test_meas_from_state_units():
    eng = FakeEngine()
    br = ZmqFarmBridge(eng, 2)
    meas = br.read(1)
    m1 = meas[1]
    assert abs(m1.genpwr_kw - 2500.0) < 1.0
    assert abs(m1.wind_x - 8.0) < 1e-9
    assert abs(m1.nacyaw_deg - math.degrees(1.0)) < 1e-9
    assert m1.moop1_knm > 100.0  # 推力/1000 载荷代理