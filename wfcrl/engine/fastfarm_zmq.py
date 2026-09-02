"""
fastfarm_zmq.py — 离线 ZeroMQ 主闭环的仿真引擎（Phase 7）

职责：
  1) 生成 FAST.Farm case 并部署 FCR 集成 DLL（复用 wfcrl case 生成）；
  2) 启动 FAST.Farm 子进程（FCR 集成 DLL 自动开启 ZMQ transport，端口由
     环境变量 FCR_ZMQ_CMD_PORT/FCR_ZMQ_STATE_PORT 配置）；
  3) 提供 get_state()（解析 FarmStateFrame）与 send_yaw_delta/send_induction；
  4) stop/close 优雅退出。

彻底取消 controls.txt 依赖（FCR 模式下 DISCON 不再读写该文件）。
"""
from __future__ import annotations

import os
import subprocess
import time
import warnings
from typing import Dict, List, Optional

import numpy as np

from wfcrl.config import WindConfig
from wfcrl.config.simulator import FastFarmConfig
from wfcrl.engine import ContinuousFastFarmInterface
from wfcrl.engine.angle_utils import misalignment_from_nacyaw
from wfcrl.transport import FarmZmqClient
from wfcrl.transport.farm_protocol import bytes_to_frame

# 状态帧解析辅助（C FcrFarmTurbineState 布局；见 fcr_state_types.h）
# 简化：只按固定偏移读取必需字段（f64 对齐）

def _parse_state_frame(data: bytes) -> dict:
    """解析 FarmStateFrame → dict（Time/fast_step/low_step/每机关键量）。"""
    import ctypes
    class Stats(ctypes.Structure):
        _fields_ = [("instant", ctypes.c_double), ("mean", ctypes.c_double),
                    ("rms", ctypes.c_double), ("min", ctypes.c_double),
                    ("max", ctypes.c_double), ("n_samples", ctypes.c_uint32),
                    ("valid", ctypes.c_uint32)]
    class Turb(ctypes.Structure):
        _fields_ = [
            ("turbine_id", ctypes.c_int32), ("valid", ctypes.c_uint32),
            ("source_time_s", ctypes.c_double),
            ("gen_power_w", Stats), ("rotor_thrust_n", Stats),
            ("tower_base_fa_moment_nm", Stats), ("tower_base_ss_moment_nm", Stats),
            ("nacelle_heading_rad", ctypes.c_double), ("nacelle_vane_rad", ctypes.c_double),
            ("hub_wind_speed_mps", ctypes.c_double), ("rotor_speed_rad_s", ctypes.c_double),
            ("gen_speed_rad_s", ctypes.c_double), ("rotor_ct", ctypes.c_double),
            ("rotor_cp", ctypes.c_double), ("blade_pitch_rad", ctypes.c_double * 3),
            ("yaw_target_heading_rad", ctypes.c_double),
            ("yaw_seq_applied", ctypes.c_uint64), ("induction_seq_applied", ctypes.c_uint64),
            ("controller_status_flags", ctypes.c_uint32),
            ("disk_ambient_wind_mps", ctypes.c_double), ("disk_disturbed_wind_mps", ctypes.c_double),
            ("disk_ambient_ti", ctypes.c_double), ("flow_source_time_s", ctypes.c_double),
            ("flow_seq", ctypes.c_uint64),
        ]
    class Frame(ctypes.Structure):
        _fields_ = [
            ("protocol_version", ctypes.c_uint32), ("frame_seq", ctypes.c_uint32),
            ("sim_time_s", ctypes.c_double), ("fast_step", ctypes.c_uint64),
            ("low_step", ctypes.c_uint64), ("rt_health_flags", ctypes.c_uint32),
            ("n_turbines", ctypes.c_uint32), ("flow_source_time_s", ctypes.c_double),
            ("flow_seq", ctypes.c_uint64), ("turbines", Turb * 64),
        ]
    if len(data) < ctypes.sizeof(Frame):
        raise ValueError(f"short state frame {len(data)}")
    fr = Frame.from_buffer_copy(data[:ctypes.sizeof(Frame)])
    out = {
        "sim_time_s": fr.sim_time_s, "fast_step": fr.fast_step, "low_step": fr.low_step,
        "rt_health_flags": fr.rt_health_flags, "flow_seq": fr.flow_seq,
        "flow_source_time_s": fr.flow_source_time_s,
        "turbines": [],
    }
    for i in range(fr.n_turbines):
        t = fr.turbines[i]
        out["turbines"].append({
            "turbine_id": t.turbine_id, "valid": t.valid,
            "source_time_s": t.source_time_s,
            "gen_power_w_mean": t.gen_power_w.mean, "gen_power_w": t.gen_power_w.instant,
            "rotor_thrust_n": t.rotor_thrust_n.instant,
            "nacelle_heading_rad": t.nacelle_heading_rad,
            "hub_wind_speed_mps": t.hub_wind_speed_mps,
            "rotor_speed_rad_s": t.rotor_speed_rad_s,
            "yaw_target_heading_rad": t.yaw_target_heading_rad,
            "yaw_seq_applied": t.yaw_seq_applied, "induction_seq_applied": t.induction_seq_applied,
            "disk_disturbed_wind_mps": t.disk_disturbed_wind_mps,
            "disk_ambient_wind_mps": t.disk_ambient_wind_mps,
        })
    return out

class FastFarmZmqEngine:
    """ZeroMQ 离线主闭环引擎。"""

    def __init__(self, config: FastFarmConfig,
                 cmd_port: int = 5556, state_port: int = 5557,
                 libzmq_path: Optional[str] = None):
        self.config = config
        self.cmd_port = cmd_port
        self.state_port = state_port
        self._ff: Optional[ContinuousFastFarmInterface] = None
        self._proc: Optional[subprocess.Popen] = None
        self._client: Optional[FarmZmqClient] = None
        self._log = None
        if libzmq_path:
            os.environ["FCR_LIBZMQ_PATH"] = libzmq_path

    # ---------- 生命周期 ----------
    def setup(self) -> None:
        """生成 case、部署 FCR 集成 DLL。"""
        if self._ff is None:
            self._ff = ContinuousFastFarmInterface(self.config)
        self._ff.setup()
        self._ff.reset(self.config.wind)

    def start(self, env_extra: Optional[Dict[str, str]] = None) -> None:
        """启动 FAST.Farm（FCR 集成 DLL 内 ZMQ transport 自动 bind 端口）。"""
        if self._ff is None or self._ff._farm_base is None:
            raise RuntimeError("call setup() first")
        env = dict(os.environ)
        env["FCR_ZMQ_CMD_PORT"] = str(self.cmd_port)
        env["FCR_ZMQ_STATE_PORT"] = str(self.state_port)
        env.setdefault("FCR_CFG_DT_LOW", str(getattr(self.config, "dt", 3.0)))
        if env_extra:
            env.update(env_extra)
        log_path = os.path.join(self.config.output_dir or ".", "fastfarm_zmq.log")
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        self._log = open(log_path, "w", buffering=1)
        self._proc = subprocess.Popen(
            [self._ff._fastfarm_exe, os.path.basename(self._ff._fstf_file)],
            cwd=self._ff._farm_base, stdout=self._log, stderr=subprocess.STDOUT,
            env=env, text=True,
        )
        # 连接 ZMQ 客户端
        self._client = FarmZmqClient(cmd_port=self.cmd_port, state_port=self.state_port)
        self._client.start()
        print(f"FAST.Farm pid {self._proc.pid}, ZMQ cmd://:{self.cmd_port} state://:{self.state_port}")

    def get_state(self, timeout_s: float = 10.0) -> Optional[dict]:
        """最新聚合状态（解析后的 dict；None = 超时）"""
        if self._client is None:
            raise RuntimeError("not started")
        data = self._client.get_state(timeout_s=timeout_s)
        if data is None:
            return None
        try:
            return _parse_state_frame(data)
        except ValueError as e:
            warnings.warn(f"state parse: {e}")
            return None

    # ---------- 命令 ----------
    def send_yaw_delta(self, turbine_id: int, delta_rad: float,
                       seq: Optional[int] = None, ttl_s: float = 60.0) -> None:
        if self._client is None:
            raise RuntimeError("not started")
        self._client.send_yaw_delta(turbine_id, delta_rad, seq=seq, ttl_s=ttl_s)

    def send_induction(self, turbine_id: int, induction_ref: float,
                       seq: Optional[int] = None, ttl_s: float = 1.0) -> None:
        if self._client is None:
            raise RuntimeError("not started")
        self._client.send_induction(turbine_id, induction_ref, seq=seq, ttl_s=ttl_s)

    # ---------- 收尾 ----------
    def stop(self, timeout_s: float = 60) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None
        if self._proc is not None:
            try:
                self._proc.wait(timeout=timeout_s)
            except Exception:
                try:
                    self._proc.terminate()
                    self._proc.wait(timeout=10)
                except Exception:
                    pass
            self._proc = None
        if self._log is not None:
            self._log.close()
            self._log = None

    def close(self) -> None:
        self.stop()

__all__ = ["FastFarmZmqEngine", "_parse_state_frame"]
