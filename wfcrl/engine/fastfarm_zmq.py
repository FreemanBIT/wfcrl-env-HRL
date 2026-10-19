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
from wfcrl.transport.farm_protocol import bytes_to_frame, state_frame_from_bytes

# 状态帧解析辅助（C FcrFarmTurbineState 布局；见 fcr_state_types.h）
# 简化：只按固定偏移读取必需字段（f64 对齐）

def _parse_state_frame(data: bytes) -> dict:
    """解析 FarmStateFrame → dict（Time/fast_step/low_step/每机关键量）。

    ICD v1.5：帧几何与字段全部取自 wfcrl.transport.farm_protocol 的权威镜像
    （该镜像已由 tools/check_abi.py 与 C 侧 offsetof 逐字段交叉校验）。

    历史缺陷：本文件曾自带一份 v1.4 ctypes 镜像，把 n_turbines 读在偏移 36
    （v1.5 实际为 44），于是恒读到 low_step 的高 4 字节 = 0 → turbines 恒为空
    列表 → 全部集成测试在 st["turbines"][id-1] 处 IndexError。此处不再重复定义。
    """
    fr = state_frame_from_bytes(data)
    out = {
        "sim_time_s": fr.state_sample_sim_time_s,
        "state_sample_sim_time_s": fr.state_sample_sim_time_s,   # ICD v1.5 S1002
        "state_frame_seq": fr.state_frame_seq,                   # ICD v1.5 S1001
        "fast_step": fr.fast_step, "low_step": fr.low_step,
        "rt_health_flags": fr.rt_health_flags, "flow_seq": fr.flow_seq,
        "flow_source_time_s": fr.flow_source_time_s,
        "turbines": [],
    }
    for i in range(fr.n_turbines):
        t = fr.turbines[i]
        out["turbines"].append({
            "turbine_id": t.turbine_id, "valid": t.valid,
            "source_time_s": t.source_time_s,
            "gen_power_w": t.gen_power_w.instant,
            "gen_power_w_mean": t.gen_power_w.mean,
            "gen_power_w_rms": t.gen_power_w.rms,
            "gen_power_w_min": t.gen_power_w.min,
            "gen_power_w_max": t.gen_power_w.max,
            # ICD v1.5 改名 S1010；保留旧键 rotor_thrust_n 兼容既有消费方
            "rotor_thrust_n": t.rotor_aero_thrust_n.instant,
            "rotor_thrust_n_mean": t.rotor_aero_thrust_n.mean,
            "rotor_aero_thrust_n": t.rotor_aero_thrust_n.instant,
            "tower_base_fa_moment_nm": t.tower_base_fa_moment_nm.instant,
            "tower_base_ss_moment_nm": t.tower_base_ss_moment_nm.instant,
            # ICD v1.5 改名 S1009；保留旧键 nacelle_heading_rad 兼容既有消费方
            "nacelle_heading_rad": t.nacelle_heading_actual_rad,
            "nacelle_heading_actual_rad": t.nacelle_heading_actual_rad,
            "nacelle_vane_rad": t.nacelle_vane_rad,
            "hub_wind_speed_mps": t.hub_wind_speed_mps,
            "rotor_speed_rad_s": t.rotor_speed_rad_s,
            "gen_speed_rad_s": t.gen_speed_rad_s,
            "rotor_ct": t.rotor_ct,
            "rotor_cp": t.rotor_cp,
            "collective_pitch_actual_rad": t.collective_pitch_actual_rad,   # S1008
            "blade_pitch_rad": [float(t.blade_pitch_rad[k]) for k in range(3)],
            "yaw_target_heading_rad": t.yaw_target_heading_rad,
            "yaw_seq_applied": t.yaw_seq_applied, "induction_seq_applied": t.induction_seq_applied,
            "controller_status_flags": t.controller_status_flags,
            "turbine_status_flags": t.turbine_status_flags,                 # S1015
            "estimated_wind_mps": t.estimated_wind_mps,                     # S1011
            "wind_source": t.wind_source,                                   # S1016
            "wind_quality_flags": t.wind_quality_flags,                     # S1017
            "ct_hat": t.ct_hat,                                             # S1012
            "disk_disturbed_wind_mps": t.disk_disturbed_wind_mps,
            "disk_ambient_wind_mps": t.disk_ambient_wind_mps,
            "disk_ambient_ti": t.disk_ambient_ti,
            "flow_source_time_s": t.flow_source_time_s,
            "flow_seq": t.flow_seq,
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
        # FCR 偏航命令路径要求 YawDOF=True（Phase 3 惯例）
        try:
            import glob
            from wfcrl.engine._outlist import _set_fast_scalar as _sfs
            for ed in glob.glob(os.path.join(self._ff._farm_base, "*ElastoDyn*.dat")):
                try:
                    _sfs(ed, "YawDOF", "True")
                except Exception:
                    pass
        except Exception:
            pass
        # 集成 DLL 从运行目录动态加载 libzmq.dll（FCR_LIBZMQ_PATH 可覆盖）
        try:
            import shutil
            _root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
            _zmq_bin = os.path.join(_root, "farm_control_runtime", "transport", "zmq",
                                    "bin", "libzmq.dll")
            if os.path.exists(_zmq_bin) and not os.environ.get("FCR_LIBZMQ_PATH"):
                shutil.copy(_zmq_bin, os.path.join(self._ff._farm_base, "libzmq.dll"))
        except Exception:
            pass
        # 仿真时长 = 用户时长（dt*max_iter）+ 一个低步。
        # 注：Grid4D 时间维与 Case.fstf TMax 无关（实测抬到 390/850s 均无法推迟
        # ~260s 崩溃点，最终以 bts 周期场路线解决），故不再加 90s 平流余量——
        # 仿真精确在用户时长后不久结束，engine.stop() 可立即收尾；
        # 此前 +90s 会让 Done 后进程多跑约 40s（6T_A_1789028169 实测 390s）。
        try:
            import glob as _glob
            _tm = float(getattr(self.config, "dt", 3.0)) * int(getattr(self.config, "max_iter", 0))
            _dt = float(getattr(self.config, "dt", 3.0))
            _tm_new = _tm + _dt   # 用户时长 + 一个低步（保证 low_step=max_iter 状态帧落盘）
            for _fst in [os.path.join(self._ff._farm_base, "Case.fstf")] + _glob.glob(os.path.join(self._ff._farm_base, "FFTest_WT*.fst")):
                try:
                    _txt = open(_fst, "r", encoding="ascii", errors="replace").read()
                    _lines = _txt.splitlines()
                    import re as _re
                    for _li, _ln in enumerate(_lines):
                        if _re.match(r"^\s*\d+[\d.]*\s+TMax\s", _ln):
                            _lines[_li] = _re.sub(r"^\s*\d+[\d.]*", f"{_tm_new:10.2f}", _ln)
                            break
                    open(_fst, "w", encoding="ascii").write("\n".join(_lines) + "\n")
                except Exception:
                    pass
        except Exception:
            pass
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
        # 连接 ZMQ 客户端（bind 模式；各 WT 进程 connect）
        n_t = getattr(self.config, "num_turbines", 0) or 0
        self._client = FarmZmqClient(cmd_port=self.cmd_port, state_port=self.state_port,
                                     n_turbines=n_t)
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
                       seq: Optional[int] = None, ttl_s: float = 5.0) -> None:
        if self._client is None:
            raise RuntimeError("not started")
        self._client.send_induction(turbine_id, induction_ref, seq=seq, ttl_s=ttl_s)

    # ---------- 收尾 ----------
    def stop(self, timeout_s: float = 60) -> None:
        # 先终止仿真进程再关闭 ZMQ client（避免对端连接重置触发 pyzmq assert abort）
        if self._proc is not None:
            try:
                self._proc.wait(timeout=timeout_s)
            except Exception:
                try:
                    self._proc.terminate()
                    self._proc.wait(timeout=15)
                except Exception:
                    pass
            self._proc = None
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None
        if self._log is not None:
            try:
                self._log.close()
            except Exception:
                pass
            self._log = None

    def close(self) -> None:
        self.stop()

__all__ = ["FastFarmZmqEngine", "_parse_state_frame"]