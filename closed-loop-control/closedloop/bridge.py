"""
M2 — FAST.Farm file-bridge interface (the core integration layer).

Wraps the file protocol implemented by ``DISCON_bridge.f90``:

    write controls.txt              controller -> FAST.Farm
        step=<N>
        T1 yaw=<deg> pitch=<deg> torque=<Nm> power=<MW>
        ...
        END

    read measurements_T<id>.txt     FAST.Farm -> controller
        step=<N> t=<s> genpwr=<kW> genspd=<rpm> gentq=<Nm> rotspd=<rpm>
        wind_x=<m/s>
        blpitch=<deg> nacyaw=<deg>
        mip1=<kNm> moop1=<kNm> mzb1=<kNm>

Key correctness requirements (see plan §M2):
  * controls.txt must be written atomically (temp file + os.replace) so the
    Fortran bridge never reads a half-written line.
  * ``step`` must be strictly increasing; the bridge only applies a new file
    when read_step > applied_step.
  * measurement parsing must tolerate the instant the bridge is overwriting a
    file (retry on short/partial reads).
"""
from __future__ import annotations

import os
import re
import time
from pathlib import Path
from typing import Iterable, Optional

from .types import TurbineMeas, Cmd

# tokens like "genpwr=123.45" possibly run together without spaces; parse robustly
_NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"
_FIELD_RE = {
    "step": re.compile(r"step=(" + r"[-+]?\d+" + r")"),
    "t": re.compile(r"\bt=(" + _NUM + r")"),
    "genpwr": re.compile(r"genpwr=(" + _NUM + r")"),
    "genspd": re.compile(r"genspd=(" + _NUM + r")"),
    "gentq": re.compile(r"gentq=(" + _NUM + r")"),
    "rotspd": re.compile(r"rotspd=(" + _NUM + r")"),
    "wind_x": re.compile(r"wind_x=(" + _NUM + r")"),
    "blpitch": re.compile(r"blpitch=(" + _NUM + r")"),
    "nacyaw": re.compile(r"nacyaw=(" + _NUM + r")"),
    "mip1": re.compile(r"mip1=(" + _NUM + r")"),
    "moop1": re.compile(r"moop1=(" + _NUM + r")"),
    "mzb1": re.compile(r"mzb1=(" + _NUM + r")"),
}


class FarmBridge:
    """File-based bridge to a running FAST.Farm simulation.

    Parameters
    ----------
    run_dir:
        Directory in which FAST.Farm runs (where controls.txt and
        measurements_T*.txt live).
    n_turbines:
        Number of turbines (6 for the 2x3 layout).
    """

    def __init__(self, run_dir: os.PathLike | str, n_turbines: int):
        self.run_dir = Path(run_dir)
        self.n_turbines = int(n_turbines)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._last_step_written = -1

    # -- controls ---------------------------------------------------------------
    def write_controls(self, step: int, cmds: dict[int, Cmd]) -> None:
        """Atomically write ``controls.txt`` for the given step.

        ``step`` must be strictly greater than any previously written step, or
        the bridge will ignore the file.
        """
        if step <= self._last_step_written:
            raise ValueError(
                f"step must be strictly increasing; got {step} "
                f"after {self._last_step_written}"
            )
        lines = [f"step={step}"]
        for tid in range(1, self.n_turbines + 1):
            cmd = cmds.get(tid)
            if cmd is None:
                cmd = Cmd(turbine_id=tid)  # empty -> bridge keeps previous values
            lines.append(cmd.to_line())
        lines.append("END")
        text = "\n".join(lines) + "\n"

        target = self.run_dir / "controls.txt"
        tmp = self.run_dir / f".controls.txt.tmp.{os.getpid()}"
        tmp.write_text(text)
        os.replace(tmp, target)  # atomic on POSIX and Windows
        self._last_step_written = step

    # -- measurements -----------------------------------------------------------
    def read_measurements(
        self, retries: int = 5, retry_wait: float = 0.02
    ) -> dict[int, TurbineMeas]:
        """Read and parse ``measurements_T<id>.txt`` for every turbine.

        Returns a dict ``{turbine_id: TurbineMeas}``. Turbines whose file is not
        yet present are omitted. Short/partial reads (the bridge overwriting the
        file) are retried.
        """
        out: dict[int, TurbineMeas] = {}
        for tid in range(1, self.n_turbines + 1):
            meas = self._read_one(tid, retries, retry_wait)
            if meas is not None:
                out[tid] = meas
        return out

    def _read_one(
        self, tid: int, retries: int, retry_wait: float
    ) -> Optional[TurbineMeas]:
        path = self.run_dir / f"measurements_T{tid}.txt"
        for _ in range(retries):
            if not path.exists():
                time.sleep(retry_wait)
                continue
            try:
                text = path.read_text()
            except (OSError, UnicodeDecodeError):
                time.sleep(retry_wait)
                continue
            meas = self._parse(tid, text)
            if meas is not None:
                return meas
            time.sleep(retry_wait)
        return None

    @staticmethod
    def _parse(tid: int, text: str) -> Optional[TurbineMeas]:
        """Parse one measurement file's text into a TurbineMeas (or None if
        essential fields are missing / file is mid-write)."""
        vals: dict[str, float] = {}
        for key, rgx in _FIELD_RE.items():
            m = rgx.search(text)
            if m:
                try:
                    vals[key] = float(m.group(1))
                except ValueError:
                    return None
        # require at least step + power to consider the record usable
        if "step" not in vals or "genpwr" not in vals:
            return None
        return TurbineMeas(
            turbine_id=tid,
            step=int(vals.get("step", -1)),
            t=vals.get("t", 0.0),
            genpwr_kw=vals.get("genpwr", 0.0),
            genspd_rpm=vals.get("genspd", 0.0),
            gentq_nm=vals.get("gentq", 0.0),
            rotspd_rpm=vals.get("rotspd", 0.0),
            wind_x=vals.get("wind_x", 0.0),
            blpitch_deg=vals.get("blpitch", 0.0),
            nacyaw_deg=vals.get("nacyaw", 0.0),
            mip1_knm=vals.get("mip1", 0.0),
            moop1_knm=vals.get("moop1", 0.0),
            mzb1_knm=vals.get("mzb1", 0.0),
        )

    # -- synchronization --------------------------------------------------------
    def wait_step(
        self, step: int, timeout: float = 60.0, poll: float = 0.05
    ) -> dict[int, TurbineMeas]:
        """Block until every turbine reports measurements at ``>= step``.

        Returns the measurements once all turbines have advanced. Raises
        TimeoutError if the deadline passes first.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            meas = self.read_measurements()
            if len(meas) == self.n_turbines and all(
                m.step >= step for m in meas.values()
            ):
                return meas
            time.sleep(poll)
        raise TimeoutError(
            f"Timed out waiting for step {step} from {self.n_turbines} turbines"
        )

    def latest_step(self) -> int:
        """Return the minimum step currently reported across turbines (-1 if
        none)."""
        meas = self.read_measurements()
        if len(meas) < self.n_turbines:
            return -1
        return min(m.step for m in meas.values())

    # -- housekeeping -----------------------------------------------------------
    def reset(self) -> None:
        """Remove stale controls/measurement files from the run directory."""
        for p in [self.run_dir / "controls.txt"] + [
            self.run_dir / f"measurements_T{tid}.txt"
            for tid in range(1, self.n_turbines + 1)
        ]:
            try:
                p.unlink()
            except FileNotFoundError:
                pass
        self._last_step_written = -1


# ---------------------------------------------------------------------------
# ZeroMQ 新架构桥（Phase 7+）：farm_control_runtime <-> ClosedLoop 适配层
# ---------------------------------------------------------------------------
# 新架构通道（FAST.Farm 集成 DLL 内运行时）：
#   yaw:       yaw_delta_rad（CW+ 增量，弧度）+ seq（每机独立、单调）
#   induction: induction_ref（无量纲轴向诱导因子 a）+ seq
# 本层把旧 Cmd（绝对 yaw_deg / power_mw / pitch_deg / torque_nm）转换为
# 新通道命令；把 FarmStateFrame（engine.get_state() dict）转换为 TurbineMeas。
#
# 通道可用性（红线）：
#   * yaw_deg    -> yaw_delta（基于最近状态帧机舱方位换算绝对目标）
#   * power_mw   -> power_to_induction() -> induction_ref（首选降载通道）
#   * pitch_deg  -> 工程近似（Cp 比 -> a；与 MockPlant 同公式，标注为近似）
#   * torque_nm  -> 新架构无扭矩直接通道；忽略并告警（每条命令记一次）
#
# 不可得量说明（状态帧无此字段，填 0/占位并注明）：
#   * gentq_nm（发电机扭矩）——状态帧未包含；
#   * mip1_knm / mzb1_knm（叶根弯矩）——状态帧未包含；
#   * moop1_knm 以 rotor_thrust_n 的千牛值作为载荷代理（见 meas_from_state）。

import math
import time
import warnings as _warnings
from typing import Optional

from .types import TurbineMeas, Cmd
from .induction import power_to_induction, A_GREEDY


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
            if cmd.yaw_deg is not None:
                self._send_yaw(tid, i, cmd.yaw_deg)
            if cmd.power_mw is not None:
                self._send_power(tid, i, cmd.power_mw)
            elif cmd.pitch_deg is not None:
                self._send_pitch(tid, i, cmd.pitch_deg)
            elif cmd.torque_nm is not None:
                self._warn_once(tid, 'torque',
                                '新架构（ZeroMQ）无发电机扭矩直接通道，已忽略 torque 命令')

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