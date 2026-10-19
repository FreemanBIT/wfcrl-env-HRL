"""
farm_zmq_client.py — Farm Controller 侧 ZeroMQ 客户端（Phase 7）

拓扑：
   命令：本客户端 PUB -> runtime SUB (tcp://127.0.0.1:CMD_PORT)
   状态：runtime PUB -> 本客户端 SUB (tcp://127.0.0.1:STATE_PORT)

API：start / get_state / send_yaw_delta / send_induction / send_frame /
      stop / close。yaw 与 induction 各自独立 seq（自动递增，也可显式给定）。
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import struct
import zmq

from .farm_protocol import (
    FarmCommandFrame,
    FarmStateFrame,
    FcrFarmTurbineState,
    FCR_MAX_TURBINES,
    FCR_STATE_FRAME_SIZE,
    FCR_STATE_FRAME_HEADER_SIZE,
    FCR_TURBINE_STATE_SIZE,
    frame_to_bytes,
    new_command_frame,
    set_yaw_cmd,
    set_induction_cmd,
    yaw_delta_cw_to_absolute_ccw,
)

class FarmZmqClient:
    """ZeroMQ 客户端（命令 PUB + 状态 SUB）。"""

    def __init__(self, cmd_port: int = 5556, state_port: int = 5557,
                 n_turbines: int = 0, timeout_s: float = 2.0):
        self._cmd_port = cmd_port
        self._state_port = state_port
        self._n_turbines = n_turbines
        self._timeout_s = timeout_s
        self._ctx: Optional[zmq.Context] = None
        self._cmd_sock: Optional[zmq.Socket] = None
        self._state_sock: Optional[zmq.Socket] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._latest_state: Optional[bytes] = None
        self._turbine_slots: dict = {}
        self._last_heading: dict = {}      # tid -> 最新实际机舱方位 (rad, CCW+)
        self._last_state_sim_time = 0.0    # S1002: 最近状态帧采样仿真时刻（供 S0102）
        self._frame_header: Optional[bytes] = None
        self._state_lock = threading.Lock()
        self._frame_seq = 0
        self._yaw_seq = 0
        self._ind_seq = 0
        self._started = False

    # ---------------- 生命周期 ----------------
    @staticmethod
    def _bind_with_retry(sock, addr, tag, tries: int = 10, delay: float = 1.0) -> None:
        """bind 重试（Windows 端口释放延迟）"""
        import time
        for i in range(tries):
            try:
                sock.bind(addr)
                return
            except zmq.ZMQError:
                if i == tries - 1:
                    raise
                time.sleep(delay)
    def start(self) -> None:
        """绑定端口等待 FAST.Farm 各 WT 进程连接（命令 PUB bind / 状态 SUB bind）。"""
        if self._started:
            return
        self._ctx = zmq.Context()
        self._cmd_sock = self._ctx.socket(zmq.PUB)
        self._cmd_sock.setsockopt(zmq.LINGER, 0)
        self._bind_with_retry(self._cmd_sock, f"tcp://*:{self._cmd_port}", "cmd")
        self._state_sock = self._ctx.socket(zmq.SUB)
        self._state_sock.setsockopt(zmq.LINGER, 0)
        self._state_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._bind_with_retry(self._state_sock, f"tcp://*:{self._state_port}", "state")
        self._stop.clear()
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        self._started = True

    # ICD v1.5：全部由 farm_protocol.py 的 ctypes 镜像派生，禁止再硬编码。
    # （v1.4 遗留的 56/360/23096 与 36 偏移曾导致所有状态帧被静默丢弃）
    _FRAME_HEADER = FCR_STATE_FRAME_HEADER_SIZE        # FarmStateFrame.turbines 偏移 = 64
    _TURBINE_SIZE = FCR_TURBINE_STATE_SIZE             # FcrFarmTurbineState = 400
    _FRAME_SIZE = FCR_STATE_FRAME_SIZE                 # FarmStateFrame = 25664
    _N_TURBINES_OFF = FarmStateFrame.n_turbines.offset            # = 44
    _HEADING_OFF = FcrFarmTurbineState.nacelle_heading_actual_rad.offset  # = 232

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._state_sock.recv(copy=False)
            except zmq.ZMQError:
                time.sleep(0.05)
                continue
            if msg is None or len(msg) == 0:
                continue
            raw = bytes(msg)
            if len(raw) != self._FRAME_SIZE:
                continue
            with self._state_lock:
                self._latest_state = raw
                self._frame_header = raw[: self._FRAME_HEADER]
                # S1002: 记录采样仿真时刻，供 S0102 command_basis 使用（G-4 禁墙钟）
                self._last_state_sim_time = struct.unpack_from(
                    "<d", raw, FarmStateFrame.state_sample_sim_time_s.offset)[0]
                for i in range(FCR_MAX_TURBINES):
                    base = self._FRAME_HEADER + i * self._TURBINE_SIZE
                    tid = int.from_bytes(raw[base : base + 4], "little", signed=True)
                    valid = int.from_bytes(raw[base + 4 : base + 8], "little")
                    if tid >= 1 and valid:
                        self._turbine_slots[tid] = raw[base : base + self._TURBINE_SIZE]
                        # 记录该机最新实际机舱方位（CCW+），供 yaw 增量→绝对目标换算
                        self._last_heading[tid] = struct.unpack_from(
                            "<d", raw, base + self._HEADING_OFF)[0]
            self._recv_thread = None
        for s in (self._state_sock, self._cmd_sock):
            if s is not None:
                s.close(linger=0)
        if self._ctx is not None:
            self._ctx.term()
        self._ctx = None
        self._state_sock = None
        self._cmd_sock = None
        self._started = False

    def stop(self) -> None:
        """停止接收线程与 socket（可重 start）。"""
        self._stop.set()
        if self._recv_thread is not None:
            self._recv_thread.join(timeout=1.0)
            self._recv_thread = None
        for s in (self._state_sock, self._cmd_sock):
            if s is not None:
                s.close(linger=0)
        if self._ctx is not None:
            self._ctx.term()
        self._ctx = None
        self._state_sock = None
        self._cmd_sock = None
        self._recv_thread = None
        self._started = False

    def close(self) -> None:
        """销毁客户端（不可再 start）。"""
        self.stop()


    def _merged_frame(self) -> Optional[bytes]:
        if self._frame_header is None or not self._turbine_slots:
            return self._latest_state
        buf = bytearray(self._frame_header)
        # FarmStateFrame.n_turbines（v1.5 偏移 44；由镜像派生，勿硬编码）
        buf[self._N_TURBINES_OFF : self._N_TURBINES_OFF + 4] = \
            int(max(self._turbine_slots)).to_bytes(4, "little")
        for i in range(FCR_MAX_TURBINES):
            tid = i + 1
            if tid in self._turbine_slots:
                buf.extend(self._turbine_slots[tid])
            else:
                buf.extend(b"\x00" * self._TURBINE_SIZE)
        return bytes(buf)
    # ---------------- 状态 ----------------
    def get_state(self, timeout_s: Optional[float] = None) -> Optional[bytes]:
        """跨帧合并后的 FarmStateFrame 原始字节（各机槽位取各自最新发布）。"""
        t0 = time.time()
        timeout = timeout_s if timeout_s is not None else self._timeout_s
        while timeout is None or time.time() - t0 < timeout:
            with self._state_lock:
                m = self._merged_frame()
                if m is not None:
                    return m
            time.sleep(0.02)
        with self._state_lock:
            return self._merged_frame()

    # ---------------- 命令 ----------------
    def _fill_command_basis(self, frame: FarmCommandFrame) -> None:
        """S0102（ICD v1.5 G-4）：命令依据时刻必须是**仿真时刻**，不得用墙钟。

        取最近一次状态帧的 state_sample_sim_time_s。尚未收到任何状态帧时保持
        0.0，由 ZMQ 传输层 WIN-05 校验拒收该帧——基于未知状态的指令本就无意义。
        这与 runtime 侧 command_store 的"命令超龄"判定（sim_time - basis >
        stale_threshold）在同一时间轴上，故墙钟替换会静默废掉该保护。
        """
        basis = self._last_state_sim_time
        if basis <= 0.0:
            return
        n = min(int(frame.n_turbines), FCR_MAX_TURBINES)
        for i in range(n):
            t = frame.turbines[i]
            if t.command_basis_sim_time_s <= 0.0:
                t.command_basis_sim_time_s = basis

    def _send_frame(self, frame: FarmCommandFrame) -> None:
        if not self._started or self._cmd_sock is None:
            raise RuntimeError("client not started")
        self._frame_seq += 1
        frame.command_frame_seq = self._frame_seq   # ICD v1.5 S0101（commit_seq 已删除）
        self._fill_command_basis(frame)
        self._cmd_sock.send(frame_to_bytes(frame))

    def send_yaw_delta(self, turbine_id: int, delta_rad: float,
                       seq: Optional[int] = None, ttl_s: float = 60.0,
                       heading_now_rad: Optional[float] = None) -> None:
        """异步下发单机偏航**增量**（CW+，动作指令语义；全场帧按槽位领取）。

        ICD v1.5 线协议只承载**绝对目标**（S0108, CCW+），因此这里在客户端
        把增量换算为绝对目标：
            target = wrap_pm_pi(heading_now - delta)
        与 bridge/yaw_adapter.c 的 fcr_yaw_adapter_delta_to_target 严格等价。

        heading_now_rad 缺省取最近一次收到的该机实际机舱方位；若尚未收到任何
        状态帧则按 0.0（机舱初始正对风，与基线场景 NacYaw=0 一致）。
        """
        if seq is None:
            self._yaw_seq += 1
            seq = self._yaw_seq
        if heading_now_rad is None:
            heading_now_rad = self._last_heading.get(turbine_id, 0.0)
        target_rad = yaw_delta_cw_to_absolute_ccw(delta_rad, heading_now_rad)
        nt = self._n_turbines or turbine_id
        frame = new_command_frame(nt, self._frame_seq + 1)
        set_yaw_cmd(frame, turbine_id, target_rad, seq, ttl_s)
        self._send_frame(frame)

    def send_yaw_target(self, turbine_id: int, target_heading_rad: float,
                        seq: Optional[int] = None, ttl_s: float = 60.0) -> None:
        """异步下发单机偏航**绝对目标**（CCW+，ICD v1.5 S0108 正式通道）。"""
        if seq is None:
            self._yaw_seq += 1
            seq = self._yaw_seq
        nt = self._n_turbines or turbine_id
        frame = new_command_frame(nt, self._frame_seq + 1)
        set_yaw_cmd(frame, turbine_id, target_heading_rad, seq, ttl_s)
        self._send_frame(frame)

    def send_induction(self, turbine_id: int, induction_ref: float,
                       seq: Optional[int] = None, ttl_s: float = 5.0) -> None:
        """异步下发单机诱导因子（全场帧；induction 通道独立 seq）"""
        if seq is None:
            self._ind_seq += 1
            seq = self._ind_seq
        nt = self._n_turbines or turbine_id
        frame = new_command_frame(nt, self._frame_seq + 1)
        set_induction_cmd(frame, turbine_id, induction_ref, seq, ttl_s)
        self._send_frame(frame)

    def send_command_frame(self, frame: FarmCommandFrame) -> None:
        """下发整帧（高级用法：多机/多通道）"""
        self._send_frame(frame)

    # ---------------- 诊断 ----------------
    @property
    def started(self) -> bool:
        return self._started

    @property
    def stats(self) -> dict:
        return {"frame_seq": self._frame_seq,
                "yaw_seq": self._yaw_seq,
                "ind_seq": self._ind_seq}

    def __enter__(self) -> "FarmZmqClient":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

__all__ = ["FarmZmqClient"]