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

import zmq

from .farm_protocol import (
    FarmCommandFrame,
    frame_to_bytes,
    new_command_frame,
    set_yaw_cmd,
    set_induction_cmd,
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

    _FRAME_HEADER = 64
    _TURBINE_SIZE = 360

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
            if len(raw) != 23096:
                continue
            with self._state_lock:
                self._latest_state = raw
                self._frame_header = raw[: self._FRAME_HEADER]
                for i in range(64):
                    base = self._FRAME_HEADER + i * self._TURBINE_SIZE
                    tid = int.from_bytes(raw[base : base + 4], "little", signed=True)
                    valid = int.from_bytes(raw[base + 4 : base + 8], "little")
                    if tid >= 1 and valid:
                        self._turbine_slots[tid] = raw[base : base + self._TURBINE_SIZE]
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
        # FarmStateFrame.n_turbines 在偏移 36（u32）
        buf[36:40] = int(max(self._turbine_slots)).to_bytes(4, "little")
        for i in range(64):
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
    def _send_frame(self, frame: FarmCommandFrame) -> None:
        if not self._started or self._cmd_sock is None:
            raise RuntimeError("client not started")
        self._frame_seq += 1
        frame.frame_seq = self._frame_seq
        frame.commit_seq = self._frame_seq
        self._cmd_sock.send(frame_to_bytes(frame))

    def send_yaw_delta(self, turbine_id: int, delta_rad: float,
                       seq: Optional[int] = None, ttl_s: float = 60.0) -> None:
        """异步下发单机偏航增量（全场帧：各 WT 进程按槽位领取自己命令）"""
        if seq is None:
            self._yaw_seq += 1
            seq = self._yaw_seq
        nt = self._n_turbines or turbine_id
        frame = new_command_frame(nt, self._frame_seq + 1)
        set_yaw_cmd(frame, turbine_id, delta_rad, seq, ttl_s)
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