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
                 host: str = "127.0.0.1", timeout_s: float = 2.0):
        self._host = host
        self._cmd_port = cmd_port
        self._state_port = state_port
        self._timeout_s = timeout_s
        self._ctx: Optional[zmq.Context] = None
        self._cmd_sock: Optional[zmq.Socket] = None
        self._state_sock: Optional[zmq.Socket] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._latest_state: Optional[bytes] = None
        self._state_lock = threading.Lock()
        self._frame_seq = 0
        self._yaw_seq = 0
        self._ind_seq = 0
        self._started = False

    # ---------------- 生命周期 ----------------
    def start(self) -> None:
        """连接 runtime（命令 PUB connect / 状态 SUB connect + 订阅线程）。"""
        if self._started:
            return
        self._ctx = zmq.Context()
        self._cmd_sock = self._ctx.socket(zmq.PUB)
        self._cmd_sock.setsockopt(zmq.LINGER, 0)
        self._cmd_sock.connect(f"tcp://{self._host}:{self._cmd_port}")
        self._state_sock = self._ctx.socket(zmq.SUB)
        self._state_sock.setsockopt(zmq.LINGER, 0)
        self._state_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._state_sock.connect(f"tcp://{self._host}:{self._state_port}")
        self._stop.clear()
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()
        self._started = True

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._state_sock.recv(copy=False)
            except zmq.ZMQError:
                time.sleep(0.05)
                continue
            if msg is None or len(msg) == 0:
                continue
            with self._state_lock:
                self._latest_state = bytes(msg)

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
        self._started = False

    def close(self) -> None:
        """销毁客户端（不可再 start）。"""
        self.stop()

    # ---------------- 状态 ----------------
    def get_state(self, timeout_s: Optional[float] = None) -> Optional[bytes]:
        """最新 FarmStateFrame 原始字节（None = 尚未收到）。"""
        t0 = time.time()
        timeout = timeout_s if timeout_s is not None else self._timeout_s
        while timeout is None or time.time() - t0 < timeout:
            with self._state_lock:
                if self._latest_state is not None:
                    return self._latest_state
            time.sleep(0.02)
        with self._state_lock:
            return self._latest_state

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
        """异步下发单机偏航增量（不阻塞仿真；yaw 通道独立 seq）"""
        if seq is None:
            self._yaw_seq += 1
            seq = self._yaw_seq
        frame = new_command_frame(1, self._frame_seq + 1)
        set_yaw_cmd(frame, turbine_id, delta_rad, seq, ttl_s)
        self._send_frame(frame)

    def send_induction(self, turbine_id: int, induction_ref: float,
                       seq: Optional[int] = None, ttl_s: float = 1.0) -> None:
        """异步下发单机诱导因子（induction 通道独立 seq）"""
        if seq is None:
            self._ind_seq += 1
            seq = self._ind_seq
        frame = new_command_frame(1, self._frame_seq + 1)
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
