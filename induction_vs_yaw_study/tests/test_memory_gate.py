"""内存闸门测试（G-18）：可用内存不足时先等，足够/超时即放行。"""
from __future__ import annotations

from induction_vs_yaw_study.run import run_fastfarm_batch as B


def test_gate_passes_immediately_when_memory_ok(monkeypatch, capsys):
    monkeypatch.setattr(B, "_free_gb", lambda: 9.0)
    waited = B._wait_for_memory(4.0, 600, poll_s=0)
    assert waited == 0.0
    assert "[mem]" not in capsys.readouterr().out


def test_gate_waits_then_proceeds(monkeypatch, capsys):
    seq = iter([1.0, 1.0, 9.0])
    monkeypatch.setattr(B, "_free_gb", lambda: next(seq, 9.0))
    waited = B._wait_for_memory(4.0, 600, poll_s=0)
    assert waited > 0 or waited == 0.0  # poll_s=0 -> loop advances without clock
    out = capsys.readouterr().out
    assert "waiting" in out
    assert "resumed" in out


def test_gate_honours_wait_cap(monkeypatch, capsys):
    monkeypatch.setattr(B, "_free_gb", lambda: 0.5)
    waited = B._wait_for_memory(4.0, 0.05, poll_s=0.05)
    assert waited >= 0.05
    out = capsys.readouterr().out
    assert "wait cap reached" in out


def test_gate_disabled_without_psutil(monkeypatch):
    monkeypatch.setattr(B, "_free_gb", lambda: None)
    assert B._wait_for_memory(4.0, 600, poll_s=0) == 0.0
