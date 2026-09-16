"""_proc_utils.py — 进程清理工具（审查 P1-4：PID/age-scope，禁止全局误杀）

研究侧每次新建案例前，清理"过期且仍存活"的 FAST.Farm 进程（例如上一轮
ZMQ assert 后遗留的僵尸进程会持有 FarmInputs 文件句柄，导致 clean_folder
PermissionError WinError 32）。只按 StartTime 过滤旧进程，不杀新进程。
"""
from __future__ import annotations

import subprocess
import time
from typing import List


def stale_fastfarm_pids(min_age_s: float = 300.0) -> List[int]:
    """列出 StartTime 早于 now-min_age_s 存活 FAST.Farm_x64_OMP 的 PID。"""
    cmd = ("powershell -NoProfile -Command "
           "Get-Process FAST.Farm_x64_OMP -ErrorAction SilentlyContinue | "
           "Where-Object { (Get-Date) - $_.StartTime -gt "
           f"[TimeSpan]::FromSeconds({float(min_age_s):.0f}) }} | "
           "ForEach-Object { Write-Output $_.Id }")
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except Exception:
        return []
    pids = []
    for line in (out.stdout or "").splitlines():
        s = line.strip()
        if s.isdigit():
            pids.append(int(s))
    return pids


def kill_pids(pids: List[int]) -> None:
    if not pids:
        return
    for pid in pids:
        try:
            subprocess.run(
                ["taskkill", "/PID", str(pid), "/T", "/F"],
                capture_output=True, timeout=30,
            )
        except Exception:
            pass


def cleanup_stale_fastfarm(min_age_s: float = 300.0) -> int:
    """清理过期 FAST.Farm 进程；返回清理数量。"""
    pids = stale_fastfarm_pids(min_age_s)
    if pids:
        print(f"[proc_utils] 清理 {len(pids)} 个过期 FAST.Farm 进程: {pids}")
    kill_pids(pids)
    return len(pids)


def wait_all_fastfarm_exit(timeout_s: float = 60.0) -> None:
    """等待所有 FAST.Farm 退出（新案例启动前调用，防文件句柄冲突）。"""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if not stale_fastfarm_pids(0.0):
            return
        time.sleep(1.0)
