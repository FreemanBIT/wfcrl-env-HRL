"""bench_modbus_throughput.py — M4/DR-8 Modbus 吞吐测量（30/100 台）

测量内容（先测吞吐 → 再冻结刷新策略与寄存器布局，DR-8）：
  1. 单轮询往返延迟：RCP 读 N 台状态区 + 写 1 台命令区
  2. 帧率：RCP 以不同轮询周期可达到的状态刷新率
  3. 完整性校验开销：CRC16 计算 + 比对
  4. 对比 30 台 vs 100 台（扩展能力，DR-8）

实现：纯 Python 计算寄存器规模 + 时间测量（无需真实 Modbus 服务，
按 Modbus TCP 报文尺寸估算网络延迟；实际 RT 联调时再测真值）。

用法：
    python -m induction_vs_yaw_study.run.bench_modbus_throughput
输出：results/_modbus_bench/benchmark.json + 控制台汇总
"""
from __future__ import annotations

import json
import time
from pathlib import Path

_STUDY = Path(__file__).resolve().parents[1]

# 寄存器布局（与 modbus_register_map.h 同步）
STATE_PER_TURBINE = 128
CMD_PER_TURBINE = 64
AWAE_PER_POINT = 32
MODBUS_PDU_OVERHEAD = 9       # MBAP(7) + FC(1) + ...
MODBUS_REG_BYTES = 2

# 假设网络：100Mbps 以太网，RTT 0.5ms（同机房）
NET_BYTES_PER_SEC = 12.5e6
RTT_S = 0.5e-3


def poll_size(n_turbs: int) -> int:
    """单次状态轮询报文大小（读全部机组状态区）"""
    regs = n_turbs * STATE_PER_TURBINE
    return regs * MODBUS_REG_BYTES + MODBUS_PDU_OVERHEAD


def cmd_write_size() -> int:
    return CMD_PER_TURBINE * MODBUS_REG_BYTES + MODBUS_PDU_OVERHEAD


def one_cycle_latency(n_turbs: int) -> float:
    """一轮（读 N 台状态 + 写 1 台命令）的传输时间（s）"""
    read_bytes = poll_size(n_turbs)
    write_bytes = cmd_write_size()
    t_net = (read_bytes + write_bytes) / NET_BYTES_PER_SEC
    return t_net + 2.0 * RTT_S  # 两次往返


def bench(n_turbs: int, iterations: int = 1000) -> dict:
    sizes = {
        "state_regs": n_turbs * STATE_PER_TURBINE,
        "state_bytes": poll_size(n_turbs),
        "cmd_bytes": cmd_write_size(),
    }
    # 传输延迟（测量）
    t_start = time.perf_counter()
    for _ in range(iterations):
        _ = one_cycle_latency(n_turbs)
    # 由于是解析模型，直接计算单轮延迟
    lat = one_cycle_latency(n_turbs)
    # 估算最大刷新率（单客户端轮询）
    max_rate = 1.0 / lat if lat > 0 else float("inf")
    # 完整性校验（CRC16-CCITT 软件实现，模拟 RCP 端）
    state_bytes = bytearray(poll_size(n_turbs))
    t0 = time.perf_counter()
    N = 2000
    for _ in range(N):
        crc = 0xFFFF
        for b in state_bytes:
            crc ^= (b << 8)
            for _b in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    crc_us = (time.perf_counter() - t0) / N * 1e6
    return {
        "n_turbines": n_turbs,
        "sizes": sizes,
        "one_cycle_latency_s": lat,
        "max_poll_rate_hz": max_rate,
        "crc16_us_per_frame": crc_us,
        "awae_points_supported": (0x2FFF - 0x2009) // AWAE_PER_POINT,
    }


def main():
    out = {"method": "analytical-network-model", "rtt_s": RTT_S, "net_mbps": 100}
    rows = []
    for n in (30, 100):
        b = bench(n)
        rows.append(b)
        out[f"n{n}"] = b
    out["results"] = rows

    out_dir = _STUDY / "results" / "_modbus_bench"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "benchmark.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== Modbus throughput benchmark (DR-8, analytical model) ==")
    print(f"  network: {out['net_mbps']} Mbps, RTT {out['rtt_s']*1e3:.1f} ms")
    for b in rows:
        n = b["n_turbines"]
        print(f"  [{n:3d} turb] state={b['sizes']['state_bytes']}B cmd={b['sizes']['cmd_bytes']}B "
              f"| cycle={b['one_cycle_latency_s']*1e3:.3f} ms | max_poll={b['max_poll_rate_hz']:.1f} Hz "
              f"| crc={b['crc16_us_per_frame']:.1f} us/frame | awae_points={b['awae_points_supported']}")
    print(f"  -> {out_dir / 'benchmark.json'}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
