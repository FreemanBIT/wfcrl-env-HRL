"""_dbg2_window.py — ROSCO .RO.dbg2 窗口解析（诊断用）
用法: python -m induction_vs_yaw_study.run._dbg2_window <dbg2路径> <t0> <t1> [每N行抽样] [列名,列名...]
"""
import sys
from pathlib import Path


def main() -> int:
    path = Path(sys.argv[1])
    t0, t1 = float(sys.argv[2]), float(sys.argv[3])
    stride = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    want = (sys.argv[5].split(",") if len(sys.argv) > 5 else
            ["Time", "GenSpeed", "RotSpeed", "GenSpeedF", "PC_RefSpd", "PC_SpdErr",
             "PC_KP", "PC_KI", "PC_PitComT", "PC_PitComT_Last", "PitCom", "PitComAct",
             "VS_SpdErr", "GenTq", "VS_State", "PC_State", "NacHeading", "VS_GenPwr",
             "VS_LastGenTrq", "GenTqMeas", "iStatus"])
    with open(path, encoding="utf-8", errors="replace") as f:
        header = None
        rows = []
        for line in f:
            if header is None:
                if line.strip().startswith("Time"):
                    header = line.split()
                continue
            parts = line.split()
            if not parts:
                continue
            try:
                t = float(parts[0])
            except ValueError:
                continue
            if t0 <= t <= t1:
                rows.append(parts)
    if header is None:
        print("no header found")
        return 2
    idx = {name: header.index(name) for name in want if name in header}
    missing = [n for n in want if n not in header]
    if missing:
        print("missing cols:", missing)
    def fmt(v: str) -> str:
        try:
            x = float(v)
        except ValueError:
            return v[:10]
        if x != x:
            return "NaN"
        if abs(x) > 1e6:
            return f"{x:.3e}"
        return f"{x:10.4f}"
    print("t".rjust(10) + "".join(n.rjust(11) for n in idx))
    for i, parts in enumerate(rows):
        if i % stride:
            continue
        line = f"{float(parts[0]):10.3f}"
        for name in idx:
            j = idx[name]
            line += fmt(parts[j]).rjust(11) if j < len(parts) else "MISS".rjust(11)
        print(line)
    print(f"rows in window: {len(rows)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
