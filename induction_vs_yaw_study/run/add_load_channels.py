"""add_load_channels.py — 向已生成 case 的模块 OutList 追加 DEL 载荷通道（研究侧 post-process）

用法 A（对单个 run/case 目录）:
    python -m induction_vs_yaw_study.run.add_load_channels --farm-inputs <dir>/FarmInputs
用法 B（对 replay/批跑前由 diag/replay 调用，engine.setup() 后 start() 前自动执行）:
    from induction_vs_yaw_study.run.add_load_channels import add_load_channels
    add_load_channels(farm_inputs_dir)

背景（EXECUTION_LOG 2026-09-16 / Phase 7）：turbine .outb 现缺塔架/主轴/偏航轴承
载荷通道，DEL（rainflow + S-N）无法计算。本工具在不修改 wfcrl 模板的前提下，
对 engine.setup() 物化出的 FarmInputs 目录就地追加 OutList 行（幂等）。

通道（NREL 5MW onshore ElastoDyn 标准输出）:
    TwrBsMyt / TwrBsMxt / TwrBsFxt   塔根弯矩/剪力（DEL 主通道）
    LSSGagMxa / LSSGagFxa            低速轴力矩/推力（RotThrust 等价量）
    YawBrTDxp / YawBrTDyp            偏航轴承处塔架位移（机舱载荷代理）
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
from pathlib import Path

ED_LOAD_CHANNELS = [
    "TwrBsMyt", "TwrBsMxt", "TwrBsFxt",
    "LSSGagMxa", "LSSGagFxa",
    "YawBrTDxp", "YawBrTDyp",
]


def _append_outlist(module_path: str, channels) -> list:
    """向模块 dat 的首个 OutList 段追加缺失通道（幂等），返回实际新增列表。"""
    p = Path(module_path)
    if not p.exists():
        return []
    text = p.read_bytes().decode("ascii", errors="replace").replace("\r\n", "\n")
    lines = text.split("\n")
    # 找第一个 OutList 段头（排除通道行/END；主 OutList 段在 nodal optional 段之前）
    head = None
    for i, l in enumerate(lines):
        s = l.strip()
        if s.startswith('"') or s.upper().startswith("END"):
            continue
        toks = s.split()
        if toks and toks[0] == "OutList":
            head = i
            break
    if head is None:
        return []
    end = None
    for i in range(head + 1, len(lines)):
        if lines[i].strip().upper().startswith("END"):
            end = i
            break
    if end is None:
        end = len(lines)
    existing = set()
    for i in range(head + 1, end):
        s = lines[i].strip()
        if s.startswith('"'):
            nm = s.split('"')
            if len(nm) >= 2:
                existing.add(nm[1].strip())
    to_add = [c for c in channels if c not in existing]
    if not to_add:
        return []
    ins = ['"' + c + '"' + "    " + c for c in to_add]
    lines = lines[:end] + ins + lines[end:]
    out = "\n".join(lines).encode("ascii", errors="replace")
    out = out.replace(b"\n", b"\r\n")
    p.write_bytes(out)
    return to_add


def add_load_channels(farm_inputs: str, verbose: bool = False) -> dict:
    """对 farm_inputs 下全部 FFTest_WT*.fst 的 ElastoDyn 模块追加载荷通道。"""
    fi = Path(farm_inputs)
    if not fi.exists():
        raise FileNotFoundError(f"FarmInputs not found: {fi}")
    report = {}
    for fst in sorted(glob.glob(str(fi / "FFTest_WT*.fst"))):
        wt = Path(fst)
        ed = None
        for line in wt.read_bytes().decode("ascii", errors="replace").splitlines():
            toks = line.split()
            if len(toks) >= 2 and toks[1] == "EDFile":
                ed = toks[0].strip('"')
                break
        if not ed:
            continue
        ed_path = fi / ed
        added = _append_outlist(str(ed_path), ED_LOAD_CHANNELS)
        report[wt.name] = {"ed": ed, "added": added}
        if verbose or added:
            print(f"[load-ch] {wt.name} -> {ed}: +{added}")
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--farm-inputs", required=True,
                    help="engine.setup() 物化的 FarmInputs 目录")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    rep = add_load_channels(a.farm_inputs, verbose=not a.quiet)
    n = sum(len(v["added"]) for v in rep.values())
    print(f"[load-ch] files={len(rep)} channels_added={n}")


if __name__ == "__main__":
    main()
