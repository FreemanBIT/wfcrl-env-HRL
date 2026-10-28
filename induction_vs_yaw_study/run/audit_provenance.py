"""audit_provenance.py — 已跑 case 的溯源一致性审计（G-21）。

回答三个问题：
1. 这批数据分别由哪个 DISCON_WT1.dll 跑出来（sha256）？
   早期 case 的 metadata.json 没有该字段 → 退化为按 timeseries.csv mtime 划分年代。
2. git commit / 控制器约定版本是否一致？
3. 有没有 case 用了不同的 bts 却声称同一个盒子（sha 复用/冲突）？

用法：
    python -m induction_vs_yaw_study.run.audit_provenance \\
        --run induction_vs_yaw_study/results/run_full_doe

输出：stdout 摘要 + <run>/provenance_audit.md
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

# 外部会重建该 DLL；这个时间点之后完成的 case 属于"新 DLL 年代"
DLL_REBUILD_EPOCH = datetime(2026, 10, 27, 16, 14)


def _load_meta(case_dir: Path) -> dict:
    p = case_dir / "metadata.json"
    if not p.exists():
        return {}
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _case_era(case_dir: Path, meta: dict) -> str:
    """有 sha 就用 sha；没有就按 timeseries.csv 的 mtime 划年代。"""
    if meta.get("discon_dll_sha256"):
        return "sha:" + meta["discon_dll_sha256"][:16]
    ts = case_dir / "timeseries.csv"
    if not ts.exists():
        return "unknown"
    mt = datetime.fromtimestamp(ts.stat().st_mtime)
    tag = "pre-rebuild" if mt < DLL_REBUILD_EPOCH else "post-rebuild"
    return f"{tag} (~10-27 16:14)"


def audit(run_dir: Path) -> dict:
    status_csv = run_dir / "manifest_status.csv"
    rows: list = []
    if status_csv.exists():
        with open(status_csv, encoding="utf-8") as f:
            rows = list(csv.DictReader(f))

    cases_root = run_dir / "cases"
    era_counts: Counter = Counter()
    commit_counts: Counter = Counter()
    bts_by_sha: dict = defaultdict(set)
    bts_by_name: dict = defaultdict(set)
    n_with_sha = 0
    completed = 0

    for r in rows:
        if r.get("status") != "done":
            continue
        name = f"{r['case_id']}_{r['control']}"
        cdir = cases_root / name
        if not (cdir / "timeseries.csv").exists():
            continue
        completed += 1
        meta = _load_meta(cdir)
        era_counts[_case_era(cdir, meta)] += 1
        if meta.get("discon_dll_sha256"):
            n_with_sha += 1
        commit_counts[(meta.get("git_commit") or "?")[:7]] += 1
        bsha = meta.get("bts_sha256")
        bname = meta.get("bts_file")
        if bsha and bname:
            bts_by_sha[bsha].add(bname)
            bts_by_name[bname].add(bsha)

    return {
        "run_dir": str(run_dir),
        "status_rows": len(rows),
        "completed_cases": completed,
        "cases_with_dll_sha": n_with_sha,
        "era": dict(era_counts),
        "git_commit": dict(commit_counts),
        "bts_names": len(bts_by_name),
        "bts_sha_conflicts": {k: sorted(v) for k, v in bts_by_name.items() if len(v) > 1},
        "bts_sha_shared": {k[:16]: sorted(v) for k, v in bts_by_sha.items() if len(v) > 1},
    }


def render_report(a: dict) -> str:
    L = ["# 溯源一致性审计（G-21）", ""]
    L.append(f"- run 目录：{a['run_dir']}")
    L.append(f"- 状态行：{a['status_rows']}；有 timeseries 的 done case：{a['completed_cases']}")
    L.append(f"- 其中带 discon_dll_sha256 的：{a['cases_with_dll_sha']}")
    L.append("")
    L.append("## 控制器 DLL 年代分布")
    L.append("")
    L.append("| 年代 | case 数 |")
    L.append("|---|---|")
    for k, v in sorted(a["era"].items(), key=lambda kv: -kv[1]):
        L.append(f"| {k} | {v} |")
    L.append("")
    L.append("> 早期 case 无 sha 字段，只能按 timeseries.csv mtime 划分；")
    L.append("> 重建后新增的 case 一律带 sha，可精确比对。")
    L.append("")
    L.append("## git commit 分布")
    L.append("")
    L.append("| commit | case 数 |")
    L.append("|---|---|")
    for k, v in sorted(a["git_commit"].items(), key=lambda kv: -kv[1]):
        L.append(f"| {k} | {v} |")
    L.append("")
    L.append("## 风盒 sha 一致性")
    L.append("")
    L.append(f"- 出现过的 bts 文件名：{a['bts_names']}")
    L.append(f"- 同名 bts 却有多个 sha（不该有）：{len(a['bts_sha_conflicts'])}")
    for k, v in list(a["bts_sha_conflicts"].items())[:10]:
        L.append(f"  - {k}: {len(v)} 个 sha")
    L.append(f"- 同一个 sha 对应多个文件名（改名副本，需留意）：{len(a['bts_sha_shared'])}")
    L.append("")
    return "\n".join(L)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="已跑 case 的溯源一致性审计")
    ap.add_argument("--run", type=Path, required=True, help="run 目录")
    ap.add_argument("--no-write", action="store_true", help="只打摘要，不写 md")
    args = ap.parse_args(argv)

    a = audit(args.run)
    print(f"completed={a['completed_cases']} with_dll_sha={a['cases_with_dll_sha']}")
    for k, v in sorted(a["era"].items(), key=lambda kv: -kv[1]):
        print(f"  era {k}: {v}")
    print(f"  bts names={a['bts_names']} conflicts={len(a['bts_sha_conflicts'])}")
    if not args.no_write:
        out = args.run / "provenance_audit.md"
        out.write_text(render_report(a), encoding="utf-8")
        print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
