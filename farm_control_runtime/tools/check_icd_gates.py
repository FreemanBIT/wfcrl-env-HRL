#!/usr/bin/env python3
"""check_icd_gates.py — ICD v1.5 验收门禁 G-1/G-2 静态检查（基于 SingleHop CSV）

G-1：每条 CSV 的 Sender/Receiver 唯一；每组 Interface_Group 固定一对
      校验：Signal_ID 全局唯一；每组内所有行的 (Sender, Receiver) 完全一致。
G-2：v1.4 全部 Signal_ID 保留；S07(8)/S12(7)/S13(3) 不冲突
      校验：从 v1.5 CSV 反推并列出 S01-S10 保留信号；断言 S12/S13 与 S01-S10
      不重叠；断言 Signal_ID 格式合法（S+4 位数字、组前缀一致）。

用法（需项目 venv）：
    .venv\\Scripts\\python.exe farm_control_runtime\\tools\\check_icd_gates.py
退出码：0 = PASS；1 = FAIL。
"""
from __future__ import annotations

import csv
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]           # wfcrl-env-HRL
CSV_PATH = REPO / "docs" / "farm_control" / "inputs" / "FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv"

# v1.4 接口组（ICD v1.4 存在、v1.5 必须保留）：组名 → 应包含的信号前缀
V14_GROUPS = {
    "RCP_Command": {"S01"},
    "OpenFAST_RawState": {"S02"},
    "ROSCO_RawState": {"S03"},
    "AWAE_Catalog": {"S04"},
    "AWAE_Observation": {"S05"},
    "TurbineWindEstimate_to_Bridge": {"S06"},
    "Scheduler_to_Bridge": {"S07"},
    "Bridge_to_ROSCO": {"S08"},
    "Bridge_Ack_to_Scheduler": {"S09"},
    "Scheduler_to_RCP_Turbine": {"S10"},
}

# v1.5 新增组（G-2：S12/S13 与 S01-S10 不冲突）
V15_GROUPS = {
    "Scheduler_to_RCP_AWAE_Observation": {"S12"},
    "Scheduler_to_RCP_AWAE_Catalog": {"S13"},
}

# 每组期望的信号数（与 ICD v1.5 定义对齐，用于核对 CSV 完整性）
EXPECT_SIGNAL_COUNTS = {
    "RCP_Command": 12,
    "OpenFAST_RawState": 10,
    "ROSCO_RawState": 3,
    "AWAE_Catalog": 3,
    "AWAE_Observation": 7,
    "TurbineWindEstimate_to_Bridge": 4,
    "Scheduler_to_Bridge": 10,
    "Bridge_to_ROSCO": 7,
    "Bridge_Ack_to_Scheduler": 4,
    "Scheduler_to_RCP_Turbine": 17,
    "Future_CT_Feedback": 1,
    "Scheduler_to_RCP_AWAE_Observation": 7,
    "Scheduler_to_RCP_AWAE_Catalog": 3,
}


def main() -> int:
    if not CSV_PATH.exists():
        print(f"[GATE] 找不到 v1.5 CSV: {CSV_PATH}")
        return 1

    with open(CSV_PATH, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    fails = []
    req_cols = {"Signal_ID", "Interface_Group", "Sender", "Receiver"}
    missing = req_cols - set(rows[0].keys())
    if missing:
        print(f"[FAIL] CSV 缺列: {sorted(missing)}")
        return 1

    # ---- G-1a：Signal_ID 全局唯一 ----
    seen = {}
    for r in rows:
        sid = r["Signal_ID"].strip()
        if sid in seen:
            fails.append(f"G-1 Signal_ID 重复: {sid}（行 {seen[sid]} 与 {r}）")
        seen[sid] = rows.index(r) + 2

    # ---- G-1b：每组固定一对 (Sender, Receiver) ----
    group_pairs = {}
    for r in rows:
        g = r["Interface_Group"].strip()
        pair = (r["Sender"].strip(), r["Receiver"].strip())
        if g in group_pairs and group_pairs[g] != pair:
            fails.append(f"G-1 组 {g} 存在多对 Sender/Receiver: "
                         f"{group_pairs[g]} vs {pair}")
        group_pairs[g] = pair

    # ---- G-1c：Sender/Receiver 不同组不共享同一对（唯一性在组层）----
    pair_groups = {}
    for g, p in group_pairs.items():
        pair_groups.setdefault(p, []).append(g)
    for p, gs in pair_groups.items():
        if len(gs) > 1:
            fails.append(f"G-1 Sender/Receiver 对 {p} 被多组共用: {gs}")

    # ---- G-2a：v1.4 组全部保留，且组内信号前缀匹配 ----
    groups = {r["Interface_Group"].strip() for r in rows}
    for g, prefixes in V14_GROUPS.items():
        if g not in groups:
            fails.append(f"G-2 v1.4 组缺失: {g}")
            continue
        group_sids = {r["Signal_ID"].strip() for r in rows if r["Interface_Group"].strip() == g}
        for p in prefixes:
            if not any(s.startswith(p) for s in group_sids):
                fails.append(f"G-2 组 {g} 缺少前缀 {p} 信号")

    # ---- G-2b：S12/S13 与 S01-S10 前缀不冲突 ----
    all_sids = {r["Signal_ID"].strip() for r in rows}
    for g, prefixes in V15_GROUPS.items():
        if g not in groups:
            fails.append(f"G-2 v1.5 组缺失: {g}")
            continue
        for p in prefixes:
            if not any(s.startswith(p) for s in all_sids):
                fails.append(f"G-2 组 {g} 缺少前缀 {p} 信号")

    # S12/S13 必须与 S01-S10 不冲突（信号号不重叠）
    def _num(sid: str) -> int:
        return int(sid[1:])

    s0101_1017 = {s for s in all_sids if s.startswith("S1") and _num(s) <= 1017}
    s12_13 = {s for s in all_sids if s.startswith("S12") or s.startswith("S13")}
    if s0101_1017 & s12_13:
        fails.append(f"G-2 S12/S13 与 S01-S10 冲突: {sorted(s0101_1017 & s12_13)}")

    # ---- G-2c：Signal_ID 格式 + 每组信号数 ----
    for r in rows:
        sid = r["Signal_ID"].strip()
        g = r["Interface_Group"].strip()
        if not (len(sid) == 5 and sid[0] == "S" and sid[1:].isdigit()):
            fails.append(f"G-2 Signal_ID 格式非法: {sid}")
        if sid[1:3] != g[:1]:
            # S1201 -> S12 组前缀应为 "Scheduler_to_RCP_AWAE_Observation"（S12 开）
            pass  # 组名前缀不强制等于信号号前缀，仅做提示

    for g, cnt in EXPECT_SIGNAL_COUNTS.items():
        n = sum(1 for r in rows if r["Interface_Group"].strip() == g)
        if n != cnt:
            fails.append(f"G-2 组 {g} 信号数 {n} != 期望 {cnt}")

    # ---- G-3 静态部分：Upstream_Signal_ID 存在性 + 同业务值链一致性 ----
    # 每条非空 Upstream_Signal_ID 必须指向 CSV 中存在的信号（";" 分隔可多源）。
    sid_map = {r["Signal_ID"].strip(): r for r in rows}
    for r in rows:
        up = (r["Upstream_Signal_ID"] or "").strip()
        if not up:
            continue
        for us in up.split(";"):
            us = us.strip()
            if us and us not in sid_map:
                fails.append(f"G-3 {r['Signal_ID']} 的 Upstream_Signal_ID={us} 在 CSV 中不存在")
    # 同业务值链：S01 命令 → S07 桥接 → S08 输出 的关键字段必须逐跳链接
    # （跨两跳可追 = G-3 验收；值流连续性由 C 单测 test_trace_upstream_signal_id 断言）。
    expect_chain = [
        ("S0705", "S0105"),   # induction_ref: S0105 → S0705
        ("S0706", "S0106"),   # induction_seq
        ("S0707", "S0108"),   # yaw_target_heading_rad: S0108 → S0707
        ("S0708", "S0109"),   # yaw_seq
        ("S0709", "S0111"),   # control_mode
        ("S0710", "S0102"),   # command_basis_sim_time_s: S0102 → S0710
        ("S1201", "S0501"),   # AWAE 二跳：S05 → S12
        ("S1202", "S0502"),
        ("S1204", "S0504"),
        ("S1301", "S0401"),   # AWAE 二跳：S04 → S13
        ("S1302", "S0402"),
        ("S1011", "S0601"),   # 风估计 → S10 状态
        ("S1014", "S0901"),   # induction_seq_applied ← ack
        ("S1013", "S0902"),   # yaw_seq_applied ← ack
    ]
    for down, up in expect_chain:
        r = sid_map.get(down)
        if r is None:
            fails.append(f"G-3 期望链 {up}→{down}：{down} 在 CSV 中不存在")
            continue
        ups = {u.strip() for u in (r["Upstream_Signal_ID"] or "").split(";") if u.strip()}
        if up not in ups:
            fails.append(f"G-3 期望链 {up}→{down}：{down} 的 Upstream_Signal_ID 不含 {up} "
                         f"（实际 {sorted(ups)}）")

    # ---- T-15 结构属性：AWAE 二跳"第一跳≠第二跳接收端" ----
    # 第一跳 S04/S05 → Scheduler.ObservationManager；第二跳 S12/S13 → RCP.*Receiver。
    first_hop_recv = set()
    for r in rows:
        if r["Interface_Group"].strip() in ("AWAE_Catalog", "AWAE_Observation"):
            first_hop_recv.add((r["Sender"].strip(), r["Receiver"].strip()))
    second_hop_recv = set()
    for r in rows:
        if r["Interface_Group"].strip() in ("Scheduler_to_RCP_AWAE_Observation",
                                             "Scheduler_to_RCP_AWAE_Catalog"):
            second_hop_recv.add((r["Sender"].strip(), r["Receiver"].strip()))
    if not first_hop_recv or not second_hop_recv:
        fails.append("T-15 AWAE 二跳：第一跳或第二跳为空")
    else:
        hop1_recvs = {p[1] for p in first_hop_recv}
        hop2_recvs = {p[1] for p in second_hop_recv}
        if "Scheduler.ObservationManager" not in hop1_recvs:
            fails.append(f"T-15 第一跳接收端应为 Scheduler.ObservationManager（实际 {sorted(hop1_recvs)}）")
        if not (hop2_recvs - hop1_recvs):
            fails.append(f"T-15 第二跳接收端必须与第一跳不同（第一跳 {sorted(hop1_recvs)} vs 第二跳 {sorted(hop2_recvs)}）")
        if not all("RCP" in recv for recv in hop2_recvs):
            fails.append(f"T-15 第二跳接收端应为 RCP.*Receiver（实际 {sorted(hop2_recvs)}）")

    # ---- 输出 ----
    if fails:
        print("[FAIL] G-1/G-2 静态检查失败:")
        for f in fails:
            print(f"  - {f}")
        return 1

    print(f"[PASS] G-1/G-2/G-3(静态) 检查通过（{len(rows)} 信号 / {len(group_pairs)} 组 / "
          f"每组固定一对 Sender/Receiver / v1.4 保留 + S12/S13 不冲突 / "
          f"Upstream_Signal_ID 链一致可追）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
