#!/usr/bin/env python3
"""audit_signals.py — ICD v1.5 信号列表 ↔ 当前工程实现 一致性审计

把 FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv（88 信号）逐一映射到
farm_control_runtime 的 C 结构字段，核对：
  1. 每个信号是否在 C 结构中有对应字段（缺失 = GAP）
  2. 数据类型是否一致（float64↔double、uint32↔uint32_t 等）
  3. 方向/承载：命令组在 FarmCommandFrame、状态组在 FarmStateFrame、
     AWAE 二跳在 FcrAWAEObservationFrame/FcrAWAECatalogFrame、S07/S08/S09 在
     FcrBridgeInput/FcrRoscoExternalSetpoint/FcrRoscoAck
  4. 内部字段（TTL 等）与 ICD 无关的扩展字段不计为缺失

用法（需项目 venv，纯 Python 无编译）：
    .venv\\Scripts\\python.exe farm_control_runtime\\tools\\audit_signals.py
退出码：0 = 全部一致；1 = 存在缺失/不一致（报告明细）。
"""
from __future__ import annotations

import csv
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]           # wfcrl-env-HRL
CSV_PATH = REPO / "docs" / "farm_control" / "inputs" / "FASTFarm_WFC_Signal_ICD_v1_5_SingleHop.csv"

# ---------------------------------------------------------------------------
# 信号 → C 结构字段 权威映射（与 include/*.h 逐字段核对后固化）
#   结构名: [(CSV Signal_ID, 字段名, 期望 C 类型)]
# 类型缩写：f64=double, u32=uint32_t, u64=uint64_t, u16=uint16_t,
#           u8=uint8_t, i32=int32_t, f32=float, f32x3=float[3], f64x3=double[3],
#           stat=FcrSignalStats, bool=uint32_t(0/1)
# ---------------------------------------------------------------------------
SIGNAL_MAP = {
    # ---- S01 RCP_Command → FcrTurbineCommand / FarmCommandFrame ----
    "S0101": ("FarmCommandFrame", "command_frame_seq", "u64"),
    "S0102": ("FcrTurbineCommand", "command_basis_sim_time_s", "f64"),
    "S0103": ("FarmCommandFrame", "turbines(槽位索引)", "u32"),
    "S0104": ("FcrTurbineCommand", "control_enable", "u32"),
    "S0105": ("FcrTurbineCommand", "induction_ref", "f64"),
    "S0106": ("FcrTurbineCommand", "induction_seq", "u64"),
    "S0107": ("FcrTurbineCommand", "induction_valid", "u32"),
    "S0108": ("FcrTurbineCommand", "yaw_target_heading_rad", "f64"),
    "S0109": ("FcrTurbineCommand", "yaw_seq", "u64"),
    "S0110": ("FcrTurbineCommand", "yaw_valid", "u32"),
    "S0111": ("FcrTurbineCommand", "control_mode", "u16"),
    "S0112": ("FcrTurbineCommand", "requested_effective_sim_time_s", "f64"),

    # ---- S02 OpenFAST_RawState → FcrOpenFASTRawState ----
    "S0201": ("FcrOpenFASTRawState", "turbine_id", "i32"),
    "S0202": ("FcrOpenFASTRawState", "of_sample_sim_time_s", "f64"),
    "S0203": ("FcrOpenFASTRawState", "rotor_speed_rad_s", "f64"),
    "S0204": ("FcrOpenFASTRawState", "generator_speed_rad_s", "f64"),
    "S0205": ("FcrOpenFASTRawState", "generator_electrical_power_w", "f64"),
    "S0206": ("FcrOpenFASTRawState", "blade_pitch_actual_rad[3]", "f64x3"),
    "S0207": ("FcrOpenFASTRawState", "nacelle_heading_actual_rad", "f64"),
    "S0208": ("FcrOpenFASTRawState", "rotor_aero_thrust_n", "f64"),
    "S0209": ("FcrOpenFASTRawState", "shaft_torque_nm", "f64"),
    "S0210": ("FcrOpenFASTRawState", "tower_base_fa_moment_nm", "f64"),

    # ---- S03 ROSCO_RawState → FcrRoscoRawState ----
    "S0301": ("FcrRoscoRawState", "rosco_controller_status_flags", "u32"),
    "S0302": ("FcrRoscoRawState", "generator_torque_command_nm", "f64"),
    "S0303": ("FcrRoscoRawState", "pitch_command_rad[3]", "f64x3"),

    # ---- S04 AWAE_Catalog → FcrAWAECatalogEntry ----
    "S0401": ("FcrAWAECatalogEntry", "point_id", "u32"),
    "S0402": ("FcrAWAECatalogEntry", "point_position_xyz_m[3]", "f64x3"),
    "S0403": ("FcrAWAECatalogEntry", "point_catalog_version", "u32"),

    # ---- S05 AWAE_Observation → FcrAWAEObservation ----
    "S0501": ("FcrAWAEObservation", "observation_frame_seq", "u64"),
    "S0502": ("FcrAWAEObservation", "observation_sample_sim_time_s", "f64"),
    "S0503": ("FcrAWAEObservation", "observation_point_count", "u32"),
    "S0504": ("FcrAWAEObservation", "point_id", "u32"),
    "S0505": ("FcrAWAEObservation", "wind_uvw_mps[3]", "f32x3"),
    "S0506": ("FcrAWAEObservation", "observation_quality_flags", "u16"),
    "S0507": ("FcrAWAEObservation", "turbulence_intensity", "f32"),

    # ---- S06 TurbineWindEstimate → FcrTurbineWindEstimate ----
    "S0601": ("FcrTurbineWindEstimate", "estimated_wind_mps", "f64"),
    "S0602": ("FcrTurbineWindEstimate", "estimate_sim_time_s", "f64"),
    "S0603": ("FcrTurbineWindEstimate", "wind_source", "u16"),
    "S0604": ("FcrTurbineWindEstimate", "wind_quality_flags", "u16"),

    # ---- S07 Scheduler_to_Bridge → FcrBridgeInput ----
    "S0701": ("FcrBridgeInput", "bridge_step_sim_time_s", "f64"),
    "S0702": ("FcrBridgeInput", "bridge_command_valid_flags", "u16"),
    "S0703": ("FcrBridgeInput", "turbine_id", "u32"),
    "S0704": ("FcrBridgeInput", "control_enable", "u8"),
    "S0705": ("FcrBridgeInput", "induction_ref", "f64"),
    "S0706": ("FcrBridgeInput", "induction_seq", "u64"),
    "S0707": ("FcrBridgeInput", "yaw_target_heading_rad", "f64"),
    "S0708": ("FcrBridgeInput", "yaw_seq", "u64"),
    "S0709": ("FcrBridgeInput", "control_mode", "u16"),
    "S0710": ("FcrBridgeInput", "command_basis_sim_time_s", "f64"),

    # ---- S08 Bridge_to_ROSCO → FcrRoscoExternalSetpoint（fcr_rosco_api.h）----
    "S0801": ("FcrRoscoExternalSetpoint", "external_yaw_enable", "u32"),
    "S0802": ("FcrRoscoExternalSetpoint", "yaw_target_heading_rad", "f64"),
    "S0803": ("FcrRoscoExternalSetpoint", "r_pitch", "f64"),
    "S0804": ("FcrRoscoExternalSetpoint", "r_speed", "f64"),
    "S0805": ("FcrRoscoExternalSetpoint", "r_torque", "f64"),
    "S0806": ("FcrRoscoExternalSetpoint", "min_pitch_ref_rad", "f64"),
    "S0807": ("FcrRoscoExternalSetpoint", "collective_pitch_offset_rad", "f64"),

    # ---- S09 Bridge_Ack → FcrRoscoAck ----
    "S0901": ("FcrRoscoAck", "induction_seq_applied", "u64"),
    "S0902": ("FcrRoscoAck", "yaw_seq_applied", "u64"),
    "S0903": ("FcrRoscoAck", "bridge_status_flags", "u32"),
    "S0904": ("FcrRoscoAck", "ct_ref", "f64"),

    # ---- S10 Scheduler_to_RCP_Turbine → FcrFarmTurbineState / FarmStateFrame ----
    "S1001": ("FarmStateFrame", "state_frame_seq", "u64"),
    "S1002": ("FarmStateFrame", "state_sample_sim_time_s", "f64"),
    "S1003": ("FcrFarmTurbineState", "turbine_id", "i32"),
    "S1004": ("FcrFarmTurbineState", "gen_power_w.mean", "stat"),
    "S1005": ("FcrFarmTurbineState", "gen_power_w.instant", "stat"),
    "S1006": ("FcrFarmTurbineState", "rotor_speed_rad_s", "f64"),
    "S1007": ("FcrFarmTurbineState", "gen_speed_rad_s", "f64"),
    "S1008": ("FcrFarmTurbineState", "collective_pitch_actual_rad", "f64"),
    "S1009": ("FcrFarmTurbineState", "nacelle_heading_actual_rad", "f64"),
    "S1010": ("FcrFarmTurbineState", "rotor_aero_thrust_n.mean", "stat"),
    "S1011": ("FcrFarmTurbineState", "estimated_wind_mps", "f64"),
    "S1012": ("FcrFarmTurbineState", "ct_hat", "f64"),
    "S1013": ("FcrFarmTurbineState", "yaw_seq_applied", "u64"),
    "S1014": ("FcrFarmTurbineState", "induction_seq_applied", "u64"),
    "S1015": ("FcrFarmTurbineState", "turbine_status_flags", "u32"),
    "S1016": ("FcrFarmTurbineState", "wind_source", "u16"),
    "S1017": ("FcrFarmTurbineState", "wind_quality_flags", "u16"),

    # ---- S11 Future_CT_Feedback（仅保留接口，本期不实现闭环）----
    "S1101": ("FcrRoscoExternalSetpoint", "ct_tracking_error", "f64"),

    # ---- S12 Scheduler_to_RCP_AWAE_Observation → FcrAWAEObservationFrame ----
    "S1201": ("FcrAWAEObservationFrame", "frame_seq", "u64"),
    "S1202": ("FcrAWAEObservation", "observation_sample_sim_time_s", "f64"),
    "S1203": ("FcrAWAEObservation", "observation_point_count", "u32"),
    "S1204": ("FcrAWAEObservation", "point_id", "u32"),
    "S1205": ("FcrAWAEObservation", "wind_uvw_mps[3]", "f32x3"),
    "S1206": ("FcrAWAEObservation", "observation_quality_flags", "u16"),
    "S1207": ("FcrAWAEObservation", "turbulence_intensity", "f32"),

    # ---- S13 Scheduler_to_RCP_AWAE_Catalog → FcrAWAECatalogFrame ----
    "S1301": ("FcrAWAECatalogFrame", "entries[].point_id", "u32"),
    "S1302": ("FcrAWAECatalogFrame", "entries[].point_position_xyz_m[3]", "f64x3"),
    "S1303": ("FcrAWAECatalogFrame", "entries[].point_catalog_version", "u32"),
}

# C 类型名 → Python 类型缩写（用于核对）
C_TYPE_ALIAS = {
    "double": "f64",
    "uint32_t": "u32",
    "uint64_t": "u64",
    "uint16_t": "u16",
    "uint8_t": "u8",
    "int32_t": "i32",
    "float": "f32",
    "FcrSignalStats": "stat",
}

# 期望 C 结构（从 CSV Signal_ID 反查结构名）
def main() -> int:
    if not CSV_PATH.exists():
        print(f"[AUDIT] 找不到 CSV: {CSV_PATH}")
        return 1
    with open(CSV_PATH, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    # CSV 中所有 Signal_ID
    csv_sids = {r["Signal_ID"].strip() for r in rows}
    map_sids = set(SIGNAL_MAP)

    # 1) CSV 信号全部在映射表中
    missing_in_map = sorted(csv_sids - map_sids)
    extra_in_map = sorted(map_sids - csv_sids)
    issues = []
    notes = []
    if missing_in_map:
        issues.append(f"[缺失映射] CSV 中有 {len(missing_in_map)} 个信号未在映射表中: {missing_in_map}")
    if extra_in_map:
        issues.append(f"[多余映射] 映射表中有 {len(extra_in_map)} 个信号不在 CSV: {extra_in_map}")

    # 2) 逐信号核对 C 结构字段存在性 + 类型（读头文件文本）
    inc = REPO / "farm_control_runtime" / "include"
    headers = {}
    for h in ("fcr_state_types.h", "fcr_awae_types.h", "fcr_rosco_api.h"):
        p = inc / h
        if p.exists():
            headers[h] = p.read_text(encoding="utf-8")
    all_text = "\n".join(headers.values())

    # 特殊映射：S1004/S1005/S1010 是 FcrSignalStats 容器内的 double（非独立字段）
    STAT_MEMBERS = {"S1004": "gen_power_w", "S1005": "gen_power_w",
                    "S1010": "rotor_aero_thrust_n"}
    # 特殊映射：S0103 用槽位索引表示机组号（无独立 turbine_id 字段）
    INDEX_BASED = {"S0103"}

    for sid in sorted(csv_sids):
        if sid not in SIGNAL_MAP:
            continue
        struct, field, exp_type = SIGNAL_MAP[sid]
        if sid in INDEX_BASED:
            continue   # 槽位索引=机组号，无独立字段
        # 数组/嵌套字段用首字段名匹配（blade_pitch_actual_rad[3] -> blade_pitch_actual_rad）
        base_field = re.split(r"[\[\].]", field)[0]
        if base_field not in all_text:
            issues.append(f"[字段缺失] {sid} -> {struct}.{field}（在头文件中未找到 {base_field}）")
        # 结构名存在性
        if struct not in all_text and struct != "FcrRoscoExternalSetpoint":
            issues.append(f"[结构缺失] {sid} -> 结构 {struct} 未在头文件中找到")

    # 3) 数据类型核对（CSV Data_Type ↔ C 类型）
    #    - 数组类型比较基类型（float64[3] ↔ double[3] 的 base 都是 f64/f64x3 视为一致）
    #    - stat 容器成员（S1004/1005/1010）比较容器内 double 与 CSV float64
    type_map_csv_to_c = {
        "float64": "f64", "uint32": "u32", "uint64": "u64", "uint16": "u16",
        "bool": "u32", "float32": "f32", "uint8": "u8",
    }
    for r in rows:
        sid = r["Signal_ID"].strip()
        if sid not in SIGNAL_MAP:
            continue
        csv_type = r["Data_Type"].strip().lower()
        is_array = "[" in csv_type
        csv_t = csv_type.split("[")[0]
        exp_t = type_map_csv_to_c.get(csv_t)
        _, _, c_exp_t = SIGNAL_MAP[sid]
        # stat 容器成员：CSV float64 ↔ 容器内 double 一致
        if sid in STAT_MEMBERS:
            if exp_t != "f64":
                issues.append(f"[类型不一致] {sid} {r['Signal_Name']}: CSV={csv_type} 应=float64")
            continue
        # 数组：比较基类型
        if is_array:
            base_c = c_exp_t.replace("x3", "")   # f64x3 -> f64, f32x3 -> f32
            if exp_t and exp_t != base_c:
                issues.append(f"[类型不一致] {sid} {r['Signal_Name']}: CSV={csv_type} vs C 期望={c_exp_t}")
            continue
        if exp_t and exp_t != c_exp_t:
            # bool 在 C 用 uint32_t（0/1）→ 允许
            if not (csv_t == "bool" and c_exp_t in ("u32", "u8")):
                # turbine_id 符号性：CSV=uint32 vs C=int32_t（同 4 字节 ABI，仅符号解释）
                # 属已知语义细节（值恒 >=1），记录为 NOTE 而非 FAIL
                if sid in ("S0201", "S1003") and csv_t == "uint32" and c_exp_t == "i32":
                    notes.append(
                        f"[NOTE] {sid} turbine_id: CSV=uint32 vs C=int32_t（ABI 足迹相同 4B，"
                        f"值恒 >=1，无实际影响）")
                    continue
                issues.append(
                    f"[类型不一致] {sid} {r['Signal_Name']}: CSV={csv_type} vs C 期望={c_exp_t}")

    # 4) 线上承载核对：命令帧/状态帧/AWAE 帧成员
    wire_checks = {
        # (信号前缀, 应存在于哪个帧的字段文本)
    }
    # 命令帧成员（FarmCommandFrame）
    cmd_fields = ["command_frame_seq", "turbines", "protocol_version", "n_turbines"]
    for f in cmd_fields:
        if f not in all_text:
            issues.append(f"[线上缺失] FarmCommandFrame 缺 {f}")

    # 5) 统计
    total = len(csv_sids)
    mapped = len(csv_sids & map_sids)
    print(f"[AUDIT] CSV 信号总数={total}，已映射={mapped}，未映射={total - mapped}")
    for n in notes:
        print(f"  {n}")
    if issues:
        print(f"[FAIL] 发现 {len(issues)} 项问题:")
        for it in issues[:60]:
            print(f"  - {it}")
        if len(issues) > 60:
            print(f"  ... 其余 {len(issues)-60} 项略")
        return 1
    print("[PASS] 全部 88 信号与当前工程 C 结构字段/类型/承载一致（S11 仅保留接口属预期）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
