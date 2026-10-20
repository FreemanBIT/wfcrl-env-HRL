#!/usr/bin/env python3
"""check_abi.py — 校验 farm_control_runtime ABI（ICD v1.5）

三件事：
  1. 编译 C 探针，实测各公共结构体 sizeof，与冻结值比对；
  2. 逐字段交叉校验 Python ctypes 镜像（wfcrl/transport/farm_protocol.py）
     的**字段偏移**与 C 侧 offsetof 完全一致；
  3. 校验 Python 镜像自身的尺寸断言。

尺寸/偏移不一致会导致 ZMQ 帧被全部拒收或按错误偏移解析出垃圾数据
（历史上发生过，见 WIN-05）。任何 MISMATCH 都表示 ABI 破坏，必须同步更新
本文件 + tests/unit/test_abi.c + wfcrl/transport/farm_protocol.py + ICD。

期望值为 ICD v1.5 冻结值（64 位 TDM-GCC-64 10.3.0 实测，2026-10-19）。
"""
import ctypes
import pathlib
import subprocess
import sys
import tempfile
import textwrap

ROOT = pathlib.Path(__file__).resolve().parent.parent      # farm_control_runtime/
REPO_ROOT = ROOT.parent                                    # wfcrl-env-HRL/

# C 类型名 → 冻结字节数
TYPE_EXPECT = {
    # ---- 命令链路 (S01) ----
    "FcrTurbineCommand": 88,
    "FarmCommandFrame": 5664,
    # ---- 状态链路 (S02/S03) ----
    "FcrOpenFASTRawState": 104,
    "FcrRoscoRawState": 232,
    "FcrSignalStats": 48,
    "FcrTurbineExtraState": 840,
    "FcrFlowState": 5152,
    # ---- Bridge 链路 (S07/S08/S09) ----
    "FcrBridgeInput": 72,
    "FcrBridgeOutput": 56,
    "FcrBridgeAck": 32,
    "FcrRoscoExternalSetpoint": 112,
    "FcrRoscoAck": 32,
    # ---- 状态帧 (S10) ----
    "FcrFarmTurbineState": 400,
    "FarmStateFrame": 25664,
    # ---- AWAE (S04/S05) + 风估计 (S06) ----
    "FcrAWAECatalogEntry": 40,
    "FcrAWAECatalogFrame": 2584,
    "FcrAWAEObservation": 48,
    "FcrAWAEObservationFrame": 3096,
    "FcrTurbineWindEstimate": 32,
    # ---- 诱导映射 ----
    "FcrInductionMapConfig": 56,
    "FcrInductionMapOutput": 56,
    "FcrInductionState": 80,
}

# 需要与 Python 镜像逐字段对齐的结构体（C 名 == Python 名）
MIRRORED_TYPES = [
    "FcrTurbineCommand",
    "FarmCommandFrame",
    "FcrSignalStats",
    "FcrFarmTurbineState",
    "FarmStateFrame",
    # W-08: AWAE 二跳帧（GG-12/GG-13）镜像
    "FcrAWAECatalogEntry",
    "FcrAWAECatalogFrame",
    "FcrAWAEObservation",
    "FcrAWAEObservationFrame",
]

# 逐字段偏移校验清单：(C 类型名, 字段名)
FIELD_OFFSETS = [
    ("FcrTurbineCommand", f) for f in (
        "yaw_target_heading_rad", "yaw_seq", "yaw_valid",
        "induction_ref", "induction_seq", "induction_valid",
        "control_enable", "control_mode", "requested_effective_sim_time_s",
        "command_basis_sim_time_s", "yaw_ttl_s", "induction_ttl_s",
    )
] + [
    ("FarmCommandFrame", f) for f in (
        "protocol_version", "command_frame_seq", "n_turbines",
        "receive_time_s", "turbines",
    )
] + [
    ("FcrSignalStats", f) for f in (
        "instant", "mean", "rms", "min", "max", "n_samples", "valid",
    )
] + [
    ("FcrFarmTurbineState", f) for f in (
        "turbine_id", "valid", "source_time_s",
        "gen_power_w", "rotor_aero_thrust_n",
        "tower_base_fa_moment_nm", "tower_base_ss_moment_nm",
        "rotor_speed_rad_s", "gen_speed_rad_s",
        "collective_pitch_actual_rad", "nacelle_heading_actual_rad",
        "estimated_wind_mps", "wind_source", "wind_quality_flags", "ct_hat",
        "yaw_seq_applied", "induction_seq_applied", "turbine_status_flags",
        "nacelle_vane_rad", "hub_wind_speed_mps", "rotor_ct", "rotor_cp",
        "blade_pitch_rad", "yaw_target_heading_rad", "controller_status_flags",
        "disk_ambient_wind_mps", "disk_disturbed_wind_mps", "disk_ambient_ti",
        "flow_source_time_s", "flow_seq",
    )
] + [
    ("FarmStateFrame", f) for f in (
        "protocol_version", "state_frame_seq", "state_sample_sim_time_s",
        "fast_step", "low_step", "rt_health_flags", "n_turbines",
        "flow_source_time_s", "flow_seq", "turbines",
    )
] + [
    # W-08: AWAE 二跳帧字段偏移（GG-12/GG-13）
    ("FcrAWAECatalogEntry", f) for f in (
        "point_id", "point_position_xyz_m", "point_catalog_version",
    )
] + [
    ("FcrAWAECatalogFrame", f) for f in (
        "protocol_version", "frame_seq", "n_points", "entries",
    )
] + [
    ("FcrAWAEObservation", f) for f in (
        "observation_frame_seq", "observation_sample_sim_time_s",
        "observation_point_count", "point_id", "wind_uvw_mps",
        "observation_quality_flags", "turbulence_intensity",
    )
] + [
    ("FcrAWAEObservationFrame", f) for f in (
        "protocol_version", "frame_seq", "n_observations", "observations",
    )
]


def probe_c() -> tuple:
    """编译并运行 C 探针，返回 ({类型: sizeof}, {(类型, 字段): offset})"""
    inc = ROOT / "include"
    prog = textwrap.dedent("""\
    #include <stdio.h>
    #include <stddef.h>
    #include "fcr_protocol.h"
    #include "fcr_state_types.h"
    #include "fcr_awae_types.h"
    #include "fcr_transport.h"
    #include "fcr_rosco_api.h"
    int main(void) {
    """)
    for t in TYPE_EXPECT:
        prog += f'    printf("SIZE {t} %zu\\n", sizeof({t}));\n'
    for t, f in FIELD_OFFSETS:
        prog += f'    printf("OFF {t} {f} %zu\\n", offsetof({t}, {f}));\n'
    prog += "    return 0;\n}\n"

    tmp = tempfile.NamedTemporaryFile("w", suffix=".c", delete=False)
    tmp.write(prog)
    tmp.close()
    exe = tempfile.NamedTemporaryFile(suffix=".exe", delete=False).name
    try:
        r = subprocess.run(["gcc", "-std=c11", f"-I{inc}", "-o", exe, tmp.name],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print("[check_abi] C probe compile FAILED:")
            print(r.stderr)
            sys.exit(2)
        out = subprocess.check_output([exe], text=True)
    finally:
        pathlib.Path(tmp.name).unlink(missing_ok=True)
        pathlib.Path(exe).unlink(missing_ok=True)

    sizes, offsets = {}, {}
    for line in out.strip().splitlines():
        parts = line.split()
        if parts[0] == "SIZE":
            sizes[parts[1]] = int(parts[2])
        elif parts[0] == "OFF":
            offsets[(parts[1], parts[2])] = int(parts[3])
    return sizes, offsets


def import_python_mirror():
    """导入 Python ctypes 镜像模块"""
    sys.path.insert(0, str(REPO_ROOT))
    try:
        from wfcrl.transport import farm_protocol as fp
        return fp
    except Exception as e:                                  # noqa: BLE001
        print(f"[check_abi] cannot import Python mirror wfcrl.transport.farm_protocol: {e}")
        print("[check_abi] hint: run with the project venv python "
              r"(D:\HR_Project\wfcrl-env-HRL\.venv\Scripts\python.exe)")
        return None


def check_python_offsets(fp, c_offsets: dict) -> bool:
    """逐字段比较 Python ctypes 偏移 vs C offsetof"""
    ok = True
    for (tname, fname) in FIELD_OFFSETS:
        cls = getattr(fp, tname, None)
        if cls is None:
            print(f"  [MISS] python class {tname} not found")
            ok = False
            continue
        try:
            py_off = getattr(cls, fname).offset
        except AttributeError:
            print(f"  [MISS] python {tname}.{fname} not found")
            ok = False
            continue
        c_off = c_offsets.get((tname, fname))
        if py_off != c_off:
            print(f"  [MISMATCH] {tname}.{fname}: python={py_off} c={c_off}")
            ok = False
    if ok:
        print(f"python<->C field offsets: all {len(FIELD_OFFSETS)} fields OK")
    return ok


def main() -> int:
    sizes, c_offsets = probe_c()
    ok = True

    for name, exp in TYPE_EXPECT.items():
        size = sizes.get(name)
        if size is None:
            print(f"{name:32s} {'--':>6s}  expected={exp}  MISSING")
            ok = False
            continue
        status = "OK" if size == exp else "MISMATCH"
        if status != "OK":
            ok = False
        print(f"{name:32s} {size:6d}  expected={exp}  {status}")

    print("-" * 64)
    fp = import_python_mirror()
    if fp is None:
        ok = False
    else:
        # Python 镜像自身的尺寸
        for tname in MIRRORED_TYPES:
            cls = getattr(fp, tname)
            py_size = ctypes.sizeof(cls)
            exp = TYPE_EXPECT[tname]
            status = "OK" if py_size == exp else "MISMATCH"
            if status != "OK":
                ok = False
            print(f"{'python:' + tname:32s} {py_size:6d}  expected={exp}  {status}")
        print("-" * 64)
        if not check_python_offsets(fp, c_offsets):
            ok = False

    print("-" * 64)
    print("ABI CHECK:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
