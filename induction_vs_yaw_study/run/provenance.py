"""provenance.py — 可复现性元数据（审查 P2-1）

run 级：run_provenance.json（git commit、exe/bts/config hash、版本）
case 级：metadata.json（工况键、bts hash、时长参数+派生计数、约定版本）
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

_STUDY_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = _STUDY_ROOT.parent

# 约定版本（控制语义变更时递增/记录）
YAW_COMMAND_CONVENTION = "nacayaw_misdeg_v1"       # 失准角，NacYaw 系（对齐=0°）
INDUCTION_MAPPING_VERSION = "power_to_induction_v1"  # 下发 a = power_to_induction(P,U)


def git_commit() -> Optional[str]:
    try:
        out = subprocess.run(["git", "-C", str(_REPO_ROOT), "rev-parse", "HEAD"],
                             capture_output=True, text=True, timeout=10)
        return out.stdout.strip() or None
    except Exception:
        return None


def sha256(path: Path) -> Optional[str]:
    try:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()
    except Exception:
        return None


def fastfarm_exe_path() -> Path:
    return _REPO_ROOT / "wfcrl" / "simulators" / "fastfarm" / "bin" / "FAST.Farm_x64_OMP.exe"


def discon_dll_path() -> Path:
    """控制器 DLL（FCR 集成版）。运行时团队会重建它，故必须逐 case 记录。"""
    return _REPO_ROOT / "wfcrl" / "simulators" / "fastfarm" / "servo_dll" / "DISCON_WT1.dll"


_SHA_CACHE: dict = {}


def cached_sha256(path: Path) -> Optional[str]:
    """按 (path, mtime, size) 缓存的哈希：同一进程内多 case 不重复读 2.4MB。"""
    try:
        st = path.stat()
    except Exception:
        return None
    key = (str(path), st.st_mtime_ns, st.st_size)
    if key in _SHA_CACHE:
        return _SHA_CACHE[key]
    h = sha256(path)
    _SHA_CACHE[key] = h
    return h


def run_provenance(bts_file: Optional[str] = None) -> dict:
    exe = fastfarm_exe_path()
    bts_path = _STUDY_ROOT / "FarmInputs" / bts_file if bts_file else None
    return {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": git_commit(),
        "fastfarm_exe": str(exe),
        "fastfarm_exe_sha256": sha256(exe),
        "discon_dll": str(discon_dll_path()),
        "discon_dll_sha256": cached_sha256(discon_dll_path()),
        "floris_version": _pkg_ver("floris"),
        "openfast_toolbox_version": _pkg_ver("openfast_toolbox"),
        "bts_file": bts_file,
        "bts_sha256": sha256(bts_path) if bts_path and bts_path.exists() else None,
        "yaw_command_convention": YAW_COMMAND_CONVENTION,
        "induction_mapping_version": INDUCTION_MAPPING_VERSION,
    }


def _pkg_ver(module: str) -> Optional[str]:
    try:
        import importlib
        m = importlib.import_module(module)
        return getattr(m, "__version__", None)
    except Exception:
        return None


def write_run_provenance(run_dir: Path, bts_file: Optional[str] = None,
                         extra: Optional[dict] = None) -> Path:
    run_dir.mkdir(parents=True, exist_ok=True)
    data = run_provenance(bts_file)
    if extra:
        data.update(extra)
    p = run_dir / "provenance.json"
    p.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    return p


def write_case_metadata(case_dir: Path, row: dict, cfg_time: dict = None) -> Path:
    """case_dir 下写 metadata.json（行 + 派生时长）。"""
    case_dir.mkdir(parents=True, exist_ok=True)
    t_sim = float(cfg_time.get("t_sim_s", 1200.0)) if cfg_time else None
    t_settle = float(cfg_time.get("t_settle_s", 600.0)) if cfg_time else None
    dt = float(cfg_time.get("control_dt_s", 3.0)) if cfg_time else None
    n_total = round(t_sim / dt) if (t_sim and dt) else None
    n_stat = round((t_sim - t_settle) / dt) if (t_sim and t_settle and dt) else None
    bts_full = _STUDY_ROOT / "FarmInputs" / (row.get("bts_file") or "")
    meta = {
        "case_id": row.get("case_id"),
        "wind_speed_ms": float(row.get("U")) if row.get("U") else None,
        "ti": float(row.get("TI")) if row.get("TI") else None,
        "seed_id": int(row.get("seed")) if row.get("seed") else None,
        "wd_offset_deg": float(row.get("WD")) if row.get("WD") else None,
        "spacing_D": float(row.get("spacing_D")) if row.get("spacing_D") else None,
        "control": row.get("control"),
        "bts_file": row.get("bts_file"),
        "bts_sha256": sha256(bts_full) if bts_full.exists() else None,
        "discon_dll_sha256": cached_sha256(discon_dll_path()),
        "t_sim_s": t_sim, "t_settle_s": t_settle, "control_dt_s": dt,
        "n_total_control_steps": n_total, "n_statistics_steps": n_stat,
        "yaw_command_convention": YAW_COMMAND_CONVENTION,
        "induction_mapping_version": INDUCTION_MAPPING_VERSION,
        "git_commit": git_commit(),
    }
    p = case_dir / "metadata.json"
    p.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    return p
