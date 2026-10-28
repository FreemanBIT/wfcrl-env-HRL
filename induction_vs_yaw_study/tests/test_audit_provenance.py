"""溯源审计测试（G-21）：新 case 按 sha 归类，旧 case 按 mtime 划年代。"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

from induction_vs_yaw_study.run import audit_provenance as A


def _mk_run(tmp_path: Path) -> Path:
    run = tmp_path / "run"
    cases = run / "cases"
    cases.mkdir(parents=True)
    (run / "manifest_status.csv").write_text(
        "case_id,control,status\n"
        "U08_TI05_seed01_wd0_s2D,baseline,done\n"
        "U08_TI05_seed01_wd0_s2D,yaw,done\n"
        "U08_TI05_seed01_wd0_s2D,induction,failed\n",
        encoding="utf-8",
    )
    # 新 case：带 sha
    new_dir = cases / "U08_TI05_seed01_wd0_s2D_baseline"
    new_dir.mkdir()
    (new_dir / "timeseries.csv").write_text("time_s,farm_power_MW\n3.0,4.0\n", encoding="utf-8")
    (new_dir / "metadata.json").write_text(json.dumps({
        "discon_dll_sha256": "deadbeef" * 8,
        "git_commit": "1234567890abcdef",
        "bts_sha256": "aaa" * 21,
        "bts_file": "box_a.bts",
    }), encoding="utf-8")
    # 旧 case：无 sha + 旧 mtime
    old_dir = cases / "U08_TI05_seed01_wd0_s2D_yaw"
    old_dir.mkdir()
    ts = old_dir / "timeseries.csv"
    ts.write_text("time_s,farm_power_MW\n3.0,4.0\n", encoding="utf-8")
    old = time.mktime((2026, 10, 20, 12, 0, 0, 0, 0, -1))
    os.utime(ts, (old, old))
    (old_dir / "metadata.json").write_text(json.dumps({
        "git_commit": "abcdef1234",
        "bts_sha256": "aaa" * 21,
        "bts_file": "box_a.bts",
    }), encoding="utf-8")
    return run


def test_audit_splits_eras(tmp_path):
    a = A.audit(_mk_run(tmp_path))
    assert a["completed_cases"] == 2
    assert a["cases_with_dll_sha"] == 1
    assert "sha:deadbeefdeadbeef" in a["era"]
    assert any("pre-rebuild" in k for k in a["era"])


def test_audit_ignores_failed_and_missing_series(tmp_path):
    run = _mk_run(tmp_path)
    a = A.audit(run)
    assert a["status_rows"] == 3        # failed 行计数但不进 completed
    assert a["completed_cases"] == 2


def test_audit_detects_bts_sha_conflict(tmp_path):
    run = _mk_run(tmp_path)
    # 同名 bts 出现第二个 sha -> 应被标记
    d = run / "cases" / "U08_TI05_seed01_wd0_s2D_yaw"
    (d / "metadata.json").write_text(json.dumps({
        "git_commit": "abcdef1234", "bts_sha256": "bbb" * 21, "bts_file": "box_a.bts",
    }), encoding="utf-8")
    a = A.audit(run)
    assert "box_a.bts" in a["bts_sha_conflicts"]


def test_report_renders(tmp_path):
    a = A.audit(_mk_run(tmp_path))
    md = A.render_report(a)
    assert "# 溯源一致性审计" in md
    assert "DLL 年代分布" in md
