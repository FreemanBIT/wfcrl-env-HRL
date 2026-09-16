"""test_provenance.py — P2-1 元数据纯逻辑测试
"""
from __future__ import annotations

import json

from induction_vs_yaw_study.run.provenance import (
    sha256, git_commit, run_provenance,
    YAW_COMMAND_CONVENTION, INDUCTION_MAPPING_VERSION,
)


def test_sha256_stable():
    import tempfile, os
    with tempfile.TemporaryDirectory() as d:
        f = os.path.join(d, "a.txt")
        with open(f, "w") as fh:
            fh.write("hello")
        h1 = sha256(__import__("pathlib").Path(f))
        h2 = sha256(__import__("pathlib").Path(f))
        assert h1 == h2 and len(h1) == 64


def test_git_commit_nonempty():
    c = git_commit()
    assert c and len(c) == 40


def test_run_provenance_structure():
    p = run_provenance(bts_file="coarse/inflow_U08_TI10_seed01.bts")
    assert p["git_commit"]
    assert p["bts_file"] == "coarse/inflow_U08_TI10_seed01.bts"
    assert p["yaw_command_convention"] == YAW_COMMAND_CONVENTION
    assert p["induction_mapping_version"] == INDUCTION_MAPPING_VERSION
    assert "fastfarm_exe_sha256" in p
    # json 可序列化
    json.dumps(p)
