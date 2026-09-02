# -*- coding: utf-8 -*-
"""package_realtime_delivery.py — 实时交付包生成（Phase 11）

收集：
  - farm_control_runtime/   （core lib、include、transport、tests、docs）
  - wfcrl/transport、wfcrl/engine/fastfarm_zmq.py、wfcrl/simulators/fastfarm/src（宏开关集成层）
  - servo_dll/DISCON_WT1.dll（FCR_FARM_CONTROL 构建）
  - transport/zmq/bin/libzmq.dll
  - docs/farm_control/*.md
输出：dist/wfcrl-rt-delivery-<ver>.zip
"""
import os
import shutil
import zipfile
from datetime import datetime

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
VER = "1.0.0-phase11"
OUT = os.path.join(ROOT, "dist")
ZIP = os.path.join(OUT, f"wfcrl-rt-delivery-{VER}.zip")

INCLUDES = [
    ("farm_control_runtime/src", "runtime/core/src"),
    ("farm_control_runtime/include", "runtime/core/include"),
    ("farm_control_runtime/rosco", "runtime/core/rosco"),
    ("farm_control_runtime/offline/provider", "runtime/core/offline/provider"),
    ("farm_control_runtime/offline/harness", "runtime/core/offline/harness"),
    ("farm_control_runtime/transport/zmq", "runtime/core/transport/zmq"),
    ("farm_control_runtime/transport/modbus", "runtime/core/transport/modbus"),
    ("farm_control_runtime/config", "runtime/core/config"),
    ("farm_control_runtime/tests/unit", "runtime/core/tests/unit"),
    ("farm_control_runtime/tests/integration", "runtime/core/tests/integration"),
    ("farm_control_runtime/tests/fixtures", "runtime/core/tests/fixtures"),
    ("wfcrl/transport", "python/wfcrl_transport"),
    ("wfcrl/engine/fastfarm_zmq.py", "python/wfcrl_engine_fastfarm_zmq.py"),
    ("wfcrl/simulators/fastfarm/src/DISCON.F90", "fastfarm_src/DISCON.F90"),
    ("wfcrl/simulators/fastfarm/src/Controllers.f90", "fastfarm_src/Controllers.f90"),
    ("wfcrl/simulators/fastfarm/src/FarmControlCBindings.f90", "fastfarm_src/FarmControlCBindings.f90"),
    ("wfcrl/simulators/fastfarm/src/_compile.py", "fastfarm_src/_compile.py"),
    ("wfcrl/simulators/fastfarm/servo_dll/DISCON_WT1.dll", "bin/DISCON_WT1.dll"),
    ("wfcrl/simulators/fastfarm/servo_dll/DISCON_ROSCO_TEMPLATE.IN", "bin/DISCON_ROSCO_TEMPLATE.IN"),
    ("docs/farm_control", "docs"),
    ("farm_control_runtime/README.md", "runtime/core/README.md"),
    ("farm_control_runtime/CMakeLists.txt", "runtime/core/CMakeLists.txt"),
    ("farm_control_runtime/tools/build_windows.ps1", "runtime/core/build_windows.ps1"),
    ("farm_control_runtime/tools/build_linux.sh", "runtime/core/build_linux.sh"),
    ("farm_control_runtime/tools/check_abi.py", "runtime/core/check_abi.py"),
]

SKIP_SUFFIXES = (".o", ".mod", ".exe", ".pyc", ".pytest_cache")


def main():
    if os.path.exists(OUT):
        shutil.rmtree(OUT)
    os.makedirs(OUT)
    n = 0
    with zipfile.ZipFile(ZIP, "w", zipfile.ZIP_DEFLATED) as z:
        for src, dst in INCLUDES:
            full = os.path.join(ROOT, src)
            if os.path.isfile(full):
                if full.endswith(SKIP_SUFFIXES):
                    continue
                z.write(full, os.path.join("wfcrl-rt", dst, os.path.basename(full)))
                n += 1
            elif os.path.isdir(full):
                for dp, _, fs in os.walk(full):
                    for f in fs:
                        if f.endswith(SKIP_SUFFIXES) or f in (".gitignore",):
                            continue
                        p = os.path.join(dp, f)
                        rel = os.path.relpath(p, full)
                        z.write(p, os.path.join("wfcrl-rt", dst, rel))
                        n += 1
            else:
                print(f"[warn] missing: {src}")
        manifest = f"""wfcrl FAST.Farm RT delivery {VER}
generated: {datetime.now().isoformat(timespec="seconds")}
repo: https://github.com/FreemanBIT/wfcrl-env-HRL (branch: dev/farm-control-runtime)
see docs/REALTIME_DELIVERY.md for contents & verification
"""
        z.writestr("wfcrl-rt/MANIFEST.txt", manifest)
    print(f"packed {n} files -> {ZIP} ({os.path.getsize(ZIP)//1024} KB)")


if __name__ == "__main__":
    main()
