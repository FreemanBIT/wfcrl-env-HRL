#!/usr/bin/env python3
"""check_abi.py — 打印/校验 farm_control_runtime ABI 尺寸（Phase 1）"""
import ctypes, pathlib, subprocess, sys, tempfile, textwrap

ROOT = pathlib.Path(__file__).resolve().parent.parent
TYPE_EXPECT = {
    # (C 类型名, 期望字节数)；期望值为空则仅打印
    "FcrTurbineCommand": 88,
    "FarmCommandFrame": 5672,
    "FcrSignalStats": 48,
    "FcrRoscoFastState": 232,
    "FcrRoscoExternalSetpoint": 96,
    "FcrTurbineExtraState": 840,
    "FcrFlowState": 5152,
    "FcrFarmTurbineState": 360,
    "FarmStateFrame": 23096,
}

def main() -> int:
    inc = ROOT / "include"
    src = textwrap.dedent("""\
    #include <stdio.h>
    #include "fcr_protocol.h"
    #include "fcr_state_types.h"
    #include "fcr_transport.h"
    #include "fcr_rosco_api.h"
    int main(void) {
    """)
    prog = src
    for t in TYPE_EXPECT:
        prog += f'    printf("{t} %zu\\n", sizeof({t}));\n'
    prog += "    return 0;\n}\n"
    tmp = tempfile.NamedTemporaryFile("w", suffix=".c", delete=False)
    tmp.write(prog); tmp.close()
    exe = tempfile.NamedTemporaryFile(suffix=".exe", delete=False).name
    try:
        r = subprocess.run(["gcc", "-std=c11", f"-I{inc}", "-o", exe, tmp.name],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stderr); return 1
        out = subprocess.check_output([exe], text=True)
    finally:
        pathlib.Path(tmp.name).unlink(missing_ok=True)
        pathlib.Path(exe).unlink(missing_ok=True)
    ok = True
    for line in out.strip().splitlines():
        name, _, size_s = line.partition(" ")
        size = int(size_s)
        exp = TYPE_EXPECT.get(name)
        status = "OK" if exp is None or exp == size else "MISMATCH"
        if status != "OK": ok = False
        print(f"{name:32s} {size:6d}  expected={exp}  {status}")
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
