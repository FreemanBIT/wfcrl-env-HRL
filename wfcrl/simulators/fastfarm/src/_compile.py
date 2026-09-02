"""编译 ROSCO + WFCRL Bridge → DISCON_WT1.dll

--mode rosco   : 传统 ROSCO + WFCRL controls.txt 桥（legacy，等价 baseline）
--mode farmcontrol : farm_control_runtime 新路径（-DFCR_FARM_CONTROL，默认）
"""
import subprocess, os, sys, shutil

SRC_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_DIR = os.path.join(SRC_DIR, "..", "servo_dll")
FCR_ROOT = os.path.normpath(os.path.join(SRC_DIR, "..", "..", "..", "..", "farm_control_runtime"))

# 默认：farmcontrol 模式
mode = "farmcontrol"
if len(sys.argv) > 1 and sys.argv[1] == "--mode":
    mode = sys.argv[2]
if len(sys.argv) > 1 and sys.argv[1] == "rosco":
    mode = "rosco"

f90_file = os.path.join(SRC_DIR, "DISCON.F90")
dll_file = os.path.join(DLL_DIR, "DISCON_WT1.dll")

if not os.path.exists(f90_file):
    print(f"ERROR: {f90_file} not found"); sys.exit(1)

def find_tool(name, paths):
    for p in paths:
        try:
            r = subprocess.run([p, "--version"], capture_output=True, timeout=10, encoding="utf-8", errors="replace")
            if r.returncode == 0:
                print(f"Found {name}: {p}"); return p
        except Exception:
            continue
    shutil.which(name) and print(f"Found {name}: {shutil.which(name)}")
    return shutil.which(name)

gfortran = find_tool("gfortran", [
    r"D:\TDM-GCC-64\bin\gfortran.exe",
    r"C:\mingw64\bin\gfortran.exe",
    r"C:\msys64\mingw64\bin\gfortran.exe",
    "gfortran",
])
if gfortran is None:
    print("ERROR: gfortran not found. Install MinGW-w64 or TDM-GCC."); sys.exit(1)

if mode == "farmcontrol":
    gcc = find_tool("gcc", [
        r"D:\TDM-GCC-64\bin\gcc.exe",
        r"C:\mingw64\bin\gcc.exe",
        "gcc",
    ])
    if gcc is None:
        print("ERROR: gcc not found (required for farm_control_runtime C core)."); sys.exit(1)

FLAGS = "-ffree-line-length-0 -static-libgcc -static-libgfortran -static -fdefault-real-8 -fdefault-double-8 -cpp -DIMPLICIT_DLLEXPORT -O2"

f90_sources = [
    "Constants.f90", "ROSCO_Types.f90", "SysGnuWin.f90",
    "Filters.f90", "Functions.f90", "ControllerBlocks.f90",
    "ROSCO_Helpers.f90", "ReadSetParameters.f90", "ROSCO_IO.f90",
    "ExtControl.f90", "ZeroMQInterface.f90",
]
# Controllers 在 FCR 路径下 USE FarmControlInterface，须在桥模块之后编译
controllers_entry = ["Controllers.f90"]
# DISCON.F90 依赖 FarmControlInterface（module 先后顺序），由各模式放最后
dll_entry = ["DISCON.F90"]

cmd = [gfortran, "-shared", "-static"] + FLAGS.split() + ["-o", dll_file]

if mode == "farmcontrol":
    # 1) 编译 C 核心（farm_control_runtime）
    c_sources = [
        "angle_convention.c", "command_store.c", "state_store.c",
        "watchdog.c", "runtime.c", "rosco_api.c",
        "signal_statistics.c", "state_aggregator.c", "induction_supervisor.c",
        "shm_command_source.c",
        "offline_fastfarm_provider.c",
    ]
    c_objs = []
    # C++ ZMQ transport（Phase 7；Windows 动态加载 libzmq）
    zmq_cpp = os.path.join(FCR_ROOT, "transport", "zmq", "zmq_transport.cpp")
    if os.path.exists(zmq_cpp):
        gpp = find_tool("g++", [r"D:\TDM-GCC-64\bin\g++.exe", r"C:\mingw64\bin\g++.exe", "g++"])
        if gpp is None:
            print("ERROR: g++ not found (required for zmq_transport.cpp)."); sys.exit(1)
        zmq_obj = os.path.join(SRC_DIR, "zmq_transport.o")
        rcpp = subprocess.run([gpp, "-std=c++17", "-O2", "-c", zmq_cpp,
                               "-I", os.path.join(FCR_ROOT, "include"),
                               "-I", os.path.join(FCR_ROOT, "src"),
                               "-o", zmq_obj], cwd=SRC_DIR, capture_output=True,
                              encoding="utf-8", errors="replace")
        if rcpp.returncode != 0:
            print("C++ compile FAIL (zmq_transport.cpp):"); print(rcpp.stderr); sys.exit(1)
        c_objs.append(zmq_obj)
    for src in c_sources:
        obj = os.path.join(SRC_DIR, os.path.splitext(src)[0] + ".o")
        src_file = os.path.join(FCR_ROOT, "src", src)
        if not os.path.exists(src_file):
            src_file = os.path.join(FCR_ROOT, "offline", "harness", src)
        if not os.path.exists(src_file):
            src_file = os.path.join(FCR_ROOT, "offline", "provider", src)
        if not os.path.exists(src_file):
            print(f"C source not found: {src}"); sys.exit(1)
        r = subprocess.run([gcc, "-std=c11", "-O2", "-c",
                            src_file,
                            "-I", os.path.join(FCR_ROOT, "include"),
                            "-I", os.path.join(FCR_ROOT, "src"),
                            "-o", obj], cwd=SRC_DIR, capture_output=True, encoding="utf-8", errors="replace")
        if r.returncode != 0:
            print(f"C compile FAIL ({src}):"); print(r.stderr); sys.exit(1)
        c_objs.append(obj)
    cmd += c_objs
    cmd += ["-D", "FCR_FARM_CONTROL"]
    cmd += ["-I", os.path.join(FCR_ROOT, "rosco")]
    cmd += ["-I", SRC_DIR]
    f90_sources = f90_sources + [
        os.path.join(FCR_ROOT, "rosco", "FarmControlCBindings.f90"),
        os.path.join(FCR_ROOT, "rosco", "FarmControlInterface.f90"),
        "Controllers.f90",
    ] + dll_entry
    controllers_entry = []
    print("== farm_control_runtime integration build (FCR_FARM_CONTROL) ==")
else:
    print("== legacy ROSCO + WFCRL bridge build ==")
    cmd += ["-UFCR_FARM_CONTROL"]
    f90_sources = f90_sources + controllers_entry + dll_entry

cmd += f90_sources
cmd += ["-lstdc++"]   # C++ ZMQ transport 运行时
cmd += ["-lws2_32"]
print(f"Running: {' '.join(cmd[:6])} ... ({len(cmd)} args)")
result = subprocess.run(cmd, cwd=SRC_DIR, capture_output=True, encoding="utf-8", errors="replace", timeout=300)

# 清理中间 .o/.mod
for f in os.listdir(SRC_DIR):
    if f.endswith(".o") or f.endswith(".mod"):
        os.remove(os.path.join(SRC_DIR, f))

if result.returncode == 0:
    size_kb = os.path.getsize(dll_file) / 1024
    print(f"OK: {dll_file} ({size_kb:.1f} KB)")
else:
    print(f"FAIL (code {result.returncode}):")
    print(result.stderr or result.stdout)
    sys.exit(1)