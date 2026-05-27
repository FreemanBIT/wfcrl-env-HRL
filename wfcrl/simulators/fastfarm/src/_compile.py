"""编译 ROSCO + WFCRL Bridge → DISCON_WT1.dll"""
import subprocess, os, sys

SRC_DIR = os.path.dirname(os.path.abspath(__file__))
DLL_DIR = os.path.join(SRC_DIR, "..", "servo_dll")

f90_file = os.path.join(SRC_DIR, "DISCON.F90")
dll_file = os.path.join(DLL_DIR, "DISCON_WT1.dll")

if not os.path.exists(f90_file):
    print(f"ERROR: {f90_file} not found")
    sys.exit(1)

# 查找 gfortran
gfortran_paths = [
    r"D:\TDM-GCC-64\bin\gfortran.exe",
    r"C:\mingw64\bin\gfortran.exe",
    r"C:\msys64\mingw64\bin\gfortran.exe",
    "gfortran",
]

gfortran = None
for p in gfortran_paths:
    try:
        r = subprocess.run([p, "--version"], capture_output=True, timeout=10)
        if r.returncode == 0:
            gfortran = p
            print(f"Found gfortran: {p}")
            break
    except Exception:
        continue

if gfortran is None:
    print("ERROR: gfortran not found. Install MinGW-w64 or TDM-GCC.")
    sys.exit(1)

# 编译
cmd = [gfortran, "-shared", "-static", "-o", dll_file, f90_file]
print(f"Running: {' '.join(cmd)}")
result = subprocess.run(cmd, cwd=SRC_DIR, capture_output=True, text=True, timeout=120)

if result.returncode == 0:
    size_kb = os.path.getsize(dll_file) / 1024
    print(f"OK: {dll_file} ({size_kb:.1f} KB)")
else:
    print(f"FAIL (code {result.returncode}):")
    print(result.stderr or result.stdout)
    sys.exit(1)
