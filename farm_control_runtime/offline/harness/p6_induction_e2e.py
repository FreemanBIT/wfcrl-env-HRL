"""
Phase 6 端到端验证：induction_ref -> 功率降载（via ROSCO VS_RtPwr/ratio）
"""
import ctypes, os, sys, time, subprocess
import numpy as np
from openfast_toolbox.io.fast_output_file import FASTOutputFile

sys.path.insert(0, r"D:\HR_Project\wfcrl-env-HRL")
from wfcrl.config import WindConfig
from wfcrl.config.layout import LayoutRegistry
from wfcrl.config.simulator import FastFarmConfig
from wfcrl.engine import ContinuousFastFarmInterface

class FcrTurbineCommand(ctypes.Structure):
    _fields_ = [("yaw_delta_rad", ctypes.c_double), ("yaw_seq", ctypes.c_uint64), ("yaw_valid", ctypes.c_uint32), ("_p1", ctypes.c_uint32),
                ("yaw_effective_low_step", ctypes.c_int64), ("yaw_ttl_s", ctypes.c_double), ("induction_ref", ctypes.c_double),
                ("induction_seq", ctypes.c_uint64), ("induction_valid", ctypes.c_uint32), ("_p2", ctypes.c_uint32),
                ("induction_effective_low_step", ctypes.c_int64), ("induction_ttl_s", ctypes.c_double), ("source_time_s", ctypes.c_double)]
class FarmCommandFrame(ctypes.Structure):
    _fields_ = [("protocol_version", ctypes.c_uint32), ("_p0", ctypes.c_uint32), ("frame_seq", ctypes.c_uint64), ("commit_seq", ctypes.c_uint64),
                ("n_turbines", ctypes.c_uint32), ("_p3", ctypes.c_uint32), ("receive_time_s", ctypes.c_double),
                ("turbines", FcrTurbineCommand * 64)]

from ctypes import wintypes
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
kernel32.CreateFileMappingW.restype = wintypes.HANDLE
kernel32.CreateFileMappingW.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD, wintypes.DWORD, wintypes.LPCWSTR]
kernel32.MapViewOfFile.restype = ctypes.c_void_p
kernel32.MapViewOfFile.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD, wintypes.DWORD, ctypes.c_size_t]

SHM = "Local\\FCR_CMD_V1"

class Writer:
    def __init__(self):
        self.h = kernel32.CreateFileMappingW(-1, None, 0x04, 0, ctypes.sizeof(FarmCommandFrame), SHM)
        self.addr = kernel32.MapViewOfFile(self.h, 0x0006, 0, 0, ctypes.sizeof(FarmCommandFrame))
        self.frame = FarmCommandFrame.from_address(self.addr)
        self.seq = 0
    def send(self, ref, ind_seq, ttl=60.0):
        self.seq += 1
        f = self.frame
        f.protocol_version = 1; f.frame_seq = self.seq; f.commit_seq = self.seq; f.n_turbines = 3
        refs = ref if isinstance(ref, list) else [ref]*3
        for i in range(3):
            t = f.turbines[i]
            t.induction_valid = 1
            t.induction_seq = ind_seq
            t.induction_ref = refs[i]
            t.induction_ttl_s = ttl
            t.yaw_valid = 0
        time.sleep(0.05)

def main():
    out_root = "__simul__/fastfarm/p6_induction"
    os.makedirs(out_root, exist_ok=True)
    registry = LayoutRegistry.from_builtin()
    layout = registry.get("3T")
    wind = WindConfig(speed=8.0, direction=270.0)
    config = FastFarmConfig(case_name=layout.name, num_turbines=3,
                            xcoords=layout.xcoords, ycoords=layout.ycoords,
                            dt=3.0, max_iter=30, wind=wind, output_dir=out_root)
    ff = ContinuousFastFarmInterface(config)
    ff.setup()
    ff.reset(wind)
    farm_base = ff._farm_base
    from wfcrl.engine._outlist import _set_fast_scalar
    import glob
    for ed in glob.glob(os.path.join(farm_base, "*ElastoDyn*.dat")):
        try: _set_fast_scalar(ed, "YawDOF", "True")
        except Exception: pass
    exe = r"D:\HR_Project\wfcrl-env-HRL\wfcrl\simulators\fastfarm\bin\FAST.Farm_x64_OMP.exe"
    log = open(os.path.join(out_root, "fastfarm_p6.log"), "w", buffering=1)
    proc = subprocess.Popen([exe, os.path.basename(ff._fstf_file)], cwd=farm_base,
                            stdout=log, stderr=subprocess.STDOUT)
    print("FAST.Farm pid", proc.pid)
    w = Writer()
    # 命令时间表（现实时间 ≈ 仿真/2.1）：
    #   t=3s  (sim~6s) : seq1 a=0.20  <- 降载（power_ratio≈0.864）
    #   t=11s (sim~23s): seq2 a=0.30  <- 轻降载
    #   t=15s (sim~32s): dup seq2     <- 幂等
    #   t=17s (sim~36s): seq3 a=1.2   <- 越界 fallback（回额定）
    sched = [
        (3.0, lambda: w.send(0.20, 1)),
        (11.0, lambda: w.send(0.30, 2)),
        (15.0, lambda: w.send(0.30, 2)),
        (17.0, lambda: w.send(1.20, 3)),
    ]
    t0 = time.time(); ix = 0
    while proc.poll() is None:
        el = time.time() - t0
        if ix < len(sched) and el >= sched[ix][0]:
            print(f"[t={el:.1f}s] send #{ix+1}"); sched[ix][1](); ix += 1
        time.sleep(0.2)
    proc.wait(timeout=60); log.close()
    print("exit", proc.returncode)
    # 功率分析
    for tid in [1, 2, 3]:
        p = os.path.join(farm_base, f"Case.T{tid}.outb")
        if not os.path.exists(p): continue
        df = FASTOutputFile(p).toDataFrame()
        t = df["Time_[s]"].to_numpy()
        gp = df["GenPwr_[kW]"].to_numpy() / 1000.0
        def mean_between(a, b):
            m = (t >= a) & (t <= b)
            return float(gp[m].mean()) if m.sum() else float('nan')
        p_base = mean_between(2, 5)     # before commands (sim 2-5s)
        p_de1 = mean_between(16, 20)    # seq1 active (sim 16-20s)
        p_de2 = mean_between(27, 31)    # seq2 active
        p_fb  = mean_between(38, 41)    # fallback period
        print(f"T{tid}: P[2-5s]={p_base:.3f}MW  P[16-20s]={p_de1:.3f}MW  P[27-31s]={p_de2:.3f}MW  P[38-41s]={p_fb:.3f}MW")
        if p_base > 0:
            print(f"    ratios: seq1={p_de1/p_base:.3f} (expect ~0.864)  seq2={p_de2/p_base:.3f} (expect ~0.986)  fallback={p_fb/p_base:.3f} (expect ~1.0)")
    print("done")

if __name__ == "__main__":
    main()