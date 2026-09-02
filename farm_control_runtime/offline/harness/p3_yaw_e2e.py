"""
离线端到端验证脚本：偏航增量动作（Phase 3 验收）。
用法: python p3_yaw_e2e.py   (需要 FAST.Farm 与 FCR 集成 DLL)
成果: 打印每机 YawPzn 关键点；
  Phase 3 验收记录（方向/latch/seq）见 docs/farm_control/OFFLINE_VALIDATION.md。
""";
"""
Phase 3 验证：偏航增量动作端到端（共享内存命令通道）

流程：
  1) 用 wfcrl 模板生成 Row3T（3 机）case（新模板 YCMode=5 / Y_ControlMode=1）；
  2) 启动 FAST.Farm（FCR DLL）；
  3) shm 注入 yaw 命令序列（seq/delta 时间表）；
  4) 等仿真结束，解析 Case.T*.outb 的 YawPzn 验证方向/latch/seq。
"""
import ctypes, os, sys, time, subprocess, struct
from ctypes import wintypes

import numpy as np
from openfast_toolbox.io.fast_output_file import FASTOutputFile

sys.path.insert(0, r"D:\HR_Project\wfcrl-env-HRL")
from wfcrl.config import WindConfig
from wfcrl.config.layout import LayoutRegistry
from wfcrl.config.simulator import FastFarmConfig
from wfcrl.engine import ContinuousFastFarmInterface

# ---------------------------------------------------------------------------
# FarmCommandFrame ctypes 镜像（与 fcr_state_types.h 一致；ABI 冻结）
# ---------------------------------------------------------------------------
class FcrTurbineCommand(ctypes.Structure):
    _fields_ = [
        ("yaw_delta_rad", ctypes.c_double),
        ("yaw_seq", ctypes.c_uint64),
        ("yaw_valid", ctypes.c_uint32),
        ("_pad1", ctypes.c_uint32),
        ("yaw_effective_low_step", ctypes.c_int64),
        ("yaw_ttl_s", ctypes.c_double),
        ("induction_ref", ctypes.c_double),
        ("induction_seq", ctypes.c_uint64),
        ("induction_valid", ctypes.c_uint32),
        ("_pad2", ctypes.c_uint32),
        ("induction_effective_low_step", ctypes.c_int64),
        ("induction_ttl_s", ctypes.c_double),
        ("source_time_s", ctypes.c_double),
    ]

class FarmCommandFrame(ctypes.Structure):
    _fields_ = [
        ("protocol_version", ctypes.c_uint32),
        ("_pad0", ctypes.c_uint32),
        ("frame_seq", ctypes.c_uint64),
        ("commit_seq", ctypes.c_uint64),
        ("n_turbines", ctypes.c_uint32),
        ("_pad3", ctypes.c_uint32),
        ("receive_time_s", ctypes.c_double),
        ("turbines", FcrTurbineCommand * 64),
    ]

assert ctypes.sizeof(FcrTurbineCommand) == 88, ctypes.sizeof(FcrTurbineCommand)
assert ctypes.sizeof(FarmCommandFrame) == 5672, ctypes.sizeof(FarmCommandFrame)

# ---------------------------------------------------------------------------
# 共享内存封装
# ---------------------------------------------------------------------------
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

CreateFileMappingW = kernel32.CreateFileMappingW
CreateFileMappingW.restype = wintypes.HANDLE
CreateFileMappingW.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD,
                               wintypes.DWORD, wintypes.DWORD, wintypes.LPCWSTR]
MapViewOfFile = kernel32.MapViewOfFile
MapViewOfFile.restype = ctypes.c_void_p
MapViewOfFile.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD,
                          wintypes.DWORD, ctypes.c_size_t]

SHM_NAME = "Local\\FCR_CMD_V1"
SHM_SIZE = ctypes.sizeof(FarmCommandFrame)

class ShmCommandWriter:
    def __init__(self):
        self.handle = CreateFileMappingW(-1, None, 0x04, 0, SHM_SIZE, SHM_NAME)
        if not self.handle:
            raise OSError("CreateFileMappingW failed", ctypes.get_last_error())
        addr = MapViewOfFile(self.handle, 0x0006, 0, 0, SHM_SIZE)  # FILE_MAP_WRITE
        if not addr:
            raise OSError("MapViewOfFile failed", ctypes.get_last_error())
        self.frame = FarmCommandFrame.from_address(addr)
        self.seq = 0

    def send_yaw(self, delta_deg, yaw_seq, ttl_s=60.0, n_turbines=1):
        self.seq += 1
        f = self.frame
        f.protocol_version = 1
        f.frame_seq = self.seq
        f.commit_seq = self.seq
        f.n_turbines = n_turbines
        f.receive_time_s = time.time()
        for i in range(n_turbines):
            t = f.turbines[i]
            if i == 0:
                t.yaw_valid = 1
                t.yaw_seq = yaw_seq
                t.yaw_delta_rad = np.deg2rad(delta_deg)
                t.yaw_ttl_s = ttl_s
                t.source_time_s = 0.0   # 由 runtime 侧时钟判断
            else:
                t.yaw_valid = 0
                t.induction_valid = 0
        time.sleep(0.05)  # 给 DLL 线程一个轮询周期
        return self.seq

# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main():
    out_root = os.path.join("__simul__", "fastfarm", "p3_yawtest")
    os.makedirs(out_root, exist_ok=True)

    registry = LayoutRegistry.from_builtin()
    layout = registry.get("3T")
    n_turbs = layout.num_turbines
    wind = WindConfig(speed=8.0, direction=270.0)
    config = FastFarmConfig(case_name=layout.name, num_turbines=n_turbs,
                            xcoords=layout.xcoords, ycoords=layout.ycoords,
                            dt=3.0, max_iter=14, wind=wind, output_dir=out_root)

    ff = ContinuousFastFarmInterface(config)
    ff.setup()
    ff.reset(wind)
    farm_base = ff._farm_base

    # set_fixed_yaw(0) 把 YawDOF 锁为 False；Phase 3 需要解锁 DOF 让 DLL 偏航命令生效
    from wfcrl.engine._outlist import _set_fast_scalar
    import glob as _glob
    for ed in _glob.glob(os.path.join(farm_base, "*ElastoDyn*.dat")) + _glob.glob(os.path.join(farm_base, "**", "*ElastoDyn*.dat"), recursive=True):
        try:
            _set_fast_scalar(ed, "YawDOF", "True")
        except Exception as e:
            print("skip", ed, e)
    print("YawDOF unlocked, case dir:", farm_base)

    exe = r"D:\HR_Project\wfcrl-env-HRL\wfcrl\simulators\fastfarm\bin\FAST.Farm_x64_OMP.exe"
    log = open(os.path.join(out_root, "fastfarm_p3.log"), "w", buffering=1)
    proc = subprocess.Popen([exe, os.path.basename(ff._fstf_file)],
                            cwd=farm_base, stdout=log, stderr=subprocess.STDOUT)
    print("FAST.Farm started pid", proc.pid)

    writer = ShmCommandWriter()
    print("shm writer ready")

    # 命令时间表用"现实时间"；仿真加速比约 2.1x（42s 仿真 ≈ 20s 现实）。
    # 仿真时刻 ≈ 现实时刻 × 2.1。
    #   t=4s  : seq=1, +10° CW   → target = 0 - 10 = -10°（内部）
    #   t=16s : seq=2, -15° CW   → target = 当前 + 15°（内部）
    #   t=28s : seq=3, +5° CW    → target = 当前 - 5°
    #   t=34s : 重复 seq=3 不同 delta（幂等验证：target 不变）
    schedule = [
        (4.0,  "seq1", lambda: writer.send_yaw(+10.0, 1)),
        (9.0,  "seq2", lambda: writer.send_yaw(-15.0, 2)),
        (13.0, "seq3", lambda: writer.send_yaw(+5.0, 3)),
        (15.0, "dup3", lambda: writer.send_yaw(+30.0, 3)),  # 重复 seq，应被忽略
    ]
    t0 = time.time()
    ix = 0
    while proc.poll() is None:
        el = time.time() - t0
        if ix < len(schedule) and el >= schedule[ix][0]:
            _, name, fn = schedule[ix]
            fn()
            print(f"[t={el:.1f}s] sent {name}")
            ix += 1
        time.sleep(0.2)
    proc.wait(timeout=30)
    log.close()
    print("FAST.Farm exit", proc.returncode)

    # 解析每机输出
    for tid in range(1, n_turbs + 1):
        p = os.path.join(farm_base, f"Case.T{tid}.outb")
        if not os.path.exists(p):
            print(f"T{tid}: no output"); continue
        df = FASTOutputFile(p).toDataFrame()
        t = df["Time_[s]"].to_numpy()
        yaw = df["YawPzn_[deg]"].to_numpy()
        rot = df["RotSpeed_[rpm]"].to_numpy()
        # 采样关键点
        def at(tt):
            i = int(np.argmin(np.abs(t - tt)))
            return yaw[i]
        print(f"T{tid}: yaw@4s={at(4):.2f}  yaw@12s={at(12):.2f}  yaw@20s={at(20):.2f}  "
              f"yaw@28s={at(28):.2f}  yaw@36s={at(36):.2f}  yaw@41s={at(41):.2f}")
        print(f"T{tid}: yaw min={yaw.min():.2f} max={yaw.max():.2f} final={yaw[-1]:.2f}  rot(final)={rot[-1]:.1f}rpm")
    print("done")

if __name__ == "__main__":
    main()