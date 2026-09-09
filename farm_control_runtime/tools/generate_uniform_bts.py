# -*- coding: utf-8 -*-
"""generate_uniform_bts.py — 生成周期性均匀 8 m/s 合成的 TurbSim .bts（长时宽盒）

用途：FAST.Farm 稳态（WindType=1）在 Grid4D 时间维上于 ~260 s 越界中止；
改用 TurbSim bts 路径（WindType=3, 周期性 ID=7）后 InflowWind 时间维按周期循环，
可支撑任意时长仿真。这里生成均匀场（u=8, v=w=0），物理上等效稳态入流。
"""
import numpy as np
import os, sys
sys.path.insert(0, r'D:\HR_Project\wfcrl-env-HRL')
from openfast_toolbox.io.turbsim_file import TurbSimFile

def main():
    U = 8.0
    t_end = 700.0
    dt = 0.1
    dY = 9.0
    dZ = 9.0
    y_half = 945.0            # 覆盖 6T 列展宽 + 3D 蜿蜒余量
    y = np.arange(-y_half, y_half + dY/2, dY)
    z = np.arange(0.0, 340.0 + dZ/2, dZ)
    t = np.arange(0.0, t_end + dt/2, dt)
    ny, nz, nt = len(y), len(z), len(t)
    u = np.full((3, nt, ny, nz), U, dtype=np.float32)
    u[1] = 0.0; u[2] = 0.0
    out = TurbSimFile()
    out['y'] = y; out['z'] = z; out['t'] = t; out['u'] = u
    out['ID'] = 8    # 与模板 90m_08mps.bts 一致（FAST.Farm 可用的周期 full-field）
    out['zRef'] = 90.0; out['uRef'] = U
    try:
        out['zTwr'] = np.array([]); out['uTwr'] = np.zeros((3, nt, 0))
    except Exception:
        pass
    p = r'D:\HR_Project\wfcrl-env-HRL\wfcrl\simulators\fastfarm\inputs\template\FarmInputs\uniform8_700s.bts'
    print('writing', p, 'nt=%d ny=%d nz=%d' % (nt, ny, nz), flush=True)
    out.write(p)
    print('done', os.path.getsize(p)//1024//1024, 'MB', flush=True)

if __name__ == '__main__':
    main()