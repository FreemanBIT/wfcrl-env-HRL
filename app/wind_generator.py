"""
湍流风场生成器
==============
生成 FAST.Farm 兼容的湍流风场 (.bts 文件)。
使用 Kaimal 谱（IEC 标准）生成三维湍流风场。

用法:
    from wind_generator import generate_turbulent_wind
    bts_path = generate_turbulent_wind(
        wind_speed=8.0, ti=0.15,
        output_path="my_wind.bts"
    )
"""

import numpy as np
from pathlib import Path
from typing import Optional, Tuple


def kaimal_spectrum(f: np.ndarray, sigma: float, L: float, U: float) -> np.ndarray:
    """Kaimal 湍流谱。

    参数:
        f: 频率数组 (Hz)
        sigma: 风速标准差 (m/s)
        L: 积分尺度 (m)
        U: 平均风速 (m/s)

    返回:
        S: 功率谱密度 (m^2/s)
    """
    S = 4 * sigma**2 * L / U / (1 + 6 * f * L / U) ** (5 / 3)
    return S


def coherence(dy: float, dz: float, f: float, U: float, C_y=8.0, C_z=6.0) -> float:
    """IEC 相干模型。"""
    # 点对点相干——这里简化为均匀相干
    return np.exp(-np.sqrt((C_y * dy * f / U) ** 2 + (C_z * dz * f / U) ** 2))


def generate_kaimal_turbulence(
    n_t: int,
    ny: int,
    nz: int,
    dt: float,
    dy: float,
    dz: float,
    U_mean: float,
    TI: float,
    seed: int = 42,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """生成 Kaimal 谱三维湍流风场。

    参数:
        n_t: 时间步数
        ny: 横向网格点数
        nz: 垂直网格点数
        dt: 时间步长 (s)
        dy: 横向网格间距 (m)
        dz: 垂直网格间距 (m)
        U_mean: 平均风速 (m/s)
        TI: 湍流强度
        seed: 随机种子

    返回:
        u, v, w: 三维风速分量 (3, n_t, ny, nz)
    """
    rng = np.random.default_rng(seed)

    # 湍流标准差
    sigma_u = U_mean * TI
    sigma_v = 0.8 * sigma_u
    sigma_w = 0.5 * sigma_u

    # 积分尺度 (IEC 61400-1)
    L_u = 340.0  # 340m for hub height > 60m
    L_v = 0.5 * L_u
    L_w = 0.3 * L_u

    # 网格坐标
    y = np.arange(ny) * dy - (ny - 1) * dy / 2
    z = np.arange(nz) * dz  # 从 0 开始
    z0 = z.mean()  # 参考高度（近似轮毂高度）

    # 频率轴
    nf = n_t // 2 + 1
    f = np.fft.rfftfreq(n_t, d=dt)
    df = f[1] - f[0] if len(f) > 1 else 1.0 / (n_t * dt)
    f[0] = f[1]  # 避免 0 Hz 问题

    # 目标谱
    S_u_target = kaimal_spectrum(f, sigma_u, L_u, U_mean)
    S_v_target = kaimal_spectrum(f, sigma_v, L_v, U_mean)
    S_w_target = kaimal_spectrum(f, sigma_w, L_w, U_mean)

    def _generate_component(S_target, sigma, name="u"):
        """生成单个速度分量的场。"""
        # 随机振幅和相位
        F = np.zeros((nf, ny, nz), dtype=complex)
        for iy in range(ny):
            for iz in range(nz):
                # 随机相位
                phi = rng.uniform(0, 2 * np.pi, nf)
                # 振幅 = sqrt(S * df) * random_amplitude
                amp = np.sqrt(S_target * df) * np.sqrt(2) / 2
                amp[0] = 0  # 零频分量为 0
                F[:, iy, iz] = amp * np.exp(1j * phi)

        # 添加相干性（简化：沿横向和垂直方向的指数相关）
        for iy in range(ny):
            for iz in range(nz):
                y_dist = y[iy]
                z_dist = z[iz] - z0
                gamma = np.exp(-(y_dist ** 2 + z_dist ** 2) / (2 * (L_u / 2) ** 2))
                # 调整幅值以反映相干性
                F[:, iy, iz] *= np.sqrt(gamma + (1 - gamma) * rng.random(nf))

        # IFFT 回到时域
        u_comp = np.zeros((n_t, ny, nz))
        for iy in range(ny):
            for iz in range(nz):
                u_comp[:, iy, iz] = np.fft.irfft(F[:, iy, iz], n=n_t) * n_t

        # 调整标准差到目标值
        std_actual = u_comp.std()
        if std_actual > 0:
            u_comp = u_comp * (sigma / std_actual)

        return u_comp

    u_fluct = _generate_component(S_u_target, sigma_u, "u")
    v_fluct = _generate_component(S_v_target, sigma_v, "v")
    w_fluct = _generate_component(S_w_target, sigma_w, "w")

    # 加入平均风速
    u = u_fluct + U_mean
    v = v_fluct
    w = w_fluct

    # 堆叠为 (3, nt, ny, nz)
    result = np.stack([u, v, w], axis=0)

    return result


def calc_grid_from_layout(
    xcoords: list,
    ycoords: list,
    rotor_diameter: float = 126.0,
    meander_margin: float = 126.0,
    corridor_margin: float = 0.0,
    extra_margin: float = 50.0,
) -> Tuple[float, float, float]:
    """根据风电场布局自动计算 .bts 需要的网格尺寸。

    参数:
        xcoords: 风机 x 坐标列表 (m)
        ycoords: 风机 y 坐标列表 (m)
        rotor_diameter: 转子直径 (m)，NREL 5MW = 126m
        meander_margin: 尾流蜿蜒余量 (m)，~1D
        corridor_margin: 走廊余量 (m)，0 或列间距/2
        extra_margin: 额外余量 (m)

    返回:
        (grid_width, grid_height, hub_height)
    """
    xs = np.array(xcoords)
    ys = np.array(ycoords)
    n = len(xs)

    # 风机分布范围
    x_range = xs.max() - xs.min()
    y_range = ys.max() - ys.min()
    max_abs_y = max(abs(ys.min()), abs(ys.max()))

    # 转子半径
    rotor_radius = rotor_diameter / 2

    # 需要覆盖的横向半宽
    # 1) 转子覆盖: 最外侧风机 + 转子半径
    half_rotor = max_abs_y + rotor_radius
    # 2) 尾流覆盖: 列半展宽 + 走廊 + 蜿蜒
    #    FAST.Farm 低分辨率域需要的走廊宽度 = 列间距 / 2
    #    即使单行布局，FAST.Farm template 中也预设了走廊宽度
    if n > 1 and len(set(ys)) > 1:
        col_spacing = max(set(ys)) - min(set(ys))
    else:
        # 单行布局（如 3T）：使用典型 4D 列间距
        col_spacing = 4 * rotor_diameter  # 4 * 126 = 504m
    corridor = col_spacing / 2
    column_half_span = 0.0  # 偏航导致的列偏移
    half_wake = column_half_span + corridor + meander_margin

    half_width = max(half_rotor, half_wake) + extra_margin

    # 网格宽度 = 2 * half_width
    # 对于 4D 间距的风场，FAST.Farm 低分辨率域至少需要 756m 宽
    min_width = max(2 * half_width, 800.0)  # 800m > 756m
    grid_width = min_width

    # 网格高度: 转子直径 + 轮毂高度 + 余量
    # NREL 5MW 轮毂高 90m, 转子直径 126m
    hub_height = 90.0
    grid_height = max(hub_height + rotor_diameter + 100, 300.0)

    return grid_width, grid_height, hub_height


def generate_turbulent_wind(
    wind_speed: float = 8.0,
    ti: float = 0.15,
    direction: float = 270.0,
    hub_height: float = 90.0,
    grid_height: float = 340.0,
    grid_width: float = 680.0,
    nz: Optional[int] = None,
    ny: Optional[int] = None,
    duration: float = 600.0,
    dt: float = 0.1,
    seed: int = 42,
    output_path: Optional[str] = None,
    name: str = "turbulent_wind",
    layout_x: Optional[list] = None,
    layout_y: Optional[list] = None,
) -> str:
    """生成湍流风场并保存为 .bts 文件。

    参数:
        wind_speed: 平均风速 (m/s)
        ti: 湍流强度
        direction: 风向 (°)
        hub_height: 轮毂高度 (m)
        grid_height: 网格高度 (m) — 传入 layout 时自动计算
        grid_width: 网格宽度 (m) — 传入 layout 时自动计算
        nz: 垂直网格数 (None=自动)
        ny: 横向网格数 (None=自动)
        duration: 风场时长 (s)
        dt: 时间步长 (s)
        seed: 随机种子
        output_path: 输出路径（None=自动生成）
        name: 文件名（不含扩展名）
        layout_x: 风机 x 坐标列表 — 传入后自动计算网格尺寸
        layout_y: 风机 y 坐标列表 — 传入后自动计算网格尺寸

    返回:
        .bts 文件的路径
    """
    # 如果有布局信息，自动计算网格尺寸
    if layout_x is not None and layout_y is not None:
        grid_width, grid_height, hub_height = calc_grid_from_layout(
            layout_x, layout_y,
            rotor_diameter=126.0,
            meander_margin=126.0,
            extra_margin=60.0,
        )
        print(f"  根据布局自动计算网格: {grid_width:.0f}x{grid_height:.0f}m")

    # 自动确定网格点数（~10m 间距）
    if nz is None:
        nz = max(25, int(grid_height / 10) + 1)
    if ny is None:
        ny = max(41, int(grid_width / 10) + 1)
    # 确保奇数
    if nz % 2 == 0:
        nz += 1
    if ny % 2 == 0:
        ny += 1

    n_t = int(duration / dt)
    dy = grid_width / (ny - 1) if ny > 1 else grid_width
    dz = grid_height / (nz - 1) if nz > 1 else grid_height

    # 检查网格参数
    if hub_height > grid_height:
        raise ValueError(f"轮毂高度 ({hub_height}m) 不能超过网格高度 ({grid_height}m)")

    print(f"生成湍流风场: U={wind_speed}m/s, TI={ti}, {n_t}步, {duration:.0f}s")
    print(f"  网格: {nz}x{ny}, dy={dy:.1f}m, dz={dz:.1f}m")

    # 生成湍流数据
    uvw = generate_kaimal_turbulence(
        n_t=n_t, ny=ny, nz=nz, dt=dt, dy=dy, dz=dz,
        U_mean=wind_speed, TI=ti, seed=seed,
    )

    # 构造 TurbSimFile (OrderedDict 子类，使用 dict-like 赋值)
    from openfast_toolbox.io.turbsim_file import TurbSimFile

    ts = TurbSimFile()
    ts['dt'] = dt
    ts['dy'] = dy
    ts['dz'] = dz
    ts['nt'] = n_t
    ts['ny'] = ny
    ts['nz'] = nz

    # 网格坐标
    ts['y'] = np.arange(ny) * dy - (ny - 1) * dy / 2
    ts['z'] = np.arange(nz) * dz
    z_offset = hub_height - np.array(ts['z']).mean()
    ts['z'] = np.array(ts['z']) + z_offset
    ts['t'] = np.arange(n_t) * dt

    # 存储数据 (3, nt, ny, nz)
    ts['u'] = uvw.astype(np.float32)
    ts['ID'] = 8  # 非周期全流场格式（FAST.Farm 兼容）

    # 确定输出路径
    if output_path is None:
        wind_dir = Path(__file__).resolve().parent.parent / "custom_winds"
        wind_dir.mkdir(parents=True, exist_ok=True)
        output_path = str(wind_dir / f"{name}.bts")

    # 保存
    ts.write(output_path)
    print(f"  -> 已保存: {output_path}")

    return output_path


def create_turbsim_inp(
    wind_speed: float = 8.0,
    ti: float = 0.15,
    hub_height: float = 90.0,
    grid_height: float = 340.0,
    grid_width: float = 680.0,
    duration: float = 600.0,
    dt: float = 0.05,
    seed: int = 12345,
    output_path: Optional[str] = None,
    name: str = "turbulent_wind",
) -> str:
    """生成 TurbSim 输入文件 (.inp)。

    如果有 TurbSim 可执行文件，可以用这个生成更精确的湍流风场。
    """
    if output_path is None:
        wind_dir = Path(__file__).resolve().parent.parent / "custom_winds"
        wind_dir.mkdir(parents=True, exist_ok=True)
        output_path = str(wind_dir / f"{name}.inp")

    nz = int(grid_height / 10) + 1  # 10m 间距
    ny = int(grid_width / 10) + 1

    inp_content = f"""--------- TurbSim v2.00 Input File -------------------------
Generated by WFCRL Dashboard - {name}
--------- Runtime Options ------------------------------------
False       Echo
{seed}        RandSeed1
{-seed}       RandSeed2
False       WrBHHTP
False       WrFHHTP
False       WrADHH
True        WrADFF
False       WrBLFF
False       WrADTWR
False       WrFMTFF
False       WrACT
True        Clockwise
1           ScaleIEC
--------- Turbine/Model Specifications ------------------------
{nz}          NumGrid_Z
{ny}          NumGrid_Y
{dt}          TimeStep
{duration}    AnalysisTime
"ALL"       UsableTime
{hub_height}  HubHt
{grid_height} GridHeight
{grid_width}  GridWidth
0           VFlowAng
0           HFlowAng
--------- Meteorological Boundary Conditions --------------------
"IECKAI"    TurbModel
"default"   IECstandard
{ti*100:.0f}%        IECturbc
"NTM"       IEC_WindType
"default"   ETMc
"PL"        WindProfileType
{hub_height}  RefHt
{wind_speed}  URef
"""

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(inp_content)

    print(f"TurbSim .inp 已生成: {output_path}")
    return output_path


def generate_turbulent_wind_schedule(
    schedule: list,
    grid_width: Optional[float] = None,
    grid_height: Optional[float] = None,
    hub_height: float = 90.0,
    dt: float = 0.1,
    seed: int = 42,
    name: str = "schedule_wind",
    output_path: Optional[str] = None,
    layout_x: Optional[list] = None,
    layout_y: Optional[list] = None,
) -> str:
    """生成随时间变化的湍流风场（风速、风向按调度变化）。

    参数:
        schedule: 风调度列表，每项为 (time, speed, direction, ti)
                  例如: [(0, 8, 270, 0.15), (100, 10, 260, 0.12), (200, 6, 280, 0.18)]
                  time: 该段结束时间 (s)
                  speed: 该段平均风速 (m/s)
                  direction: 该段风向 (°)
                  ti: 该段湍流强度
        grid_width: 网格宽度 (m)
        grid_height: 网格高度 (m)
        hub_height: 轮毂高度 (m)
        dt: 时间步长 (s)
        seed: 随机种子
        name: 文件名
        output_path: 输出路径
        layout_x/layout_y: 风场坐标（传入后自动计算 grid 尺寸）

    返回:
        .bts 文件路径
    """
    if layout_x and layout_y:
        grid_width, grid_height, hub_height = calc_grid_from_layout(
            layout_x, layout_y
        )
        print(f"  根据布局自动计算网格: {grid_width:.0f}x{grid_height:.0f}m")
    elif grid_width is None or grid_height is None:
        grid_width, grid_height, hub_height = 800.0, 300.0, 90.0
        print(f"  使用默认网格: {grid_width:.0f}x{grid_height:.0f}m")

    # 计算总时长
    total_time = max(s[0] for s in schedule)
    n_t = int(total_time / dt)

    # 网格参数
    nz = max(25, int(grid_height / 10) + 1)
    ny = max(41, int(grid_width / 10) + 1)
    if nz % 2 == 0: nz += 1
    if ny % 2 == 0: ny += 1
    dy = grid_width / (ny - 1)
    dz = grid_height / (nz - 1)

    print(f"生成调度湍流风场: {len(schedule)}段, {total_time:.0f}s")
    print(f"  网格: {nz}x{ny}, dy={dy:.1f}m, dz={dz:.1f}m")

    # 用参考风速生成基础湍流场
    ref_speed = schedule[0][1]
    uvw = generate_kaimal_turbulence(
        n_t=n_t, ny=ny, nz=nz, dt=dt, dy=dy, dz=dz,
        U_mean=ref_speed, TI=schedule[0][3], seed=seed,
    )

    # 时间轴
    t = np.arange(n_t) * dt

    # 对每个时间步，计算当前目标风速和风向
    # 然后在湍流场上叠加变化
    for i in range(n_t):
        current_t = t[i]
        # 线性插值找到当前段
        for seg_idx in range(len(schedule) - 1):
            t_start = schedule[seg_idx][0] if seg_idx == 0 else 0
            t_end = schedule[seg_idx + 1][0]
            t_prev = schedule[seg_idx][0] if seg_idx > 0 else 0
            if t_prev <= current_t <= t_end:
                frac = (current_t - t_prev) / (t_end - t_prev) if t_end > t_prev else 0
                s0 = schedule[seg_idx]
                s1 = schedule[seg_idx + 1]
                target_speed = s0[1] + (s1[1] - s0[1]) * frac
                target_dir = s0[2] + (s1[2] - s0[2]) * frac
                target_ti = s0[3] + (s1[3] - s0[3]) * frac
                break
        else:
            target_speed = schedule[-1][1]
            target_dir = schedule[-1][2]
            target_ti = schedule[-1][3]

        # 缩放湍流以匹配目标速度
        scale = target_speed / ref_speed if ref_speed > 0 else 1.0
        # u 分量
        uvw[0, i] = (uvw[0, i] - ref_speed) * (target_ti / schedule[0][3]) * scale + target_speed
        # v, w 分量按 TI 比例缩放
        uvw[1, i] *= (target_ti / schedule[0][3])
        uvw[2, i] *= (target_ti / schedule[0][3])

        # 风向变化: 旋转 u,v 分量
        dir_change = np.deg2rad(target_dir - schedule[0][2])
        if abs(dir_change) > 0.01:
            u_old = uvw[0, i].copy()
            v_old = uvw[1, i].copy()
            uvw[0, i] = u_old * np.cos(dir_change) - v_old * np.sin(dir_change)
            uvw[1, i] = u_old * np.sin(dir_change) + v_old * np.cos(dir_change)

    # 构造 TurbSimFile
    from openfast_toolbox.io.turbsim_file import TurbSimFile
    ts = TurbSimFile()
    ts['dt'] = dt
    ts['dy'] = dy
    ts['dz'] = dz
    ts['nt'] = n_t
    ts['ny'] = ny
    ts['nz'] = nz
    ts['y'] = np.arange(ny) * dy - (ny - 1) * dy / 2
    ts['z'] = np.arange(nz) * dz
    z_offset = hub_height - np.array(ts['z']).mean()
    ts['z'] = np.array(ts['z']) + z_offset
    ts['t'] = t
    ts['u'] = uvw.astype(np.float32)
    ts['ID'] = 8

    if output_path is None:
        wind_dir = Path(__file__).resolve().parent.parent / "custom_winds"
        wind_dir.mkdir(parents=True, exist_ok=True)
        output_path = str(wind_dir / f"{name}.bts")

    ts.write(output_path)
    print(f"  -> 已保存: {output_path}")

    # 打印调度信息
    print(f"\n  风调度:")
    for i, seg in enumerate(schedule):
        label = f"  Segment {i+1}: t={seg[0]:.0f}s"
        label += f"  {seg[1]:.1f} m/s"
        label += f"  {seg[2]:.0f} deg"
        label += f"  TI={seg[3]:.2f}"
        print(label)

    return output_path


# 快捷测试
if __name__ == "__main__":
    # 测试调度风
    bts = generate_turbulent_wind_schedule(
        schedule=[
            (0, 8, 270, 0.15),
            (60, 8, 270, 0.15),
            (120, 10, 260, 0.12),
            (180, 6, 280, 0.18),
        ],
        grid_width=400, grid_height=300, hub_height=90,
        dt=0.1, seed=42, name="test_schedule",
    )
    print(f"\n生成完成: {bts}")
