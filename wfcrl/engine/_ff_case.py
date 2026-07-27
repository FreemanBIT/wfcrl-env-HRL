"""
Adapted from openfast_toolbox example `Ex2_FFarmInputSetup.py`
@ https://github.com/OpenFAST/openfast_toolbox/blob/main/openfast_toolbox/fastfarm/examples/
"""
import glob
import os
import shutil
import warnings
from pathlib import Path
from typing import Dict

import numpy as np
import yaml
from openfast_toolbox.fastfarm import (
    fastFarmBoxExtent,
    fastFarmTurbSimExtent,
    writeFastFarm,
)
from openfast_toolbox.io.fast_input_file import FASTInputFile

LOCAL_DIR = Path(__file__).resolve().parent.parent  # wfcrl/simulator/ → wfcrl/

TEMPLATE_DIR = str(LOCAL_DIR / "simulators/{}/inputs/template/") + "/"
CASE_DIR = str(LOCAL_DIR / "simulators/{}/inputs/") + "/"
SERVO_DIR = str(LOCAL_DIR / "simulators/{}/servo_dll/") + "/"
FARMINPUTS_DIR = os.environ.get("WFCRL_FARMINPUTS_DIR",
    str(LOCAL_DIR / "simulators/{}/inputs/template/FarmInputs").format("fastfarm"))


def clean_folder(path):
    for subpath in glob.glob(path, recursive=True):
        if not os.path.isdir(subpath):
            os.remove(subpath)


def create_floris_case(case: Dict, output_dir=None):
    template_dir = TEMPLATE_DIR.format("floris")
    output_dir = Path(CASE_DIR.format("floris") if output_dir is None else output_dir)
    with open(f"{template_dir}case.yaml", "r") as fp:
        config = yaml.safe_load(fp)
    config["farm"]["layout_x"] = case["xcoords"]
    config["farm"]["layout_y"] = case["ycoords"]
    if case.get("direction") is not None:
        config["flow_field"]["wind_directions"] = [case["direction"]]
    if case.get("speed") is not None:
        config["flow_field"]["wind_speeds"] = [case["speed"]]
    if case.get("turbine_type") is not None:
        config["farm"]["turbine_type"] = case["turbine_type"]
        # Auto-resolve turbine_library_path to the bundled turbine_library
        turbine_lib = str(LOCAL_DIR / "simulators/floris/inputs/turbine_library/")
        config["farm"]["turbine_library_path"] = case.get("turbine_library_path", turbine_lib)
    elif case.get("turbine_library_path") is not None:
        config["farm"]["turbine_library_path"] = case["turbine_library_path"]
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(output_dir / "case.yaml", "w") as fp:
        yaml.safe_dump(config, fp)
    return str(output_dir / "case.yaml")


def read_simul_info(fstf_file):
    fstf = FASTInputFile(fstf_file)
    num_iter = fstf["TMax"] // fstf["DT_Low"]
    num_turbines = fstf["NumTurbines"]
    return num_turbines, num_iter


def get_inflow_file_path(fstf_file):
    fstf = FASTInputFile(fstf_file)
    inflow_file_name = fstf["InflowFile"].replace('"', "")
    inflow_file_path = (Path(fstf_file).parent / inflow_file_name).resolve()
    return inflow_file_path


def read_inflow_info(inflow_file):
    inflow = FASTInputFile(inflow_file)
    return inflow["HWindSpeed"]


def write_inflow_info(inflow_file, wind_speed):
    inflow = FASTInputFile(inflow_file)
    inflow["HWindSpeed"] = wind_speed
    inflow.write(os.path.join(inflow_file))
    return inflow_file


def create_dll(fstf_file):
    fstf = FASTInputFile(fstf_file)
    base = Path(fstf_file).parent

    # 为每台风机部署 DISCON bridge DLL + ROSCO 参数文件
    for idx, ref_path in enumerate(fstf["WindTurbines"][:, 3]):
        turbine_id = idx + 1
        fst = FASTInputFile(str((base / ref_path.replace('"', "")).resolve()))
        servo_file_name = fst["ServoFile"]
        servo = FASTInputFile(str((base / servo_file_name.replace('"', "")).resolve()))
        servo_dll_filename = servo["DLL_FileName"]
        path_to_servo_dll = (base / servo_dll_filename.replace('"', "")).resolve()
        servo_dir = path_to_servo_dll.parent

        # 总是覆盖 DISCON DLL（确保使用最新 bridge）
        servo_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy(
            f'{SERVO_DIR.format("fastfarm")}/DISCON_WT1.dll', path_to_servo_dll
        )

        # 写入 per-turbine ROSCO 参数文件到 ServoDyn .dat 同目录 (FarmInputs/)
        # ROSCO 通过 accINFILE (DLL_InFile) 读取，路径相对 ServoDyn 输入文件
        # 第一行: 风机 ID（供 WFCRL bridge 读取）
        # 后续行: ROSCO v2.9.0 控制器参数
        farm_discon = Path(base) / f"DISCON_T{turbine_id}.IN"
        template_path = Path(SERVO_DIR.format("fastfarm")) / "DISCON_ROSCO_TEMPLATE.IN"
        if template_path.exists():
            with open(template_path, 'r') as ft:
                template_lines = ft.readlines()
            # 替换第一行风机 ID
            if template_lines:
                template_lines[0] = f"{turbine_id}\n"
            with open(farm_discon, 'w') as f:
                f.writelines(template_lines)
        else:
            # 回退: 仅写风机 ID（但 ROSCO 仍需完整参数文件，故应确保模板存在）
            with open(farm_discon, 'w') as f:
                f.write(str(turbine_id) + '\n')


def write_modambwind3_boxes(
    src_bts_path: str,
    out_farminputs_dir: str,
    FFTS: Dict,
    xcoords,
    ycoords,
    zcoords,
):
    """
    由**单个**环境 TurbSim 盒子(src_bts) 派生 Mod_AmbWind=3 所需的多盒子：
      - Low.bts          低分辨率域（大而粗，按 FFTS 的低分辨率网格重采样）
      - HighT{n}.bts     每台风机的高分辨率域（小而细，按 FFTS 的高分辨率网格，
                         空间上同步到该风机的全局 X-Y-Z 偏移）

    依据 OpenFAST 建模指南（Mod_AmbWind=3）：
      * 多个 .bts，命名固定为 Low / HighT1 / HighT2 / ...
      * InflowWind 中 FileName_BTS 只给**目录路径**（不含文件名）。
      * 要求各盒子沿传播方向**周期**，且 PropagationDir=0。
      * 高分辨率盒按 (WT_X, WT_Y) 空间同步到全局坐标。

    实现：用 openfast_toolbox.io.TurbSimFile 读源盒，按目标网格在 (y,z) 上线性
    插值重采样；低分辨率盒覆盖整个低分辨率域，高分辨率盒裁剪到每台风机周围
    的窗口。时间轴沿用源盒（已是周期湍流盒）。

    返回写出的文件名列表（相对 out_farminputs_dir）。
    """
    import numpy as _np
    from openfast_toolbox.io.turbsim_file import TurbSimFile

    ts = TurbSimFile(src_bts_path)
    y_src = _np.asarray(ts['y'], dtype=float)        # (ny,)
    z_src = _np.asarray(ts['z'], dtype=float)        # (nz,)
    u_src = _np.asarray(ts['u'], dtype=float)        # (3, nt, ny, nz)
    t_src = _np.asarray(ts['t'], dtype=float)        # (nt,)
    try:
        zref = float(ts['zRef']); uref = float(ts['uRef'])
    except Exception:
        zref = float(FFTS.get('Z0_Low', 5.0)) + (z_src[-1] - z_src[0]) / 2.0
        uref = float(_np.nanmean(u_src[0]))

    def _resample(y_tgt, z_tgt):
        """在 (y,z) 上对每个时间步、每个分量做双线性插值，得到 (3,nt,ny',nz')。"""
        from scipy.interpolate import RegularGridInterpolator
        ny2, nz2 = len(y_tgt), len(z_tgt)
        out = _np.zeros((3, u_src.shape[1], ny2, nz2), dtype=_np.float32)
        YT, ZT = _np.meshgrid(y_tgt, z_tgt, indexing='ij')
        pts = _np.column_stack([YT.ravel(), ZT.ravel()])
        pts[:, 0] = _np.clip(pts[:, 0], y_src[0], y_src[-1])
        pts[:, 1] = _np.clip(pts[:, 1], z_src[0], z_src[-1])
        for c in range(3):
            for it in range(u_src.shape[1]):
                interp = RegularGridInterpolator(
                    (y_src, z_src), u_src[c, it], method='linear',
                    bounds_error=False, fill_value=None)
                out[c, it] = interp(pts).reshape(ny2, nz2)
        return out

    def _write_bts(path, y_tgt, z_tgt, u_box):
        out = TurbSimFile()
        out['y'] = _np.asarray(y_tgt, dtype=float)
        out['z'] = _np.asarray(z_tgt, dtype=float)
        out['t'] = t_src
        out['u'] = u_box
        out['ID'] = 7   # periodic full-field TurbSim binary
        out['zRef'] = zref
        out['uRef'] = uref
        try:
            out['zTwr'] = _np.array([])
            out['uTwr'] = _np.zeros((3, u_src.shape[1], 0))
        except Exception:
            pass
        out.write(path)

    written = []

    # ---------- Low.bts ----------
    nyl = int(FFTS['NY_Low']); nzl = int(FFTS['NZ_Low'])
    y0l = float(FFTS['Y0_Low']); z0l = float(FFTS['Z0_Low'])
    dyl = float(FFTS['dY_Low']); dzl = float(FFTS['dZ_Low'])
    y_low = y0l + _np.arange(nyl) * dyl
    z_low = z0l + _np.arange(nzl) * dzl
    u_low = _resample(y_low, z_low)
    _write_bts(os.path.join(out_farminputs_dir, "Low.bts"), y_low, z_low, u_low)
    written.append("Low.bts")

    # ---------- HighT{n}.bts ----------
    # 说明：高分辨率盒按 Y/Z 网格裁剪并同步到各风机的横向偏移；**流向(X)的
    # 时空同步由 FAST.Farm 通过 WindTurbines 表的 X0_High 列 + 盒子的周期性
    # 自动完成**（Taylor 冻结湍流推进），因此盒内时间轴沿用源盒 t_src 即可，
    # 无需对盒内容预先做流向时移。
    nyh = int(FFTS['NY_High']); nzh = int(FFTS['NZ_High'])
    dyh = float(FFTS['dY_High']); dzh = float(FFTS['dZ_High'])
    z0h = float(FFTS['Z0_High'])
    Y0_High = list(FFTS['Y0_High'])
    for iWT in range(len(xcoords)):
        y0h = float(Y0_High[iWT])
        y_high = y0h + _np.arange(nyh) * dyh
        z_high = z0h + _np.arange(nzh) * dzh
        u_high = _resample(y_high, z_high)
        _write_bts(os.path.join(out_farminputs_dir, f"HighT{iWT+1}.bts"),
                   y_high, z_high, u_high)
        written.append(f"HighT{iWT+1}.bts")

    return written


def create_ff_case(case: Dict, output_dir=None):
    assert case["num_turbines"] == len(case["xcoords"])
    xcoords = case["xcoords"]
    ycoords = case["ycoords"]
    max_iter, dt = case["max_iter"], case["dt"]

    template_dir = TEMPLATE_DIR.format("fastfarm")
    servoDir = SERVO_DIR.format("fastfarm")
    templateFSTF = os.path.join(f"{template_dir}FarmInputs/", "Case.fstf")
    fstf = FASTInputFile(templateFSTF)
    if output_dir is None:
        output_dir = CASE_DIR.format("fastfarm")
    else:
        if output_dir[-1] != "/":
            output_dir += "/"

    # Create dirs
    os.makedirs(f"{output_dir}servo_dll/", exist_ok=True)
    os.makedirs(f"{output_dir}FarmInputs/", exist_ok=True)
    os.makedirs(f"{output_dir}5MW_Baseline/ServoData/", exist_ok=True)
    os.makedirs(f"{output_dir}5MW_Baseline/Airfoils/", exist_ok=True)
    clean_folder(f"{output_dir}FarmInputs/*")
    clean_folder(f"{output_dir}5MW_Baseline/*")

    outputFSTF = os.path.join(f"{output_dir}FarmInputs/", "Case.fstf")
    ref_path = fstf["WindTurbines"][0, 3]
    max_time = max_iter * dt

    fst = FASTInputFile(
        os.path.join(f"{template_dir}FarmInputs/", ref_path.replace('"', ""))
    )
    ed = FASTInputFile(
        os.path.join(f"{template_dir}FarmInputs/", fst["EDFile"].replace('"', ""))
    )
    inflow = FASTInputFile(
        os.path.join(f"{template_dir}FarmInputs/", fstf["InflowFile"].replace('"', ""))
    )

    FFTS = None
    # Turbine diameter (m)
    D = ed["TipRad"] * 2
    # Hub Height (m)
    hubHeight = ed["TowerHt"]
    # x-extent of high res box in diamter around turbine location
    extent_X_high = 2.0  # np.round((fstf['NX_High'] * dX_High)/D,2)
    # y-extent of high res box in diamter around turbine location
    extent_YZ_high = 2.0  # np.round((fstf['NY_High'] * dYZ_High)/D,2)
    # maximum blade chord (m). Turbine specific.
    chord_max = 5
    # All turbine have same z coordinates
    zcoords = [0.0 for _ in xcoords]
    # Meandering constant (-)
    Cmeander = 1.9
    if case["wind_time_series"] is not None:
        # # If TurbSim is used, retrieve wind info from `.bts` file
        inflow["WindType"] = 3
        _use_mod3_flag = bool(case.get("use_mod_ambwind3", False))
        if _use_mod3_flag:
            # Mod_AmbWind=3：FileName_BTS 只给**目录**（FAST.Farm 在该目录找
            # Low.bts / HighT{n}.bts），且要求 PropagationDir=0（风向通过几何/
            # 机舱偏航体现，而非旋转湍流盒）。盒子在 setup 阶段由源盒派生写出。
            inflow["FileName_BTS"] = '"./"'
            inflow["PropagationDir"] = 0.0
        else:
            inflow["FileName_BTS"] = f'"{os.path.join(FARMINPUTS_DIR, case["wind_time_series"])}"'
        # TurbSim .bts file to be used in FAST.Farm simulation, needs to exist.
        BTS_filename = os.path.join(
            FARMINPUTS_DIR, case["wind_time_series"]
        )
        # --- Get box extents
        FFTS = fastFarmTurbSimExtent(
            BTS_filename,
            hubHeight,
            D,
            xcoords,
            ycoords,
            Cmeander=Cmeander,
            chord_max=chord_max,
            extent_X=extent_X_high,
            extent_YZ=extent_YZ_high,
            meanUAtHubHeight=True,
        )
    else:
        inflow["WindType"] = 1
        if case["speed"] is not None:
            inflow["HWindSpeed"] = case["speed"]
        mean_wind = inflow["HWindSpeed"]
        # ===== 稳态盒构造修复：显式控制 dY_box <= dY_Low_desired =====
        # 背景：原写法用模板 fstf['dY_Low']（≈10 m）作为横向间距，导致
        #   dY_box = dY_Low*ny/(ny-1) ≈ 10.13 > dY_Low_desired（U=6 时 ~9.58），
        # openfast_toolbox.fastFarmBoxExtent 内部 floor(desired/dY_box)=0 → 抛
        #   "The Y-resolution of the box (dY_Box) is too large ... Reduce the DY of the box"。
        # 修复：用一个**显式**的、足够小的横向/垂向间距（<=9.0 m，对 6/8/10 m/s
        # 的 desired 均满足）来铺 y/z 网格，并保证横向范围覆盖（旋转后的）风机
        # 列展宽 + 尾流蜿蜒余量。
        D_loc = D   # 与上文一致（ed["TipRad"]*2）
        dY_des = 9.0   # <= 稳态 U=6 的 desired(~9.58)，对更高风速更宽松
        dZ_des = 9.0
        # 横向盒宽 = 低分辨率覆盖(风机列展宽 + 2.5D 尾流走廊) + 1D 蜿蜒采样余量。
        # 关键：盒子要比“低分辨率域覆盖范围”更宽，给尾流蜿蜒的环境风采样留余量，
        # 否则蜿蜒采样点会越出盒子（与湍流路径的 -408 同类问题）。
        turb_span_y = (max(ycoords) - min(ycoords)) if len(ycoords) else 0.0
        grid_half_need = turb_span_y / 2.0 + 2.5 * D_loc      # 低分辨率覆盖
        box_half_need = grid_half_need + 1.0 * D_loc          # + 蜿蜒采样余量
        ny = int(np.ceil((2.0 * box_half_need) / dY_des)) + 1
        ny = max(ny, int(fstf["NY_Low"]))           # 不低于模板规模
        nz = max(int(fstf["NZ_Low"]) - 1, int(np.ceil(340.0 / dZ_des)))
        width = dY_des * (ny - 1)
        height = dZ_des * (nz - 1)
        # ===== 关键修复：合成盒时间维度必须覆盖整个仿真时长 =====
        # 旧代码硬编码 time=200s。但 Mod_AmbWind=2 稳态下，FAST.Farm 会把均匀入流
        # 采样成一个带**时间维**的 Grid4D 环境场，其时长由这里的 t 数组决定。
        # 当仿真跑到 t>200s 时，InflowWind 查询 Grid4D 的时间维越界，报
        #   "Grid4DField_GetVel: Outside the grid bounds"
        # （报错只显示 X/Y/Z，越界的其实是第 4 维=时间，故 X/Y/Z 看起来都在界内）。
        # 实测 stU6_wd20_s8D 恰在 t=258s 崩溃，与 200s 盒时长耗尽吻合。
        # 修复：盒时长 = 仿真时长(max_time=max_iter*dt) + 余量，确保覆盖整个 TMax。
        time = float(max_time) + 60.0    # TMax + 60s 余量
        # =====================================================
        y = np.linspace(-width / 2.0, width / 2.0, ny)
        z = np.linspace(0.0, height, nz)
        t = np.arange(0, time, fstf["DT_High"] / 10)
        # ===== 稳态盒构造修复结束 =====
        FFTS = fastFarmBoxExtent(
            y,
            z,
            t,
            mean_wind,
            hubHeight,
            D,
            xcoords,
            ycoords,
            Cmeander=Cmeander,
            chord_max=chord_max,
            extent_X=extent_X_high,
            extent_YZ=extent_YZ_high,
        )

    # --- Write Fast Farm file with layout and Low and High res extent
    # Write directly from template to preserve v5.0.0 file format

    # ===== 关键修复：在 TurbSim 盒子内部"干净重建"低分辨率 Y 网格 =====
    #
    # 两类报错都源自低分辨率域横向与 .bts 盒子边界的关系：
    #   (1) openfast_toolbox 内部:
    #       "The Y-resolution of the box (dY_Box) is too large ... Reduce the DY"
    #       —— 当 dY_Box > dY_Low_desired(随风速增大) 时, floor(desired/dY_Box)=0。
    #       这发生在 fastFarmTurbSimExtent() 内部(本函数上方),无法事后补救,
    #       必须靠把 .bts 的 dY_Box 调小(增大 NumGrid_Y)来避免 —— 见随附 .inp。
    #   (2) FAST.Farm 运行期:
    #       "GF wind array boundaries violated: Grid too small in Y direction.
    #        Y=-370.8; Y boundaries = [-370, 370]"
    #       —— toolbox 把低分辨率域一直铺到盒子边缘,Y0_Low 取整后最外侧节点
    #          落到边界外 0.x m。仅靠"微调 Y0_Low"很脆弱(取整/节点列差异会漏)。
    #
    # 这里改为最稳健的做法:已知盒子真实半宽与 toolbox 选定的 dY_Low 后,
    # 从零重建一个"居中、对称、严格落在盒子内部、并尽量占满盒子"的低分辨率
    # Y 网格,彻底摆脱 FFTS 原始 Y0_Low/NY_Low 的取整瑕疵。X/Z/高分辨率盒等
    # 其余量保持 toolbox 原值不动。
    # Mod_AmbWind=3 下，Low.bts 就是按低分辨率网格生成的，不存在"外部盒子边界"
    # 约束，故无需对 Y 网格做收缩夹紧（夹紧仅对 Mod2 单盒情形有意义）。
    _skip_clamp_mod3 = bool(case.get("use_mod_ambwind3", False)) and (
        case["wind_time_series"] is not None
    )
    if FFTS is not None and not _skip_clamp_mod3:
        if case["wind_time_series"] is not None:
            try:
                from openfast_toolbox.io.turbsim_file import TurbSimFile
                _ts = TurbSimFile(BTS_filename)
                _yvec = np.asarray(_ts['y'], dtype=float)
                box_half = (float(_yvec[-1]) - float(_yvec[0])) / 2.0
            except Exception:
                box_half = (int(FFTS['NY_Low']) - 1) * float(FFTS['dY_Low']) / 2.0
        else:
            # 稳态：合成盒的横向半宽（前面 else 分支构造的 y 数组）
            try:
                box_half = (float(y[-1]) - float(y[0])) / 2.0
            except Exception:
                box_half = (int(FFTS['NY_Low']) - 1) * float(FFTS['dY_Low']) / 2.0

        dY_low = float(FFTS['dY_Low'])
        dY_high = float(FFTS['dY_High'])
        nyh = int(FFTS['NY_High'])
        D_loc = D

        # ---------------------------------------------------------------
        # 关键认识（本轮修复）：本工程用"风场旋转法"处理非零风向 —— 风机列
        # 绕质心旋转 -offset，使入流沿 +X。于是**外侧风机的 Y 坐标不再是 0**，
        # 在 8D/20° 这类大间距大偏差工况可达 ±345 m（列展宽 5.47D）。
        #
        # 旧夹紧代码基于"机位均在 Y=0"的错误假设，把高分辨率盒**整体平移**回
        # 盒子内部。当风机本身已接近/超出盒子横向范围时，这一平移把盒子推离转
        # 子中心，导致叶片节点落到高分辨率盒之外：
        #   "Grid4DField_GetVel: Outside the grid bounds:(−9.87, 48.74, 62.51);
        #    box bounds:(−129.23,−205.43,5.00) to (121.36, 46.26, 205.51)"
        #   （高分辨率盒中心被推到 Y=−79.6，而 T3 转子在 Y=+345）。
        #
        # 正确做法：高分辨率盒必须**始终居中于其风机**，绝不为迁就 .bts 边界而
        # 平移。若某台风机的转子(±1D + 叶尖余量)放不进 .bts 盒子，说明这个盒子
        # 对该(偏差×间距)工况**太窄**，应当**明确报错**提示加宽盒子/生成专用盒，
        # 而不是静默产出错误几何。
        # ---------------------------------------------------------------

        # (1) 每台风机所需的高分辨率盒横向范围（居中于风机，不平移）
        #     high-res 盒半宽 ≈ (nyh-1)*dY_high/2（toolbox 设为 ~1D，含叶尖）
        halfspan_high = (nyh - 1) * dY_high / 2.0
        yc_list = [float(v) for v in ycoords]  # 旋转后的风机 Y（含 ±345 等）
        need_high = max(abs(yc) + halfspan_high for yc in yc_list) if yc_list else halfspan_high

        # (2) 低分辨率域所需范围：覆盖"风机列展宽 + 尾流走廊"，并给蜿蜒采样留 ~1D
        turb_span_y = (max(yc_list) - min(yc_list)) if yc_list else 0.0
        need_low = turb_span_y / 2.0 + 2.0 * D_loc          # 走廊
        need_meander = 1.0 * D_loc                          # 蜿蜒采样余量

        # (3) 盒子够不够？—— 高分辨率盒是硬约束（转子必须完整落入）
        required_half = max(need_high, need_low + need_meander)
        if required_half > box_half + 1e-6:
            raise ValueError(
                "[create_ff_case] TurbSim .bts 盒子横向太窄，无法容纳该工况的旋转风机列。\n"
                f"  盒子半宽 box_half = {box_half:.1f} m（宽 {2*box_half:.0f} m）\n"
                f"  该工况需要 half >= {required_half:.1f} m（宽 {2*required_half:.0f} m）\n"
                f"    - 高分辨率盒需 half >= {need_high:.1f} m（外侧风机 |Y|max="
                f"{max(abs(y) for y in yc_list):.1f} + 转子半跨 {halfspan_high:.1f}）\n"
                f"    - 低分辨率域需 half >= {need_low + need_meander:.1f} m（列半展宽"
                f" {turb_span_y/2.0:.1f} + 走廊 {2*D_loc:.0f} + 蜿蜒 {D_loc:.0f}）\n"
                f"  .bts = {os.path.basename(str(BTS_filename))}\n"
                "  解决办法：为该(风速,TI,间距)生成更宽的专用盒，命名如 "
                "inflow_06ms_TI05_s8D.bts（GridWidth≈"
                f"{int(np.ceil(2*required_half/10.0)*10)} m），工程会自动优先使用。\n"
                "  （大偏差×大间距的旋转列展宽是内禀的：Y-span=(n-1)*spacing*sin(offset)。）"
            )

        # (4) 低分辨率 Y 网格：居中、对称、严格落在盒子内部
        grid_half = min(need_low, box_half - need_meander)
        grid_half = max(grid_half, turb_span_y / 2.0 + 0.5 * D_loc)  # 至少覆盖风机列
        grid_half = min(grid_half, box_half - dY_low)               # 绝不触盒子边
        n_half = int(np.floor(grid_half / dY_low))
        NY_low = 2 * n_half + 1
        Y0_low = -n_half * dY_low
        far = Y0_low + (NY_low - 1) * dY_low
        assert Y0_low >= -box_half - 1e-6 and far <= box_half + 1e-6, \
            f"low-res Y grid out of box: Y0={Y0_low}, far={far}, half={box_half}"
        FFTS['Y0_Low'] = np.around(Y0_low, 3)
        FFTS['NY_Low'] = int(NY_low)

        # (5) 高分辨率盒：**保持居中于各自风机**（不再平移）。
        #     经 (3) 校验后每台转子都能完整落入盒子，这里仅按 toolbox 的居中值
        #     写回（Y0_High = 风机Y − halfspan_high），并做一次健壮性断言。
        new_y0h = []
        for iWT, yc in enumerate(yc_list):
            yh = yc - halfspan_high
            far_h = yh + (nyh - 1) * dY_high
            assert yh >= -box_half - 1e-6 and far_h <= box_half + 1e-6, (
                f"high-res box for T{iWT+1} out of .bts after centering: "
                f"[{yh:.1f},{far_h:.1f}] vs box_half={box_half:.1f}"
            )
            new_y0h.append(np.around(yh, 3))
        FFTS['Y0_High'] = new_y0h
    # ===== 关键修复结束 =====


    import shutil
    shutil.copy2(templateFSTF, outputFSTF)
    fstf_out = FASTInputFile(outputFSTF)

    # 是否启用 Mod_AmbWind=3（多盒：Low + HighT{n}）。仅对湍流(.bts)有意义；
    # 稳态(WindType=1)下 Mod_AmbWind 无影响（OpenFAST 文档明确说明）。
    use_mod3 = bool(case.get("use_mod_ambwind3", False)) and (
        case["wind_time_series"] is not None
    )

    # Set layout and box extent
    if FFTS is not None:
        fstf_out['Mod_AmbWind'] = 3 if use_mod3 else 2
        for k in ['DT_Low', 'DT_High', 'NX_Low', 'NY_Low', 'NZ_Low',
                   'X0_Low', 'Y0_Low', 'Z0_Low', 'dX_Low', 'dY_Low', 'dZ_Low',
                   'NX_High', 'NY_High', 'NZ_High']:
            if isinstance(FFTS[k], int):
                fstf_out[k] = FFTS[k]
            else:
                fstf_out[k] = np.around(FFTS[k], 3)
        fstf_out['WrDisDT'] = FFTS['DT_Low']

    # ===== 关键修复：让 FAST.Farm 自适应尾流低通截止频率 f_c =====
    # 模板里 f_c=0.17 Hz 是**固定值**。FAST.Farm 推荐 f_c ≈ 1.28*U/R：
    #   U=6 → 0.122，U=8 → 0.163，U=10 → 0.203。
    # 对最低风速 U=6，0.17 比推荐值高 ~40%，会导致尾流面平流过快而**相互穿越**
    # （日志：'wake plane ... has overtaken ... Reduce f_c'），随后尾流场损坏、
    # AWAE 采样出错 → FAST.Farm 在中途（实测 ~258 s）abort。
    # 设为 DEFAULT，让 FAST.Farm 按 1.28*U0/R 自动取值，从根本上消除低风速穿越。
    try:
        fstf_out['f_c'] = 'DEFAULT'
    except Exception:
        pass
    # ===== f_c 修复结束 =====
    nWT = len(xcoords)
    fstf_out['NumTurbines'] = nWT
    nCol = 10 if FFTS is not None else 4
    ref_path = fstf_out['WindTurbines'][0, 3]
    WT = np.array([''] * nWT * nCol, dtype='object').reshape((nWT, nCol))
    from openfast_toolbox.fastfarm.fastfarm import insertTN
    for iWT, (x, y, z) in enumerate(zip(xcoords, ycoords, zcoords)):
        WT[iWT, 0] = x
        WT[iWT, 1] = y
        WT[iWT, 2] = z
        WT[iWT, 3] = insertTN(ref_path, iWT + 1, nWT, noLeadingZero=False)
        if FFTS is not None:
            WT[iWT, 4] = FFTS['X0_High'][iWT]
            WT[iWT, 5] = FFTS['Y0_High'][iWT]
            WT[iWT, 6] = FFTS['Z0_High']
            WT[iWT, 7] = FFTS['dX_High']
            WT[iWT, 8] = FFTS['dY_High']
            WT[iWT, 9] = FFTS['dZ_High']
    fstf_out['WindTurbines'] = WT
    fstf_out.write(outputFSTF)
    # Fix: FASTInputFile.write() creates bare LF line endings on Windows,
    # but FAST.Farm v5.0.0 on Windows requires CRLF line endings.
    # Also fix line 2 to avoid key-value parsing issues.
    with open(outputFSTF, 'rb') as f:
        raw = f.read()
    # Convert bare LF to CRLF (avoid double-converting existing CRLF)
    raw = raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
    # Read lines and fix line 2
    try:
        lines = raw.decode('utf-8').split('\r\n')
    except UnicodeDecodeError:
        lines = raw.decode('gbk', errors='replace').split('\r\n')
    if len(lines) > 1:
        lines[1] = '! FAST.Farm input file'
    raw = '\r\n'.join(lines).encode('utf-8')
    with open(outputFSTF, 'wb') as f:
        f.write(raw)
    print("Created FAST.Farm input file:", outputFSTF)

    servo_file_name = fst["ServoFile"]
    servo = FASTInputFile(
        os.path.join(f"{template_dir}FarmInputs/", servo_file_name.replace('"', ""))
    )
    servo_dll_filename = servo["DLL_FileName"].split("/")[-1]
    servo_dll_file = os.path.join(servoDir, servo_dll_filename.replace('"', ""))
    servo_template_path = os.path.join(f"{template_dir}FarmInputs/", servo_file_name.replace('"', ""))
    with open(servo_template_path, 'rb') as _sf:
        servo_template_raw = _sf.read()
    servo_template_raw = servo_template_raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
    servo_dll_filename_encoded = servo_dll_filename.encode('utf-8')

    # FAST.Farm v5.0.0: SC_DLL (SuperController) no longer used

    # Copy all other files
    shutil.copytree(
        f"{template_dir}5MW_Baseline/", f"{output_dir}5MW_Baseline/", dirs_exist_ok=True
    )
    if use_mod3:
        # Mod_AmbWind=3：不复制单盒，改为从源盒派生 Low.bts + HighT{n}.bts
        out_fi_dir = f"{output_dir}FarmInputs/"
        try:
            written_boxes = write_modambwind3_boxes(
                BTS_filename, out_fi_dir, FFTS, xcoords, ycoords, zcoords
            )
            print("Mod_AmbWind=3 boxes written:", written_boxes)
        except Exception as _e:
            raise RuntimeError(
                f"Mod_AmbWind=3 box generation failed: {_e}. "
                f"源盒={BTS_filename}"
            )
    # .bts files are NOT copied — InflowWind.dat points directly to FARMINPUTS_DIR
    for file in glob.glob(f"{template_dir}FarmInputs/*.dat"):
        shutil.copy2(file, f"{output_dir}FarmInputs/")
    # Write InflowWind
    inflow_path = os.path.join(f"{output_dir}FarmInputs/", "InflowWind.dat")
    inflow.write(inflow_path)
    # Fix: FASTInputFile corrupts multi-valued parameters like RotorApexOffsetPos
    with open(inflow_path, 'rb') as f:
        iw_raw = f.read()
    iw_raw = iw_raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
    try:
        iw_text = iw_raw.decode('utf-8')
    except UnicodeDecodeError:
        iw_text = iw_raw.decode('gbk', errors='replace')
    # Restore RotorApexOffsetPos to 3 values if corrupted to single value
    import re as _re
    iw_text = _re.sub(
        r'^(\s*)[0-9.+-]+\s+RotorApexOffsetPos',
        r'\1 0.0 0.0 0.0   RotorApexOffsetPos',
        iw_text,
        flags=_re.MULTILINE
    )
    with open(inflow_path, 'wb') as f:
        f.write(iw_text.encode('utf-8'))
    # Get needed .fst files — use raw copy + string replace to preserve v5.0.0 format
    out_fstf = FASTInputFile(outputFSTF)
    out_fstf.write(outputFSTF)
    fst_files = [row[3].replace('"', "") for row in out_fstf["WindTurbines"]]
    template_fst_path = os.path.join(f"{template_dir}FarmInputs/", ref_path.replace('"', ""))
    with open(template_fst_path, 'rb') as _tf:
        template_fst_raw = _tf.read()
    template_fst_raw = template_fst_raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')

    for i, file in enumerate(fst_files):
        # Raw copy: replace ServoFile name and write directly (preserves v5.0.0 params)
        fst_raw = template_fst_raw.replace(
            servo_file_name.encode('utf-8'),
            servo_file_name.replace("1", str(i + 1)).encode('utf-8')
        )

        # ===== 关键修复：每台风机独立的 ElastoDyn 文件 =====
        # 原代码所有风机的 .fst 都指向**同一个** ED .dat（只替换了 ServoFile 名，
        # 没替换 EDFile 名）。于是 set_fixed_yaw / 初始 NacYaw 对每台风机写 NacYaw 时，
        # 三台机解析到的是**同一个文件** → 最后一台覆盖前面 → 所有风机都用最后一台的
        # 偏航值（wd=0 时为 0）→ **yaw 结果与 baseline 完全相同**。
        # 修复：为每台风机复制一份独立 ED 文件（EDFile_T{i}），并在 .fst 中改指向它。
        ed_template_name = fst["EDFile"].replace('"', "")
        ed_stem, ed_ext = os.path.splitext(os.path.basename(ed_template_name))
        ed_name_i = f"{ed_stem}_T{i+1}{ed_ext}"
        # 用二进制原样复制模板 ED（保留 v5.0.0 格式），CRLF 规整
        _ed_src = os.path.join(f"{template_dir}FarmInputs/", ed_template_name)
        with open(_ed_src, 'rb') as _edf:
            _ed_raw = _edf.read()
        _ed_raw = _ed_raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
        with open(os.path.join(f"{output_dir}FarmInputs/", ed_name_i), 'wb') as _edw:
            _edw.write(_ed_raw)
        # 在 .fst 里把 EDFile 指向本机专属 ED 文件
        fst_raw = fst_raw.replace(
            f'"{ed_template_name}"'.encode('utf-8'),
            f'"{ed_name_i}"'.encode('utf-8'),
        )
        # =====================================================

        with open(os.path.join(f"{output_dir}FarmInputs/", file), 'wb') as _fw:
            _fw.write(fst_raw)

        servo_dll_filename_i = servo_dll_filename.replace("1", str(i + 1))
        servo_i_name = servo_file_name.replace("1", str(i + 1)).replace('"', '')
        # Raw copy: replace DLL_FileName + DLL_InFile in servo file (preserves v5.0.0 format)
        dll_infile_old = b'"DISCON.IN"'
        dll_infile_new = f'"DISCON_T{i+1}.IN"'.encode('utf-8')
        servo_raw = servo_template_raw.replace(
            servo_dll_filename_encoded,
            servo_dll_filename_i.encode('utf-8')
        )
        servo_raw = servo_raw.replace(dll_infile_old, dll_infile_new)
        with open(os.path.join(f"{output_dir}FarmInputs/", servo_i_name), 'wb') as _sw:
            _sw.write(servo_raw)
        shutil.copy2(
            servo_dll_file,
            os.path.join(
                f"{output_dir}5MW_Baseline/ServoData/",
                servo_dll_filename_i.replace('"', ""),
            ),
        )

    out_fstf["TMax"] = max_time
    out_fstf["DT_Low"] = dt
    out_fstf["WrDisDT"] = out_fstf["DT_Low"]
    # OpenFAST 5.0.0 requires RotorDiamRef
    rotor_diameter = ed["TipRad"] * 2
    out_fstf["RotorDiamRef"] = rotor_diameter

    # 启用 VTK 输出（用于尾流可视化）
    out_fstf["WrDisWind"] = True           # 写入 disturbed wind VTK 文件
    out_fstf["NOutDisWindXY"] = 1           # 输出一个水平切片
    out_fstf["OutDisWindZ"] = hubHeight      # 在轮毂高度切片
    out_fstf["NOutDisWindYZ"] = 0
    out_fstf["NOutDisWindXZ"] = 0

    out_fstf.write(outputFSTF)
    return outputFSTF


def reset_simul_file(simul_file, it):
    new_path = path = Path(simul_file)
    if it > 0:
        new_path = path.with_stem(f"{path.stem}_iter{it}")
        shutil.copy2(path, new_path)
    return str(new_path)
