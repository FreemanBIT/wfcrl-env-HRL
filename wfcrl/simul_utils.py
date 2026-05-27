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

LOCAL_DIR = Path(__file__).resolve().parent

TEMPLATE_DIR = str(LOCAL_DIR / "simulators/{}/inputs/template/") + "/"
CASE_DIR = str(LOCAL_DIR / "simulators/{}/inputs/") + "/"
SERVO_DIR = str(LOCAL_DIR / "simulators/{}/servo_dll/") + "/"


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

    # FAST.Farm v5.0.0 不再需要 SC_DLL（SuperController 已移除）

    # 为每台风机部署 DISCON bridge DLL + DISCON.IN
    for idx, ref_path in enumerate(fstf["WindTurbines"][:, 3]):
        turbine_id = idx + 1
        fst = FASTInputFile(str((base / ref_path.replace('"', "")).resolve()))
        servo_file_name = fst["ServoFile"]
        servo = FASTInputFile(str((base / servo_file_name.replace('"', "")).resolve()))
        servo_dll_filename = servo["DLL_FileName"]
        path_to_servo_dll = (base / servo_dll_filename.replace('"', "")).resolve()

        # 总是覆盖 DISCON DLL（确保使用最新 bridge）
        path_to_servo_dll.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(
            f'{SERVO_DIR.format("fastfarm")}/DISCON_WT1.dll', path_to_servo_dll
        )

        # 写入 DISCON.IN（风机 ID）到 ServoData 目录
        discon_in_path = path_to_servo_dll.parent / "DISCON.IN"
        with open(discon_in_path, 'w') as f:
            f.write(str(turbine_id) + '\n')

        # 写入 per-turbine DISCON 文件到 FarmInputs/（cwd）
        # 修改后的 DISCON_bridge.f90 使用 accINFILE（即 DLL_InFile 的值）
        # 读取各风机独立的 ID 文件（如 DISCON_T1.IN → turbine_id=1）
        farm_discon = Path(base) / f"DISCON_T{turbine_id}.IN"
        with open(farm_discon, 'w') as f:
            f.write(str(turbine_id) + '\n')


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
        inflow["FileName_BTS"] = f'"{case["wind_time_series"]}"'
        # TurbSim .bts file to be used in FAST.Farm simulation, needs to exist.
        BTS_filename = os.path.join(
            f"{template_dir}FarmInputs/", case["wind_time_series"]
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
        min_width = max(ycoords) - min(ycoords)
        min_ny = int(np.ceil(min_width / fstf["dY_Low"]))
        ny, nz = max(min_ny, fstf["NY_Low"] - 1), fstf["NZ_Low"] - 1
        width, height = fstf["dY_Low"] * ny, fstf["dZ_Low"] * nz
        time = 200
        y = np.linspace(-width / 2, width / 2, ny)
        z = np.linspace(0, height, nz)
        t = np.arange(0, time, fstf["DT_High"] / 10)
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
    import shutil
    shutil.copy2(templateFSTF, outputFSTF)
    fstf_out = FASTInputFile(outputFSTF)
    # Set layout and box extent
    if FFTS is not None:
        fstf_out['Mod_AmbWind'] = 2
        for k in ['DT_Low', 'DT_High', 'NX_Low', 'NY_Low', 'NZ_Low',
                   'X0_Low', 'Y0_Low', 'Z0_Low', 'dX_Low', 'dY_Low', 'dZ_Low',
                   'NX_High', 'NY_High', 'NZ_High']:
            if isinstance(FFTS[k], int):
                fstf_out[k] = FFTS[k]
            else:
                fstf_out[k] = np.around(FFTS[k], 3)
        fstf_out['WrDisDT'] = FFTS['DT_Low']
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
    lines = raw.decode('ascii').split('\r\n')
    if len(lines) > 1:
        lines[1] = '! FAST.Farm input file'
    raw = '\r\n'.join(lines).encode('ascii')
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
    servo_dll_filename_encoded = servo_dll_filename.encode('ascii')

    # FAST.Farm v5.0.0: SC_DLL (SuperController) no longer used

    # Copy all other files
    shutil.copytree(
        f"{template_dir}5MW_Baseline/", f"{output_dir}5MW_Baseline/", dirs_exist_ok=True
    )
    for file in glob.glob(f"{template_dir}FarmInputs/*.bts"):
        shutil.copy2(file, f"{output_dir}FarmInputs/")
    for file in glob.glob(f"{template_dir}FarmInputs/*.dat"):
        shutil.copy2(file, f"{output_dir}FarmInputs/")
    # Write InflowWind
    inflow_path = os.path.join(f"{output_dir}FarmInputs/", "InflowWind.dat")
    inflow.write(inflow_path)
    # Fix: FASTInputFile corrupts multi-valued parameters like RotorApexOffsetPos
    with open(inflow_path, 'rb') as f:
        iw_raw = f.read()
    iw_raw = iw_raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
    iw_text = iw_raw.decode('ascii')
    # Restore RotorApexOffsetPos to 3 values if corrupted to single value
    import re as _re
    iw_text = _re.sub(
        r'^(\s*)[0-9.+-]+\s+RotorApexOffsetPos',
        r'\1 0.0 0.0 0.0   RotorApexOffsetPos',
        iw_text,
        flags=_re.MULTILINE
    )
    with open(inflow_path, 'wb') as f:
        f.write(iw_text.encode('ascii'))
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
            servo_file_name.encode('ascii'), 
            servo_file_name.replace("1", str(i + 1)).encode('ascii')
        )
        with open(os.path.join(f"{output_dir}FarmInputs/", file), 'wb') as _fw:
            _fw.write(fst_raw)

        servo_dll_filename_i = servo_dll_filename.replace("1", str(i + 1))
        servo_i_name = servo_file_name.replace("1", str(i + 1)).replace('"', '')
        # Raw copy: replace DLL_FileName + DLL_InFile in servo file (preserves v5.0.0 format)
        dll_infile_old = b'"DISCON.IN"'
        dll_infile_new = f'"DISCON_T{i+1}.IN"'.encode('ascii')
        servo_raw = servo_template_raw.replace(
            servo_dll_filename_encoded,
            servo_dll_filename_i.encode('ascii')
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
    out_fstf.write(outputFSTF)
    return outputFSTF


def reset_simul_file(simul_file, it):
    new_path = path = Path(simul_file)
    if it > 0:
        new_path = path.with_stem(f"{path.stem}_iter{it}")
        shutil.copy2(path, new_path)
    return str(new_path)
