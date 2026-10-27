#!/usr/bin/env bash
# build_rosco_linux.sh — 在 WSL/Linux 构建 ROSCO + 场控桥接的 Fortran 静态库
#
# 产物：build-linux/libfcr_rosco.a
#      （ROSCO 控制器 + FarmControlCBindings/FarmControlInterface）
#
# 依赖：gfortran（sudo apt-get install -y gfortran）
#
# 用法：
#   ./tools/build_rosco_linux.sh            # 构建 libfcr_rosco.a
#   ./tools/build_rosco_linux.sh --clean    # 先清理对象目录
#
# 与 Windows DLL 构建（wfcrl/simulators/fastfarm/src/_compile.py）保持一致的关键选项：
#   -cpp                 SysSubs.f90 内的 #ifdef _WIN32 需要预处理
#   -fdefault-real-8      REAL 默认 8 字节（ROSCO 依赖此约定）
#   -fdefault-double-8    与上一条配套
#   -ffree-line-length-0 上游源码行长不受限
#   -fPIC                便于后续链接进共享库
#
# 说明（与 Windows 构建的差异）：
#   - 不含 SysGnuWin.f90：该文件定义同名 MODULE SysSubs 且调用 Windows API；
#     Linux 使用 SysSubs.f90（PathSep 由 _WIN32 预处理自动切换为 '/'）。
#   - 不含 DISCON.F90：那是 ServoDyn 的 avrSWAP 入口；fastfarmRT 中由平台直接
#     调用控制器（ROSCO_Update/ROSCO_Init），FCR 钩子通过
#     FarmControlInterface 的 FCR_Init/FCR_ApplyReferences 暴露。
#   - 不含 ROSCO_IO.f90 / ZeroMQInterface.f90：文件 I/O 与 ZMQ 传输在 RT 路径
#     不需要（传输归调度器）；若需要日志输出请自行加回。
#   - 不含 ExtControl.f90：外部 DLL 控制器接口（Windows 动态加载）。
#
# 与 C 侧一起链接（推荐）：
#   gfortran your_main.o libfcr_rosco.a libfarm_control_runtime.a -lpthread -lm
#   （先把 libfarm_control_runtime.a 用 build_linux.sh 构建出来）
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
SRC="${ROOT}/rosco/src"
BUILD="${ROOT}/build-linux"
OBJ="${BUILD}/rosco_obj"
LIB="${BUILD}/libfcr_rosco.a"

if [[ "${1:-}" == "--clean" ]]; then
    rm -rf "${OBJ}" "${LIB}"
fi

command -v gfortran >/dev/null 2>&1 || {
    echo "ERROR: gfortran not found. Install with: sudo apt-get install -y gfortran" >&2
    exit 1
}
[[ -d "${SRC}" ]] || { echo "ERROR: ${SRC} not found" >&2; exit 1; }

mkdir -p "${OBJ}"

FLAGS=(-cpp -O2 -fPIC -ffree-line-length-0 -fdefault-real-8 -fdefault-double-8
       -J "${OBJ}" -I "${OBJ}" -I "${ROOT}/rosco" -I "${ROOT}/include")

# 编译顺序 = 模块依赖顺序（被 USE 的模块先编）
SOURCES=(
    "Constants.f90"
    "ROSCO_Types.f90"
    "SysSubs.f90"
    "Functions.f90"
    "Filters.f90"
    "ControllerBlocks.f90"
    "ROSCO_Helpers.f90"
    "ReadSetParameters.f90"
    "${ROOT}/rosco/FarmControlCBindings.f90"
    "${ROOT}/rosco/FarmControlInterface.f90"
    "Controllers.f90"
)

objs=()
for s in "${SOURCES[@]}"; do
    if [[ "${s}" = /* ]]; then path="${s}"; base="$(basename "${s}")"
    else path="${SRC}/${s}"; base="${s}"; fi
    [[ -f "${path}" ]] || { echo "ERROR: missing ${path}" >&2; exit 1; }
    obj="${OBJ}/${base%.*}.o"
    echo "  [f90] ${base}"
    gfortran "${FLAGS[@]}" -c "${path}" -o "${obj}"
    objs+=("${obj}")
done

ar rcs "${LIB}" "${objs[@]}"
echo "OK: ${LIB}"
echo "    链接示例: gfortran your_main.o ${LIB} ${BUILD}/libfarm_control_runtime.a -lpthread -lm"
