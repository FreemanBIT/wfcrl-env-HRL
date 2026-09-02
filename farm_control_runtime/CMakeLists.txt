cmake_minimum_required(VERSION 3.16)
project(farm_control_runtime VERSION 0.1.0 LANGUAGES C)

# farm_control_runtime — 场控中间层可移植核心（实时交付源）
# 依赖：仅 C11 编译器（MSVC / GCC / Clang 均可）。
# 构建：
#   cmake -S . -B build && cmake --build build
# 测试：
#   ctest --test-dir build

set(FCR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(FCR_SRC
    src/angle_convention.c
    src/command_store.c
    src/state_store.c
    src/watchdog.c
    src/runtime.c
)

add_library(farm_control_runtime STATIC ${FCR_SRC})
target_include_directories(farm_control_runtime PUBLIC ${FCR_INCLUDE_DIR})
target_compile_features(farm_control_runtime PUBLIC c_std_11)
set_target_properties(farm_control_runtime PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden)

# ---- 单元测试 ----
enable_testing()
add_executable(fcr_unit_tests
    tests/unit/test_framework.c
    tests/unit/test_main.c
    tests/unit/test_angle_convention.c
    tests/unit/test_command_store.c
    tests/unit/test_abi.c
    tests/unit/test_watchdog.c
)
target_include_directories(fcr_unit_tests PRIVATE ${FCR_INCLUDE_DIR} src tests/unit)
target_link_libraries(fcr_unit_tests PRIVATE farm_control_runtime m)
add_test(NAME unit COMMAND fcr_unit_tests)
