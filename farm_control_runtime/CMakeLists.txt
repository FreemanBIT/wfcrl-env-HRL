cmake_minimum_required(VERSION 3.16)
# WIN-04: C++ needed for zmq_transport.cpp
project(farm_control_runtime VERSION 0.1.0 LANGUAGES C CXX)

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
    src/rosco_api.c
    src/signal_statistics.c
    src/state_aggregator.c
    src/induction_supervisor.c
    src/yaw_action_manager.c
    bridge/fcr_bridge_core.c
    bridge/yaw_adapter.c
    bridge/rt_snapshot.c
    src/bridge_runtime_adapter.c
    src/fcr_rt_adapter.c
    awae/observation_manager.c
    awae/wind_mapper.c
    awae/awae_frame_builder.c
    offline/provider/offline_fastfarm_provider.c
    offline/harness/fcr_mock_command_source.c
)

# ---- 平台相关源（WSL/Linux 无 Windows 共享内存与 Winsock）----
if(WIN32)
    list(APPEND FCR_SRC
        offline/harness/shm_command_source.c
        transport/modbus/modbus_transport.c)
else()
    # RT/WSL：传输由调度器负责，离线 SHM 注入源替换为 no-op
    list(APPEND FCR_SRC offline/harness/shm_command_source_stub.c)
endif()

add_library(farm_control_runtime STATIC ${FCR_SRC})
# W-07: 库自身需要 src/bridge/awae/offline/transport 等子目录头文件
target_include_directories(farm_control_runtime PUBLIC
    ${FCR_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/bridge
    ${CMAKE_CURRENT_SOURCE_DIR}/awae
    ${CMAKE_CURRENT_SOURCE_DIR}/offline/harness
    ${CMAKE_CURRENT_SOURCE_DIR}/offline/provider
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/modbus)
target_compile_features(farm_control_runtime PUBLIC c_std_11)
set_target_properties(farm_control_runtime PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    C_VISIBILITY_PRESET hidden)

# rosco_api.c 在非 Windows 下使用 pthread 互斥锁
if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(farm_control_runtime PUBLIC Threads::Threads)
endif()

# WIN-04: offline ZMQ transport as separate shared library
# (loaded at runtime via LoadLibrary, avoids static link dependency)
if(WIN32)
    add_library(fcr_offline_zmq SHARED transport/zmq/zmq_transport.cpp)
    target_include_directories(fcr_offline_zmq PRIVATE ${FCR_INCLUDE_DIR})
    target_compile_features(fcr_offline_zmq PRIVATE cxx_std_11)
    set_target_properties(fcr_offline_zmq PROPERTIES
        POSITION_INDEPENDENT_CODE ON
        PREFIX "")
endif()

# ---- 单元测试 ----
enable_testing()
add_executable(fcr_unit_tests
    tests/unit/test_framework.c
    tests/unit/test_main.c
    tests/unit/test_angle_convention.c
    tests/unit/test_command_store.c
    tests/unit/test_abi.c
    tests/unit/test_watchdog.c
    tests/unit/test_rosco_api.c
    tests/unit/test_yaw_action_manager.c
    tests/unit/test_offline_provider.c
    tests/unit/test_signal_statistics.c
    tests/unit/test_state_aggregator.c
    tests/unit/test_induction_supervisor.c
    tests/unit/test_bridge_core.c
    tests/unit/test_bridge_adapter.c
    tests/unit/test_awae.c
    tests/unit/test_rt_snapshot.c
    tests/unit/test_rt_adapter.c
    tests/unit/test_mock_command_source.c
)
# Modbus 单测依赖 Winsock（transport/modbus/modbus_transport.h -> windows.h），仅 Windows
if(WIN32)
    target_sources(fcr_unit_tests PRIVATE tests/unit/test_modbus.c)
else()
    # test_main.c 在 MB_STUB 下不注册 Modbus 用例
    target_compile_definitions(fcr_unit_tests PRIVATE MB_STUB)
endif()
target_include_directories(fcr_unit_tests PRIVATE ${FCR_INCLUDE_DIR} src bridge awae offline/harness offline/provider transport/modbus tests/unit)
target_link_libraries(fcr_unit_tests PRIVATE farm_control_runtime m)
if(WIN32)
    target_link_libraries(fcr_unit_tests PRIVATE ws2_32)
else()
    find_package(Threads REQUIRED)
    target_link_libraries(fcr_unit_tests PRIVATE Threads::Threads)
endif()
add_test(NAME unit COMMAND fcr_unit_tests)
