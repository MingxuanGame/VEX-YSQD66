cmake_minimum_required(VERSION 3.20)
project(VexSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(VEX_BUILD_TESTS "Build host-side unit tests" ON)
option(VEX_ENABLE_SANITIZERS "Enable ASan/UBSan for host builds (GCC/Clang only)" OFF)

set(VEX_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(VEX_INCLUDE ${VEX_ROOT}/include)
set(VEX_SIM ${VEX_ROOT}/sim)

function(vex_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

if(VEX_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined)
    else()
        message(WARNING "VEX_ENABLE_SANITIZERS is only supported for GCC/Clang")
    endif()
endif()

# ---------------------------------------------------------------------------
# 纯软件业务代码：排除依赖 PROS 的硬件适配文件
# ---------------------------------------------------------------------------
file(GLOB_RECURSE VEX_CORE_SOURCES
    "${VEX_ROOT}/src/auton/*.cpp"
    "${VEX_ROOT}/src/collision/*.cpp"
    "${VEX_ROOT}/src/control/*.cpp"
    "${VEX_ROOT}/src/tuning/*.cpp"
    "${VEX_ROOT}/src/robot/*.cpp"
    "${VEX_ROOT}/src/vxmath/*.cpp"
    "${VEX_ROOT}/src/motion/*.cpp"
    "${VEX_ROOT}/src/io/*.cpp"
)
# pros_controller / clock / battery 只在固件构建里使用；host 用 sim 实现替代
list(FILTER VEX_CORE_SOURCES EXCLUDE REGEX "/pros_controller\\.cpp$")
list(FILTER VEX_CORE_SOURCES EXCLUDE REGEX "/pros_hardware\\.cpp$")
list(FILTER VEX_CORE_SOURCES EXCLUDE REGEX "/auton/session\\.cpp$")
list(FILTER VEX_CORE_SOURCES EXCLUDE REGEX "/src/robot/(clock|battery|display)\\.cpp$")
# host 时钟实现放进 core，避免静态库链接顺序导致 clock_sim 不被拉入
list(APPEND VEX_CORE_SOURCES "${VEX_SIM}/compat/clock_sim.cpp")

add_library(vex_core STATIC ${VEX_CORE_SOURCES})
target_include_directories(vex_core PUBLIC
    ${VEX_ROOT}
    ${VEX_INCLUDE}
    ${VEX_SIM}
)
target_compile_definitions(vex_core PUBLIC ENABLE_SIMULATOR)
vex_set_warnings(vex_core)

# ---------------------------------------------------------------------------
# 仿真硬件、时钟、物理引擎（不含 main）
# ---------------------------------------------------------------------------
file(GLOB_RECURSE VEX_SIM_SOURCES "${VEX_SIM}/*.cpp")
list(FILTER VEX_SIM_SOURCES EXCLUDE REGEX "/sim/main\\.cpp$")
list(FILTER VEX_SIM_SOURCES EXCLUDE REGEX "/sim/tuning_main\\.cpp$")
list(FILTER VEX_SIM_SOURCES EXCLUDE REGEX "/sim/motion_main\\.cpp$")
list(FILTER VEX_SIM_SOURCES EXCLUDE REGEX "/sim/controller_ui_main\\.cpp$")
list(FILTER VEX_SIM_SOURCES EXCLUDE REGEX "/sim/compat/clock_sim\\.cpp$")

add_library(vex_simlib STATIC ${VEX_SIM_SOURCES})
target_include_directories(vex_simlib PUBLIC
    ${VEX_ROOT}
    ${VEX_INCLUDE}
    ${VEX_SIM}
)
target_compile_definitions(vex_simlib PUBLIC ENABLE_SIMULATOR)
target_link_libraries(vex_simlib PUBLIC vex_core)
vex_set_warnings(vex_simlib)

# ---------------------------------------------------------------------------
# 仿真可执行文件
# ---------------------------------------------------------------------------
add_executable(vex_sim "${VEX_SIM}/main.cpp")
target_link_libraries(vex_sim PRIVATE vex_simlib)
vex_set_warnings(vex_sim)

# 只测调参链路：ParamsManager -> CollisionTuning -> ParamApply -> 存盘/读盘
add_executable(vex_sim_tuning "${VEX_SIM}/tuning_main.cpp")
target_link_libraries(vex_sim_tuning PRIVATE vex_simlib)
vex_set_warnings(vex_sim_tuning)

add_executable(vex_sim_motion "${VEX_SIM}/motion_main.cpp")
target_link_libraries(vex_sim_motion PRIVATE vex_simlib)
vex_set_warnings(vex_sim_motion)

add_executable(vex_sim_ui "${VEX_SIM}/controller_ui_main.cpp")
target_link_libraries(vex_sim_ui PRIVATE vex_simlib)
if(WIN32)
    target_link_libraries(vex_sim_ui PRIVATE user32 gdi32)
endif()
vex_set_warnings(vex_sim_ui)

# 可执行文件统一输出到 sim/bin，避免和 CMake 中间文件混在一起。
set(VEX_SIM_BIN "${VEX_SIM}/bin")
set_target_properties(vex_sim vex_sim_tuning vex_sim_motion vex_sim_ui
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${VEX_SIM_BIN}")

# ---------------------------------------------------------------------------
# Host 单元测试
# ---------------------------------------------------------------------------
if(VEX_BUILD_TESTS)
    enable_testing()
    file(GLOB VEX_TEST_SOURCES "${VEX_ROOT}/tests/*.cpp")
    if(VEX_TEST_SOURCES)
        add_executable(vex_tests ${VEX_TEST_SOURCES})
        target_link_libraries(vex_tests PRIVATE vex_simlib)
        vex_set_warnings(vex_tests)
        set_target_properties(vex_tests PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${VEX_SIM_BIN}")
        add_test(NAME vex_tests COMMAND vex_tests)
        add_test(NAME vex_sim_tuning COMMAND vex_sim_tuning)
        add_test(NAME vex_sim_motion COMMAND vex_sim_motion)
    else()
        message(STATUS "No tests found under ${VEX_ROOT}/tests")
    endif()
endif()
