# =====================================================================
# RTStitcher 跨平台 CMake 工程
# 支持: Windows (MSVC, vcpkg) / Linux x86_64 / Linux aarch64 (RK3588)
#
# 用法:
#   Linux:
#     mkdir build && cd build
#     cmake .. -DCMAKE_BUILD_TYPE=Release
#     make -j$(nproc)
#   Windows (vcpkg):
#     cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
#     cmake --build build --config Release
#
# 选项:
#   -DMODULE_GAZE_AWARE=0/1   是否启用眼动仪注视感知缝合（板子上默认 0）
# =====================================================================

cmake_minimum_required(VERSION 3.16)
project(RTStitcher LANGUAGES CXX)

# ---------- C++ 标准 ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译命令导出（方便 clangd / VSCode 跳转）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 默认 Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------- 编译选项 ----------
if(MSVC)
    add_compile_options(/utf-8 /EHsc /W3)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unused-variable)
    # 修复：不使用生成器表达式，改用条件判断
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        add_compile_options(-O0 -g)
    else()
        add_compile_options(-O2)
    endif()

    # ARM64 (RK3588 = Cortex-A76 + Cortex-A55) 优化
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
        message(STATUS "Detected aarch64 (probably RK3588), enabling NEON & A76/A55 tune")
        add_compile_options(-mcpu=cortex-a76.cortex-a55)
    endif()
endif()

# ---------- 选项 ----------
option(MODULE_GAZE_AWARE "Enable gaze-aware seam finder" ON)
if(MODULE_GAZE_AWARE)
    add_compile_definitions(MODULE_GAZE_AWARE=1)
    message(STATUS "Gaze-aware seam finder: ENABLED")
else()
    add_compile_definitions(MODULE_GAZE_AWARE=0)
    message(STATUS "Gaze-aware seam finder: DISABLED")
endif()


option(MODULE_SALIENCY_U2NET "Enable U2-Net saliency seam finder" ON)
if(MODULE_SALIENCY_U2NET)
    add_compile_definitions(MODULE_SALIENCY_U2NET=1)
    message(STATUS "U2-Net saliency: ENABLED")
else()
    add_compile_definitions(MODULE_SALIENCY_U2NET=0)
    message(STATUS "U2-Net saliency: DISABLED")
endif()

# ---------- 依赖查找 ----------
find_package(OpenCV 4.5 REQUIRED
    COMPONENTS core imgproc highgui videoio features2d
               calib3d stitching imgcodecs flann)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

message(STATUS "OpenCV version : ${OpenCV_VERSION}")
message(STATUS "OpenCV libs    : ${OpenCV_LIBS}")
message(STATUS "OpenCV include : ${OpenCV_INCLUDE_DIRS}")

# ---------- 源文件 ----------
set(RTSTITCHER_SOURCES
    RTStitcher/RTStitcher.cpp
    VideoCapture/VideoCapture.cpp
    SeamFinder/SeamFinder.cpp
    SeamFinder/GazeAwareGraphCutSeamFinder.cpp
    Blender/Blender.cpp
    Config/Config.cpp
    FeatureFinder/FeatureFinder.cpp
    Warper/Warper.cpp
    CameraParamEst/CameraParamEst.cpp
    CameraParamEst/VisualizeCameraStitchingParams.cpp
    Displayer/Displayer.cpp
    Utility/Utility.cpp
    GazeDataReader/GazeDataReader.cpp
    SaliencyDetector/SaliencyMapReader.cpp
)

add_executable(RTStitcher ${RTSTITCHER_SOURCES})

# ---------- include 路径 ----------
target_include_directories(RTStitcher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Platform
    ${CMAKE_CURRENT_SOURCE_DIR}/RTStitcher
    ${CMAKE_CURRENT_SOURCE_DIR}/VideoCapture
    ${CMAKE_CURRENT_SOURCE_DIR}/CircularBuffer
    ${CMAKE_CURRENT_SOURCE_DIR}/CircularBufferSync
    ${CMAKE_CURRENT_SOURCE_DIR}/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/Types
    ${CMAKE_CURRENT_SOURCE_DIR}/Exposure
    ${CMAKE_CURRENT_SOURCE_DIR}/SeamFinder
    ${CMAKE_CURRENT_SOURCE_DIR}/Blender
    ${CMAKE_CURRENT_SOURCE_DIR}/FeatureFinder
    ${CMAKE_CURRENT_SOURCE_DIR}/Warper
    ${CMAKE_CURRENT_SOURCE_DIR}/CameraParamEst
    ${CMAKE_CURRENT_SOURCE_DIR}/Displayer
    ${CMAKE_CURRENT_SOURCE_DIR}/Utility
    ${CMAKE_CURRENT_SOURCE_DIR}/GazeDataReader
    ${CMAKE_CURRENT_SOURCE_DIR}/SaliencyDetector
    ${OpenCV_INCLUDE_DIRS}
)

# ---------- 链接库 ----------
target_link_libraries(RTStitcher PRIVATE
    ${OpenCV_LIBS}
    spdlog::spdlog
    yaml-cpp
    Threads::Threads
)

# Linux 上 dlopen / clock_gettime 需要这两个库
if(UNIX AND NOT APPLE)
    target_link_libraries(RTStitcher PRIVATE dl rt)
endif()

# Windows 上不需要额外的 GDI 库，windows.h 自带
if(MSVC)
    target_link_libraries(RTStitcher PRIVATE Gdi32 User32 ws2_32)  # ws2_32: gaze socket
endif()

# ---------- 输出目录 ----------
set_target_properties(RTStitcher PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# [新增] VS 调试时把工作目录设为工程根（config.yaml 所在处），
#        这样默认相对路径 "config.yaml" 能直接命中；命令行运行不受影响。
set_target_properties(RTStitcher PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

# ---------- 安装 ----------
install(TARGETS RTStitcher RUNTIME DESTINATION bin)
