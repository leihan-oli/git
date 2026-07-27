cmake_minimum_required(VERSION 3.16)
project(YamlConfigEditor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 手动查找 yaml-cpp 库
find_path(YAML_CPP_INCLUDE_DIR
    NAMES yaml-cpp/yaml.h
    PATHS /usr/include /usr/local/include
)

find_library(YAML_CPP_LIBRARY
    NAMES yaml-cpp
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

if(NOT YAML_CPP_INCLUDE_DIR OR NOT YAML_CPP_LIBRARY)
    message(FATAL_ERROR "yaml-cpp not found. Please install libyaml-cpp-dev.")
endif()

qt6_add_executable(YamlConfigEditor
    main.cpp
    AllConfig.cpp
    MainWindow.cpp
    dialogs/BasicSettingsDialog.cpp
    dialogs/ScaleDialog.cpp
    dialogs/FeatureDialog.cpp
    dialogs/CameraAdjustDialog.cpp
    dialogs/ExposureDialog.cpp
    dialogs/SeamDialog.cpp
    dialogs/GazeDialog.cpp
    dialogs/CameraInfoDialog.cpp
    dialogs/CameraParamsDialog.cpp
    dialogs/FullScreenDialog.cpp
)

target_include_directories(YamlConfigEditor PRIVATE ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(YamlConfigEditor
    Qt5::Widgets
    ${YAML_CPP_LIBRARY}
)
