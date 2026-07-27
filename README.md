# Git 仓库

本仓库包含三个独立项目：

---

## 1. Launcher — 启动器

一个基于 **Qt (C++11)** 的桌面启动器程序。

- **框架：** Qt (core, gui, widgets)
- **文件：** `launcher.cpp` / `launcher.h` / `main.cpp`

---

## 2. RT-Stitching — 实时图像拼接

一个跨平台的**实时视频拼接系统**，支持多相机实时拼接，运行于 RK3588（ARM64）及 x86_64 平台。

### 特性
- 多相机实时拼接（支持 3+ 相机）
- **注视感知拼接**（Gaze-Aware Seam Finder，可选模块）
- **U2-Net 显著性检测**（Saliency-based Seam Finder，可选模块）
- OpenCV 特征提取与配准（SIFT / ORB）
- 曝光补偿与多频段融合（Multi-band Blender）
- 支持 Windows (MSVC + vcpkg) / Linux x86_64 / Linux ARM64

### 依赖
- OpenCV ≥ 4.5
- spdlog
- yaml-cpp
- CMake ≥ 3.16, C++17

### 构建
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

### 配置
通过 `config.yaml` 灵活配置相机数、特征类型、拼接参数等。

---

## 3. YamlConfigEditor — YAML 配置文件编辑器

一个基于 **Qt (C++11)** 的图形化 YAML 配置编辑工具，用于可视化编辑拼接系统的配置文件。

- **框架：** Qt (core, gui, widgets)
- **依赖：** yaml-cpp
- **功能：** 提供多个配置对话框，涵盖相机参数、曝光、特征、拼接、注视野等设置

### 对话框列表
| 对话框 | 用途 |
|--------|------|
| BasicSettingsDialog | 基础设置 |
| CameraAdjustDialog | 相机调整 |
| CameraInfoDialog / CameraInfoEditDialog | 相机信息查看与编辑 |
| CameraParamsDialog | 相机参数 |
| ExposureDialog | 曝光设置 |
| FeatureDialog | 特征提取配置 |
| FullScreenDialog | 全屏显示 |
| GazeDialog | 注视感知设置 |
| ScaleDialog | 缩放设置 |
| SeamDialog | 拼接缝设置 |

---

## 构建要求

| 项目 | 语言标准 | 构建系统 | 主要依赖 |
|------|----------|----------|----------|
| Launcher | C++11 | qmake | Qt |
| RT-Stitching | C++17 | CMake ≥ 3.16 | OpenCV, spdlog, yaml-cpp |
| YamlConfigEditor | C++11 | qmake | Qt, yaml-cpp |
