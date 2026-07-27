#ifndef VISUALIZECAMERASTITCHINGPARAMS_HPP
#define VISUALIZECAMERASTITCHINGPARAMS_HPP

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <thread>

// [删除] 原代码包含 <conio.h>/<termios.h> 但本头文件实际未使用 _kbhit/_getch
// 跨平台键盘按键已经统一封装在 Utility.hpp 的 kbHit() / getKey()

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "Types.hpp"
#include "CircularBuffer.hpp"
#include "CircularBufferSync.hpp"
#include "FeatureFinder.hpp"
#include "Config.hpp"
#include "VideoCapture.hpp"
//#include "Blender.hpp"
#include "Displayer.hpp"
#include "SeamFinder.hpp"
#include "Warper.hpp"
//#include "CameraParamEst.hpp"
//#include "YamlLoader.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

// 显示窗口尺寸宏定义
#define DISPLAY_WINDOW_WIDTH 1280
#define DISPLAY_WINDOW_HEIGHT 720
#define DISPLAY_ROWS 3
#define DISPLAY_COLS 2

// 全局显示图像（在其他文件中定义）
extern cv::Mat global_display_image;
extern bool display_initialized;

extern void update_display_image(const std::vector<RTStitching::CameraStitchParams>& stitching_params);
extern void init_display_window();

#endif  // VISUALIZECAMERASTITCHINGPARAMS_HPP
