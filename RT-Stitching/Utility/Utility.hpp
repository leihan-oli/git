#ifndef UTILITY_HPP
#define UTILITY_HPP

#ifdef _WIN32
#include <conio.h>           // Windows平台专用：键盘按键检测与获取
#else
#include <termios.h>         // Linux/Unix：终端属性
#include <unistd.h>          // Linux/Unix：系统调用
#include <fcntl.h>           // [新增] F_GETFL / F_SETFL / O_NONBLOCK，原代码遗漏
#include <cstdio>            // [新增] EOF / getchar / ungetc
#endif

#include <spdlog/spdlog.h>

// 1. OpenCV核心数据结构
#include <opencv2/core.hpp>
// 2. OpenCV图像变形相关
#include <opencv2/stitching/detail/warpers.hpp>
// 3. OpenCV图像处理
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
// 4. 项目自定义类型
#include "Types.hpp"
// 5. 项目自定义变形器模块
#include "../Warper/Warper.hpp"
// 6. C++标准库
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int kbHit();
int getKey();

// ============================================================
// [新增] 近景视差补偿的球面 warp（RTStitching::parallax_d0）
//   语义：不再假设场景在无穷远，而是假设在以 world(=middle 光心) 为球心、
//   半径 d0(米) 的球面上。建映射时把相机光心在 world 系的位置 C（即
//   camera_params[i].T，由标定外参导出）代入 —— d0 处的物体各路严格对齐，
//   偏离 d0 的深度错位为 f·b·|1/d − 1/d0|（对比纯旋转的 f·b/d）。
//   约定：R = 相机→world；C = 相机光心在 world 系坐标（米）；
//         d0 <= 0 时退化为与 OpenCV SphericalWarper 完全相同的纯旋转映射。
//   仅支持 spherical 投影（与本工程 warper_type 一致）。
// ============================================================
namespace RTStitching {

// 构建后向 remap 映射表（等价 RotationWarper::buildMaps 的补偿版）。
// 返回目标 ROI；xmap/ymap 为 CV_32FC1，尺寸 (roi.height+1, roi.width+1)，
// 与 OpenCV RotationWarperBase::buildMaps 约定一致。
cv::Rect buildSphericalMapsD0(
    cv::Size src_size,
    const cv::Mat& K, const cv::Mat& R, const cv::Mat& T_center,
    float scale, double d0,
    cv::Mat& xmap, cv::Mat& ymap);

// 一次性 warp 一幅图（等价 RotationWarper::warp 的补偿版），返回 corner。
cv::Point warpSphericalD0(
    cv::InputArray src,
    const cv::Mat& K, const cv::Mat& R, const cv::Mat& T_center,
    float scale, double d0,
    int interp_mode, int border_mode,
    cv::OutputArray dst);

} // namespace RTStitching

void update_stitching_params(
    RTStitching::ConfigParams& config_params,
    std::vector<RTStitching::CameraStitchParams>& stitching_params,
    RTStitching::scale_t base_scale);
std::string typeToString_tmp(int type);
std::string matToString(const cv::Mat& mat);
cv::Vec3d matrixToEuler(const cv::Mat& R);
std::string cameraParamsToString(const cv::Mat& K, const cv::Mat& R);
inline double rad2deg(double rad);
cv::Mat eulerToMatrix(double roll, double pitch, double yaw);
#endif
