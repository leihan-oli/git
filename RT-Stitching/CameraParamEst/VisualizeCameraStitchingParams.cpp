#include "CameraParamEst.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "../Utility/DebugDump.hpp"   // [新增] 调试图像统一写盘 /root/build/debug/

// 静态成员变量定义（必须在类外定义）
cv::Mat CameraParamEst::global_display_image;
bool CameraParamEst::display_initialized = false;

void CameraParamEst::visualize_stitching_params(
    const std::vector<RTStitching::CameraStitchParams>& stitching_params)
{
    // 初始化显示画布（namedWindow/resizeWindow 属 HighGUI，Linux 工作线程调用
    // 会与 Qt 抢占 GUI 资源，故仅 Windows 创建窗口）
    if (!display_initialized) {
        global_display_image = cv::Mat::zeros(DISPLAY_WINDOW_HEIGHT, DISPLAY_WINDOW_WIDTH, CV_8UC3);
#ifdef _WIN32
        cv::namedWindow("Stitching Masks Overview", cv::WINDOW_NORMAL);
        cv::resizeWindow("Stitching Masks Overview", DISPLAY_WINDOW_WIDTH, DISPLAY_WINDOW_HEIGHT);
#endif
        display_initialized = true;
    }

    // 清空显示图像
    global_display_image.setTo(cv::Scalar(0, 0, 0));

    // 计算子图尺寸
    const int subplot_width = DISPLAY_WINDOW_WIDTH / DISPLAY_COLS;
    const int subplot_height = DISPLAY_WINDOW_HEIGHT / DISPLAY_ROWS;

    // 定义缩放比例映射
    std::vector<double> scale_ratios = { 1.0, 0.8, 0.8, 0.32, 1.0 };

    // 处理前5个scale_cnt
    for (int scale_cnt = 0; scale_cnt < std::min(5, (int)stitching_params.size()); scale_cnt++) {
        if (scale_cnt >= stitching_params.size() ||
            stitching_params[scale_cnt].masks.size() < 2) {
            continue;
        }

        // 计算子图位置
        int row = scale_cnt / DISPLAY_COLS;
        int col = scale_cnt % DISPLAY_COLS;
        cv::Rect subplot_roi(col * subplot_width, row * subplot_height,
            subplot_width, subplot_height);

        // 获取两个mask（索引0和1）
        const cv::UMat& mask1 = stitching_params[scale_cnt].masks[0];
        const cv::UMat& mask2 = stitching_params[scale_cnt].masks[1];
        const cv::Point& corner1 = stitching_params[scale_cnt].corners[0];
        const cv::Point& corner2 = stitching_params[scale_cnt].corners[1];

        // 计算最小外切矩阵
        cv::Rect rect1(corner1, stitching_params[scale_cnt].sizes[0]);
        cv::Rect rect2(corner2, stitching_params[scale_cnt].sizes[1]);
        cv::Rect union_rect = rect1 | rect2;

        if (union_rect.width <= 0 || union_rect.height <= 0) {
            continue;
        }

        // 计算缩放比例以适应子图（保留10%的边距）
        double scale_x = (subplot_width * 0.9) / union_rect.width;
        double scale_y = (subplot_height * 0.9) / union_rect.height;
        double scale = std::min(scale_x, scale_y);

        // 创建缩放后的画布
        cv::Mat resized_canvas;
        cv::Mat canvas = cv::Mat::zeros(union_rect.size(), CV_8UC3);

        // 绘制两个mask
        cv::Mat mask1_mat, mask2_mat;
        mask1.getMat(cv::ACCESS_READ).copyTo(mask1_mat);
        mask2.getMat(cv::ACCESS_READ).copyTo(mask2_mat);

        // 创建彩色mask
        cv::Mat red_mask = cv::Mat::zeros(union_rect.size(), CV_8UC3);
        cv::Mat blue_mask = cv::Mat::zeros(union_rect.size(), CV_8UC3);

        cv::Rect roi1(corner1.x - union_rect.x, corner1.y - union_rect.y,
            mask1_mat.cols, mask1_mat.rows);
        cv::Rect roi2(corner2.x - union_rect.x, corner2.y - union_rect.y,
            mask2_mat.cols, mask2_mat.rows);

        red_mask(roi1).setTo(cv::Scalar(0, 0, 255), mask1_mat);
        blue_mask(roi2).setTo(cv::Scalar(255, 0, 0), mask2_mat);

        // 混合（50%透明度）
        cv::addWeighted(red_mask, 0.5, blue_mask, 0.5, 0, canvas);

        // 缩放
        int new_width = cvRound(union_rect.width * scale);
        int new_height = cvRound(union_rect.height * scale);
        cv::resize(canvas, resized_canvas, cv::Size(new_width, new_height), 0, 0, cv::INTER_AREA);

        // 将缩放后的图像放置到子图中心
        int start_x = subplot_roi.x + (subplot_width - resized_canvas.cols) / 2;
        int start_y = subplot_roi.y + (subplot_height - resized_canvas.rows) / 2;

        if (start_x >= 0 && start_y >= 0 &&
            start_x + resized_canvas.cols <= global_display_image.cols &&
            start_y + resized_canvas.rows <= global_display_image.rows) {

            cv::Rect dest_roi(start_x, start_y, resized_canvas.cols, resized_canvas.rows);
            resized_canvas.copyTo(global_display_image(dest_roi));
        }

        // 添加标题
        std::string title = "Scale " + std::to_string(scale_cnt) + " (Ratio:" +
            std::to_string(scale_ratios[scale_cnt]) + ")";
        cv::putText(global_display_image, title,
            cv::Point(subplot_roi.x + 5, subplot_roi.y + 20),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

        // 添加尺寸信息
        std::string size_info = "Size: " + std::to_string(new_width) + "x" + std::to_string(new_height);
        cv::putText(global_display_image, size_info,
            cv::Point(subplot_roi.x + 5, subplot_roi.y + 40),
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
    }

    // 在最后一个子图位置显示详细信息
    int info_col = 1;  // 第二列
    int info_row = 2;  // 第三行
    cv::Rect info_roi(info_col * subplot_width, info_row * subplot_height,
        subplot_width, subplot_height);

    // 绘制信息背景
    global_display_image(info_roi).setTo(cv::Scalar(50, 50, 50));

    // 添加详细信息标题
    cv::putText(global_display_image, "Mask Details:",
        cv::Point(info_roi.x + 10, info_roi.y + 25),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 1);

    int text_y = info_roi.y + 50;
    int line_height = 20;

    // 显示每个scale_cnt的详细信息
    for (int scale_cnt = 0; scale_cnt < std::min(5, (int)stitching_params.size()); scale_cnt++) {
        if (scale_cnt >= stitching_params.size() ||
            stitching_params[scale_cnt].masks.size() < 2) {
            continue;
        }

        std::string info = "Scale" + std::to_string(scale_cnt) + ": ";
        info += "C0=(" + std::to_string(stitching_params[scale_cnt].corners[0].x) + "," +
            std::to_string(stitching_params[scale_cnt].corners[0].y) + ") ";
        info += "S0=" + std::to_string(stitching_params[scale_cnt].sizes[0].width) + "x" +
            std::to_string(stitching_params[scale_cnt].sizes[0].height);

        cv::putText(global_display_image, info,
            cv::Point(info_roi.x + 10, text_y),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        text_y += line_height;

        info = "         C1=(" + std::to_string(stitching_params[scale_cnt].corners[1].x) + "," +
            std::to_string(stitching_params[scale_cnt].corners[1].y) + ") ";
        info += "S1=" + std::to_string(stitching_params[scale_cnt].sizes[1].width) + "x" +
            std::to_string(stitching_params[scale_cnt].sizes[1].height);

        cv::putText(global_display_image, info,
            cv::Point(info_roi.x + 10, text_y),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        text_y += line_height;
    }

    // [修改] 掩膜总览写盘 /root/build/debug/Stitching_Masks_Overview.jpg（Windows 仍 imshow）
    RTStitching::debugDump("Stitching Masks Overview", global_display_image);
}