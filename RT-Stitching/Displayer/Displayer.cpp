#include "Displayer.hpp"
#include <stdexcept>
#include <iostream>
#include <spdlog/spdlog.h>
#include <opencv2/imgcodecs.hpp>   // cv::imwrite（Linux 写盘用）

Displayer::Displayer(
    CircularBuffer<RTStitching::Image>& input_buffer,
    RTStitching::ConfigParams& config_params,
    void* hwnd,
    const std::string& window_name,
    bool show_window)
    : input_buffer_(input_buffer),
    window_name_(window_name),
    show_window_(show_window),
    display_hwnd_(hwnd),
    is_running_(false),
    is_paused_(false),
    stop_requested_(false),
    output_width_(config_params.output_width),
    output_height_(config_params.output_height) {
}

Displayer::~Displayer() {
    stop();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    // 注意：Windows 下窗口现在由主线程创建/销毁，这里不再 destroyWindow
}

void Displayer::start() {
    if (!is_running_) {
        stop_requested_ = false;
        is_running_ = true;
        worker_thread_ = std::thread(&Displayer::run, this);
    }
}

void Displayer::stop() {
    if (is_running_) {
        stop_requested_ = true;
        condition_var_.notify_all();
    }
}

void Displayer::pause() {
    if (is_running_ && !is_paused_) is_paused_ = true;
}

void Displayer::resume() {
    if (is_running_ && is_paused_) {
        is_paused_ = false;
        condition_var_.notify_one();
    }
}

bool Displayer::isRunning() const { return is_running_; }
bool Displayer::isPaused() const { return is_paused_; }

// [新增] 主线程取走最新一帧
bool Displayer::getLatestFrame(cv::Mat& out) {
    if (!has_new_frame_.load(std::memory_order_acquire)) return false;
    std::lock_guard<std::mutex> lk(frame_mutex_);
    if (latest_frame_.empty()) return false;
    latest_frame_.copyTo(out);
    has_new_frame_.store(false, std::memory_order_release);
    return true;
}

void Displayer::run() {
    // 注意：不在工作线程里创建 OpenCV 窗口（Windows 下窗口归主线程管）

    while (!stop_requested_) {
        if (!waitForWork()) break;

        try {
            RTStitching::Image img;
            if (!input_buffer_.empty())
            {
                img = input_buffer_.pop_front();
                if (img.data.empty()) continue;

                cv::Mat result;
                cv::Mat result_mask;
                img.data.copyTo(result);
                if (!img.mask.empty()) img.mask.copyTo(result_mask);
                else result_mask = cv::Mat::ones(result.size(), CV_8U) * 255;

                cv::Mat result_display;
                if (result.type() == CV_16SC3) {
                    result.convertTo(result_display, CV_8UC3, 255.0 / 65535.0, 127.5);
                }
                else {
                    if (result.channels() == 3) result.convertTo(result_display, CV_8UC3);
                    else cv::cvtColor(result, result_display, cv::COLOR_GRAY2BGR);
                }

                cv::Mat final_output_image;
                cv::Mat scaled_mask = scaleMaskToFitRectangle(result_mask, output_width_, output_height_);

                if (!scaled_mask.empty() && !result_display.empty()) {
                    cv::Mat scaled_result;
                    cv::resize(result_display, scaled_result, scaled_mask.size(), 0, 0, cv::INTER_LINEAR);

                    int img_w = scaled_result.cols;
                    int img_h = scaled_result.rows;
                    int target_x = (img_w - output_width_) / 2;
                    int target_y = (img_h - output_height_) / 2;

                    cv::Rect target_rect(target_x, target_y, output_width_, output_height_);
                    target_rect &= cv::Rect(0, 0, img_w, img_h);

                    if (target_rect.width > 0 && target_rect.height > 0) {
                        final_output_image = scaled_result(target_rect).clone();
                    }
                    else {
                        final_output_image = scaled_result;
                    }
                }
                else {
                    final_output_image = result_display;
                }

                // ============ 显示逻辑：按平台分开 ============
#ifdef _WIN32
                // ---------- Windows ----------
                if (display_hwnd_ != nullptr) {
                    // 若外部传入了窗口句柄（嵌入到某个 HWND），仍走 GDI 渲染
                    drawToHwnd(display_hwnd_, final_output_image);
                }
                else if (show_window_ && !final_output_image.empty()) {
                    // imshow 必须在主线程调用，这里只把最新一帧缓存起来，
                    // 由 main() 的循环取走并 imshow（见 RTStitcher.cpp）。
                    std::lock_guard<std::mutex> lk(frame_mutex_);
                    final_output_image.copyTo(latest_frame_);
                    has_new_frame_.store(true, std::memory_order_release);
                }
#else
                // ---------- Linux 开发板：imshow 与 Qt 多窗口线程冲突 ----------
                // 不调用任何 HighGUI，改为周期性把结果写成 result.jpg。
                if (!final_output_image.empty()) {
                    static int s_save_counter = 0;
                    if ((s_save_counter++ % 15) == 0) {
                        try {
                            cv::imwrite("result.jpg", final_output_image);
                        }
                        catch (const std::exception& e) {
                            spdlog::warn("[DISPLAYER] imwrite failed: {}", e.what());
                        }
                    }
                }
#endif
                // =============================================
            }
        }
        catch (const std::exception& e) {
            spdlog::error("[DISPLAYER] error: {}", e.what());
            continue;
        }
    }
    is_running_ = false;
}

bool Displayer::waitForWork() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_var_.wait(lock, [this]() {
        return !is_paused_ || stop_requested_ || !input_buffer_.empty();
    });
    return !stop_requested_;
}

cv::Mat Displayer::scaleMaskToFitRectangle(const cv::Mat& mask, int output_width_, int output_height_) {
    if (mask.empty()) return cv::Mat();

    try {
        cv::Mat mask_gray;
        if (mask.channels() > 1) cv::cvtColor(mask, mask_gray, cv::COLOR_BGR2GRAY);
        else mask_gray = mask.clone();

        cv::threshold(mask_gray, mask_gray, 128, 255, cv::THRESH_BINARY);

        std::vector<cv::Point> mask_points;
        cv::findNonZero(mask_gray, mask_points);
        if (mask_points.empty()) return cv::Mat();

        cv::Rect mask_rect = cv::boundingRect(mask_points);
        double scale_x = static_cast<double>(output_width_) / static_cast<double>(mask_rect.width);
        double scale_y = static_cast<double>(output_height_) / static_cast<double>(mask_rect.height);
        double scale = std::max(scale_x, scale_y) * 1.2;
        scale = std::max(1.0, std::min(scale, 5.0));

        int new_width  = static_cast<int>(mask_gray.cols * scale);
        int new_height = static_cast<int>(mask_gray.rows * scale);

        cv::Mat scaled_mask;
        cv::resize(mask_gray, scaled_mask, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);

        int canvas_width  = new_width  + 200;
        int canvas_height = new_height + 200;
        cv::Mat canvas = cv::Mat::zeros(canvas_height, canvas_width, CV_8UC1);

        int start_x = (canvas_width  - new_width)  / 2;
        int start_y = (canvas_height - new_height) / 2;
        cv::Rect roi_rect(start_x, start_y, new_width, new_height);

        if (roi_rect.x >= 0 && roi_rect.width <= canvas_width && roi_rect.height <= canvas_height) {
            scaled_mask.copyTo(canvas(roi_rect));
        }
        return canvas;
    }
    catch (const std::exception& e) {
        spdlog::error("[DISPLAYER] Error scaling mask: {}", e.what());
        return cv::Mat();
    }
}

// =====================================================================
// drawToHwnd —— 整段 GDI 渲染用 #ifdef _WIN32 包起来
// =====================================================================
void Displayer::drawToHwnd(void* hwnd, const cv::Mat& img) {
#ifdef _WIN32
    if (hwnd == nullptr || img.empty()) return;
    HWND hWindow = static_cast<HWND>(hwnd);
    if (!IsWindow(hWindow)) return;

    cv::Mat render_img = img;
    if (render_img.type() != CV_8UC3) {
        if (render_img.channels() == 1) cv::cvtColor(render_img, render_img, cv::COLOR_GRAY2BGR);
        else render_img.convertTo(render_img, CV_8UC3);
    }
    if (!render_img.isContinuous()) render_img = render_img.clone();

    int width = render_img.cols;
    int height = render_img.rows;
    int channels = 3;
    int gdi_stride = ((width * 24 + 31) / 32) * 4;

    std::vector<uint8_t> aligned_buffer;
    const uint8_t* pData = nullptr;

    if (render_img.step != gdi_stride) {
        aligned_buffer.resize(gdi_stride * height);
        uint8_t* dst_ptr = aligned_buffer.data();
        for (int y = 0; y < height; ++y) {
            const uint8_t* src_row = render_img.ptr<uint8_t>(y);
            memcpy(dst_ptr + (y * gdi_stride), src_row, width * channels);
        }
        pData = aligned_buffer.data();
    }
    else {
        pData = render_img.data;
    }

    HDC hdc = GetDC(hWindow);
    if (hdc == NULL) return;

    RECT rect;
    GetClientRect(hWindow, &rect);
    int win_w = rect.right - rect.left;
    int win_h = rect.bottom - rect.top;

    int dst_x = 0, dst_y = 0, dst_w = win_w, dst_h = win_h;
    if (width > 0 && height > 0) {
        double aspect_ratio = (double)width / height;
        double win_aspect = (double)win_w / win_h;

        if (win_aspect > aspect_ratio) {
            dst_h = win_h;
            dst_w = (int)(win_h * aspect_ratio);
            dst_x = (win_w - dst_w) / 2;
        }
        else {
            dst_w = win_w;
            dst_h = (int)(win_w / aspect_ratio);
            dst_y = (win_h - dst_h) / 2;
        }
    }

    BITMAPINFO bmi = { 0 };
    bmi.bmiHeader.biSize        = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth       = width;
    bmi.bmiHeader.biHeight      = -height;
    bmi.bmiHeader.biPlanes      = 1;
    bmi.bmiHeader.biBitCount    = 24;
    bmi.bmiHeader.biCompression = BI_RGB;
    bmi.bmiHeader.biSizeImage   = gdi_stride * height;

    SetStretchBltMode(hdc, HALFTONE);

    StretchDIBits(hdc, dst_x, dst_y, dst_w, dst_h,
        0, 0, width, height,
        pData, &bmi, DIB_RGB_COLORS, SRCCOPY);

    if (dst_x > 0 || dst_y > 0) {
        HBRUSH hBrush = CreateSolidBrush(RGB(0, 0, 0));
        RECT r1 = { 0, 0, win_w, dst_y };
        RECT r2 = { 0, dst_y + dst_h, win_w, win_h };
        RECT r3 = { 0, dst_y, dst_x, dst_y + dst_h };
        RECT r4 = { dst_x + dst_w, dst_y, win_w, dst_y + dst_h };
        FillRect(hdc, &r1, hBrush);
        FillRect(hdc, &r2, hBrush);
        FillRect(hdc, &r3, hBrush);
        FillRect(hdc, &r4, hBrush);
        DeleteObject(hBrush);
    }
    ReleaseDC(hWindow, hdc);
#else
    (void)hwnd; (void)img;
#endif
}
