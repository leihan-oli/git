#ifndef DISPLAYER_H
#define DISPLAYER_H

// [修改] 用 Platform.hpp 统一处理 windows.h，Linux 上不再 include windows.h
#include "Platform.hpp"

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "CircularBuffer.hpp"
#include "Types.hpp"
#include "../Config/Config.hpp"

class Displayer {
public:
    explicit Displayer(
        CircularBuffer<RTStitching::Image>& input_buffer,
        RTStitching::ConfigParams& config_params,
        void* hwnd = nullptr,                     // Windows 下可传 HWND，Linux 下永远 nullptr
        const std::string& window_name = "Stitching Result",
        bool show_window = true);

    std::condition_variable condition_var_;

    ~Displayer();

    void start();
    void stop();
    void pause();
    void resume();
    bool isRunning() const;
    bool isPaused() const;

    // [新增] Windows 专用：由主线程取走最新处理好的一帧来 imshow
    // （imshow 必须在主线程调用，工作线程只负责把帧缓存进来）
    // 返回 true 表示拿到了新的一帧，out 被填充；false 表示暂时没有新帧。
    bool getLatestFrame(cv::Mat& out);

    const std::string& getWindowName() const { return window_name_; }

private:
    void run();
    bool waitForWork();

    void drawToHwnd(void* hwnd, const cv::Mat& img);

    CircularBuffer<RTStitching::Image>& input_buffer_;
    std::string window_name_;
    bool show_window_;
    void* display_hwnd_;

    std::thread worker_thread_;
    std::atomic<bool> is_running_;
    std::atomic<bool> is_paused_;
    std::atomic<bool> stop_requested_;

    std::mutex mutex_;

    int output_width_;
    int output_height_;

    // [新增] 最新一帧的缓存（供主线程取走显示）
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;
    std::atomic<bool> has_new_frame_{ false };

    cv::Mat scaleMaskToFitRectangle(const cv::Mat& mask, int target_width, int target_height);
};

#endif // DISPLAYER_H
