#ifndef VIDEOCAPTURE_H
#define VIDEOCAPTURE_H

// [修改] 把 windows.h / HMODULE / CALLBACK 抽到 Platform.hpp 里跨平台处理
#include "Platform.hpp"

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <string>
#include <mutex>
#include <condition_variable>
#include "Types.hpp"
#include "CircularBuffer.hpp"
#include "CircularBufferSync.hpp"

// RTSP_CX (CmPlay.dll) 相关回调签名
typedef void (CALLBACK* CMPDataStreamCallbackFuncDt)(int iCh, int iFlag, void** data, int iW, int iH, void* user);

struct CMPStruct {
    int iFlag;
    int iFrameGop;
    void* pUserData;
    CMPDataStreamCallbackFuncDt func;
};

class VideoCapture {
public:
    VideoCapture(
        bool show_window = true,
        bool record_video = true,
        const std::string& module_name = "");

    VideoCapture(CircularBuffer<RTStitching::Image>& output_buffer,
        bool show_window = true,
        bool record_video = true,
        const std::string& module_name = "");

    VideoCapture(RTStitching::CameraInfo camera_info,
        const RTStitching::CameraParams& camera_params,   // [新增] 传入 K/D 用于去畸变
        CircularBuffer<RTStitching::Image>& output_buffer,
        CircularBufferSync<RTStitching::Image>& sync_buffer_to_camest,
        size_t sync_buffer_to_camest_index,
        CircularBufferSync<RTStitching::Image>& sync_buffer_to_expest,
        size_t sync_buffer_to_expest_index,
        CircularBufferSync<RTStitching::Image>& sync_buffer_to_seamfinder,
        size_t sync_buffer_to_seamfinder_index,
        bool show_window = true,
        bool record_video = true,
        const std::string& module_name = "");

    ~VideoCapture();

    void start();
    void stop();
    void pause();
    void resume();
    bool isRunning() const;
    bool isPaused() const;
    bool stopRequested() const noexcept;
    std::string getModuleName() const;

    void stopRecording();
    void resumeRecording();
    void pauseRecording();

private:
    void run();
    void runImpl();

    // [新增] 去畸变：按需构建映射表 + 对单帧原地去畸变
    void buildUndistortMaps(const cv::Size& frame_size);
    void applyUndistort(cv::Mat& frame);

    bool waitForWork();

    // RTSP_CX 私有方法
    bool initRtspCx();
    void cleanupRtspCx();
    void processRtspCxFrame();
    static void CALLBACK rtspCxDataCallback(int iCh, int iFlag, void** data, int iW, int iH, void* user);
    void onRtspCxFrameReceived(int iCh, int iFlag, void** data, int iW, int iH);

    std::mutex mutex_;

    RTStitching::CameraInfo camera_info_;
    RTStitching::CameraParams camera_params_;   // [新增] 内参 K + 畸变系数 D
    // [新增] 去畸变映射表（initUndistortRectifyMap 预计算一次，每帧 remap 复用）
    cv::Mat undistort_map1_;
    cv::Mat undistort_map2_;
    bool    undistort_ready_   = false;   // 映射表是否已构建
    bool    undistort_enabled_ = false;   // 是否需要去畸变（开关 + D 非零）
    cv::Size undistort_size_;             // 映射表对应的帧尺寸
    CircularBuffer<RTStitching::Image>& output_buffer_;
    CircularBufferSync<RTStitching::Image>& sync_buffer_to_camest_;
    size_t sync_buffer_to_camest_index_;
    CircularBufferSync<RTStitching::Image>& sync_buffer_to_expest_;
    size_t sync_buffer_to_expest_index_;
    CircularBufferSync<RTStitching::Image>& sync_buffer_to_seamfinder_;
    size_t sync_buffer_to_seamfinder_index_;
    std::thread worker_thread_;
    std::atomic<bool> is_running_;
    std::atomic<bool> is_paused_;
    std::atomic<bool> stop_requested_;
    std::condition_variable cv_;
    bool show_window_;
    std::string module_name_;
    cv::VideoCapture cap_;
    bool need_resize_;
    cv::Mat image_frame_;
    uint64_t frame_index_;

    bool record_video_;

    bool initializeRecording(const cv::Mat& frame);
    void writeVideoFrame(const cv::Mat& frame);

    bool stop_record_;

    cv::VideoWriter video_writer_;
    std::string video_filename_;
    std::chrono::steady_clock::time_point recording_start_time_;
    bool recording_initialized_;
    int recorded_frames_;

    std::chrono::steady_clock::time_point last_frame_time_;
    int target_interval_ms_;
    bool first_frame_;

    static std::string formatTimestamp(const std::chrono::high_resolution_clock::time_point& tp);

    void* cm_dev_ = nullptr;
    std::mutex  cm_mtx_;
    cv::Mat     cm_frame_;

    // CmPlay.dll 函数指针
    void* (*CMP_InitDev)(const char*, int) = nullptr;
    void* (*CMP_UnInitDev)(void*) = nullptr;
    int   (*CMP_OpenDevMedia)(void*, int, void*) = nullptr;
    int   (*CMP_CloseDevMedia)(void*, int) = nullptr;
    int   (*CMP_SetMediaCallbackFunc)(void*, CMPStruct*) = nullptr;

    // [修改] 用 Platform.hpp 定义的 DllHandle 替代 HMODULE
    DllHandle cm_dll_handle_ = nullptr;
    bool rtsp_cx_initialized_ = false;
};

#endif // VIDEOCAPTURE_H
