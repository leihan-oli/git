#ifndef WARPING_MODULE_HPP
#define WARPING_MODULE_HPP

#include <opencv2/core.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "Types.hpp"
#include "CircularBuffer.hpp"
#include "CircularBufferSync.hpp"

class WarperModule {
public:
    WarperModule(CircularBuffer<RTStitching::Image>& input_buffer,
        CircularBuffer<RTStitching::Image>& output_buffer,
        RTStitching::ConfigParams& config_params,
        std::vector<RTStitching::CameraStitchParams>& camera_stitch_params,
        size_t camera_index,
        bool 
        
        = true,
        const std::string& module_name = "WarperModule");

    WarperModule(CircularBuffer<RTStitching::Image>& input_buffer,
        CircularBufferSync<RTStitching::Image>& sync_buffer_to_blender,
        RTStitching::ConfigParams& config_params,
        std::vector<RTStitching::CameraStitchParams>& camera_stitch_params,
        size_t camera_index,
        bool show_window = true,
        const std::string& module_name = "WarperModule");
    ~WarperModule();

    // 线程管理接口
    bool start();           // 启动线程
    void stop();            // 停止线程
    void pause();           // 暂停线程  
    void resume();          // 恢复线程
    bool isRunning() const; // 检查线程是否运行中
    bool isPaused() const;  // 检查线程是否暂停

    // 使用StitchingParams设置变形器
    bool initialize(const RTStitching::ConfigParams& params);



    // 执行单次图像变形（同步接口）
    bool warpImage(
        const RTStitching::Image& input_image,
        RTStitching::Image& output_image,
        cv::Point& corner,
        int camera_index
    );

    // 获取错误信息
    std::string getLastError() const { return last_error_; }

    // 获取处理时间
    double getLastProcessingTime() const { return last_processing_time_; }

    static cv::Ptr<cv::WarperCreator> initWarperCreator(
        std::string warper_type, bool try_cuda);

    static bool doWarp(
        cv::Ptr<cv::detail::RotationWarper> warper,
        cv::InputArray src,  // 使用 InputArray
        RTStitching::CameraParams camera,
        double trans_ratio,
        int interp_mode, int border_mode, 
        cv::OutputArray dst,  // 使用 OutputArray
        cv::Point& corner,
        double parallax_d0 = 0.0,   // [新增] >0 时启用近景视差补偿（需 spherical）
        float warper_scale = 0.0f   // [新增] 补偿路径所需的 warper 焦距（该尺度 mid_focal）
    );

    // [优化] 带缓存的逐帧变形：静态标定参数(K/R/focal/输入尺寸)不变时，
    //   只在首帧/参数变化时调用 buildMaps 预建映射表，之后每帧仅 remap。
    //   几何结果(corner/size)与静态 doWarp 完全一致，下游零改动。
    bool doWarpCached(
        const cv::Mat& src,
        const RTStitching::CameraParams& camera,
        double trans_ratio,
        int interp_mode, int border_mode,
        cv::Mat& dst,
        cv::Point& corner
    );

private:
    cv::Ptr<cv::WarperCreator> warper_creator_;
    cv::Ptr<cv::detail::RotationWarper> warper_;
    float mid_focal_;       // 用于记录前一次mid_focal_，构造时设为-1
    std::string last_error_;

    // [优化] 预建映射表缓存（doWarpCached 使用）
    cv::Mat warp_xmap_, warp_ymap_;          // 预建的 remap 映射表
    cv::Rect warp_dst_roi_;                  // buildMaps 返回的目标 ROI（corner=tl, size=roi+1）
    bool warp_maps_valid_ = false;           // 映射表是否有效
    cv::Size warp_cached_src_size_;          // 上次构建映射表时的(resize后/虚拟)输入尺寸
    bool     warp_cached_fold_ = false;      // [新增] 上次构建是否折叠了去畸变
    cv::Size warp_cached_raw_size_;          // [新增] 上次构建时的原始帧尺寸（折叠时使用）
    float warp_cached_focal_ = -1.0f;        // 上次构建时的 mid_focal_
    cv::Mat warp_cached_K_, warp_cached_R_;  // 上次构建时的缩放后 K / R
    double warp_cached_d0_ = -1.0;           // [新增] 上次构建时的 parallax_d0
    cv::Mat warp_cached_T_;                  // [新增] 上次构建时的相机光心 T（world 系）
    double last_processing_time_;

    // 变形参数
    std::string warper_type_;
    double warp_scale_;
    bool try_cuda_;

    // 相机参数
    RTStitching::ConfigParams& config_params_;
    std::vector<RTStitching::CameraStitchParams>& camera_stitch_params_;

    // 输入输出缓冲区
    CircularBuffer<RTStitching::Image>& input_buffer_;
    CircularBuffer<RTStitching::Image>& output_buffer_;
    CircularBufferSync<RTStitching::Image>& sync_buffer_to_blender_;
    size_t camera_index_;

    // 线程管理成员变量
    std::atomic<bool> is_running_;      // 线程运行状态
    std::atomic<bool> is_paused_;       // 线程暂停状态
    std::atomic<bool> stop_requested_;  // 停止请求标志
    std::thread worker_thread_;         // 工作线程
    mutable std::mutex mutex_;          // 互斥锁
    std::condition_variable condition_; // 条件变量
    bool show_window_;
    std::string module_name_;

    // 显示相关参数
    struct ProcessingTimestamps {
        std::chrono::high_resolution_clock::time_point image_original_timestamp; // 从Image结构体中获取的原始时间戳
        std::chrono::high_resolution_clock::time_point input_fetched_time;       // 从input_buffer取出的时间
        std::chrono::high_resolution_clock::time_point warping_start_time;       // warp开始时间
        std::chrono::high_resolution_clock::time_point warping_end_time;         // warp结束时间
        std::chrono::high_resolution_clock::time_point output_ready_time;        // 输出准备完成时间
    };

    struct DisplayInfo {
        std::vector<std::string> additional_messages;
        ProcessingTimestamps timestamps;
        cv::Size input_size;
        cv::Size output_size;
        cv::Point corner;
        uint64_t img_idx;
        cv::Mat input_image;
        cv::Mat output_image;

        // 添加CameraStitchParams版本信息
        int csp_ver;
        int sm_ver;

        // 添加相机参数信息
        cv::Mat K_matrix;  // 相机内参矩阵
        cv::Mat R_matrix;  // 相机旋转矩阵
        cv::Size stitch_size; // camera_stitch_params中的size
        cv::Point stitch_corner; // camera_stitch_params中的corner
    };

    // 成员变量
    std::vector<DisplayInfo> display_info_history_;
    static const size_t MAX_HISTORY_SIZE = 10; // 保留最近10帧的信息

    // 线程执行函数
    void runImpl();  // 线程主循环实现

    // 显示相关函数
    void addDisplayInfo(const DisplayInfo& info);
    cv::Mat createDisplayImage(const DisplayInfo& info);
    void drawTextWithBackground(cv::Mat& image, const std::string& text,
        cv::Point position, cv::Scalar text_color = cv::Scalar(255, 255, 255),
        cv::Scalar bg_color = cv::Scalar(0, 0, 0));
};

#endif // WARPING_MODULE_HPP
