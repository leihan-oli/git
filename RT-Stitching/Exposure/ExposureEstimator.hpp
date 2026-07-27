#ifndef EXPOSURE_ESTIMATOR_HPP
#define EXPOSURE_ESTIMATOR_HPP

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "../Types/Types.hpp"
#include "../CircularBufferSync/CircularBufferSync.hpp"
#include "../CircularBuffer/CircularBuffer.hpp"

class ExposureEstimator {
public:
    ExposureEstimator(
        const RTStitching::ConfigParams& config_params,
        std::vector<RTStitching::CameraStitchParams>& camera_stitch_params,
        int index,
        CircularBufferSync<RTStitching::Image>& input_buffer,
        CircularBuffer<RTStitching::ExposureCompensatorWrapper>& out_buffer
    );
    ~ExposureEstimator();

    // 线程管理接口
    bool start();
    void stop();
    void pause();
    void resume();
    bool isRunning() const;
    bool isPaused() const;

    // 初始化配置参数
    bool initialize(const RTStitching::ConfigParams& params);

    // 错误信息和处理时间
    std::string getLastError() const { return last_error_; }
    double getLastProcessingTime() const { return last_processing_time_; }

private:
    // 曝光补偿参数
    std::string exp_type_;
    int exp_nr_feeds_;
    int exp_nr_filtering_;
    int exp_block_size_;
    bool try_cuda_;
    double work_scale_;  // 工作尺度，使用 EXP_EST_SCALE

    std::string last_error_;
    double last_processing_time_;
    const RTStitching::ConfigParams& config_params_;
    std::vector<RTStitching::CameraStitchParams>& camera_stitch_params_;
    int index_;
    // 线程管理
    std::atomic<bool> is_running_;
    std::atomic<bool> is_paused_;
    std::atomic<bool> stop_requested_;
    std::thread worker_thread_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;

    // 缓冲区
    CircularBufferSync<RTStitching::Image>& input_buffer_;
    CircularBuffer<RTStitching::ExposureCompensatorWrapper>& output_buffer_;

    // 线程执行函数
    void runImpl();

    // 创建新的补偿器实例
    bool createNewCompensatorInstance(cv::Ptr<cv::detail::ExposureCompensator>& out_compensator);

    // 图像预处理函数 - 使用 EXP_EST_SCALE 进行缩放
    bool preprocessImages(const std::vector<RTStitching::Image>& input_images,
        std::vector<cv::UMat>& processed_images);

    // Warp图像函数 - 使用预处理后的图像
    bool warpImages(const std::vector<cv::UMat>& images,
        std::vector<cv::UMat>& warped_images);

    // 执行曝光估计
    bool estimateExposure(const std::vector<cv::UMat>& warped_images,
        RTStitching::ExposureCompensatorWrapper& result);

    // 获取当前帧ID
    uint64_t getCurrentFrameId();
};

#endif // EXPOSURE_ESTIMATOR_HPP