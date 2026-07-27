#ifndef EXPOSURE_COMPENSATOR_HPP
#define EXPOSURE_COMPENSATOR_HPP

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "../Types/Types.hpp"
#include "../CircularBuffer/CircularBuffer.hpp"
#include "../CircularBufferSync/CircularBufferSync.hpp"  // 添加同步环形缓冲区头文件

class ExposureCompensator {
public:
    ExposureCompensator();
    ~ExposureCompensator();

    // 线程管理接口
    bool start();
    void stop();
    void pause();
    void resume();
    bool isRunning() const;
    bool isPaused() const;

    // 设置输入缓冲区 - 使用Image数据结构，普通环形缓冲区
    void setInputBuffer(CircularBuffer<std::vector<RTStitching::Image>>* buffer) {
        input_buffer_ = buffer;
    }

    // 设置曝光数据缓冲区 - 存储补偿器包装结构，普通环形缓冲区
    void setExposureDataBuffer(CircularBuffer<RTStitching::ExposureCompensatorWrapper>* buffer) {
        exposure_data_buffer_ = buffer;
    }

    // 设置输出缓冲区 - 输出补偿后的Image数据，使用同步环形缓冲区
    void setOutputBuffer(CircularBufferSync<std::vector<RTStitching::Image>>* buffer) {
        output_buffer_ = buffer;
    }

    // 设置相机参数和拼接参数 - 用于更新全局状态
    void setCameraParams(std::vector<RTStitching::CameraParams>* params) {
        config_params_. = params;
    }

    void setStitchParams(std::vector<RTStitching::CameraStitchParams>* params) {
        stitch_params_ = params;
    }

    // 设置相机索引 - 标识处理哪个相机的数据，也用于同步环形缓冲区的写入索引
    void setCameraIndex(int index) {
        camera_index_ = index;
    }

    // 错误信息和处理时间
    std::string getLastError() const { return last_error_; }
    double getLastProcessingTime() const { return last_processing_time_; }

private:
    std::string last_error_;
    double last_processing_time_;
    int camera_index_;  // 相机索引，用于标识处理哪个相机的数据，也用于同步环形缓冲区的写入索引

    // 当前曝光补偿数据
    RTStitching::ExposureCompensatorWrapper current_exposure_data_;
    mutable std::mutex exposure_data_mutex_;

    // 全局参数指针
    std::vector<RTStitching::CameraStitchParams>& stitch_params_;

    // 线程管理
    std::atomic<bool> is_running_;
    std::atomic<bool> is_paused_;
    std::atomic<bool> stop_requested_;
    std::thread worker_thread_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;

    // 缓冲区
    CircularBuffer<std::vector<RTStitching::Image>>* input_buffer_;  // 输入：普通环形缓冲区
    CircularBuffer<RTStitching::ExposureCompensatorWrapper>* exposure_data_buffer_;  // 曝光数据：普通环形缓冲区
    CircularBufferSync<std::vector<RTStitching::Image>>* output_buffer_;  // 输出：同步环形缓冲区

    // 线程执行
    void runImpl();

    // 更新曝光补偿数据
    void updateExposureData();

    // 执行曝光补偿 - 直接使用输入图像，不进行缩放
    bool applyExposureCompensation(const std::vector<RTStitching::Image>& input_images,
        std::vector<RTStitching::Image>& output_images);

    // 更新全局掩码
    void updateGlobalMasks(const std::vector<cv::UMat>& masks);
};

#endif // EXPOSURE_COMPENSATOR_HPP