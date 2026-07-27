#include "ExposureEstimator.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <chrono>
#include <iostream>

using namespace cv;

ExposureEstimator::ExposureEstimator(
    const RTStitching::ConfigParams& config_params,
    std::vector<RTStitching::CameraStitchParams>& camera_stitch_params,
    CircularBufferSync<RTStitching::Image>& input_buffer,
    CircularBuffer<RTStitching::ExposureCompensatorWrapper>& out_buffer
) : 
    config_params_(config_params),
    camera_stitch_params_(camera_stitch_params),
    last_processing_time_(0.0), is_running_(false), 
    is_paused_(false),
    stop_requested_(false), 
    try_cuda_(config_params.try_cuda), 
    exp_nr_feeds_(1),
    exp_nr_filtering_(2), 
    exp_block_size_(32), 
    work_scale_(config_params.scale_megapix[RTStitching::EXP_EST_SCALE]),
    input_buffer_(input_buffer), 
    output_buffer_(out_buffer) {
} 

ExposureEstimator::~ExposureEstimator() {
    stop();
}

// 线程管理实现保持不变
bool ExposureEstimator::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_running_) return false;

    is_running_.store(true);
    is_paused_.store(false);
    stop_requested_.store(false);

    worker_thread_ = std::thread(&ExposureEstimator::runImpl, this);
    std::cout << "Exposure estimator thread started" << std::endl;
    return true;
}

void ExposureEstimator::stop() {
    if (!is_running_.load()) return;

    stop_requested_.store(true);
    condition_.notify_all();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    is_running_.store(false);
    is_paused_.store(false);
    std::cout << "Exposure estimator thread stopped" << std::endl;
}

void ExposureEstimator::pause() {
    is_paused_.store(true);
    std::cout << "Exposure estimator thread paused" << std::endl;
}

void ExposureEstimator::resume() {
    if (is_paused_.load()) {
        is_paused_.store(false);
        condition_.notify_one();
        std::cout << "Exposure estimator thread resumed" << std::endl;
    }
}

bool ExposureEstimator::isRunning() const {
    return is_running_.load(std::memory_order_acquire);
}

bool ExposureEstimator::isPaused() const {
    return is_paused_.load(std::memory_order_acquire);
}

void ExposureEstimator::runImpl() {
    std::cout << "Exposure estimator thread main loop started" << std::endl;

    while (!stop_requested_.load()) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] {
                return stop_requested_.load() || !is_paused_.load();
                });
        }

        if (stop_requested_.load()) {
            break;
        }

        // 处理逻辑
        if (!config_params_.camera_params.empty()) {
            try {
                std::vector<RTStitching::Image> images_batch;
                std::vector<bool> new_data_flags;

                // 从同步环形缓冲区获取最新数据
                if (input_buffer_.back(images_batch, new_data_flags)) {
                    // 取最新的一帧数据
                    if (!images_batch.empty()) {

                        // 步骤1: 图像预处理（缩放）- 使用 EXP_EST_SCALE
                        std::vector<cv::UMat> scaled_images;
                        if (preprocessImages(images_batch, scaled_images)) {

                            // 步骤2: Warp图像变换
                            std::vector<cv::UMat> warped_images;
                            if (warpImages(scaled_images, warped_images)) {

                                // 步骤3: 执行曝光估计
                                RTStitching::ExposureCompensatorWrapper result;
                                if (estimateExposure(warped_images, result)) {
                                    // 设置帧ID和时间戳
                                    result.frame_id = images_batch[0].img_idx;
                                    result.timestamp = std::chrono::high_resolution_clock::now();

                                    // 存入输出缓冲区
                                    if (output_buffer_.try_push_back(result)) {
                                        std::cout << "Exposure estimation completed for frame "
                                            << result.frame_id << std::endl;
                                    }
                                    else {
                                        std::cerr << "Failed to push exposure data to buffer" << std::endl;
                                    }
                                }
                                else {
                                    std::cerr << "Exposure estimation failed: " << last_error_ << std::endl;
                                }
                            }
                            else {
                                std::cerr << "Image warping failed: " << last_error_ << std::endl;
                            }
                        }
                        else {
                            std::cerr << "Image preprocessing failed: " << last_error_ << std::endl;
                        }
                    }
                }
            }
            catch (const std::exception& e) {
                last_error_ = std::string("Processing exception: ") + e.what();
                std::cerr << last_error_ << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Exposure estimator thread main loop ended" << std::endl;
}

bool ExposureEstimator::initialize(const RTStitching::ConfigParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 保存参数
    exp_type_ = params.exp_type;
    exp_nr_feeds_ = params.exp_nr_feeds;
    exp_nr_filtering_ = params.exp_nr_filtering;
    exp_block_size_ = params.exp_block_size;
    try_cuda_ = params.try_cuda;
    work_scale_ = params.scale_megapix[RTStitching::EXP_EST_SCALE]; // 使用曝光估计尺度

    // 验证补偿器类型是否有效
    cv::Ptr<cv::detail::ExposureCompensator> test_compensator;
    if (!createNewCompensatorInstance(test_compensator)) {
        last_error_ = "Failed to create exposure compensator with type: " + exp_type_;
        return false;
    }

    if (params.verbose_output) {
        std::cout << "Exposure estimator initialized with type: " << exp_type_
            << ", work scale: " << work_scale_ << std::endl;
    }

    return true;
}

bool ExposureEstimator::createNewCompensatorInstance(cv::Ptr<cv::detail::ExposureCompensator>& out_compensator) {
    if (exp_type_ == "gain") {
        out_compensator = cv::makePtr<cv::detail::GainCompensator>();
    }
    else if (exp_type_ == "gain_blocks") {
        out_compensator = cv::makePtr<cv::detail::BlocksGainCompensator>(exp_block_size_, exp_block_size_);
    }
    else if (exp_type_ == "channel") {
        out_compensator = cv::makePtr<cv::detail::ChannelsCompensator>(exp_nr_feeds_);
    }
    else if (exp_type_ == "channel_blocks") {
        out_compensator = cv::makePtr<cv::detail::BlocksChannelsCompensator>(
            exp_block_size_, exp_block_size_, exp_nr_feeds_);
    }
    else if (exp_type_ == "no") {
        out_compensator = cv::makePtr<cv::detail::NoExposureCompensator>();
    }
    else {
        last_error_ = "Unknown exposure compensator type: " + exp_type_;
        return false;
    }
    return (out_compensator != nullptr);
}

// 修改：图像预处理函数 - 使用 EXP_EST_SCALE 进行缩放
bool ExposureEstimator::preprocessImages(const std::vector<RTStitching::Image>& input_images,
    std::vector<cv::UMat>& processed_images) {
    processed_images.clear();
    processed_images.reserve(input_images.size());

    for (const auto& img : input_images) {
        if (img.data.empty()) {
            last_error_ = "Empty input image";
            return false;
        }

        cv::UMat processed;

        // 关键修改：根据 work_scale_（EXP_EST_SCALE）进行缩放
        if (work_scale_ > 0 && std::abs(work_scale_ - 1.0) > 1e-3) {
            // 计算缩放后的尺寸
            cv::Size new_size(
                static_cast<int>(img.data.cols * work_scale_),
                static_cast<int>(img.data.rows * work_scale_)
            );

            // 缩放图像到曝光估计尺度
            cv::resize(img.data, processed, new_size, 0, 0, cv::INTER_LINEAR_EXACT);

            if (config_params_.verbose_output) {
                std::cout << "Resized image from " << img.data.size()
                    << " to " << processed.size()
                    << " using EXP_EST_SCALE: " << work_scale_ << std::endl;
            }
        }
        else {
            // 不缩放，直接复制
            img.data.copyTo(processed);
        }
        processed_images.push_back(processed);
    }

    return true;
}

// 修改：Warp图像变换函数 - 不再进行缩放，使用预处理后的图像
// 修改 warpImages 函数
bool ExposureEstimator::warpImages(const std::vector<cv::UMat>& images,
    std::vector<cv::UMat>& warped_images) {
    if (images.size() != config_params_.camera_params.size()) {
        last_error_ = "Image count doesn't match camera parameters count";
        return false;
    }

    warped_images.clear();
    warped_images.resize(images.size());

    try {
        // 计算 warp 尺度
        double warp_scale = 1.0;
        if (!config_params_.camera_params.empty() && !images.empty()) {
            cv::Size image_size = images[0].size();
            double image_area = image_size.area();
            warp_scale = std::min(1.0, std::sqrt(1.0 * 1e6 / image_area));
        }

        // 创建 warper
        auto warper = cv::makePtr<cv::detail::SphericalWarper>(static_cast<float>(warp_scale));

        for (size_t i = 0; i < images.size(); ++i) {
            if (images[i].empty()) {
                last_error_ = "Empty input image for warping at index " + std::to_string(i);
                return false;
            }

            // 关键修复：确保相机参数正确转换
            cv::Mat K, R;

            // 转换内参矩阵 K 到 CV_32F
            if (config_params_.camera_params[i].K.empty()) {
                // 如果没有内参，创建默认内参
                K = (cv::Mat_<float>(3, 3) << 1000, 0, images[i].cols / 2,
                    0, 1000, images[i].rows / 2,
                    0, 0, 1);
            }
            else {
                config_params_.camera_params[i].K.convertTo(K, CV_32F);

                // 应用工作尺度到内参矩阵
                float swa = static_cast<float>(work_scale_);
                K.at<float>(0, 0) *= swa;
                K.at<float>(0, 2) *= swa;
                K.at<float>(1, 1) *= swa;
                K.at<float>(1, 2) *= swa;
            }

            // 关键修复：确保旋转矩阵 R 是 3x3 CV_32F
            if (config_params_.camera_params[i].R.empty() ||
                config_params_.camera_params[i].R.rows != 3 ||
                config_params_.camera_params[i].R.cols != 3) {
                // 如果没有旋转矩阵或尺寸不对，创建单位矩阵
                R = cv::Mat::eye(3, 3, CV_32F);
            }
            else {
                // 确保旋转矩阵是 3x3 并转换为 CV_32F
                config_params_.camera_params[i].R.convertTo(R, CV_32F);
                if (R.rows != 3 || R.cols != 3) {
                    // 如果转换后尺寸不对，重置为单位矩阵
                    R = cv::Mat::eye(3, 3, CV_32F);
                }
            }

            // 验证参数
            if (K.rows != 3 || K.cols != 3 || K.type() != CV_32F) {
                last_error_ = "Invalid intrinsic matrix K for image " + std::to_string(i);
                return false;
            }

            if (R.rows != 3 || R.cols != 3 || R.type() != CV_32F) {
                last_error_ = "Invalid rotation matrix R for image " + std::to_string(i);
                return false;
            }

            // 应用 warp 变换
            cv::Point corner;
            try {
                corner = warper->warp(images[i], K, R,
                    cv::INTER_LINEAR, cv::BORDER_REFLECT,
                    warped_images[i]);
            }
            catch (const cv::Exception& e) {
                last_error_ = std::string("Warping failed for image ") + std::to_string(i) + ": " + e.what();
                return false;
            }

            // 更新角点信息
            if (i < camera_stitch_params_[index_].corners.size()) {
                camera_stitch_params_[index_].corners[i] = corner;
            }
            else {
                camera_stitch_params_[index_].corners.push_back(corner);
            }

            if (config_params_.verbose_output) {
                std::cout << "Warped image " << i << " from " << images[i].size()
                    << " to " << warped_images[i].size()
                    << ", corner: " << corner << std::endl;
            }
        }

        return true;
    }
    catch (const cv::Exception& e) {
        last_error_ = std::string("Warping failed: ") + e.what();
        return false;
    }
}

bool ExposureEstimator::estimateExposure(
    const std::vector<cv::UMat>& warped_images,
    RTStitching::ExposureCompensatorWrapper& result) {

    auto start = std::chrono::high_resolution_clock::now();

    try {
        // 创建新的补偿器实例
        cv::Ptr<cv::detail::ExposureCompensator> new_compensator;
        if (!createNewCompensatorInstance(new_compensator)) {
            last_error_ = "Failed to create new exposure compensator instance";
            return false;
        }

        // 用 warp 后的图像训练补偿器
        new_compensator->feed(camera_stitch_params_[index_].corners, warped_images, masks_);

        // 存储补偿器数据
        result.compensator = new_compensator;
        result.corners = corners_;
        result.masks = masks_;
        result.is_estimated = true;
        result.frame_id = getCurrentFrameId();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        last_processing_time_ = duration.count() / 1000.0;

        std::cout << "Created new exposure compensator for frame " << result.frame_id
            << ", object address: " << new_compensator.get() << std::endl;

        return true;
    }
    catch (const cv::Exception& e) {
        last_error_ = std::string("OpenCV exception: ") + e.what();
        return false;
    }
    catch (const std::exception& e) {
        last_error_ = std::string("Standard exception: ") + e.what();
        return false;
    }
}

// 辅助函数：获取当前帧ID
uint64_t ExposureEstimator::getCurrentFrameId() {
    static std::atomic<uint64_t> frame_counter{ 0 };
    return frame_counter.fetch_add(1, std::memory_order_relaxed);
}