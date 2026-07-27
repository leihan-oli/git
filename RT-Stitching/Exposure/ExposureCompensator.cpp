#include "ExposureCompensator.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <chrono>
#include <iostream>

using namespace cv;

ExposureCompensator::ExposureCompensator()
    : last_processing_time_(0.0), is_running_(false), is_paused_(false),
    stop_requested_(false), camera_index_(-1),
    input_buffer_(nullptr), exposure_data_buffer_(nullptr), output_buffer_(nullptr),
    camera_params_(nullptr), stitch_params_(nullptr) {
}

ExposureCompensator::~ExposureCompensator() {
    stop();
}

// 线程管理实现保持不变
bool ExposureCompensator::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_running_) return false;

    is_running_.store(true);
    is_paused_.store(false);
    stop_requested_.store(false);

    worker_thread_ = std::thread(&ExposureCompensator::runImpl, this);
    std::cout << "Exposure compensator thread started for camera " << camera_index_ << std::endl;
    return true;
}

void ExposureCompensator::stop() {
    if (!is_running_.load()) return;

    stop_requested_.store(true);
    condition_.notify_all();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    is_running_.store(false);
    is_paused_.store(false);
    std::cout << "Exposure compensator thread stopped for camera " << camera_index_ << std::endl;
}

void ExposureCompensator::pause() {
    is_paused_.store(true);
    std::cout << "Exposure compensator thread paused for camera " << camera_index_ << std::endl;
}

void ExposureCompensator::resume() {
    if (is_paused_.load()) {
        is_paused_.store(false);
        condition_.notify_one();
        std::cout << "Exposure compensator thread resumed for camera " << camera_index_ << std::endl;
    }
}

bool ExposureCompensator::isRunning() const {
    return is_running_.load(std::memory_order_acquire);
}

bool ExposureCompensator::isPaused() const {
    return is_paused_.load(std::memory_order_acquire);
}

void ExposureCompensator::runImpl() {
    std::cout << "Exposure compensator thread main loop started for camera " << camera_index_ << std::endl;

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

        // 更新曝光补偿数据
        updateExposureData();

        // 处理逻辑 - 直接从环形缓冲区获取图像，不进行缩放处理
        if (input_buffer_ && !input_buffer_->empty()) {
            try {
                // 获取当前相机的图像数据
                std::vector<RTStitching::Image> input_images;
                if (input_buffer_->try_pop_front(input_images)) {
                    // 检查是否有有效的曝光数据
                    {
                        std::lock_guard<std::mutex> lock(exposure_data_mutex_);
                        if (!current_exposure_data_.is_estimated) {
                            // 没有曝光补偿数据，直接输出原图像到同步环形缓冲区
                            std::cout << "No exposure compensation data available for camera "
                                << camera_index_ << ", outputting original images" << std::endl;
                            if (output_buffer_ && camera_index_ >= 0) {
                                // 关键修改：使用同步环形缓冲区写入补偿后的图像
                                output_buffer_->push_back(static_cast<size_t>(camera_index_), input_images);
                            }
                            continue;
                        }
                    }

                    // 执行曝光补偿 - 直接使用输入图像，不进行缩放
                    std::vector<RTStitching::Image> output_images;
                    if (applyExposureCompensation(input_images, output_images)) {
                        // 将结果放入同步环形缓冲区 - 关键修改
                        if (output_buffer_ && camera_index_ >= 0) {
                            if (output_buffer_->push_back(static_cast<size_t>(camera_index_), output_images)) {
                                std::cout << "Exposure compensation completed for camera "
                                    << camera_index_ << ", frame written to sync buffer" << std::endl;
                            }
                            else {
                                std::cerr << "Failed to write to sync buffer for camera " << camera_index_ << std::endl;
                            }
                        }
                    }
                    else {
                        std::cerr << "Exposure compensation failed for camera "
                            << camera_index_ << ": " << last_error_ << std::endl;
                        // 补偿失败时输出原图像到同步环形缓冲区
                        if (output_buffer_ && camera_index_ >= 0) {
                            output_buffer_->push_back(static_cast<size_t>(camera_index_), input_images);
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

    std::cout << "Exposure compensator thread main loop ended for camera " << camera_index_ << std::endl;
}

// 更新曝光补偿数据 - 从环形缓冲区获取最新的补偿器
void ExposureCompensator::updateExposureData() {
    if (exposure_data_buffer_ && !exposure_data_buffer_->empty()) {
        RTStitching::ExposureCompensatorWrapper new_data;
        if (exposure_data_buffer_->try_pop_front(new_data)) {
            std::lock_guard<std::mutex> lock(exposure_data_mutex_);
            current_exposure_data_ = new_data;
            std::cout << "Updated exposure compensation data for camera " << camera_index_
                << ", frame ID: " << new_data.frame_id << std::endl;

            // 更新全局掩码信息
            if (stitch_params_ && !new_data.masks.empty()) {
                updateGlobalMasks(new_data.masks);
            }
        }
    }
}

// 执行曝光补偿 - 直接使用输入图像，不进行缩放处理
bool ExposureCompensator::applyExposureCompensation(
    const std::vector<RTStitching::Image>& input_images,
    std::vector<RTStitching::Image>& output_images) {

    auto start = std::chrono::high_resolution_clock::now();

    try {
        // 获取当前曝光补偿器数据
        cv::Ptr<cv::detail::ExposureCompensator> compensator;
        std::vector<cv::Point> comp_corners;
        std::vector<cv::UMat> comp_masks;

        {
            std::lock_guard<std::mutex> lock(exposure_data_mutex_);
            if (!current_exposure_data_.is_estimated) {
                last_error_ = "No exposure compensation data available";
                return false;
            }
            compensator = current_exposure_data_.compensator;
            comp_corners = current_exposure_data_.corners;
            comp_masks = current_exposure_data_.masks;
        }

        if (!compensator) {
            last_error_ = "Exposure compensator is null";
            return false;
        }

        if (input_images.empty()) {
            last_error_ = "No input images to compensate";
            return false;
        }

        // 准备输出图像
        output_images.clear();
        output_images.reserve(input_images.size());

        // 对每个输入图像应用曝光补偿 - 直接使用输入图像，不进行缩放
        for (size_t i = 0; i < input_images.size(); ++i) {
            // 直接将Mat转换为UMat用于曝光补偿
            cv::UMat processed_image;
            input_images[i].data.copyTo(processed_image);  // 直接复制，不缩放

            // 应用曝光补偿
            // 关键：使用正确的角点和掩码索引
            int comp_index = (camera_index_ >= 0 && camera_index_ < static_cast<int>(comp_corners.size()))
                ? camera_index_ : static_cast<int>(i);

            if (comp_index < static_cast<int>(comp_corners.size()) &&
                comp_index < static_cast<int>(comp_masks.size())) {

                // 关键改进：直接使用输入图像进行曝光补偿
                compensator->apply(comp_index, comp_corners[comp_index], processed_image, comp_masks[comp_index]);
            }
            else {
                std::cerr << "Warning: Compensation index out of range for camera " << camera_index_ << std::endl;
                // 如果索引超出范围，使用第一个补偿器
                compensator->apply(0, comp_corners[0], processed_image, comp_masks[0]);
            }

            // 创建输出图像 - 将处理后的UMat转换回Mat
            RTStitching::Image output_image;
            processed_image.copyTo(output_image.data);  // 转换回Mat格式
            output_image.timestamp = input_images[i].timestamp;
            output_image.img_idx = input_images[i].img_idx;
            output_images.push_back(output_image);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        last_processing_time_ = duration.count() / 1000.0;

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

// 更新全局掩码信息
void ExposureCompensator::updateGlobalMasks(const std::vector<cv::UMat>& masks) {
    if (!stitch_params_) {
        return;
    }

    // 确保全局参数数组大小足够
    if (stitch_params_->size() < masks.size()) {
        stitch_params_->resize(masks.size());
    }

    // 更新对应相机的掩码
    for (size_t i = 0; i < masks.size() && i < stitch_params_->size(); ++i) {
        // 将UMat转换为Mat并存储到全局参数中
        cv::Mat mask_mat;
        masks[i].copyTo(mask_mat);
        (*stitch_params_)[i].masks = mask_mat;

        // 输出调试信息
        if (i == static_cast<size_t>(camera_index_)) {
            std::cout << "Updated global mask for camera " << camera_index_
                << ", size: " << mask_mat.size() << std::endl;
        }
    }
}