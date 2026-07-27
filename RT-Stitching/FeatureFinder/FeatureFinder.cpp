#include "FeatureFinder.hpp"
#include <iostream>
#include <chrono>
#include <spdlog/spdlog.h>
#include "../Utility/DebugDump.hpp"   // [新增] 调试图像统一写盘 /root/build/debug/
#define HAVE_OPENCV_XFEATURES2D
#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#endif

// ThreadPool implementation
ThreadPool::ThreadPool(size_t num_threads) : stop(false) {
    for (size_t i = 0; i < num_threads; ++i) {
        workers.emplace_back([this] {
            for (;;) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(this->queue_mutex);
                    this->condition.wait(lock, [this] {
                        return this->stop || !this->tasks.empty();
                        });
                    if (this->stop && this->tasks.empty())
                        return;
                    task = std::move(this->tasks.front());
                    this->tasks.pop();
                }
                task();
            }
            });
    }
}

void ThreadPool::shutdown() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
}

ThreadPool::~ThreadPool() {
    shutdown();
    for (std::thread& worker : workers)
        worker.join();
}

template<class F, class... Args>
auto ThreadPool::enqueue(F&& f, Args&&... args)
-> std::future<typename std::result_of<F(Args...)>::type> {
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        if (stop)
            throw std::runtime_error("enqueue on stopped ThreadPool");
        tasks.emplace([task]() { (*task)(); });
    }
    condition.notify_one();
    return res;
}
FeatureFinder::FeatureFinder(RTStitching::ConfigParams config_params,
    CircularBufferSync<RTStitching::Image>& input_buffer,
    CircularBuffer<std::vector<ImageFeatures>>& output_buffer,
    bool show_window,
    const std::string& module_name) :
    config_params_(config_params),
    input_buffer_(input_buffer),
    output_buffer_(output_buffer),
    show_window_(show_window),
    module_name_(module_name),
    num_threads_(4),
    work_scale_(1.0),
    work_scale_set_(false),
    thread_pool_(nullptr),
    thread_pool_initialized_(false)
{
    if (module_name_.empty())
        module_name_ = "FeatureFinder";
    setFeaturesType(config_params.features_type);
    // Decide whether to auto-start based on configuration
    if (config_.auto_start) {
        if (!start()) {
            throw std::runtime_error("FeatureFinder auto-start failed");
        }
    }
}
FeatureFinder::~FeatureFinder() {
    stop();  // Ensure thread stops
    shutdownThreadPool();  // Shutdown thread pool
}

// Work waiting function
bool FeatureFinder::waitForWork() {
    std::unique_lock<std::mutex> lock(control_mutex_);
    condition_.wait(lock, [this] {
        return stop_requested_.load() || !is_paused_.load();
        });
    return stop_requested_.load();
}

void FeatureFinder::runImpl() {
    spdlog::info("[FEATUREFINDER] FeatureFinder thread started");
    const std::chrono::milliseconds timeout(50);
    std::vector<RTStitching::Image> results;
    std::vector<bool> new_data_flags;

    // Main loop
    while (!stop_requested_.load(std::memory_order_acquire)) {
        // Wait for work condition (exit if stop requested)
        if (waitForWork()) {
            break;
        }

        // Clear previous results to avoid stale data
        results.clear();
        new_data_flags.clear();

        // Read synchronized data from all buffers
        const bool data_ready = input_buffer_.back(results, new_data_flags, timeout, config_params_.camera_count);
        if (!data_ready) {
            continue; // No complete data available yet, try again
        }

        // Validate buffer size matches expected camera count
        if (results.size() != config_params_.camera_count) {
            spdlog::error("[FEATUREFINDER] Mismatch between buffer count ({}) and camera count ({})", results.size(), config_params_.camera_count);
            continue;
        }

        // Prepare images for processing with bounds checking
        std::vector<cv::Mat> images_to_process;
        images_to_process.reserve(config_params_.camera_count);
        bool valid_data = true;

        for (size_t cam_idx = 0; cam_idx < config_params_.camera_count; ++cam_idx) {
            // Check if buffer has valid data
            if (cam_idx >= results.size() || results[cam_idx].data.empty()) {
                spdlog::error("[FEATUREFINDER] Invalid data for camera index: {}", cam_idx);
                valid_data = false;
                break;
            }
            images_to_process.push_back(results[cam_idx].data);
        }

        if (!valid_data || images_to_process.empty()) {
            continue; // Skip processing with invalid data
        }

        // Process features and handle potential errors
        try {
            auto features = computeImageFeatures(images_to_process);

            // Push results to output buffer with error checking
            if (!output_buffer_.push_back(features)) {
                spdlog::error("[FEATUREFINDER] Output buffer is full, dropping features");
            }

            // Save to local storage with thread-safe access
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                computed_features_ = std::move(features);
                processing_complete_ = true;
            }

            spdlog::info("[FEATUREFINDER] Feature extraction completed for {} images", images_to_process.size());

            if (show_window_ && images_to_process.size() >= 2 && !computed_features_.empty()) {
                // Take first two images and their corresponding feature points for comparison (adjust indices as needed)
                const cv::Mat& img1 = images_to_process[0];
                const ImageFeatures& feat1 = computed_features_[0];
                const cv::Mat& img2 = images_to_process[1];
                const ImageFeatures& feat2 = computed_features_[1];

                cv::Mat comparison_img;
                // Call drawing function, set feature point size to 2 (adjust as needed)
                drawKeypointsComparison(img1, feat1, img2, feat2, comparison_img, 2);

                // [修改] Linux 上 imshow 与 Qt 抢占 GUI 资源，改为写盘到
                //   /root/build/debug/<module>_Keypoints_Comparison.jpg（Windows 仍 imshow）
                RTStitching::debugDump(module_name_ + "_Keypoints_Comparison", comparison_img);
            }
        }
        catch (const std::exception& e) {
            spdlog::error("[FEATUREFINDER] Error during feature extraction: {}", e.what());
            // Reset state after error
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                processing_complete_ = false;
            }
        }
    }

    spdlog::info("[FEATUREFINDER] FeatureFinder thread stopped");
}


// Thread start function
bool FeatureFinder::start() {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (is_running_.load()) {
        return false;
    }

    // Reset state
    is_running_.store(true);
    is_paused_.store(false);
    stop_requested_.store(false);

    // Start worker thread
    worker_thread_ = std::thread(&FeatureFinder::runImpl, this);

    return true;
}

// Thread stop function
void FeatureFinder::stop() {
    if (!is_running_.load()) {
        return;
    }

    // Send stop request
    stop_requested_.store(true);
    condition_.notify_all();

    // Wait for thread to exit
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    // Reset state
    is_running_.store(false);
    is_paused_.store(false);
}

// Thread pause function
void FeatureFinder::pause() {
    is_paused_.store(true);
}

// Thread resume function
void FeatureFinder::resume() {
    if (is_paused_.load()) {
        is_paused_.store(false);
        condition_.notify_one();
    }
}

// Feature processing start function
bool FeatureFinder::processFeatures(const std::vector<cv::Mat>& images) {
    if (!is_running_.load()) {
        spdlog::error("[FEATUREFINDER] FeatureFinder is not running");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_images_ = images;
        computed_features_.clear();
        processing_complete_ = false;
    }

    // Wake up worker thread
    if (is_paused_.load()) {
        resume();
    }
    else {
        condition_.notify_one();
    }

    return true;
}

void FeatureFinder::initializeThreadPool() {
    if (!thread_pool_initialized_) {
        spdlog::info("[FEATUREFINDER] Initializing thread pool with {} threads", num_threads_);
        thread_pool_ = std::make_unique<ThreadPool>(num_threads_);
        thread_pool_initialized_ = true;
    }
}

void FeatureFinder::shutdownThreadPool() {
    if (thread_pool_initialized_ && thread_pool_) {
        spdlog::info("[FEATUREFINDER] Shutting down thread pool");
        thread_pool_.reset();
        thread_pool_initialized_ = false;
    }
}

void FeatureFinder::setNumThreads(int num_threads) {
    if (num_threads != num_threads_) {
        num_threads_ = num_threads;
        if (thread_pool_initialized_) {
            shutdownThreadPool();
            initializeThreadPool();
        }
    }
}

void FeatureFinder::setFeaturesType(const std::string& features_type) {
    // Set feature detector and descriptor extractor based on type
    if (features_type == "orb") {
        feature_finder_ = cv::ORB::create();
    }
    else if (features_type == "sift") {
        feature_finder_ = cv::SIFT::create();
    }
    else if (features_type == "akaze") {
        feature_finder_ = cv::AKAZE::create();
    }
#ifdef HAVE_OPENCV_XFEATURES2D
    else if (features_type == "surf") {
        feature_finder_ = cv::xfeatures2d::SURF::create();
    }
#endif
    else {
        spdlog::error("[FEATUREFINDER] Unknown features type: {}, using ORB as default", features_type);
        feature_finder_ = cv::ORB::create();
    }
}

std::vector<ImageFeatures> FeatureFinder::computeImageFeatures(const std::vector<cv::Mat>& full_imgs) {
    // Check stop request
    if (stop_requested_.load()) {
        return std::vector<ImageFeatures>();
    }

    if (full_imgs.empty()) {
        return std::vector<ImageFeatures>();
    }

    if (!thread_pool_initialized_) {
        initializeThreadPool();
    }

    // Calculate work scale if not set, using StitchingParams from config
    if (!work_scale_set_) {
        //work_scale_ = std::min(1.0, std::sqrt(config_params_.scale_megapix[RTStitching::HOMO_EST_SCALE] * 1e6 / full_imgs[0].size().area()));
        work_scale_ = 1.0;
        work_scale_set_ = true;
        spdlog::info("[FEATUREFINDER] Homo scale: {} (based on homo_est_scale_megapix = {})", work_scale_, config_params_.scale_megapix[RTStitching::HOMO_EST_SCALE]);
    }

    std::vector<ImageFeatures> features(full_imgs.size());
    std::vector<std::future<void>> results;

    for (int i = 0; i < full_imgs.size(); ++i) {
        // Check stop request
        if (stop_requested_.load()) {
            break;
        }

        results.emplace_back(
            thread_pool_->enqueue([this, i, &full_imgs, &features]() {
                // Check stop request
                if (this->stop_requested_.load()) {
                    return;
                }

                cv::Mat work_img;
                if (this->work_scale_ != 1.0) {
                    cv::resize(full_imgs[i], work_img, cv::Size(),
                        this->work_scale_, this->work_scale_, cv::INTER_LINEAR_EXACT);
                }
                else {
                    work_img = full_imgs[i];
                }
                this->extractFeaturesFromWorkImage(work_img, i, features[i]);
                })
        );
    }

    for (auto& result : results) {
        // Check stop request
        if (stop_requested_.load()) {
            break;
        }
        result.get();
    }

    return features;
}

// Feature extraction implementation
void FeatureFinder::extractFeaturesFromWorkImage(const cv::Mat& work_img, int img_idx, ImageFeatures& features) {
    auto start = std::chrono::high_resolution_clock::now();

    cv::Mat gray;
    if (work_img.channels() > 1) {
        cv::cvtColor(work_img, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = work_img;
    }

    cv::detail::computeImageFeatures(feature_finder_, gray, features);

    features.img_idx = img_idx;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    spdlog::info("[FEATUREFINDER] Image {}: {} keypoints [{}ms]", img_idx, features.keypoints.size(), duration.count());
}

// Visualization function - Draw keypoints as simple dots
void FeatureFinder::drawKeypoints(const cv::Mat& img, const ImageFeatures& features,
    cv::Mat& out_img, int point_size, const cv::Scalar& color) {
    // Make a copy of the input image
    img.copyTo(out_img);

    // Draw keypoints as simple dots instead of rich keypoints
    for (const auto& kp : features.keypoints) {
        cv::Point center(cvRound(kp.pt.x), cvRound(kp.pt.y));
        cv::circle(out_img, center, point_size, color, -1); // Filled circle
    }

    // Add information text
    std::string info = "Keypoints: " + std::to_string(features.keypoints.size());
    cv::putText(out_img, info, cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
}

// Visualization function - Draw keypoints comparison for two images side by side
void FeatureFinder::drawKeypointsComparison(const cv::Mat& img1, const ImageFeatures& features1,
    const cv::Mat& img2, const ImageFeatures& features2,
    cv::Mat& out_img, int point_size) {
    // Create a combined image
    cv::Mat combined_img(std::max(img1.rows, img2.rows), img1.cols + img2.cols, img1.type());
    combined_img = cv::Scalar::all(0);

    // Copy images side by side
    cv::Mat left_roi = combined_img(cv::Rect(0, 0, img1.cols, img1.rows));
    cv::Mat right_roi = combined_img(cv::Rect(img1.cols, 0, img2.cols, img2.rows));
    img1.copyTo(left_roi);
    img2.copyTo(right_roi);

    // Draw keypoints on left image (green)
    for (const auto& kp : features1.keypoints) {
        cv::Point center(cvRound(kp.pt.x), cvRound(kp.pt.y));
        cv::circle(combined_img, center, point_size, cv::Scalar(0, 255, 0), -1);
    }

    // Draw keypoints on right image (red)
    for (const auto& kp : features2.keypoints) {
        cv::Point center(cvRound(kp.pt.x + img1.cols), cvRound(kp.pt.y));
        cv::circle(combined_img, center, point_size, cv::Scalar(0, 0, 255), -1);
    }

    // Add information text
    std::string left_info = "Left: " + std::to_string(features1.keypoints.size()) + " keypoints";
    std::string right_info = "Right: " + std::to_string(features2.keypoints.size()) + " keypoints";

    cv::putText(combined_img, left_info, cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    cv::putText(combined_img, right_info, cv::Point(img1.cols + 10, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

    // Draw separator line
    cv::line(combined_img, cv::Point(img1.cols, 0), cv::Point(img1.cols, combined_img.rows),
        cv::Scalar(255, 255, 255), 2);

    out_img = combined_img;
}