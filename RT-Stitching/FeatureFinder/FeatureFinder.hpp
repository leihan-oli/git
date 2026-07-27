#ifndef FEATURE_FINDER_HPP
#define FEATURE_FINDER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <future>
#include <memory>
#include "Types.hpp"
#include "../CircularBuffer/CircularBuffer.hpp"
#include "../CircularBufferSync/CircularBufferSync.hpp"
using cv::detail::ImageFeatures;

class ThreadPool {
public:
    explicit ThreadPool(size_t num_threads);
    ~ThreadPool();

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<typename std::result_of<F(Args...)>::type>;

    void shutdown();  // Manual thread pool shutdown

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};

class FeatureFinder {
public:
    // Thread configuration structure
    struct Config {
        bool auto_start = false;
        int num_threads = 4;
    };
    FeatureFinder(
        RTStitching::ConfigParams config_params,
        CircularBufferSync<RTStitching::Image>& input_buffer,
        CircularBuffer<std::vector<ImageFeatures>>& output_buffer,
        bool show_window = true,
        const std::string& module_name = "");
    ~FeatureFinder();

    void setNumThreads(int num_threads);  // Reinitialize thread pool if needed
    void setFeaturesType(const std::string& features_type);
    double getWorkScale() const { return  work_scale_; }

    std::vector<ImageFeatures> computeImageFeatures(const std::vector<cv::Mat>& full_imgs);

    void initializeThreadPool();  // Manual thread pool initialization
    void shutdownThreadPool();    // Manual thread pool shutdown

    // Thread control interface
    bool start();
    void stop();
    void pause();
    void resume();
    bool isRunning() const { return is_running_.load(std::memory_order_acquire); }
    bool isPaused() const { return is_paused_.load(std::memory_order_acquire); }


    // Threaded processing interface for feature extraction
    bool processFeatures(const std::vector<cv::Mat>& images);

    // Result access interface
    const std::vector<ImageFeatures>& getComputedFeatures() const { return computed_features_; }
    bool isProcessingComplete() const { return processing_complete_; }

    // Visualization functions
    static void drawKeypoints(const cv::Mat& img, const ImageFeatures& features,
        cv::Mat& out_img, int point_size = 2,
        const cv::Scalar& color = cv::Scalar(0, 255, 0));

    static void drawKeypointsComparison(const cv::Mat& img1, const ImageFeatures& features1,
        const cv::Mat& img2, const ImageFeatures& features2,
        cv::Mat& out_img, int point_size = 2);
private:
    // Thread execution implementation
    void runImpl();

    // Work waiting function
    bool waitForWork();

    // Thread control member variables
    std::thread worker_thread_;
    mutable std::mutex control_mutex_;
    std::condition_variable condition_;
    std::atomic<bool> is_running_{ false };
    std::atomic<bool> is_paused_{ false };
    std::atomic<bool> stop_requested_{ false };

    // Data processing member variables
    RTStitching::ConfigParams config_params_;
    // Input/output buffers
    CircularBufferSync<RTStitching::Image>& input_buffer_;
    CircularBuffer<std::vector<ImageFeatures>>& output_buffer_;
    Config config_;
    std::vector<cv::Mat> current_images_;
    std::vector<ImageFeatures> computed_features_;
    bool processing_complete_ = false;
    std::mutex data_mutex_;

    cv::Ptr<cv::Feature2D> feature_finder_;

    int num_threads_;
    double work_scale_;
    bool work_scale_set_;

    std::unique_ptr<ThreadPool> thread_pool_;
    bool thread_pool_initialized_ = false;
    bool show_window_;
    std::string module_name_;
    void extractFeaturesFromWorkImage(const cv::Mat& work_img, int img_idx, ImageFeatures& features);
};

#endif