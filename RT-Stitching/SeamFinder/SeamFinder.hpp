#ifndef SEAM_FINDER_HPP
#define SEAM_FINDER_HPP

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include "../Types/Types.hpp"
#include "../CircularBufferSync/CircularBufferSync.hpp"
#include "../Warper/Warper.hpp"
#include "GazeAwareGraphCutSeamFinder.hpp"
#include "ISaliencySource.hpp"

class SeamFinder {
public:
    SeamFinder(CircularBufferSync<RTStitching::Image>& input_buffer,
        RTStitching::ConfigParams& config_params,
        std::vector<RTStitching::CameraStitchParams>& camera_stitch_params,
        std::shared_ptr<std::shared_mutex> seam_mask_mutex,
        bool show_window = true,
        const std::string& module_name = "",
        ISaliencySource* saliency_src = nullptr);
    ~SeamFinder();

    // Thread management interface
    bool start();           // Start thread
    void stop();            // Stop thread
    void pause();           // Pause thread  
    void resume();          // Resume thread
    bool isRunning() const; // Check if thread is running
    bool isPaused() const;  // Check if thread is paused

    // Initialize seam finder with ConfigParams
    bool initialize(const RTStitching::ConfigParams& params);


    // Get current camera stitch parameters (thread-safe)
    std::vector<RTStitching::CameraStitchParams> getCameraStitchParams() const {
        std::lock_guard<std::mutex> lock(stitch_params_mutex_);
        return camera_stitch_params_;
    }

    // Get last error message
    std::string getLastError() const { return last_error_; }

    // Get last processing time
    double getLastProcessingTime() const { return last_processing_time_; }
    // [新增] Windows 专用：主线程取走最新接缝对比调试图来 imshow
    bool getLatestSeamDebugFrame(cv::Mat & out);
    

private:
    cv::Ptr<cv::detail::SeamFinder> seam_finder;
    gaze_seam::GazeAwareGraphCutSeamFinder* gaze_finder_ = nullptr;
    cv::Ptr<cv::WarperCreator> warper_creator_;
    cv::Ptr<cv::detail::RotationWarper> warper_;
    float mid_focal_;       // 用于记录前一次mid_focal_，构造时设为-1
    std::string last_error_;
    double last_processing_time_;

    // Seam finding parameters
    std::string seam_find_type_;
    std::string warper_type_;
    bool try_cuda_;
    double seam_scale_;  // 添加接缝查找阶段的缩放比例参数

    // Camera parameters
    RTStitching::ConfigParams& config_params_;
    mutable std::mutex stitch_params_mutex_;  // For thread-safe access to camera_stitch_params_
    std::vector<RTStitching::CameraStitchParams>& camera_stitch_params_;

    // Thread management members
    std::atomic<bool> is_running_;      // Thread running state
    std::atomic<bool> is_paused_;       // Thread paused state
    std::atomic<bool> stop_requested_;  // Stop request flag
    std::thread worker_thread_;         // Worker thread
    mutable std::mutex mutex_;          // Mutex for condition variable
    std::condition_variable condition_; // Condition variable
    std::shared_ptr<std::shared_mutex> seam_mask_mutex_;    // mutex to protect seam mask
    bool show_window_;

    // [新增] 接缝对比调试图缓存（供主线程显示）
    cv::Mat seam_debug_frame_;
    mutable std::mutex seam_debug_mutex_;
    std::atomic<bool> has_seam_debug_{ false };



    // Input buffer
    CircularBufferSync<RTStitching::Image>& input_buffer_;

    // Thread execution function
    void runImpl();  // Thread main loop implementation

    // Set seam finder type
    bool setSeamFinderType(const std::string& seam_find_type, bool try_cuda = false);

    // 新增：按接缝查找尺度缩放图像并调整相机内参
    bool scaleImages(
        const std::vector<RTStitching::Image>& input_images,
        std::vector<cv::UMat>& scaled_images,
        std::vector<cv::Mat>& scaled_K
    );

    // Execute image warping
    bool warpImages(
        const std::vector<RTStitching::Image>& images,
        std::vector<cv::UMat>& warped_images,
        std::vector<cv::Point>& corners,
        std::vector<cv::UMat>& masks
    );

    // Execute seam finding
    bool findSeams(
        const std::vector<cv::UMat>& warped_images,
        const std::vector<cv::Point>& corners,
        std::vector<cv::UMat>& masks
    );

    // Convert RTStitching::Image to cv::UMat
    cv::UMat imageToUMat(const RTStitching::Image& image);

    // Visualize results (optional, for debugging)
    void visualizeResults(
        const std::vector<cv::UMat>& images,
        const std::vector<cv::UMat>& masks,
        const std::vector<cv::Point>& corners,
        const std::string& method_name
    );
    ISaliencySource* saliency_src_;   // 外部注入的注视数据读取器

    // === U²-Net 闭环：导出 warped 拼接画布供 Python 检测 ===
    uint64_t saliency_in_seq_ = 0;     // 写出序列号（递增）
    int      saliency_export_skip_ = 0; // 降频计数器

    // 把两路 warped 图贴成 canvas 合成图，原子写到 sal_u2net_dir/canvas_in.png
    void exportCanvasForSaliency(
        const std::vector<cv::UMat>& warped_images,
        const std::vector<cv::Point>& corners,
        int min_x, int min_y, int canvas_w, int canvas_h);


    // 注视感知后处理: 在 findSeams 之后修改 masks, 将接缝避让注视区域//已弃用
    /*void applyGazeAvoidance(
        const std::vector<cv::UMat>& warped_images,
        const std::vector<cv::Point>& corners,
        std::vector<cv::UMat>& masks
    );

	void visualizeGazeComparison(    //眼动仪注视比较可视化
        const std::vector<cv::UMat>& warped_images,
        const std::vector<cv::Point>& corners,
        const std::vector<cv::UMat>& masks_original,
        const std::vector<cv::UMat>& masks_gaze
    );
    */
    // 注视感知图割效果对比：同一组 warped 输入, 分别用 alpha=0 和 alpha>0 跑图割,
    // 把两套 seam 轮廓叠到同一张画布上
    void visualizeAlphaComparison(
        const std::vector<cv::UMat>& warped_images,
        const std::vector<cv::Point>& corners,
        const std::vector<cv::UMat>& masks_baseline,   // alpha=0 结果
        const std::vector<cv::UMat>& masks_gaze);      // alpha>0 结果


};
#endif // SEAM_FINDER_HPP