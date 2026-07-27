#ifndef FEATURE_TO_WARP_HPP
#define FEATURE_TO_WARP_HPP

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "Types.hpp"
#include "../CircularBuffer/CircularBuffer.hpp"
#include "../Warper/Warper.hpp"

// 宏定义窗口尺寸
#define DISPLAY_WINDOW_WIDTH 1280
#define DISPLAY_WINDOW_HEIGHT 720
#define DISPLAY_ROWS 3
#define DISPLAY_COLS 2

// 多尺度参数存储结构
struct MultiScaleResults {
    std::vector<RTStitching::CameraParams> camera_params_homo_est;     // 单应性估计尺度
    std::vector<RTStitching::CameraParams> camera_params_seam_find;    // 接缝查找尺度  
    std::vector<RTStitching::CameraParams> camera_params_blend;        // 融合尺度
    std::vector<RTStitching::CameraStitchParams> stitch_params_seam_find;  // 接缝查找尺度
    std::vector<RTStitching::CameraStitchParams> stitch_params_blend;      // 融合尺度

    MultiScaleResults() = default;

    void resize(size_t camera_count) {
        camera_params_homo_est.resize(camera_count);
        camera_params_seam_find.resize(camera_count);
        camera_params_blend.resize(camera_count);
        stitch_params_seam_find.resize(camera_count);
        stitch_params_blend.resize(camera_count);
    }

    void clear() {
        camera_params_homo_est.clear();
        camera_params_seam_find.clear();
        camera_params_blend.clear();
        stitch_params_seam_find.clear();
        stitch_params_blend.clear();
    }
};

class CameraParamEst {
public:
    CameraParamEst(
        RTStitching::ConfigParams& config_params,
        std::vector<RTStitching::CameraStitchParams>& stitch_params,
        CircularBuffer<std::vector<cv::detail::ImageFeatures>>& input_buffer,
        bool show_window = false   // [新增] MODULE_CAMERA_PARAM_EST_DEBUG 传入，控制掩膜总览调试图输出
    );
    ~CameraParamEst();

    // Thread management interface
    bool start();
    void stop();
    void pause();
    void resume();
    bool isRunning() const;
    bool isPaused() const;

    // Get multi-scale results
    const MultiScaleResults& getMultiScaleResults() const { return multi_scale_results_; }

    // Get last error message
    std::string getLastError() const { return last_error_; }

    // Get last processing time
    double getLastProcessingTime() const { return last_processing_time_; }

    // 可视化拼接参数
    static void visualize_stitching_params(
        const std::vector<RTStitching::CameraStitchParams>& stitching_params);





private:
    // 静态成员变量声明
    static cv::Mat global_display_image;
    static bool display_initialized;

    // Core processing components
    cv::Ptr<cv::detail::FeaturesMatcher> feature_matcher_;
    cv::Ptr<cv::detail::Estimator> motion_estimator_;
    cv::Ptr<cv::detail::BundleAdjusterBase> bundle_adjuster_;

    std::string last_error_;
    double last_processing_time_;
    bool show_window_;   // [新增] 调试可视化开关

    // Processing parameters
    std::string matcher_type_;
    std::string estimator_type_;
    std::string adjuster_type_;
    std::string warper_type_;
    std::string wave_correction_;
    bool try_cuda_;
    float match_conf_;
    float conf_thresh_;
    int range_width_;

    


    // Intermediate processing results
    std::vector<cv::detail::MatchesInfo> pairwise_matches_;

    std::vector<cv::detail::CameraParams> camera_params_;
    std::vector<RTStitching::CameraParams> RT_camera_params_;
    std::vector<RTStitching::CameraParams> calculate_RT_camera_params_;
    double warped_image_scale_;

    // Multi-scale results storage
    MultiScaleResults multi_scale_results_;

    // Thread management members
    std::atomic<bool> is_running_;
    std::atomic<bool> is_paused_;
    std::atomic<bool> stop_requested_;
    std::thread worker_thread_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;

    // Input buffer and output parameters
    CircularBuffer<std::vector<cv::detail::ImageFeatures>>& input_buffer_;

    RTStitching::ConfigParams& config_params_;

    std::vector<RTStitching::CameraStitchParams>& stitch_params_;

    // Thread execution function
    void runImpl();

    // ========== 核心处理函数 - 每个功能独立 ==========

    // 1. 特征匹配功能
    bool FeatureMatching(const std::vector<cv::detail::ImageFeatures>& features);

    // 2. 相机参数估计功能
    bool CameraEstimation(const std::vector<cv::detail::ImageFeatures>& features);

    // 3. 束调整功能
    bool BundleAdjustment(const std::vector<cv::detail::ImageFeatures>& features);

    // 4. 波形校正功能
    bool WaveCorrection();

    // ========== 初始化函数 ==========

    // 8. 特征匹配器初始化
    bool initializeFeatureMatcher(const std::string& matcher_type, bool try_cuda);

    // 9. 运动估计器初始化
    bool initializeMotionEstimator(const std::string& estimator_type);

    // 10. 束调整器初始化
    bool initializeBundleAdjuster(const std::string& adjuster_type, const std::string& refine_mask = "xxxxx");

    // ========== 工具函数 ==========
    // 13. 获取波形校正类型
    cv::detail::WaveCorrectKind getWaveCorrectKind(const std::string& wave_correction);

    void update_camera_params(void);   //主要实现camera_params滤波、输出决策逻辑

    // 18. 验证输入特征
    bool validateInputFeatures(const std::vector<cv::detail::ImageFeatures>& features);

};
#endif // FEATURE_TO_WARP_HPP