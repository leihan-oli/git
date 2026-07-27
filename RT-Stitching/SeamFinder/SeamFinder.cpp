#include "SeamFinder.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/warpers.hpp>
#include "../Utility/Utility.hpp"   // [新增] 视差补偿 warpSphericalD0
#include <chrono>
#include <thread>
#include <fstream>
#include <cstdio>
#include <spdlog/spdlog.h>  // Add spdlog header
#include "../Utility/PerfLog.hpp"   // [新增] 性能日志（CSV + 周期摘要）
#include "../Utility/ThreadAffinity.hpp" // [新增] 线程绑核（RK3588 大小核）
#include <filesystem>

#include "GazeDataReader.hpp"
#include "GazeAwareGraphCutSeamFinder.hpp"

#define MODULE_GAZE_DEBUG true

// [新增] 接缝重算限频（ms）：无有效显著性时 1Hz 兜底刷新；
//   显著性源(注视/U2-Net)报告有效数据时提升到 5Hz，把算力用在刀刃上。
#ifndef RT_SEAM_INTERVAL_IDLE_MS
#define RT_SEAM_INTERVAL_IDLE_MS   1000
#endif
#ifndef RT_SEAM_INTERVAL_ACTIVE_MS
#define RT_SEAM_INTERVAL_ACTIVE_MS 500
#endif
// 调试开关: 开启后每帧跑两次 findSeams (alpha=0 vs alpha>0), 
// 并把结果对比可视化. 生产环境请关闭 (图割耗时翻倍).
#define MODULE_SEAM_ALPHA_COMPARE 1
using namespace cv;

SeamFinder::SeamFinder(
    CircularBufferSync<RTStitching::Image>& input_buffer,
    RTStitching::ConfigParams& config_params,
    std::vector<RTStitching::CameraStitchParams>& camera_stitch_params,
    std::shared_ptr<std::shared_mutex> seam_mask_mutex,
    bool show_window,
    const std::string& module_name,
    ISaliencySource* saliency_src
)
    : last_processing_time_(0.0)
    , is_running_(false)
    , is_paused_(false)
    , stop_requested_(false)
    , try_cuda_(false)
    , seam_scale_(0.1)  // Initialize default scaling ratio
    , config_params_(config_params)
    , camera_stitch_params_(camera_stitch_params)
    , seam_mask_mutex_(seam_mask_mutex)
    , show_window_(show_window)
    , input_buffer_(input_buffer)
    , saliency_src_(saliency_src) {
}

SeamFinder::~SeamFinder() {
    stop(); // Ensure thread safe stop
}

// Thread management method implementations

bool SeamFinder::start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (is_running_) return false;

    // Reset thread state
    is_running_.store(true);
    is_paused_.store(false);
    stop_requested_.store(false);

    // Create worker thread
    worker_thread_ = std::thread(&SeamFinder::runImpl, this);

    spdlog::info("[SEAMFINDER] Seam finder thread started");
    return true;
}

void SeamFinder::stop() {
    if (!is_running_.load()) return;

    // Send stop request
    stop_requested_.store(true);
    // Wake up waiting thread
    condition_.notify_all();

    // Wait for thread to exit
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    // Reset thread state
    is_running_.store(false);
    is_paused_.store(false);

    spdlog::info("[SEAMFINDER] Seam finder thread stopped");
}

void SeamFinder::pause() {
    is_paused_.store(true);
    spdlog::info("[SEAMFINDER] Seam finder thread paused");
}

void SeamFinder::resume() {
    if (is_paused_.load()) {
        is_paused_.store(false);
        condition_.notify_one();
        spdlog::info("[SEAMFINDER] Seam finder thread resumed");
    }
}

bool SeamFinder::isRunning() const {
    return is_running_.load(std::memory_order_acquire);
}

bool SeamFinder::isPaused() const {
    return is_paused_.load(std::memory_order_acquire);
}

void SeamFinder::runImpl() {
    RTStitching::bindToBigCores("SeamFinder");   // [新增] 图割属重计算，绑大核(A76)
    spdlog::info("[SEAMFINDER] Seam finder thread main loop started");

    // [新增] 上一次接缝重算的时刻（限频用）
    auto last_seam_run = std::chrono::steady_clock::now() - std::chrono::hours(1);

    // Main loop condition: only check internal stop flag
    while (!stop_requested_.load()) {
        // Wait for work condition (check pause and stop)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] {
                return stop_requested_.load() || !is_paused_.load();
                });
        }

        // If stop request received, exit loop
        if (stop_requested_.load()) {
            break;
        }

        // [新增] 接缝重算限频：显著性源有有效数据(检测到注视点/显著图)时 5Hz，
        //   否则 1Hz 兜底。图割是本系统最重的单体计算(实测 120~180ms/次)，
        //   限频可稳定释放约 0.5 个大核给采集/变形/融合。
        {
            const bool saliency_active =
                (saliency_src_ != nullptr) && saliency_src_->hasSaliency();
            const int interval_ms = saliency_active ? RT_SEAM_INTERVAL_ACTIVE_MS
                                                    : RT_SEAM_INTERVAL_IDLE_MS;
            const auto now_tp = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    now_tp - last_seam_run).count() < interval_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            last_seam_run = now_tp;
        }

        // Processing logic: get synchronized image data from input buffer
        if (!camera_stitch_params_.empty()) {
            try {
                // Get synchronized images from input buffer
                std::vector<RTStitching::Image> images_vec;
                std::vector<bool> new_data_flags;
                if (input_buffer_.back(images_vec, new_data_flags)) {
                    // Extract the latest image from each camera
                    std::vector<RTStitching::Image> current_images;
                    for (const auto& img_vec : images_vec) {
                        if (!img_vec.data.empty()) {
                            current_images.push_back(img_vec); // Get latest image
                        }
                        else {
                            // If no image available, create a dummy one
                            RTStitching::Image dummy;
                            dummy.data = cv::Mat::zeros(600, 800, CV_8UC3); // Default size
                            current_images.push_back(dummy);
                        }
                    }

                    // Perform warping operation
                    std::vector<cv::UMat> warped_images;
                    std::vector<cv::Point> corners;
                    std::vector<cv::UMat> masks;

                    if (mid_focal_ != camera_stitch_params_[RTStitching::SEAM_FINDER_SCALE].mid_focal) {
                        mid_focal_ = camera_stitch_params_[RTStitching::SEAM_FINDER_SCALE].mid_focal;
                        warper_.release();
                        warper_ = warper_creator_->create(mid_focal_);
                    }

                    if (warpImages(current_images, warped_images, corners, masks)) {

                        // === 1. 计算 canvas 范围（两分支共用） ===
                        int min_x = corners[0].x, min_y = corners[0].y;
                        int max_x = corners[0].x, max_y = corners[0].y;
                        for (size_t i = 0; i < warped_images.size(); ++i) {
                            if (warped_images[i].empty()) continue;
                            min_x = std::min(min_x, corners[i].x);
                            min_y = std::min(min_y, corners[i].y);
                            max_x = std::max(max_x, corners[i].x + warped_images[i].cols);
                            max_y = std::max(max_y, corners[i].y + warped_images[i].rows);
                        }
                        const int canvas_w = max_x - min_x;
                        const int canvas_h = max_y - min_y;

                        // === 1b. 导出 warped 拼接画布供 Python U²-Net 检测 ===
                        //   仅 U²-Net 启用时执行。这是闭环同步的关键：Python 处理的是
                        //   本流水线当前 warp 出来的真实画布（覆盖两路相机），
                        //   而非独立视频文件。
                        if (config_params_.sal_use_u2net && canvas_w > 0 && canvas_h > 0) {
                            spdlog::error("[SEAMFINDER] U2NET export -> dir={}", config_params_.sal_u2net_dir);
                            exportCanvasForSaliency(warped_images, corners,
                                min_x, min_y, canvas_w, canvas_h);
                        }
                        else {
                            spdlog::error("[SEAMFINDER] U2NET export SKIPPED (use_u2net={})",
                                (int)config_params_.sal_use_u2net);
                        }


                        // === 2. 生成显著性图（两分支共用） ===
                        cv::Mat saliency;
                        if (saliency_src_ != nullptr && canvas_w > 0 && canvas_h > 0) {
                            const double sigma = static_cast<double>(canvas_w) *
                                static_cast<double>(config_params_.gaze_sigma_ratio);
                            saliency = saliency_src_->generateSaliencyMap(
                                canvas_w, canvas_h, sigma, /*max_age_ms=*/3000);
                        }

#if MODULE_SEAM_ALPHA_COMPARE
                        spdlog::error("[SEAMFINDER] === Alpha compare mode ACTIVE, show_window_={} ===",
                            show_window_);
                        // ========== 调试模式: 双路对比 ==========
                        // 2a. 备份 warpImages 产出的初始 masks（findSeams 会就地修改）
                        std::vector<cv::UMat> masks_baseline(masks.size());
                        std::vector<cv::UMat> masks_gaze(masks.size());
                        for (size_t i = 0; i < masks.size(); ++i) {
                            masks[i].copyTo(masks_baseline[i]);
                            masks[i].copyTo(masks_gaze[i]);
                        }

                        // 2b. 基线: alpha=0 图割
                        if (gaze_finder_ != nullptr) gaze_finder_->clearSaliencyMap();
                        const bool ok_base = findSeams(warped_images, corners, masks_baseline);

                        // 2c. 注视感知: alpha>0 图割
                        if (gaze_finder_ != nullptr && !saliency.empty()) {
                            gaze_finder_->setSaliencyMap(saliency, cv::Point(min_x, min_y),
                                config_params_.gaze_alpha);
                        }
                        else if (gaze_finder_ != nullptr) {
                            gaze_finder_->clearSaliencyMap();
                        }
                        const bool ok_gaze = findSeams(warped_images, corners, masks_gaze);

                        // 2d. 对比可视化
                        if (ok_base && ok_gaze && show_window_) {
                            visualizeAlphaComparison(warped_images, corners,
                                masks_baseline, masks_gaze);
                        }

                        // 2e. 下游使用 gaze-aware 的结果
                        if (ok_gaze) masks = std::move(masks_gaze);
                        const bool findSeams_ok = ok_gaze;
#else
                        // ========== 生产模式: 单路 ==========
                        if (gaze_finder_ != nullptr && !saliency.empty()) {
                            gaze_finder_->setSaliencyMap(saliency, cv::Point(min_x, min_y),
                                config_params_.gaze_alpha);
                        }
                        else if (gaze_finder_ != nullptr) {
                            gaze_finder_->clearSaliencyMap();
                        }
                        const bool findSeams_ok = findSeams(warped_images, corners, masks);
#endif

                        if (findSeams_ok) {
                            // ↓↓↓ 下面这段是 SEAM_FINDER_SCALE -> BLENDER_SCALE 的尺度转换，
                            //     完全沿用你现在的代码，不要动 ↓↓↓
                            cv::Mat dilated_mask, seam_mask;
                            cv::Mat mask_warped;
                            std::vector<cv::Mat> masks_warped;
                            std::lock_guard<std::mutex> lock(stitch_params_mutex_);
                            for (int i = 0; i < camera_stitch_params_[RTStitching::SEAM_FINDER_SCALE].masks.size(); i++)
                            {
                                spdlog::info("[SEAMFINDER] Here 1.");
                                spdlog::info("[SEAMFINDER] Here 1.1.");
                                cv::dilate(masks[i], dilated_mask, cv::Mat());
                                spdlog::info("[SEAMFINDER] Here 1.2.");
                                resize(dilated_mask, seam_mask, camera_stitch_params_[RTStitching::BLENDER_SCALE].masks[i].size(), 0, 0, INTER_LINEAR_EXACT);
                                spdlog::info("[SEAMFINDER] Here 1.3.");

                                camera_stitch_params_[RTStitching::BLENDER_SCALE].masks[i].copyTo(mask_warped);
                                spdlog::info("[SEAMFINDER] Here 1.4.");
                                mask_warped = seam_mask & mask_warped;
                                masks_warped.push_back(mask_warped);
                                spdlog::info("[SEAMFINDER] Here 1.5.");
                            }

                            // ============================================================
                            // [接缝空洞修复] 接缝在 SEAM 尺度(0.1MP)分割，却在 BLENDER 尺度
                            //   与有效区相与；FOV 边缘两尺度覆盖范围有细微差异，叠加注视感知
                            //   (gaze_alpha) 把缝推到某相机有效内容之外时，会出现"有效并集里
                            //   有一条缝既不属于左也不属于右"的空洞 -> 融合后为纯黑竖带。
                            //   这里在统一画布坐标下找出空洞，补给该处确有有效内容的相机，
                            //   保证整个有效并集被覆盖、不留黑缝（2/3/N 路通用）。
                            // ============================================================
                            {
                                auto& BL = camera_stitch_params_[RTStitching::BLENDER_SCALE];
                                const int n = static_cast<int>(masks_warped.size());
                                if (n > 0) {
                                    int min_x = BL.corners[0].x, min_y = BL.corners[0].y;
                                    int max_x = min_x, max_y = min_y;
                                    for (int i = 0; i < n; ++i) {
                                        const cv::Point& c = BL.corners[i];
                                        min_x = std::min(min_x, c.x);
                                        min_y = std::min(min_y, c.y);
                                        max_x = std::max(max_x, c.x + masks_warped[i].cols);
                                        max_y = std::max(max_y, c.y + masks_warped[i].rows);
                                    }
                                    const int CW = max_x - min_x, CH = max_y - min_y;
                                    if (CW > 0 && CH > 0) {
                                        cv::Mat union_valid    = cv::Mat::zeros(CH, CW, CV_8U);
                                        cv::Mat union_assigned = cv::Mat::zeros(CH, CW, CV_8U);
                                        std::vector<cv::Mat> valid_mat(n);
                                        for (int i = 0; i < n; ++i) {
                                            BL.masks[i].copyTo(valid_mat[i]);   // UMat -> Mat
                                            cv::Rect r(BL.corners[i].x - min_x, BL.corners[i].y - min_y,
                                                       masks_warped[i].cols, masks_warped[i].rows);
                                            cv::bitwise_or(union_valid(r), valid_mat[i], union_valid(r));
                                            cv::bitwise_or(union_assigned(r), masks_warped[i], union_assigned(r));
                                        }
                                        cv::Mat gap;
                                        cv::bitwise_not(union_assigned, gap);
                                        cv::bitwise_and(gap, union_valid, gap);   // 有效但未分配 = 空洞
                                        if (cv::countNonZero(gap) > 0) {
                                            for (int i = 0; i < n; ++i) {
                                                cv::Rect r(BL.corners[i].x - min_x, BL.corners[i].y - min_y,
                                                           masks_warped[i].cols, masks_warped[i].rows);
                                                cv::Mat gi;
                                                cv::bitwise_and(gap(r), valid_mat[i], gi);   // 该相机能覆盖的空洞
                                                cv::bitwise_or(masks_warped[i], gi, masks_warped[i]);
                                                cv::Mat notgi; cv::bitwise_not(gi, notgi);
                                                cv::Mat tmp; cv::bitwise_and(gap(r), notgi, tmp);
                                                tmp.copyTo(gap(r));   // 移除已补部分, 避免重复分配
                                            }
                                        }
                                    }
                                }
                            }

                            {
                                std::unique_lock<std::shared_mutex> lock(*seam_mask_mutex_);
                                for (int i = 0; i < camera_stitch_params_[RTStitching::SEAM_FINDER_SCALE].masks.size(); i++)
                                {
                                    masks_warped[i].copyTo(camera_stitch_params_[RTStitching::SEAM_FINDER_SCALE].masks[i]);
                                }
                                camera_stitch_params_[RTStitching::SEAM_FINDER_SCALE].sm_ver++;
                            }

                            config_params_.seamfinder_flag = true;

#if !MODULE_SEAM_ALPHA_COMPARE
                            // 非对比模式下保留原有的单路可视化；
                            // 对比模式下已经用 visualizeAlphaComparison 取代
                            if (show_window_) {
                                visualizeResults(warped_images, masks, corners, seam_find_type_);
                            }
#endif
                        }
                        else {
                            spdlog::error("[SEAMFINDER] Seam finding failed: {}", last_error_);
                        }
                    }
                    else {
                        spdlog::error("[SEAMFINDER] Image warping failed: {}", last_error_);
                    }

                    
                }
            }
            catch (const std::exception& e) {
                last_error_ = std::string("Processing exception: ") + e.what();
                spdlog::error("[SEAMFINDER] {}", last_error_);
            }
        }
        // Brief sleep to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    spdlog::info("[SEAMFINDER] Seam finder thread main loop ended");
}

// [新增] 主线程取走最新接缝对比调试图
bool SeamFinder::getLatestSeamDebugFrame(cv::Mat& out) {
    if (!has_seam_debug_.load(std::memory_order_acquire)) return false;
    std::lock_guard<std::mutex> lk(seam_debug_mutex_);
    if (seam_debug_frame_.empty()) return false;
    seam_debug_frame_.copyTo(out);
    has_seam_debug_.store(false, std::memory_order_release);
    return true;
}



// Seam finding method implementations

bool SeamFinder::initialize(const RTStitching::ConfigParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Save parameters
    seam_find_type_ = params.seam_find_type;
    warper_type_ = params.warper_type;
    try_cuda_ = params.try_cuda;

    // Set seam finder type
    if (!setSeamFinderType(seam_find_type_, try_cuda_)) {
        last_error_ = "Failed to initialize seam finder with type: " + seam_find_type_;
        return false;
    }

    // Only create warper_creator_
    warper_creator_ = WarperModule::initWarperCreator(warper_type_, try_cuda_);

    if (params.verbose_output) {
        spdlog::info("[SEAMFINDER] Seam finder initialized successfully with type: {}, warper: {}, try_cuda: {}, scale: {}MP",
            seam_find_type_, warper_type_, (try_cuda_ ? "true" : "false"), seam_scale_);
    }

    return true;
}

bool SeamFinder::setSeamFinderType(const std::string& seam_find_type, bool try_cuda) {
    seam_finder.release();
    gaze_finder_ = nullptr;

    if (seam_find_type == "gc_color") {
        auto p = cv::makePtr<gaze_seam::GazeAwareGraphCutSeamFinder>(
            gaze_seam::GazeAwareGraphCutSeamFinder::COST_COLOR,
            10000.f,    // terminal_cost
            1000.f);    // bad_region_penalty
        gaze_finder_ = p.get();
        seam_finder = p;
    }
    else if (seam_find_type == "gc_colorgrad") {
        auto p = cv::makePtr<gaze_seam::GazeAwareGraphCutSeamFinder>(
            gaze_seam::GazeAwareGraphCutSeamFinder::COST_COLOR_GRAD,
            10000.f, 1000.f);
        gaze_finder_ = p.get();
        seam_finder = p;
    }
    else if (seam_find_type == "voronoi" || seam_find_type == "no") {
        // 走原来的逻辑
        seam_finder = cv::makePtr<cv::detail::VoronoiSeamFinder>();
        // gaze_finder_ 保持 nullptr，注视避让自动失效
    }
    else if (seam_find_type == "dp_color") {
        seam_finder = cv::makePtr<cv::detail::DpSeamFinder>(
            cv::detail::DpSeamFinder::COLOR);
    }
    else if (seam_find_type == "dp_colorgrad") {
        seam_finder = cv::makePtr<cv::detail::DpSeamFinder>(
            cv::detail::DpSeamFinder::COLOR_GRAD);
    }
    else {
        last_error_ = "Unknown seam finder type: " + seam_find_type;
        return false;
    }

    seam_find_type_ = seam_find_type;
    try_cuda_ = try_cuda;
    return true;
}

// New: Scale images according to seam finding scale and adjust camera intrinsic parameters
bool SeamFinder::scaleImages(
    const std::vector<RTStitching::Image>& input_images,
    std::vector<cv::UMat>& scaled_images,
    std::vector<cv::Mat>& scaled_K
) {
    scaled_images.clear();
    scaled_K.clear();
    scaled_images.reserve(input_images.size());
    scaled_K.reserve(input_images.size());

    for (size_t i = 0; i < input_images.size(); ++i) {
        const auto& img = input_images[i];
        const auto& cam_params = config_params_.camera_params[i];

        // Handle empty image
        if (img.data.empty()) {
            last_error_ = "Empty input image at index: " + std::to_string(i);
            return false;
        }

        // Calculate original image pixel count
        double orig_megapix = (img.data.cols * img.data.rows) / 1e6;
        double scale_factor = 1.0;

        // Calculate scaling factor based on target megapixels (-1 means no scaling)
        if (seam_scale_ > 0 && orig_megapix > 0) {
            scale_factor = sqrt(seam_scale_ / orig_megapix);
        }

        // Scale image
        cv::UMat scaled_img;
        if (scale_factor != 1.0) {
            cv::resize(img.data, scaled_img, cv::Size(), scale_factor, scale_factor, cv::INTER_AREA);
        }
        else {
            img.data.copyTo(scaled_img);
        }

        // Adjust intrinsic parameter matrix (using the same scaling factor)
        cv::Mat K_scaled = cam_params.K.clone();
        K_scaled.at<double>(0, 0) *= scale_factor;  // fx
        K_scaled.at<double>(1, 1) *= scale_factor;  // fy
        K_scaled.at<double>(0, 2) *= scale_factor;  // cx
        K_scaled.at<double>(1, 2) *= scale_factor;  // cy

        scaled_images.push_back(scaled_img);
        scaled_K.push_back(K_scaled);
    }

    return true;
}

cv::UMat SeamFinder::imageToUMat(const RTStitching::Image& image) {
    cv::UMat result;
    if (!image.data.empty()) {
        image.data.copyTo(result);
    }
    return result;
}

bool SeamFinder::warpImages(
    const std::vector<RTStitching::Image>& images,
    std::vector<cv::UMat>& warped_images,
    std::vector<cv::Point>& corners,
    std::vector<cv::UMat>& masks) {

    if (images.size() != config_params_.camera_params.size()) {
        last_error_ = "Number of images doesn't match number of camera parameters";
        return false;
    }

    try {
        warped_images.clear();
        corners.clear();
        masks.clear();

        warped_images.reserve(images.size());
        corners.reserve(images.size());
        masks.reserve(images.size());

        // New: Scale images according to seam finding scale and get adjusted intrinsic parameters
        std::vector<cv::UMat> scaled_images;
        std::vector<cv::Mat> scaled_K;
        if (!scaleImages(images, scaled_images, scaled_K)) {
            last_error_ = "Failed to scale images: " + last_error_;
            return false;
        }

        // Replace the original loop content
        for (size_t i = 0; i < scaled_images.size(); ++i) {
            const auto& orig_uimg = scaled_images[i];  // Renamed to orig_uimg to clearly indicate original image
            const auto& K = scaled_K[i];

            cv::UMat uimg;  // Declare temporary variable to store valid image
            if (orig_uimg.empty()) {
                spdlog::error("[SEAMFINDER] Empty scaled image for camera {}", i);
                uimg = cv::UMat(600, 800, CV_8UC3, cv::Scalar(0, 0, 255)); // Assign to temporary variable
            }
            else {
                uimg = orig_uimg;  // Use original image if not empty
            }

            // The following code uses uimg for processing (unchanged)
            cv::Mat K_float;
            K.convertTo(K_float, CV_32F);

            cv::UMat warped_image;
            cv::UMat warped_mask;
            cv::Point warped_corner;

            cv::Mat R = config_params_.camera_params[i].R.clone();
            cv::Mat R_float;
            R.convertTo(R_float, CV_32F);

            // [新增] 近景视差补偿：与 Warper 热路径 / update_stitching_params 同一套
            //   （补偿后的）几何 —— 接缝必须在同一画布坐标系里计算，否则整体偏移。
            if (config_params_.parallax_d0 > 1e-9) {
                cv::Mat T64;
                if (!config_params_.camera_params[i].T.empty())
                    config_params_.camera_params[i].T.convertTo(T64, CV_64F);
                warped_corner = RTStitching::warpSphericalD0(
                    uimg, K_float, R_float, T64, mid_focal_, config_params_.parallax_d0,
                    cv::INTER_LINEAR, cv::BORDER_REFLECT, warped_image);
                cv::UMat mask_d0(uimg.size(), CV_8U, cv::Scalar(255));
                RTStitching::warpSphericalD0(
                    mask_d0, K_float, R_float, T64, mid_focal_, config_params_.parallax_d0,
                    cv::INTER_NEAREST, cv::BORDER_CONSTANT, warped_mask);
            }
            else {
                warped_corner = warper_->warp(uimg, K_float, R_float, cv::INTER_LINEAR, cv::BORDER_REFLECT, warped_image);

                cv::UMat mask(uimg.size(), CV_8U, cv::Scalar(255));
                warper_->warp(mask, K_float, R_float, cv::INTER_NEAREST, cv::BORDER_CONSTANT, warped_mask);
            }

            warped_images.push_back(warped_image);
            corners.push_back(warped_corner);
            masks.push_back(warped_mask);
        }

        return true;
    }
    catch (const cv::Exception& e) {
        last_error_ = std::string("OpenCV warping exception: ") + e.what();
        return false;
    }
    catch (const std::exception& e) {
        last_error_ = std::string("Standard warping exception: ") + e.what();
        return false;
    }
}

bool SeamFinder::findSeams(
    const std::vector<cv::UMat>& warped_images,
    const std::vector<cv::Point>& corners,
    std::vector<cv::UMat>& masks) {

    if (!seam_finder) {
        last_error_ = "Seam finder not initialized";
        return false;
    }

    if (warped_images.size() != masks.size()) {
        last_error_ = "Input vectors size mismatch";
        return false;
    }

    auto start = std::chrono::high_resolution_clock::now();

    try {
        // Convert images to float type for seam finding
        std::vector<cv::UMat> images_float(warped_images.size());
        for (size_t i = 0; i < warped_images.size(); ++i) {
            warped_images[i].convertTo(images_float[i], CV_32F);
        }

        //static int i = 0;
        // Execute seam finding (seam_finder may modify masks)
        //if (i <= 100) {
        //    i++;测试代码？不确定是不是屎山
        //}
        seam_finder->find(images_float, corners, masks);
        //std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        last_processing_time_ = duration.count() / 1000.0;

        // [新增] 性能日志：单次接缝搜索耗时
        RTStitching::perfRecord("SeamFinder", "seam_find_ms", (double)duration.count());

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
#define AF
#ifdef AF
void SeamFinder::visualizeResults(
    const std::vector<cv::UMat>& images,
    const std::vector<cv::UMat>& masks,
    const std::vector<cv::Point>& corners,
    const std::string& method_name) {

    if (images.empty() || corners.size() != images.size() || masks.size() != images.size()) {
        spdlog::error("[SEAMFINDER] Input size mismatch!");
        return;
    }

    // 1. Calculate coordinate offsets and canvas size (handle negative coordinates)
    int min_x = corners[0].x, min_y = corners[0].y;
    int max_x = corners[0].x, max_y = corners[0].y;
    for (size_t i = 0; i < images.size(); ++i) {
        if (images[i].empty()) {
            spdlog::error("[SEAMFINDER] Empty image at index {}", i);
            continue;
        }
        int x = corners[i].x;
        int y = corners[i].y;
        int w = images[i].cols;
        int h = images[i].rows;
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x + w);
        max_y = std::max(max_y, y + h);
    }

    int canvas_width = max_x - min_x;
    int canvas_height = max_y - min_y;
    if (canvas_width <= 0 || canvas_height <= 0) {
        spdlog::error("[SEAMFINDER] Invalid canvas size!");
        return;
    }

    // 2. Create black background canvas
    cv::Mat combined_canvas(canvas_height, canvas_width, CV_8UC3, cv::Scalar(0, 0, 0));

    // 3. Draw underlying images first (30% transparency)
    for (size_t i = 0; i < images.size(); ++i) {
        if (images[i].empty()) continue;

        cv::Mat img;
        images[i].copyTo(img);

        // Convert single channel to BGR
        if (img.channels() == 1) {
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        }

        // Calculate drawing position
        int x = corners[i].x - min_x;
        int y = corners[i].y - min_y;
        cv::Rect roi(x, y, img.cols, img.rows);

        // Check ROI validity
        if (roi.x < 0 || roi.y < 0 ||
            roi.x + roi.width > combined_canvas.cols ||
            roi.y + roi.height > combined_canvas.rows) {
            spdlog::error("[SEAMFINDER] ROI out of bounds for image {}", i);
            continue;
        }

        // Extract corresponding canvas area
        cv::Mat canvas_roi = combined_canvas(roi);

        // Overlay image on bottom layer with 30% transparency (black background + 30% image)
        cv::addWeighted(canvas_roi, 1.0, img, 0.3, 0, canvas_roi);
    }

    // 4. Draw top layer masks (70% transparency for visibility)
    for (size_t i = 0; i < masks.size(); ++i) {
        if (masks[i].empty() || images[i].empty()) continue;

        cv::Mat mask;
        masks[i].copyTo(mask);

        // Calculate drawing position (same as corresponding image)
        int x = corners[i].x - min_x;
        int y = corners[i].y - min_y;
        cv::Rect roi(x, y, images[i].cols, images[i].rows);

        // Check ROI validity
        if (roi.x < 0 || roi.y < 0 ||
            roi.x + roi.width > combined_canvas.cols ||
            roi.y + roi.height > combined_canvas.rows) {
            spdlog::error("[SEAMFINDER] ROI out of bounds for mask {}", i);
            continue;
        }

        // Mask color (image 1 red, image 2 green)
        cv::Scalar color = (i == 0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
        cv::Mat colored_mask(images[i].size(), CV_8UC3, color);

        // Mask preprocessing
        cv::Mat mask_8u;
        mask.convertTo(mask_8u, CV_8U);
        cv::threshold(mask_8u, mask_8u, 127, 255, cv::THRESH_BINARY);

        // Keep only valid mask areas
        colored_mask.setTo(cv::Scalar(0, 0, 0), ~mask_8u);

        // Extract corresponding canvas area
        cv::Mat canvas_roi = combined_canvas(roi);

        // Overlay mask on top layer with 70% transparency
        cv::addWeighted(canvas_roi, 1.0, colored_mask, 0.3, 0, canvas_roi);
    }

    // 5. Display results
    // [修复] HighGUI(namedWindow/imshow/waitKey)只能在 Windows 主线程安全使用；
    //   开发板(Linux)上从 SeamFinder 工作线程直接 imshow 会与 launcher 的 Qt 抢占
    //   GUI 资源，导致工作线程卡死、stop() 的 join() 永久阻塞、进程无法退出。
    //   因此板子上沿用 visualizeAlphaComparison 的做法：把可视化画布原子写到
    //   /tmp/seam_alpha_compare.jpg，由 launcher 的小窗读取显示——接缝可视化
    //   这一核心展示在板子上照常呈现，且不再卡死。
#ifdef _WIN32
    std::string win_name = "Images (30% opacity) + Masks (top) - " + method_name;
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::imshow(win_name, combined_canvas);
    cv::waitKey(1);
#else
    // Linux: 限频原子写盘（与 launcher 的 smallImagePath=/tmp/seam_alpha_compare.jpg 对应）
    {
        static auto last_save_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time).count() >= 33) {
            last_save_time = now;
            const char* tmp_path = "/tmp/seam_alpha_compare.tmp.jpg";
            const char* out_path = "/tmp/seam_alpha_compare.jpg";
            if (cv::imwrite(tmp_path, combined_canvas)) {
                std::rename(tmp_path, out_path);
            } else {
                spdlog::warn("[SEAMFINDER] visualizeResults: imwrite failed ({})", tmp_path);
            }
        }
    }
#endif
}
#else
void SeamFinder::visualizeResults(
    const std::vector<cv::UMat>& images,
    const std::vector<cv::UMat>& masks,
    const std::vector<cv::Point>& corners,
    const std::string& method_name) {

    if (images.empty() || corners.size() != images.size() || masks.size() != images.size()) {
        spdlog::error("[SEAMFINDER] Input size mismatch!");
        return;
    }

    // 1. Calculate coordinate offsets and canvas size (handle negative coordinates)
    int min_x = corners[0].x, min_y = corners[0].y;
    int max_x = corners[0].x, max_y = corners[0].y;
    for (size_t i = 0; i < images.size(); ++i) {
        if (images[i].empty()) {
            spdlog::error("[SEAMFINDER] Empty image at index {}", i);
            continue;
        }
        int x = corners[i].x;
        int y = corners[i].y;
        int w = images[i].cols;
        int h = images[i].rows;
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x + w);
        max_y = std::max(max_y, y + h);
    }

    int canvas_width = max_x - min_x;
    int canvas_height = max_y - min_y;
    if (canvas_width <= 0 || canvas_height <= 0) {
        spdlog::error("[SEAMFINDER] Invalid canvas size!");
        return;
    }

    // 2. Create black background canvas with Alpha channel (initially fully transparent)
    cv::Mat combined_canvas(canvas_height, canvas_width, CV_8UC4, cv::Scalar(0, 0, 0, 0));

    // 3. Process each image: use mask to control Alpha channel (transparent for invalid areas)
    for (size_t i = 0; i < images.size(); ++i) {
        if (images[i].empty() || masks[i].empty()) {
            spdlog::error("[SEAMFINDER] Empty image or mask at index {}", i);
            continue;
        }

        // Convert image to BGR format (3 channels)
        cv::Mat img;
        images[i].copyTo(img);
        if (img.channels() == 1) {
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        }

        // Process mask: convert to 8-bit single channel and binarize (255 inside mask, 0 outside)
        cv::Mat mask_8u;
        masks[i].copyTo(mask_8u);
        mask_8u.convertTo(mask_8u, CV_8U);
        cv::threshold(mask_8u, mask_8u, 127, 255, cv::THRESH_BINARY);

        // Convert BGR image to BGRA (add Alpha channel)
        cv::Mat img_bgra;
        cv::cvtColor(img, img_bgra, cv::COLOR_BGR2BGRA);

        // Set Alpha channel using mask: opaque (Alpha=255) inside mask, fully transparent (Alpha=0) outside
        for (int row = 0; row < img_bgra.rows; ++row) {
            for (int col = 0; col < img_bgra.cols; ++col) {
                // Get mask value (0 or 255), use directly as Alpha value
                img_bgra.at<cv::Vec4b>(row, col)[3] = mask_8u.at<uchar>(row, col);
            }
        }

        // Calculate drawing position (offset correction)
        int x = corners[i].x - min_x;
        int y = corners[i].y - min_y;
        cv::Rect roi(x, y, img_bgra.cols, img_bgra.rows);

        // Check ROI validity
        if (roi.x < 0 || roi.y < 0 ||
            roi.x + roi.width > combined_canvas.cols ||
            roi.y + roi.height > combined_canvas.rows) {
            spdlog::error("[SEAMFINDER] ROI out of bounds for image {}", i);
            continue;
        }

        // Draw image with Alpha channel to canvas (automatically handle transparency overlay)
        img_bgra.copyTo(combined_canvas(roi), mask_8u);  // Only copy valid areas inside mask
    }

    // 4. Display results (OpenCV supports displaying images with Alpha channel)
    std::string win_name = "Transparent Cropped Images - " + method_name;
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::imshow(win_name, combined_canvas);
    cv::waitKey(1);
}


#endif

void SeamFinder::exportCanvasForSaliency(
    const std::vector<cv::UMat>& warped_images,
    const std::vector<cv::Point>& corners,
    int min_x, int min_y, int canvas_w, int canvas_h)
{
    // 降频导出：每 kExportEvery 个周期写一次（接缝不需要逐帧重算显著性）
    static constexpr int kExportEvery = 5;
    if ((saliency_export_skip_++ % kExportEvery) != 0) return;

    // 1) 把各路 warped 图按 corners 贴到 canvas（重叠区后写覆盖，用于显著性检测足够）
    cv::Mat canvas(canvas_h, canvas_w, CV_8UC3, cv::Scalar(0, 0, 0));
    for (size_t i = 0; i < warped_images.size(); ++i) {
        if (warped_images[i].empty()) continue;
        cv::Mat img;
        warped_images[i].copyTo(img);
        if (img.channels() == 1)      cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        else if (img.channels() == 4) cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

        const int x = corners[i].x - min_x;
        const int y = corners[i].y - min_y;
        cv::Rect roi(x, y, img.cols, img.rows);
        if (roi.x < 0 || roi.y < 0 ||
            roi.x + roi.width > canvas_w || roi.y + roi.height > canvas_h) continue;
        img.copyTo(canvas(roi));
    }

    // 2) 原子写 canvas_in.png + canvas_in.seq（与 saliency_writer.py 的 canvas 模式约定一致）
    const std::string& dir = config_params_.sal_u2net_dir;
    const char sep =
#ifdef _WIN32
        '\\';
#else
        '/';
#endif
    const std::string png_path = dir + sep + "canvas_in.bmp";   // .png -> .bmp
    const std::string seq_path = dir + sep + "canvas_in.seq";
    const std::string png_tmp = dir + sep + "canvas_in.tmp.bmp";  // .png -> .bmp
    const std::string seq_tmp = dir + sep + "canvas_in.seq.tmp";


    // 目录不存在时自动创建（防止 imwrite 静默失败）
    std::error_code _ec;
    std::filesystem::create_directories(dir, _ec);

    if (!cv::imwrite(png_tmp, canvas)) {
        spdlog::warn("[SEAMFINDER] exportCanvasForSaliency: imwrite failed ({})", png_tmp);
        return;
    }
    std::remove(png_path.c_str());
    std::rename(png_tmp.c_str(), png_path.c_str());

    ++saliency_in_seq_;
    std::ofstream sf(seq_tmp, std::ios::trunc);
    sf << saliency_in_seq_;
    sf.close();
    std::remove(seq_path.c_str());
    std::rename(seq_tmp.c_str(), seq_path.c_str());

    spdlog::debug("[SEAMFINDER] canvas_in written ({}x{}), seq={}",
        canvas_w, canvas_h, saliency_in_seq_);
}












void SeamFinder::visualizeAlphaComparison(
    const std::vector<cv::UMat>& warped_images,
    const std::vector<cv::Point>& corners,
    const std::vector<cv::UMat>& masks_baseline,
    const std::vector<cv::UMat>& masks_gaze)
{
    if (warped_images.empty() ||
        masks_baseline.size() != warped_images.size() ||
        masks_gaze.size() != warped_images.size()) {
        spdlog::warn("[SEAMFINDER] visualizeAlphaComparison: size mismatch");
        return;
    }

    const size_t num = warped_images.size();

    // ---- 1. 计算 canvas 范围 ----
    int min_x = corners[0].x, min_y = corners[0].y;
    int max_x = corners[0].x, max_y = corners[0].y;
    for (size_t i = 0; i < num; ++i) {
        if (warped_images[i].empty()) continue;
        min_x = std::min(min_x, corners[i].x);
        min_y = std::min(min_y, corners[i].y);
        max_x = std::max(max_x, corners[i].x + warped_images[i].cols);
        max_y = std::max(max_y, corners[i].y + warped_images[i].rows);
    }
    const int cw = max_x - min_x;
    const int ch = max_y - min_y;
    if (cw <= 0 || ch <= 0) return;

    // ---- 2. 底层: warped 图像 40% 透明度, 灰度 ----
    cv::Mat canvas(ch, cw, CV_8UC3, cv::Scalar(30, 30, 30));

    for (size_t i = 0; i < num; ++i) {
        if (warped_images[i].empty()) continue;
        cv::Mat img;
        warped_images[i].copyTo(img);
        if (img.channels() == 1) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

        // 转灰度让上面的彩色轮廓更突出
        cv::Mat gray_bgr;
        cv::cvtColor(img, gray_bgr, cv::COLOR_BGR2GRAY);
        cv::cvtColor(gray_bgr, gray_bgr, cv::COLOR_GRAY2BGR);

        int x = corners[i].x - min_x;
        int y = corners[i].y - min_y;
        cv::Rect roi(x, y, img.cols, img.rows);
        if (roi.x < 0 || roi.y < 0 ||
            roi.x + roi.width > cw || roi.y + roi.height > ch) continue;

        cv::addWeighted(canvas(roi), 1.0, gray_bgr, 0.4, 0, canvas(roi));
    }

    // ---- 3. 可选: 显著性热力图叠加 ----
    if (saliency_src_ != nullptr) {
        const double sigma = cw * static_cast<double>(config_params_.gaze_sigma_ratio);
        cv::Mat saliency = saliency_src_->generateSaliencyMap(cw, ch, sigma, 3000);
        if (!saliency.empty()) {
            cv::Mat sal_8u;
            saliency.convertTo(sal_8u, CV_8U, 255.0);

            cv::Mat heatmap;
            cv::applyColorMap(sal_8u, heatmap, cv::COLORMAP_JET);

            cv::Mat sal_mask;
            cv::threshold(sal_8u, sal_mask, 25, 255, cv::THRESH_BINARY);
            heatmap.setTo(cv::Scalar(0, 0, 0), ~sal_mask);

            cv::addWeighted(canvas, 1.0, heatmap, 0.25, 0, canvas);
        }
    }

    // ---- 4. 提取 mask 轮廓并叠加到同一画布 ----
    //       baseline (alpha=0) 用黄色, gaze-aware 用青色
    auto drawContours = [&](const std::vector<cv::UMat>& masks,
        const cv::Scalar& color, int thickness)
        {
            for (size_t i = 0; i < masks.size(); ++i) {
                if (masks[i].empty() || warped_images[i].empty()) continue;

                // Mask -> 二值化 -> Canny 取边缘
                cv::Mat m;
                masks[i].copyTo(m);
                cv::Mat m8u;
                m.convertTo(m8u, CV_8U);
                cv::threshold(m8u, m8u, 127, 255, cv::THRESH_BINARY);

                cv::Mat edges;
                cv::Canny(m8u, edges, 50, 150);

                // 加粗线条
                if (thickness > 1) {
                    cv::dilate(edges, edges,
                        cv::getStructuringElement(cv::MORPH_RECT,
                            cv::Size(thickness, thickness)));
                }

                // 贴到画布
                int x = corners[i].x - min_x;
                int y = corners[i].y - min_y;
                cv::Rect roi(x, y, warped_images[i].cols, warped_images[i].rows);
                if (roi.x < 0 || roi.y < 0 ||
                    roi.x + roi.width > cw || roi.y + roi.height > ch) continue;

                canvas(roi).setTo(color, edges);
            }
        };

    // baseline 先画 (底层), gaze-aware 后画 (顶层, 相交时压过 baseline)
    drawContours(masks_baseline, cv::Scalar(0, 255, 255), 2);   // 黄色, 细
    drawContours(masks_gaze, cv::Scalar(255, 200, 0), 3);   // 青色, 粗

    // ---- 5. 注视焦点十字准星 ----
    if (saliency_src_ != nullptr) {
        double gx, gy;
        if (auto g = dynamic_cast<GazeDataReader*>(saliency_src_)){
            if (g->getGazeFocus(gx, gy)) {
                int px, py;
                GazeDataReader::gazeToPixel(gx, gy, cw, ch, px, py);

                int cs = std::max(10, cw / 40);
                cv::Scalar red(0, 0, 255);
                cv::line(canvas, cv::Point(px - cs, py), cv::Point(px + cs, py), red, 2);
                cv::line(canvas, cv::Point(px, py - cs), cv::Point(px, py + cs), red, 2);
                cv::circle(canvas, cv::Point(px, py), cs / 2, red, 2);
            }
        }
    }

    // ---- 6. 标题 + 图例 + alpha 数值 ----
    double fs = std::max(0.5, cw / 1400.0);
    int thick = std::max(1, (int)(fs * 2));

    char buf[96];
    snprintf(buf, sizeof(buf), "Seam Comparison  (alpha = %.2f)",
        config_params_.gaze_alpha);
    cv::putText(canvas, buf, cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(255, 255, 255), thick);

    // 图例色块
    cv::rectangle(canvas, cv::Rect(10, 45, 20, 20),
        cv::Scalar(0, 255, 255), cv::FILLED);
    cv::putText(canvas, "Baseline (alpha=0)", cv::Point(40, 62),
        cv::FONT_HERSHEY_SIMPLEX, fs * 0.7,
        cv::Scalar(255, 255, 255), thick);

    cv::rectangle(canvas, cv::Rect(10, 75, 20, 20),
        cv::Scalar(255, 200, 0), cv::FILLED);
    cv::putText(canvas, "Gaze-aware", cv::Point(40, 92),
        cv::FONT_HERSHEY_SIMPLEX, fs * 0.7,
        cv::Scalar(255, 255, 255), thick);

    // ---- 7. 显示 ----
#ifdef _WIN32
    // Windows: imshow 必须在主线程调用，这里只缓存最新一帧，由 main() 取走显示
    {
        std::lock_guard<std::mutex> lk(seam_debug_mutex_);
        canvas.copyTo(seam_debug_frame_);
        has_seam_debug_.store(true, std::memory_order_release);
    }
#else
    // Linux: 周期性写盘
    static auto last_save_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time).count() >= 33) {
        last_save_time = now;
        const char* filename = "/tmp/seam_alpha_compare.tmp.jpg";
        cv::imwrite(filename, canvas);
        std::rename("/tmp/seam_alpha_compare.tmp.jpg", "/tmp/seam_alpha_compare.jpg");
        std::cout << "[SEAMFINDER] Saved alpha comparison to " << filename << std::endl;
    }
#endif
}





/*
void SeamFinder::applyGazeAvoidance(
    const std::vector<cv::UMat>& warped_images,
    const std::vector<cv::Point>& corners,
    std::vector<cv::UMat>& masks)
{
    if (!gaze_reader_ || warped_images.empty() || masks.empty()) return;

    const size_t num_cameras = warped_images.size();

    //计算画布范围
    int min_x = corners[0].x, min_y = corners[0].y;
    int max_x = corners[0].x, max_y = corners[0].y;
    for (size_t i = 0; i < num_cameras; ++i) {
        if (warped_images[i].empty()) continue;
        min_x = std::min(min_x, corners[i].x);
        min_y = std::min(min_y, corners[i].y);
        max_x = std::max(max_x, corners[i].x + warped_images[i].cols);
        max_y = std::max(max_y, corners[i].y + warped_images[i].rows);
    }
    int canvas_w = max_x - min_x;
    int canvas_h = max_y - min_y;
    if (canvas_w <= 0 || canvas_h <= 0) return;

    //获取注视焦点
    double gx, gy;
    if (!gaze_reader_->getGazeFocus(gx, gy)) return;

    int focus_px, focus_py;
    GazeDataReader::gazeToPixel(gx, gy, canvas_w, canvas_h, focus_px, focus_py);

    // 找到注视点所属的相机
    int owner_cam = -1;
    for (size_t i = 0; i < num_cameras; ++i) {
        if (warped_images[i].empty()) continue;
        int lx = focus_px - (corners[i].x - min_x);
        int ly = focus_py - (corners[i].y - min_y);
        if (lx < 0 || ly < 0) continue;

        cv::Mat m = masks[i].getMat(cv::ACCESS_READ);
        if (lx < m.cols && ly < m.rows && m.at<uchar>(ly, lx) > 0) {
            owner_cam = static_cast<int>(i);
            break;
        }
    }
    if (owner_cam < 0) {
        float min_dist = 1e18f;
        for (size_t i = 0; i < num_cameras; ++i) {
            if (warped_images[i].empty()) continue;
            float cx = corners[i].x - min_x + warped_images[i].cols * 0.5f;
            float cy = corners[i].y - min_y + warped_images[i].rows * 0.5f;
            float d = (focus_px - cx) * (focus_px - cx) + (focus_py - cy) * (focus_py - cy);
            if (d < min_dist) { min_dist = d; owner_cam = static_cast<int>(i); }
        }
    }
    if (owner_cam < 0) return;

    //生成显著性热力图
    double sigma = canvas_w * static_cast<double>(config_params_.gaze_sigma_ratio);
    cv::Mat saliency = gaze_reader_->generateSaliencyMap(canvas_w, canvas_h, sigma, 3000);
    if (saliency.empty()) return;

    //高斯模糊
    int blur_k = static_cast<int>(sigma * 0.6) | 1;
    if (blur_k >= 3) {
        cv::GaussianBlur(saliency, saliency, cv::Size(blur_k, blur_k), 0);
        double max_val;
        cv::minMaxLoc(saliency, nullptr, &max_val);
        if (max_val > 1e-6) saliency /= static_cast<float>(max_val);
    }

    const float saliency_threshold = config_params_.gaze_saliency_threshold;

    // 对每个非所属相机处理重叠区域
    for (size_t j = 0; j < num_cameras; ++j) {
        if (static_cast<int>(j) == owner_cam) continue;
        if (warped_images[j].empty()) continue;

        cv::Rect roi_owner(corners[owner_cam].x - min_x, corners[owner_cam].y - min_y,
            warped_images[owner_cam].cols, warped_images[owner_cam].rows);
        cv::Rect roi_j(corners[j].x - min_x, corners[j].y - min_y,
            warped_images[j].cols, warped_images[j].rows);

        cv::Rect overlap = roi_owner & roi_j;
        if (overlap.width <= 0 || overlap.height <= 0) continue;

        //到重叠矩形边缘的几何距离权重
        int margin = std::max(15, std::min(overlap.height, overlap.width) / 5);

        cv::Mat boundary_weight(overlap.height, overlap.width, CV_32F);
        for (int r = 0; r < overlap.height; ++r) {
            float* bw = boundary_weight.ptr<float>(r);
            for (int c = 0; c < overlap.width; ++c) {
                float dist = static_cast<float>(std::min({
                    r,
                    overlap.height - 1 - r,
                    c,
                    overlap.width - 1 - c
                    }));
                bw[c] = std::min(1.0f, dist / static_cast<float>(margin));
            }
        }

        //像素级重新分配
        cv::Mat mask_owner_rw = masks[owner_cam].getMat(cv::ACCESS_RW);
        cv::Mat mask_other_rw = masks[j].getMat(cv::ACCESS_RW);

        for (int r = 0; r < overlap.height; ++r) {
            int canvas_row = overlap.y + r;
            const float* sal_ptr = saliency.ptr<float>(canvas_row);
            const float* bw_ptr = boundary_weight.ptr<float>(r);

            int lr_own = canvas_row - (corners[owner_cam].y - min_y);
            int lr_oth = canvas_row - (corners[j].y - min_y);
            if (lr_own < 0 || lr_own >= mask_owner_rw.rows) continue;
            if (lr_oth < 0 || lr_oth >= mask_other_rw.rows) continue;

            uchar* m_own = mask_owner_rw.ptr<uchar>(lr_own);
            uchar* m_oth = mask_other_rw.ptr<uchar>(lr_oth);

            for (int c = 0; c < overlap.width; ++c) {
                int canvas_col = overlap.x + c;

                // 有效显著性 = 原始显著性 × 边界权重
                float effective_sal = sal_ptr[canvas_col] * bw_ptr[c];
                if (effective_sal < saliency_threshold) continue;

                int lc_own = canvas_col - (corners[owner_cam].x - min_x);
                int lc_oth = canvas_col - (corners[j].x - min_x);
                if (lc_own < 0 || lc_own >= mask_owner_rw.cols) continue;
                if (lc_oth < 0 || lc_oth >= mask_other_rw.cols) continue;

                // 将 other 相机的像素转让给 owner
                if (m_oth[lc_oth] > 0) {
                    m_own[lc_own] = 255;
                    m_oth[lc_oth] = 0;
                }
            }
        }

        //形态学平滑
        cv::Rect local_roi_own(overlap.x - (corners[owner_cam].x - min_x),
            overlap.y - (corners[owner_cam].y - min_y),
            overlap.width, overlap.height);
        cv::Rect local_roi_oth(overlap.x - (corners[j].x - min_x),
            overlap.y - (corners[j].y - min_y),
            overlap.width, overlap.height);

        local_roi_own &= cv::Rect(0, 0, mask_owner_rw.cols, mask_owner_rw.rows);
        local_roi_oth &= cv::Rect(0, 0, mask_other_rw.cols, mask_other_rw.rows);

        if (local_roi_own.width > 0 && local_roi_own.height > 0 &&
            local_roi_oth.width > 0 && local_roi_oth.height > 0) {

            int morph_size = std::max(7, canvas_w / 80);
            cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size(morph_size * 2 + 1, morph_size * 2 + 1));

            cv::Mat r_own = mask_owner_rw(local_roi_own);
            cv::Mat r_oth = mask_other_rw(local_roi_oth);

            cv::morphologyEx(r_own, r_own, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(r_own, r_own, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(r_oth, r_oth, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(r_oth, r_oth, cv::MORPH_OPEN, kernel);

            // 互斥性：所属相机优先
            int mw = std::min(local_roi_own.width, local_roi_oth.width);
            int mh = std::min(local_roi_own.height, local_roi_oth.height);
            for (int r = 0; r < mh; ++r) {
                uchar* po = r_own.ptr<uchar>(r);
                uchar* pj = r_oth.ptr<uchar>(r);
                for (int c = 0; c < mw; ++c) {
                    if (po[c] > 0 && pj[c] > 0) pj[c] = 0;
                }
            }
        }
    }

    spdlog::info("[SEAMFINDER] Gaze-aware seam v4 (owner={}, gaze=({:.2f},{:.2f}), margin applied)",
        owner_cam, gx, gy);
}*/

/*
void SeamFinder::visualizeGazeComparison(
    const std::vector<cv::UMat>& warped_images,
    const std::vector<cv::Point>& corners,
    const std::vector<cv::UMat>& masks_original,
    const std::vector<cv::UMat>& masks_gaze)
{
    if (warped_images.empty() || masks_original.empty() || masks_gaze.empty()) return;
    if (masks_original.size() != masks_gaze.size()) return;

    const size_t num = warped_images.size();

    // 计算画布范围
    int min_x = corners[0].x, min_y = corners[0].y;
    int max_x = corners[0].x, max_y = corners[0].y;
    for (size_t i = 0; i < num; ++i) {
        if (warped_images[i].empty()) continue;
        min_x = std::min(min_x, corners[i].x);
        min_y = std::min(min_y, corners[i].y);
        max_x = std::max(max_x, corners[i].x + warped_images[i].cols);
        max_y = std::max(max_y, corners[i].y + warped_images[i].rows);
    }
    int cw = max_x - min_x;
    int ch = max_y - min_y;
    if (cw <= 0 || ch <= 0) return;

    // 每路相机的颜色
    const cv::Scalar cam_colors[] = {
        cv::Scalar(200, 80, 80),   cv::Scalar(80, 80, 200),
        cv::Scalar(80, 200, 80),   cv::Scalar(200, 200, 80),
        cv::Scalar(200, 80, 200),  cv::Scalar(80, 200, 200)
    };
    const int num_colors = 6;

    // 渲染单侧画布的 lambda
    auto renderCanvas = [&](const std::vector<cv::UMat>& masks,
        cv::Scalar seam_color, bool draw_saliency) -> cv::Mat
        {
            cv::Mat canvas(ch, cw, CV_8UC3, cv::Scalar(0, 0, 0));

            // 底层: warped 图像 30% 透明度
            for (size_t i = 0; i < num; ++i) {
                if (warped_images[i].empty()) continue;
                cv::Mat img;
                warped_images[i].copyTo(img);
                if (img.channels() == 1) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

                int x = corners[i].x - min_x;
                int y = corners[i].y - min_y;
                cv::Rect roi(x, y, img.cols, img.rows);
                if (roi.x < 0 || roi.y < 0 ||
                    roi.x + roi.width > cw || roi.y + roi.height > ch) continue;

                cv::addWeighted(canvas(roi), 1.0, img, 0.3, 0, canvas(roi));
            }

            // 中层: mask 颜色区域 20% 透明度
            for (size_t i = 0; i < masks.size(); ++i) {
                if (masks[i].empty()) continue;
                cv::Mat mask;
                masks[i].copyTo(mask);

                int x = corners[i].x - min_x;
                int y = corners[i].y - min_y;
                cv::Rect roi(x, y, warped_images[i].cols, warped_images[i].rows);
                if (roi.x < 0 || roi.y < 0 ||
                    roi.x + roi.width > cw || roi.y + roi.height > ch) continue;

                cv::Mat colored(warped_images[i].size(), CV_8UC3, cam_colors[i % num_colors]);
                cv::Mat mask_8u;
                mask.convertTo(mask_8u, CV_8U);
                cv::threshold(mask_8u, mask_8u, 127, 255, cv::THRESH_BINARY);
                colored.setTo(cv::Scalar(0, 0, 0), ~mask_8u);

                cv::addWeighted(canvas(roi), 1.0, colored, 0.2, 0, canvas(roi));
            }

            // 上层: 接缝线 (mask 边缘)
            for (size_t i = 0; i < masks.size(); ++i) {
                if (masks[i].empty()) continue;
                cv::Mat mask;
                masks[i].copyTo(mask);

                cv::Mat mask_8u;
                mask.convertTo(mask_8u, CV_8U);
                cv::threshold(mask_8u, mask_8u, 127, 255, cv::THRESH_BINARY);

                cv::Mat edges;
                cv::Canny(mask_8u, edges, 50, 150);
                cv::dilate(edges, edges, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

                int x = corners[i].x - min_x;
                int y = corners[i].y - min_y;
                cv::Rect roi(x, y, warped_images[i].cols, warped_images[i].rows);
                if (roi.x < 0 || roi.y < 0 ||
                    roi.x + roi.width > cw || roi.y + roi.height > ch) continue;

                canvas(roi).setTo(seam_color, edges);
            }

            // 显著性热力图叠加 (仅注视感知侧)
            if (draw_saliency && gaze_reader_ != nullptr) {
                double sigma = cw * 0.08;
                cv::Mat saliency = gaze_reader_->generateSaliencyMap(cw, ch, sigma, 3000);

                if (!saliency.empty()) {
                    cv::Mat sal_8u;
                    saliency.convertTo(sal_8u, CV_8U, 255.0);

                    cv::Mat heatmap;
                    cv::applyColorMap(sal_8u, heatmap, cv::COLORMAP_JET);

                    // 只在显著性 > 0.1 的区域叠加
                    cv::Mat sal_mask;
                    cv::threshold(sal_8u, sal_mask, 25, 255, cv::THRESH_BINARY);
                    heatmap.setTo(cv::Scalar(0, 0, 0), ~sal_mask);

                    cv::addWeighted(canvas, 1.0, heatmap, 0.3, 0, canvas);
                }
            }

            return canvas;
        };

    // 渲染左右画布
    cv::Mat canvas_left = renderCanvas(masks_original, cv::Scalar(0, 255, 255), false);  // 黄色接缝线
    cv::Mat canvas_right = renderCanvas(masks_gaze, cv::Scalar(0, 255, 100), true);   // 绿色接缝线 + 热力图

    // 标题文字
    double fs = std::max(0.5, cw / 1200.0);
    int thick = std::max(1, (int)(fs * 2));

    cv::putText(canvas_left, "Original Seam (GraphCut)",
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, fs,
        cv::Scalar(0, 255, 255), thick);

    cv::putText(canvas_right, "Gaze-Aware Seam",
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, fs,
        cv::Scalar(0, 255, 100), thick);

    // 注视焦点十字准星
    if (gaze_reader_ != nullptr) {
        double gx, gy;
        if (gaze_reader_->getGazeFocus(gx, gy)) {
            int px, py;
            GazeDataReader::gazeToPixel(gx, gy, cw, ch, px, py);

            int cs = std::max(10, cw / 40);
            cv::Scalar red(0, 0, 255);
            cv::line(canvas_right, cv::Point(px - cs, py), cv::Point(px + cs, py), red, 2);
            cv::line(canvas_right, cv::Point(px, py - cs), cv::Point(px, py + cs), red, 2);
            cv::circle(canvas_right, cv::Point(px, py), cs / 2, red, 2);

            char buf[64];
            snprintf(buf, sizeof(buf), "Gaze: (%.2f, %.2f)", gx, gy);
            cv::putText(canvas_right, buf,
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, fs * 0.8,
                cv::Scalar(0, 0, 255), thick);
        }
    }

    // 并排拼接 (中间白色分隔线)
    cv::Mat combined(ch, cw * 2 + 3, CV_8UC3, cv::Scalar(255, 255, 255));
    canvas_left.copyTo(combined(cv::Rect(0, 0, cw, ch)));
    canvas_right.copyTo(combined(cv::Rect(cw + 3, 0, cw, ch)));

    // 显示窗口
    std::string win_name = "GAZE DEBUG: Original vs Gaze-Aware Seam";
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    int dw = std::min(1600, combined.cols);
    double sc = (double)dw / combined.cols;
    cv::resizeWindow(win_name, dw, (int)(combined.rows * sc));
    cv::imshow(win_name, combined);
    cv::waitKey(1);

    spdlog::info("[GAZE_DEBUG] Comparison displayed ({}x{})", cw, ch);
}*/
