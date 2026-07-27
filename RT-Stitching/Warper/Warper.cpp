#include "Warper.hpp"
#include "Platform.hpp"  // [新增] 跨平台兼容: localtime_s -> localtime_r
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/highgui.hpp>
#include <iomanip>
#include <spdlog/spdlog.h>
#include "../Utility/DebugDump.hpp"   // [新增] 调试图像统一写盘 /root/build/debug/
#include "../Utility/PerfLog.hpp"     // [新增] 性能日志（CSV + 周期摘要）
#include "../Utility/ThreadAffinity.hpp" // [新增] 线程绑核（RK3588 大小核）
#include "../Utility/Utility.hpp"        // [新增] 视差补偿映射 buildSphericalMapsD0
// ---------- 静态 Dummy 对象，用于无效引用安全初始化 ----------
static CircularBuffer<RTStitching::Image> dummy_output_buffer(1);
static CircularBufferSync<RTStitching::Image> dummy_sync_buffer(1, 1);

using namespace cv;
using namespace std;
using namespace std::chrono;

WarperModule::WarperModule(
    CircularBuffer<RTStitching::Image>& input_buffer,
    CircularBuffer<RTStitching::Image>& output_buffer,
    RTStitching::ConfigParams& config_params,
    std::vector<RTStitching::CameraStitchParams>& camera_stitch_params,
    size_t camera_index,
    bool show_window,
    const std::string& module_name
)
    : last_processing_time_(0.0)
    , output_buffer_(output_buffer)
    , input_buffer_(input_buffer)
    , sync_buffer_to_blender_(dummy_sync_buffer)
    , config_params_(config_params)
    , camera_stitch_params_(camera_stitch_params)
    , camera_index_(camera_index)
    , is_running_(false)
    , is_paused_(false)
    , stop_requested_(false)
    , try_cuda_(false)
    , warp_scale_(1.0)
    , mid_focal_(-1.0)
    , show_window_(show_window)
    , module_name_(module_name)
    , display_info_history_()
{
    if (module_name_.empty())
        module_name_ = "Warper_" + std::to_string(camera_index_);
}

WarperModule::WarperModule(
    CircularBuffer<RTStitching::Image>& input_buffer,
    CircularBufferSync<RTStitching::Image>& sync_buffer_to_blender,
    RTStitching::ConfigParams& config_params,
    std::vector<RTStitching::CameraStitchParams>& camera_stitch_params,
    size_t camera_index,
    bool show_window,
    const std::string& module_name
)
    : last_processing_time_(0.0)
    , output_buffer_(dummy_output_buffer)
    , input_buffer_(input_buffer)
    , sync_buffer_to_blender_(sync_buffer_to_blender)
    , config_params_(config_params)
    , camera_stitch_params_(camera_stitch_params)
    , camera_index_(camera_index)
    , is_running_(false)
    , is_paused_(false)
    , stop_requested_(false)
    , try_cuda_(false)
    , warp_scale_(1.0)
    , mid_focal_(-1.0)
    , show_window_(show_window)
    , module_name_(module_name)
{
    if (module_name_.empty())
        module_name_ = "Warper_" + std::to_string(camera_index_);
}

WarperModule::~WarperModule() {
    stop(); // Ensure thread is stopped safely
}

bool WarperModule::start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (is_running_) return false;

    // Reset thread status
    is_running_.store(true);
    is_paused_.store(false);
    stop_requested_.store(false);

    // Create worker thread
    worker_thread_ = std::thread(&WarperModule::runImpl, this);

    spdlog::info("[WARPER] Warper thread started");
    return true;
}

void WarperModule::stop() {
    if (!is_running_.load()) return;

    // Send stop request
    stop_requested_.store(true);
    // Wake up waiting thread
    condition_.notify_all();

    // Wait for thread to exit
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    // Reset thread status
    is_running_.store(false);
    is_paused_.store(false);

    spdlog::info("[WARPER] Warper thread stopped");
}

void WarperModule::pause() {
    is_paused_.store(true);
    spdlog::info("[WARPER] Warper thread paused");
}

void WarperModule::resume() {
    if (is_paused_.load()) {
        is_paused_.store(false);
        condition_.notify_one();
        spdlog::info("[WARPER] Warper thread resumed");
    }
}

bool WarperModule::isRunning() const {
    return is_running_.load(std::memory_order_acquire);
}

bool WarperModule::isPaused() const {
    return is_paused_.load(std::memory_order_acquire);
}

// Core loop implementation of the thread
void WarperModule::runImpl() {
    RTStitching::bindToBigCores(module_name_);   // [新增] 重计算线程绑大核(A76)
    spdlog::info("[WARPER] Warper thread main loop started");

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

        if (!input_buffer_.empty()) {
            RTStitching::Image input_data;
#if 0
            if (input_buffer_.try_pop_front(input_data)) {
                // skip all other data in buffer 可优化为back + clear
                RTStitching::Image skip_data;
                while (input_buffer_.try_pop_front(skip_data)) {
                    input_data = skip_data;
                    spdlog::warn("[WARPER] Video {} skip frame {}", camera_index_, skip_data.img_idx);
                }
#else
            if (input_buffer_.back(input_data)) {
#endif

                // Record timestamps
                DisplayInfo current_info;
                // [新增] 性能统计：取帧时刻（与图像调试开关无关，始终记录）
                const auto perf_fetch_time = high_resolution_clock::now();

                if (show_window_) {
                    // Record all timestamps in ProcessingTimestamps structure
                    current_info.timestamps.image_original_timestamp = input_data.timestamp;
                    current_info.timestamps.input_fetched_time = high_resolution_clock::now();

                    current_info.img_idx = input_data.img_idx;
                    current_info.input_size = input_data.data.size();

                    // Record CameraStitchParams version information
                    if (camera_index_ < camera_stitch_params_.size()) {
                        current_info.csp_ver = camera_stitch_params_[camera_index_].csp_ver;
                        current_info.sm_ver = camera_stitch_params_[camera_index_].sm_ver;

                        // Record size and corner from camera_stitch_params
                        if (!camera_stitch_params_[camera_index_].sizes.empty()) {
                            current_info.stitch_size = camera_stitch_params_[camera_index_].sizes[0];
                        }
                        if (!camera_stitch_params_[camera_index_].corners.empty()) {
                            current_info.stitch_corner = camera_stitch_params_[camera_index_].corners[0];
                        }
                    }
                    else {
                        current_info.csp_ver = -1;
                        current_info.sm_ver = -1;
                    }

                    // Record camera parameters
                    if (camera_index_ < config_params_.camera_params.size()) {
                        current_info.K_matrix = config_params_.camera_params[camera_index_].K.clone();
                        current_info.R_matrix = config_params_.camera_params[camera_index_].R.clone();
                    }

                    // Directly reference input image to avoid copy
                    current_info.input_image = input_data.data;

                    // Add additional information
                    current_info.additional_messages.push_back("Warper Type: " + warper_type_);
                    current_info.additional_messages.push_back("Camera Index: " + std::to_string(camera_index_));
                    current_info.additional_messages.push_back("Warp Scale: " + std::to_string(warp_scale_));

                    // Record warp start time
                    current_info.timestamps.warping_start_time = high_resolution_clock::now();
                }

                RTStitching::Image output_data;
                cv::Point corner;

                // Update warper when mid_focal changes
                if (mid_focal_ != camera_stitch_params_[RTStitching::BLENDER_SCALE].mid_focal) {
                    mid_focal_ = camera_stitch_params_[RTStitching::BLENDER_SCALE].mid_focal;
                    warper_.release();
                    warper_ = warper_creator_->create(mid_focal_);
                }

                // [新增] 性能统计：变形起始时刻
                const auto perf_warp_start = high_resolution_clock::now();

                if (warpImage(input_data, output_data, corner, camera_index_)) {
                    output_data.timestamp = high_resolution_clock::now();
                    output_data.img_idx = input_data.img_idx;

                    // [新增] 性能日志：与原 info 面板 Time Analysis 同定义
                    //   input_buffer_delay_ms —— 采集时间戳 -> Warper 取帧
                    //   warp_ms               —— 变形计算耗时
                    //   total_latency_ms      —— 采集时间戳 -> 变形输出就绪
                    RTStitching::perfRecord(module_name_, "input_buffer_delay_ms",
                        duration<double, std::milli>(perf_fetch_time - input_data.timestamp).count(),
                        input_data.img_idx);
                    RTStitching::perfRecord(module_name_, "warp_ms",
                        duration<double, std::milli>(output_data.timestamp - perf_warp_start).count(),
                        input_data.img_idx);
                    RTStitching::perfRecord(module_name_, "total_latency_ms",
                        duration<double, std::milli>(output_data.timestamp - input_data.timestamp).count(),
                        input_data.img_idx);
                    if (show_window_) {
                        // Record warp end time
                        current_info.timestamps.warping_end_time = high_resolution_clock::now();
                        
                        current_info.output_size = output_data.data.size();
                        current_info.corner = corner;

                        // Directly reference output image to avoid copy
                        current_info.output_image = output_data.data;

                        // Record output ready time
                        current_info.timestamps.output_ready_time = high_resolution_clock::now();

                        // [修改] 调试信息图写盘 /root/build/debug/<module>_info.jpg（Windows 仍 imshow）
                        cv::Mat info_display = createDisplayImage(current_info);
                        int dbg_key = RTStitching::debugDump(module_name_ + "_info", info_display);

                        // Save display information to history
                        addDisplayInfo(current_info);

                        if (dbg_key == 27) {
                            stop_requested_.store(true, std::memory_order_release);
                        }
                    }
                    
                    bool pushed_to_any_buffer = false;
                    if (output_buffer_.try_push_back(output_data)) {
                        pushed_to_any_buffer = true;
                    }
                    if (sync_buffer_to_blender_.push_back(camera_index_, output_data)) {
                        pushed_to_any_buffer = true;
                    }
                    if (!pushed_to_any_buffer) {
                        spdlog::error("[WARPER] All output buffers full, dropping warped image");
                        if (show_window_) {
                            current_info.additional_messages.push_back("WARNING: Output buffer full!");
                        }
                    }
                }
                else {
                    spdlog::error("[WARPER] Image warping failed: {}", last_error_);
                    if (show_window_) {
                        current_info.additional_messages.push_back("ERROR: " + last_error_);
                        current_info.additional_messages.push_back("Warping failed!");

                        cv::Mat info_display = createDisplayImage(current_info);
                        // [修改] 失败信息图同样写盘 /root/build/debug/（Windows 仍 imshow）
                        RTStitching::debugDump(module_name_ + "_info", info_display);
                        addDisplayInfo(current_info);
                    }
                }

                auto end = high_resolution_clock::now();
                auto duration = duration_cast<milliseconds>(end - current_info.timestamps.warping_start_time);
                last_processing_time_ = duration.count();
            }
        }
        else {
            std::this_thread::sleep_for(milliseconds(1));
        }
    }

    warper_.release();
    spdlog::info("[WARPER] Warper thread main loop ended");
}

cv::Ptr<cv::WarperCreator> WarperModule::initWarperCreator(
    std::string warper_type, bool try_cuda)
{
    cv::Ptr<cv::WarperCreator> warper_creator;
    if (warper_type == "plane") {
        warper_creator = makePtr<PlaneWarper>();
    }
    else if (warper_type == "affine") {
        warper_creator = makePtr<AffineWarper>();
    }
    else if (warper_type == "cylindrical") {
        warper_creator = makePtr<CylindricalWarper>();
    }
    else if (warper_type == "spherical") {
        warper_creator = makePtr<SphericalWarper>();
    }
    else if (warper_type == "fisheye") {
        warper_creator = makePtr<FisheyeWarper>();
    }
    else if (warper_type == "stereographic") {
        warper_creator = makePtr<StereographicWarper>();
    }
    else if (warper_type == "compressedPlaneA2B1") {
        warper_creator = makePtr<CompressedRectilinearWarper>(2.0f, 1.0f);
    }
    else if (warper_type == "compressedPlaneA1.5B1") {
        warper_creator = makePtr<CompressedRectilinearWarper>(1.5f, 1.0f);
    }
    else if (warper_type == "compressedPlanePortraitA2B1") {
        warper_creator = makePtr<CompressedRectilinearPortraitWarper>(2.0f, 1.0f);
    }
    else if (warper_type == "compressedPlanePortraitA1.5B1") {
        warper_creator = makePtr<CompressedRectilinearPortraitWarper>(1.5f, 1.0f);
    }
    else if (warper_type == "paniniA2B1") {
        warper_creator = makePtr<PaniniWarper>(2.0f, 1.0f);
    }
    else if (warper_type == "paniniA1.5B1") {
        warper_creator = makePtr<PaniniWarper>(1.5f, 1.0f);
    }
    else if (warper_type == "paniniPortraitA2B1") {
        warper_creator = makePtr<PaniniPortraitWarper>(2.0f, 1.0f);
    }
    else if (warper_type == "paniniPortraitA1.5B1") {
        warper_creator = makePtr<PaniniPortraitWarper>(1.5f, 1.0f);
    }
    else if (warper_type == "mercator") {
        warper_creator = makePtr<MercatorWarper>();
    }
    else if (warper_type == "transverseMercator") {
        warper_creator = makePtr<TransverseMercatorWarper>();
    }
    else {
        spdlog::error("[WARPER] Unknown warper type: {}", warper_type);
        return nullptr;
    }

    return warper_creator;
}

bool WarperModule::initialize(const RTStitching::ConfigParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Save parameters
    warper_type_ = params.warper_type;
    try_cuda_ = params.try_cuda;
    warp_scale_ = camera_stitch_params_[RTStitching::BLENDER_SCALE].scale_ratio;

    warper_creator_ = initWarperCreator(warper_type_, try_cuda_);

    if (params.verbose_output) {
        spdlog::info("[WARPER] Warper initialized successfully with type: {}, scale: {}, try_cuda: {}",
            warper_type_, warp_scale_, (try_cuda_ ? "true" : "false"));
    }

    return true;
}

static std::string formatSystemTime(const std::chrono::system_clock::time_point& tp) {
    auto now = std::chrono::system_clock::to_time_t(tp);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        tp.time_since_epoch()) % 1000;

    std::tm tm_buf{};
    localtime_s(&tm_buf, &now);

    std::stringstream ss;
    ss << std::put_time(&tm_buf, "%H:%M:%S")
        << "." << std::setw(3) << std::setfill('0') << ms.count();
    return ss.str();
}

bool WarperModule::warpImage(
    const RTStitching::Image& input_image,
    RTStitching::Image& output_image,
    cv::Point& corner,
    int camera_index)
{
    if (!warper_) {
        last_error_ = "Warper not initialized";
        return false;
    }

    if (camera_index < 0 || camera_index >= static_cast<int>(config_params_.camera_params.size())) {
        last_error_ = "Invalid camera index: " + std::to_string(camera_index);
        return false;
    }

    // [优化] 改用带缓存的 doWarpCached：静态参数下只在首帧/参数变化时
    //   buildMaps，之后每帧仅 remap，避免每帧重建球面映射表。
    if (doWarpCached(input_image.data, config_params_.camera_params[camera_index],
        camera_stitch_params_[RTStitching::BLENDER_SCALE].scale_ratio / camera_stitch_params_[RTStitching::INPUT_SCALE].scale_ratio,
        INTER_LINEAR, BORDER_CONSTANT,
        output_image.data, corner)) {
        return true;
    }

    return false;
}

bool WarperModule::doWarpCached(
    const cv::Mat& src,
    const RTStitching::CameraParams& camera,
    double trans_ratio,
    int interp_mode, int border_mode,
    cv::Mat& dst,
    cv::Point& corner)
{
    if (!warper_) {
        last_error_ = "Warper not initialized";
        return false;
    }
    try {
        // 准备缩放后的内参矩阵（与静态 doWarp 完全一致的算法）
        Mat_<float> K;
        camera.K.convertTo(K, CV_32F);
        K(0, 0) *= static_cast<float>(trans_ratio);
        K(0, 2) *= static_cast<float>(trans_ratio);
        K(1, 1) *= static_cast<float>(trans_ratio);
        K(1, 2) *= static_cast<float>(trans_ratio);

        Mat_<float> R;
        camera.R.convertTo(R, CV_32F);

        // [修改] 去畸变折叠：相机启用去畸变且 D 非零时，把"去畸变 remap"和
        //   "球面投影 remap"两级映射在建表阶段预合成为一张联合映射表，
        //   运行时对【原始帧】只做一次 remap —— 采集线程的逐帧去畸变(≈55ms/帧)
        //   与本函数的逐帧 resize 同时消失。仅初始帧/参数变化时重建，之后零成本。
        const bool fold_undistort =
            camera_index_ >= 0 &&
            camera_index_ < static_cast<int>(config_params_.camera_info.size()) &&
            config_params_.camera_info[camera_index_].undistort &&
            !camera.D.empty() && !camera.K.empty() &&
            cv::countNonZero(camera.D != 0.0) > 0;

        const cv::Mat* psrc = &src;
        cv::Mat resized;
        cv::Size map_src_size;   // buildMaps 使用的（虚拟）源尺寸
        if (fold_undistort) {
            // 折叠路径不做实际 resize：缩放已并入联合映射，直接从原始帧采样
            map_src_size = cv::Size(cvRound(src.cols * trans_ratio),
                                    cvRound(src.rows * trans_ratio));
        }
        else {
            // 原路径：按 trans_ratio 缩放输入；trans_ratio≈1 时跳过冗余拷贝
            if (std::abs(trans_ratio - 1.0) > 1e-9) {
                resize(src, resized, Size(), trans_ratio, trans_ratio);
                psrc = &resized;
            }
            map_src_size = psrc->size();
        }

        // [新增] 视差补偿参数：d0>0 且有光心 T 时启用（Config 已保证仅 spherical）
        const double d0 = config_params_.parallax_d0;
        cv::Mat T64;
        if (!camera.T.empty()) camera.T.convertTo(T64, CV_64F);

        // 判断是否需要(重新)构建映射表：首帧、输入尺寸/焦距/K/R/折叠状态变化时
        bool need_rebuild =
            !warp_maps_valid_ ||
            map_src_size != warp_cached_src_size_ ||
            fold_undistort != warp_cached_fold_ ||
            (fold_undistort && src.size() != warp_cached_raw_size_) ||
            mid_focal_ != warp_cached_focal_ ||
            d0 != warp_cached_d0_ ||
            warp_cached_K_.empty() || warp_cached_R_.empty() ||
            cv::norm(K, warp_cached_K_, cv::NORM_INF) > 1e-4 ||
            cv::norm(R, warp_cached_R_, cv::NORM_INF) > 1e-4 ||
            (!T64.empty() && (warp_cached_T_.empty() ||
                cv::norm(T64, warp_cached_T_, cv::NORM_INF) > 1e-9));

        if (need_rebuild) {
            if (d0 > 1e-9) {
                // [新增] 近景视差补偿路径：等价 buildMaps，但世界方向按
                //   "半径 d0 球面上的点"计算，光心偏移 T 进入映射。
                warp_dst_roi_ = RTStitching::buildSphericalMapsD0(
                    map_src_size, K, R, T64, mid_focal_, d0,
                    warp_xmap_, warp_ymap_);
            }
            else {
                warp_dst_roi_ = warper_->buildMaps(map_src_size, K, R, warp_xmap_, warp_ymap_);
            }

            if (fold_undistort) {
                // (a) 去畸变映射（undistorted 全尺寸 -> 原始帧），参数与原采集端
                //     buildUndistortMaps 完全一致：newCameraMatrix=K、R=I，保证几何不变
                cv::Mat K64, D64;
                camera.K.convertTo(K64, CV_64F);
                camera.D.convertTo(D64, CV_64F);
                cv::Mat ud_x, ud_y;
                cv::initUndistortRectifyMap(K64, D64, cv::Mat(), K64,
                                            src.size(), CV_32FC1, ud_x, ud_y);

                // (b) warp 映射坐标从 resize 后空间换算回全尺寸 undistorted 空间
                cv::Mat wx = warp_xmap_, wy = warp_ymap_;
                if (std::abs(trans_ratio - 1.0) > 1e-9) {
                    wx = warp_xmap_ / trans_ratio;
                    wy = warp_ymap_ / trans_ratio;
                }

                // (c) 复合：combined(p) = undistort_map(warp_map(p))；
                //     warp 落点越界处填大负数坐标，最终 remap 时判为图外(黑边)
                cv::Mat cmb_x, cmb_y;
                cv::remap(ud_x, cmb_x, wx, wy, cv::INTER_LINEAR,
                          cv::BORDER_CONSTANT, cv::Scalar(-1e5));
                cv::remap(ud_y, cmb_y, wx, wy, cv::INTER_LINEAR,
                          cv::BORDER_CONSTANT, cv::Scalar(-1e5));

                // (d) 定点化为 CV_16SC2，remap 提速接近一倍
                cv::convertMaps(cmb_x, cmb_y, warp_xmap_, warp_ymap_, CV_16SC2);
            }

            warp_cached_src_size_ = map_src_size;
            warp_cached_fold_ = fold_undistort;
            warp_cached_raw_size_ = src.size();
            warp_cached_focal_ = mid_focal_;
            warp_cached_K_ = K.clone();
            warp_cached_R_ = R.clone();
            warp_cached_d0_ = d0;
            warp_cached_T_ = T64.clone();
            warp_maps_valid_ = true;
            spdlog::info("[WARPER] [{}] Warp maps (re)built: roi=({},{}) {}x{}, undistort folded={}, parallax_d0={}",
                module_name_, warp_dst_roi_.x, warp_dst_roi_.y,
                warp_dst_roi_.width, warp_dst_roi_.height,
                fold_undistort ? "yes" : "no",
                d0 > 1e-9 ? std::to_string(d0) : "off");
        }

        // 与 cv::detail::RotationWarper::warp 内部布局严格一致
        // 折叠路径对原始帧一次 remap 完成 去畸变+缩放+球面投影
        const cv::Mat& remap_src = fold_undistort ? src : *psrc;
        dst.create(warp_dst_roi_.height + 1, warp_dst_roi_.width + 1, remap_src.type());
        cv::remap(remap_src, dst, warp_xmap_, warp_ymap_, interp_mode, border_mode);
        corner = warp_dst_roi_.tl();
        return true;
    }
    catch (const cv::Exception& e) {
        last_error_ = std::string("OpenCV exception: ") + e.what();
        spdlog::error("[WARPER] doWarpCached OpenCV exception: {}", e.what());
        return false;
    }
    catch (const std::exception& e) {
        last_error_ = std::string("Standard exception: ") + e.what();
        spdlog::error("[WARPER] doWarpCached exception: {}", e.what());
        return false;
    }
}

bool WarperModule::doWarp(
    cv::Ptr<detail::RotationWarper> warper,
    InputArray src,
    RTStitching::CameraParams camera,
    double trans_ratio,
    int interp_mode, int border_mode,
    OutputArray dst,
    cv::Point& corner,
    double parallax_d0,
    float warper_scale)
{
    try {
        // Prepare intrinsic matrix
        Mat_<float> K;
        camera.K.convertTo(K, CV_32F);

        // Adjust scale of intrinsic matrix
        K(0, 0) *= static_cast<float>(trans_ratio);
        K(0, 2) *= static_cast<float>(trans_ratio);
        K(1, 1) *= static_cast<float>(trans_ratio);
        K(1, 2) *= static_cast<float>(trans_ratio);

        // Convert rotation matrix to CV_32F
        Mat_<float> R;
        camera.R.convertTo(R, CV_32F);

        // Perform image warping
        UMat resized_image;
        resize(src, resized_image, Size(), trans_ratio, trans_ratio);

        // [新增] 近景视差补偿路径：与 doWarpCached / SeamFinder 用同一套几何，
        //   保证掩码/角点与热路径 warp 结果严格一致（Blender 依赖此一致性放置）。
        if (parallax_d0 > 1e-9 && warper_scale > 0.0f) {
            cv::Mat T64;
            if (!camera.T.empty()) camera.T.convertTo(T64, CV_64F);
            corner = RTStitching::warpSphericalD0(
                resized_image, K, R, T64, warper_scale, parallax_d0,
                interp_mode, border_mode, dst);
            return true;
        }

        corner = warper->warp(resized_image, K, R, interp_mode, border_mode, dst);

        return true;
    }
    catch (const cv::Exception& e) {
        spdlog::error("[WARPER] OpenCV exception: {}", e.what());
        return false;
    }
    catch (const std::exception& e) {
        spdlog::error("[WARPER] Standard exception: {}", e.what());
        return false;
    }
}

// Helper function to format matrix display elements
std::string formatMatrixElement(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << value;
    return oss.str();
}

void WarperModule::drawTextWithBackground(Mat& image, const std::string& text,
    Point position, Scalar text_color,
    Scalar bg_color) {
    int font_face = FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.4;
    int thickness = 1;
    int baseline = 0;

    Size text_size = getTextSize(text, font_face, font_scale, thickness, &baseline);

    // Draw background rectangle
    rectangle(image,
        Rect(position.x, position.y - text_size.height - 2,
            text_size.width, text_size.height + baseline + 4),
        bg_color,
        FILLED);

    // Draw text
    putText(image, text,
        Point(position.x, position.y - 2),
        font_face, font_scale, text_color, thickness);
}

void WarperModule::addDisplayInfo(const DisplayInfo& info) {
    std::lock_guard<std::mutex> lock(mutex_);
    display_info_history_.push_back(info);
    if (display_info_history_.size() > MAX_HISTORY_SIZE) {
        display_info_history_.erase(display_info_history_.begin());
    }
}

// Helper function to find non-zero region of an image (minimum bounding rectangle)
Rect findNonZeroRegion(const Mat& image) {
    if (image.empty()) return Rect();

    Mat gray;
    if (image.channels() == 3) {
        cvtColor(image, gray, COLOR_BGR2GRAY);
    }
    else {
        gray = image.clone();
    }

    // Find positions of non-zero pixels
    Mat nonZeroCoords;
    findNonZero(gray, nonZeroCoords);

    if (nonZeroCoords.total() == 0) {
        return Rect(0, 0, image.cols, image.rows);
    }

    // Calculate bounding box
    int min_x = image.cols, min_y = image.rows;
    int max_x = 0, max_y = 0;

    for (int i = 0; i < nonZeroCoords.total(); i++) {
        Point pt = nonZeroCoords.at<Point>(i);
        min_x = std::min(min_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_x = std::max(max_x, pt.x);
        max_y = std::max(max_y, pt.y);
    }

    // Add some margin
    int margin = 5;
    min_x = std::max(0, min_x - margin);
    min_y = std::max(0, min_y - margin);
    max_x = std::min(image.cols - 1, max_x + margin);
    max_y = std::min(image.rows - 1, max_y + margin);

    return Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
}

// Modified createDisplayImage function, adding camera parameter display
Mat WarperModule::createDisplayImage(const DisplayInfo& info) {
    const int DISPLAY_WIDTH = 720;
    const int DISPLAY_HEIGHT = 540;

    Mat display_image = Mat::zeros(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3);

    // Calculate time information (in milliseconds)
    auto image_original_ms = time_point_cast<milliseconds>(info.timestamps.image_original_timestamp);
    auto input_fetched_ms = time_point_cast<milliseconds>(info.timestamps.input_fetched_time);
    auto warping_start_ms = time_point_cast<milliseconds>(info.timestamps.warping_start_time);
    auto warping_end_ms = time_point_cast<milliseconds>(info.timestamps.warping_end_time);
    auto output_ready_ms = time_point_cast<milliseconds>(info.timestamps.output_ready_time);

    // Calculate time differences for each stage
    double input_buffer_delay = duration_cast<milliseconds>(
        info.timestamps.input_fetched_time - info.timestamps.image_original_timestamp).count();

    double warping_time = duration_cast<milliseconds>(
        info.timestamps.warping_end_time - info.timestamps.warping_start_time).count();

    double post_warp_processing = duration_cast<milliseconds>(
        info.timestamps.output_ready_time - info.timestamps.warping_end_time).count();

    double total_processing_time = duration_cast<milliseconds>(
        info.timestamps.output_ready_time - info.timestamps.input_fetched_time).count();

    double total_latency = duration_cast<milliseconds>(
        info.timestamps.output_ready_time - info.timestamps.image_original_timestamp).count();

    // Basic information area - text on the left
    int y_pos = 20;
    int line_height = 15;

    // Frame information
    auto system_original_time = std::chrono::system_clock::now() +
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            info.timestamps.image_original_timestamp - std::chrono::high_resolution_clock::now());

    auto system_fetched_time = std::chrono::system_clock::now() +
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            info.timestamps.input_fetched_time - std::chrono::high_resolution_clock::now());

    auto system_ready_time = std::chrono::system_clock::now() +
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            info.timestamps.output_ready_time - std::chrono::high_resolution_clock::now());

    drawTextWithBackground(display_image,
        "Original Timestamp: " + formatSystemTime(system_original_time),
        Point(10, y_pos));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Input Fetched Time: " + formatSystemTime(system_fetched_time),
        Point(10, y_pos));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Output Ready Time: " + formatSystemTime(system_ready_time),
        Point(10, y_pos));
    y_pos += line_height;

    // Time delay information
    y_pos += 5; // Add some spacing

    drawTextWithBackground(display_image,
        "=== Time Analysis ===",
        Point(10, y_pos), Scalar(255, 200, 0), Scalar(50, 50, 50));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Total Latency: " + std::to_string(total_latency) + "ms",
        Point(10, y_pos), Scalar(255, 255, 0), Scalar(50, 50, 50));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Input Buffer Delay: " + std::to_string(input_buffer_delay) + "ms",
        Point(10, y_pos));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Warping Time: " + std::to_string(warping_time) + "ms",
        Point(10, y_pos));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Post-Warp Processing: " + std::to_string(post_warp_processing) + "ms",
        Point(10, y_pos));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Total Processing: " + std::to_string(total_processing_time) + "ms",
        Point(10, y_pos));
    y_pos += line_height;

    // Image size information
    y_pos += 5; // Add some spacing

    drawTextWithBackground(display_image,
        "=== Image Info ===",
        Point(10, y_pos), Scalar(255, 200, 0), Scalar(50, 50, 50));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Input Size: " + std::to_string(info.input_size.width) + "x" +
        std::to_string(info.input_size.height),
        Point(10, y_pos));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Output Size: " + std::to_string(info.output_size.width) + "x" +
        std::to_string(info.output_size.height),
        Point(10, y_pos));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Corner: (" + std::to_string(info.corner.x) + ", " +
        std::to_string(info.corner.y) + ")",
        Point(10, y_pos));
    y_pos += line_height;

    // Version information
    drawTextWithBackground(display_image,
        "CSP Version: " + std::to_string(info.csp_ver),
        Point(10, y_pos));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "SM Version: " + std::to_string(info.sm_ver),
        Point(10, y_pos));
    y_pos += line_height;

    // Camera parameter information
    y_pos += 5; // Add some spacing

    drawTextWithBackground(display_image,
        "=== Camera Parameters ===",
        Point(10, y_pos), Scalar(255, 200, 0), Scalar(50, 50, 50));
    y_pos += line_height;

    // Display K matrix
    if (!info.K_matrix.empty() && info.K_matrix.rows == 3 && info.K_matrix.cols == 3) {
        drawTextWithBackground(display_image, "K Matrix:", Point(10, y_pos));
        y_pos += line_height;

        for (int i = 0; i < 3; i++) {
            std::string row = "  ";
            for (int j = 0; j < 3; j++) {
                row += formatMatrixElement(info.K_matrix.at<double>(i, j));
                if (j < 2) row += ", ";
            }

            drawTextWithBackground(display_image, row, Point(10, y_pos));
            y_pos += line_height;
        }
    }
    else {
        drawTextWithBackground(display_image, "K Matrix: Not available", Point(10, y_pos));
        y_pos += line_height;
    }

    // Display R matrix
    if (!info.R_matrix.empty() && info.R_matrix.rows == 3 && info.R_matrix.cols == 3) {
        drawTextWithBackground(display_image, "R Matrix:", Point(10, y_pos));
        y_pos += line_height;

        for (int i = 0; i < 3; i++) {
            std::string row = "  ";
            for (int j = 0; j < 3; j++) {
                row += formatMatrixElement(info.R_matrix.at<double>(i, j));
                if (j < 2) row += ", ";
            }
            drawTextWithBackground(display_image, row, Point(10, y_pos));
            y_pos += line_height;
        }
    }
    else {
        drawTextWithBackground(display_image, "R Matrix: Not available", Point(10, y_pos));
        y_pos += line_height;
    }

    // Display size and corner from camera_stitch_params
    drawTextWithBackground(display_image,
        "Stitch Size: " + std::to_string(info.stitch_size.width) + "x" +
        std::to_string(info.stitch_size.height),
        Point(10, y_pos));
    y_pos += line_height;

    drawTextWithBackground(display_image,
        "Stitch Corner: (" + std::to_string(info.stitch_corner.x) + ", " +
        std::to_string(info.stitch_corner.y) + ")",
        Point(10, y_pos));
    y_pos += line_height;

    // Reserved area for additional information
    y_pos += 5; // Add some spacing

    drawTextWithBackground(display_image, "=== Additional Info ===",
        Point(10, y_pos), Scalar(255, 200, 0), Scalar(50, 50, 50));
    y_pos += line_height;

    // Display additional text information
    for (const auto& msg : info.additional_messages) {
        drawTextWithBackground(display_image, msg, Point(20, y_pos));
        y_pos += line_height;
        if (y_pos > 250) break; // Prevent text from exceeding image area
    }

    // Display thumbnails of input and output images in the right area
    int thumbnail_area_x = 360; // Start from the middle
    int thumbnail_width = 340;  // Thumbnail width
    int thumbnail_height = 200; // Thumbnail height

    // Display input image thumbnail - using minimum bounding rectangle
    if (!info.input_image.empty()) {
        Mat input_thumbnail;

        // Find non-zero region of input image
        Rect input_roi = findNonZeroRegion(info.input_image);
        Mat input_roi_image = info.input_image(input_roi);

        // Resize image to thumbnail size while maintaining aspect ratio
        double scale = std::min(
            (double)thumbnail_width / input_roi_image.cols,
            (double)thumbnail_height / input_roi_image.rows
        );
        resize(input_roi_image, input_thumbnail, Size(), scale, scale);

        // Create thumbnail with border
        Mat input_display = Mat::zeros(thumbnail_height, thumbnail_width, CV_8UC3);

        int x_offset = (thumbnail_width - input_thumbnail.cols) / 2;
        int y_offset = (thumbnail_height - input_thumbnail.rows) / 2;
        input_thumbnail.copyTo(input_display(Rect(x_offset, y_offset, input_thumbnail.cols, input_thumbnail.rows)));

        // Draw ROI boundary information
        std::string roi_info = "ROI: " + std::to_string(input_roi.x) + "," + std::to_string(input_roi.y) +
            " " + std::to_string(input_roi.width) + "x" + std::to_string(input_roi.height);
        drawTextWithBackground(input_display, roi_info,
            Point(5, thumbnail_height - 10), Scalar(255, 255, 255), Scalar(0, 0, 0, 128));

        // Add title
        drawTextWithBackground(input_display, "Input Image (ROI)",
            Point(10, 20), Scalar(255, 255, 255), Scalar(0, 0, 255));

        // Place input thumbnail into display image
        input_display.copyTo(display_image(Rect(thumbnail_area_x, 50, thumbnail_width, thumbnail_height)));
    }
    else {
        drawTextWithBackground(display_image, "No Input Image",
            Point(thumbnail_area_x + 120, 150), Scalar(255, 255, 255), Scalar(100, 100, 100));
    }

    // Display output image thumbnail - using minimum bounding rectangle
    if (!info.output_image.empty()) {
        Mat output_thumbnail;

        // Find non-zero region of output image (effective area after warp)
        Rect output_roi = findNonZeroRegion(info.output_image);
        Mat output_roi_image = info.output_image(output_roi);

        // Resize image to thumbnail size while maintaining aspect ratio
        double scale = std::min(
            (double)thumbnail_width / output_roi_image.cols,
            (double)thumbnail_height / output_roi_image.rows
        );
        resize(output_roi_image, output_thumbnail, Size(), scale, scale);

        // Create thumbnail with border
        Mat output_display = Mat::zeros(thumbnail_height, thumbnail_width, CV_8UC3);
        int x_offset = (thumbnail_width - output_thumbnail.cols) / 2;
        int y_offset = (thumbnail_height - output_thumbnail.rows) / 2;
        output_thumbnail.copyTo(output_display(Rect(x_offset, y_offset, output_thumbnail.cols, output_thumbnail.rows)));

        // Draw ROI boundary information
        std::string roi_info = "ROI: " + std::to_string(output_roi.x) + "," + std::to_string(output_roi.y) +
            " " + std::to_string(output_roi.width) + "x" + std::to_string(output_roi.height);
        drawTextWithBackground(output_display, roi_info,
            Point(5, thumbnail_height - 10), Scalar(255, 255, 255), Scalar(0, 0, 0, 128));

        // Add title
        drawTextWithBackground(output_display, "Output Image (ROI)",
            Point(10, 20), Scalar(255, 255, 255), Scalar(0, 165, 255));

        // Place output thumbnail into display image
        output_display.copyTo(display_image(Rect(thumbnail_area_x, 270, thumbnail_width, thumbnail_height)));

        // Add ROI information to info area
        y_pos += 5;
        drawTextWithBackground(display_image,
            "Output ROI: (" + std::to_string(output_roi.x) + ", " + std::to_string(output_roi.y) + ") " +
            std::to_string(output_roi.width) + "x" + std::to_string(output_roi.height),
            Point(10, y_pos));
        y_pos += line_height;
    }
    else {
        drawTextWithBackground(display_image, "No Output Image",
            Point(thumbnail_area_x + 120, 370), Scalar(255, 255, 255), Scalar(100, 100, 100));
    }

    // Add dividing lines
    line(display_image, Point(350, 0), Point(350, DISPLAY_HEIGHT), Scalar(100, 100, 100), 2);
    line(display_image, Point(0, 260), Point(350, 260), Scalar(100, 100, 100), 1);

    return display_image;
}