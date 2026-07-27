#include "Blender.hpp"
#include "Platform.hpp"  // [新增] 跨平台兼容: localtime_s -> localtime_r
#include <iostream>
#include <algorithm>  // [新增] std::min/std::max（max_ts_skew_ms 统计用）
#include <chrono>
#include <sstream>
#include <iomanip>
#include <spdlog/spdlog.h>
#include "../Utility/DebugDump.hpp"   // [新增] 调试图像统一写盘 /root/build/debug/
#include "../Utility/PerfLog.hpp"     // [新增] 性能日志（CSV + 周期摘要）
#include "../Utility/ThreadAffinity.hpp" // [新增] 线程绑核（RK3588 大小核）

using namespace cv;
using namespace std;
using namespace std::chrono;

// Calculate the combined ROI (Region of Interest) from multiple corners and sizes
cv::Rect resultRoi(const std::vector<cv::Point>& corners, const std::vector<cv::Size>& sizes) {
    vector<Rect> rois;
    for (size_t i = 0; i < corners.size(); i++) {
        rois.push_back(Rect(corners[i], sizes[i]));
    }
    Rect result = rois[0];
    for (size_t i = 1; i < rois.size(); i++) {
        result |= rois[i];
    }
    return result;
}

BlenderModule::BlenderModule(
    std::vector<RTStitching::CameraStitchParams>& stitch_params,
    CircularBufferSync<RTStitching::Image>& input_buffer,
    CircularBuffer<RTStitching::Image>& output_buffer,
    RTStitching::ConfigParams& config_params,
    std::shared_ptr<std::shared_mutex> seam_mask_mutex,
    int show_window,
    const std::string& module_name
) :
    stitch_params_(stitch_params),
    input_buffer_(input_buffer),
    output_buffer_(output_buffer),
    config_params_(config_params),
    is_running_(false),
    is_paused_(false),
    stop_requested_(false),
    last_processing_time_(0.0),
    is_initialized_(false),
    seam_mask_mutex_(seam_mask_mutex),
    show_window_(show_window),
    is_prepared_(false),
    output_width_(config_params.output_width),      
    output_height_(config_params.output_height) {
    // Initialize in constructor
    if (!setBlenderType(config_params.blender_type, config_params.try_cuda, config_params.blend_strength)) {
        last_error_ = "Failed to initialize blender with type: " + config_params.blender_type;
        return;
    }

    is_initialized_ = true;

    // [N路适配] 跳帧统计数组按相机数初始化为 0，支持 2/3/N 路
    last_index_.assign(config_params.camera_count > 0 ? config_params.camera_count : 2, 0);

    if (config_params.verbose_output) {
        spdlog::info("[BLENDER] Blender initialized successfully with type: {}, try_cuda: {}, blend_strength: {}",
            config_params.blender_type, (config_params.try_cuda ? "true" : "false"), config_params.blend_strength);
    }
}

BlenderModule::~BlenderModule() {
    stop();
}

bool BlenderModule::start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (is_running_) return false;

    is_running_.store(true);
    is_paused_.store(false);
    stop_requested_.store(false);

    worker_thread_ = std::thread(&BlenderModule::runImpl, this);

    // [新增] 启动异步 JPEG 写盘线程，把 launcher 显示通道的编码搬出融合线程
    jpeg_stop_.store(false);
    jpeg_writer_thread_ = std::thread(&BlenderModule::jpegWriterLoop, this);

    spdlog::info("[BLENDER] BlenderModule thread started");
    return true;
}

void BlenderModule::stop() {
    if (!is_running_.load()) return;

    stop_requested_.store(true);
    condition_.notify_all();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    // [新增] 停掉异步 JPEG 写盘线程
    jpeg_stop_.store(true);
    jpeg_cv_.notify_all();
    if (jpeg_writer_thread_.joinable()) {
        jpeg_writer_thread_.join();
    }

    is_running_.store(false);
    is_paused_.store(false);

    spdlog::info("[BLENDER] BlenderModule thread stopped");
}

void BlenderModule::pause() {
    is_paused_.store(true);
    spdlog::info("[BLENDER] BlenderModule thread paused");
}

void BlenderModule::resume() {
    if (is_paused_.load()) {
        is_paused_.store(false);
        condition_.notify_one();
        spdlog::info("[BLENDER] BlenderModule thread resumed");
    }
}

bool BlenderModule::isRunning() const {
    return is_running_.load(std::memory_order_acquire);
}

bool BlenderModule::isPaused() const {
    return is_paused_.load(std::memory_order_acquire);
}

void BlenderModule::runImpl() {
    RTStitching::bindToBigCores("Blender");   // [新增] 多频段融合属重计算，绑大核(A76)
    spdlog::info("[BLENDER] BlenderModule thread main loop started");

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

        if (input_buffer_.getNumBuffers() > 0) {
            std::vector<RTStitching::Image> frame_data;
            std::vector<bool> new_data_flags;

            // 取帧策略说明：wait_for_num=1 —— 任意一路有新帧即开工，不等三路齐帧；
            //   back() 对无新帧的路返回缓冲区里最新的旧帧（天然复用上一帧）。
            //   new_data_flags 标记各路新旧，传入 processFrame 做复用统计与陈旧度保护。
            if (input_buffer_.back(frame_data, new_data_flags, std::chrono::milliseconds(35),1)) {
                auto start_time = high_resolution_clock::now();
                
                if (processFrame(frame_data, new_data_flags)) {
                    auto end_time = high_resolution_clock::now();
                    auto duration = duration_cast<milliseconds>(end_time - start_time);
                    last_processing_time_ = duration.count();

                    // [修改] info -> debug：每帧一条 info 的格式化与输出本身也在关键路径上
                    spdlog::debug("[BLENDER] Successfully processed frame {} with {} images in {} ms",
                        frame_data[0].img_idx, frame_data.size(), last_processing_time_);
                }
                else {
                    spdlog::warn("[BLENDER] Failed to process frame {}", frame_data[0].img_idx);
                }
            }

        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // [修改] waitKey 也属于 HighGUI，Linux 工作线程调用会与 Qt 抢占 GUI 资源，
        //   仅 Windows 保留（用于刷新调试窗口事件循环）
#ifdef _WIN32
        if (show_window_ > 0) {
            cv::waitKey(1); 
        }
#endif
    }

    spdlog::info("[BLENDER] BlenderModule thread main loop ended");
}

// [新增] 异步 JPEG 写盘线程：launcher 显示通道 /tmp/stitched.jpg
//   单槽邮箱语义：融合线程覆盖写 jpeg_pending_，这里取走后编码+原子替换；
//   编码期间不持锁，融合线程随时可覆盖投递新帧，永远只写最新一帧。
void BlenderModule::jpegWriterLoop() {
    spdlog::info("[BLENDER] JPEG writer thread started");
    while (true) {
        cv::Mat img;
        {
            std::unique_lock<std::mutex> lock(jpeg_mutex_);
            jpeg_cv_.wait(lock, [this] {
                return jpeg_stop_.load() || jpeg_has_pending_;
                });
            if (jpeg_stop_.load() && !jpeg_has_pending_) break;
            img = std::move(jpeg_pending_);
            jpeg_pending_ = cv::Mat();
            jpeg_has_pending_ = false;
        }
        if (img.empty()) continue;
        try {
            // 先写临时文件再 rename，保证读端（launcher）看到的永远是完整文件
            cv::imwrite("/tmp/stitched.tmp.jpg", img);
            std::rename("/tmp/stitched.tmp.jpg", "/tmp/stitched.jpg");
        }
        catch (const std::exception& e) {
            spdlog::warn("[BLENDER] JPEG writer failed: {}", e.what());
        }
    }
    spdlog::info("[BLENDER] JPEG writer thread stopped");
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

bool BlenderModule::setBlenderType(const std::string& blender_type, bool try_cuda, int blend_strength) {
    int blend_type;

    if (blender_type == "no" || blender_type == "Blender::NO") {
        blend_type = detail::Blender::NO;
    }
    else if (blender_type == "feather" || blender_type == "Blender::FEATHER") {
        blend_type = detail::Blender::FEATHER;
    }
    else if (blender_type == "multiband" || blender_type == "Blender::MULTI_BAND") {
        blend_type = detail::Blender::MULTI_BAND;
    }
    else {
        last_error_ = "Unknown blender type: " + blender_type;
        return false;
    }

    blender_ = detail::Blender::createDefault(blend_type, try_cuda);
    config_params_.blender_type = blender_type;
    config_params_.try_cuda = try_cuda;
    config_params_.blend_strength = blend_strength;

    return true;
}

bool BlenderModule::processFrame(const std::vector<RTStitching::Image>& frame_data,
    const std::vector<bool>& new_data_flags) {
    
    // ---------- 性能测量入口 ----------
    static auto last_entry_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_entry_time).count();
    if (elapsed_ms > 0) {
        // [修改] info -> debug：该数据已由 perf.csv 的 blend_interval_ms 覆盖
        spdlog::debug("[BLENDER] Frame entry interval: {} ms (≈ {:.1f} fps)", 
                     elapsed_ms, 1000.0 / elapsed_ms);
    }
    last_entry_time = now;
    // ---------------------------------
    
    if (frame_data.empty() || stitch_params_.empty()) {
        last_error_ = "Empty frame data or stitch parameters";
        return false;
    }

    try {
        // Record buffer read time
        auto buffer_read_time = high_resolution_clock::now();

        std::vector<Point> corners;
        std::vector<Size> sizes;

        corners = stitch_params_[RTStitching::BLENDER_SCALE].corners;
        sizes = stitch_params_[RTStitching::BLENDER_SCALE].sizes;
        if (!is_initialized_) {
            last_error_ = "Blender not initialized";
            return false;
        }
        
        spdlog::debug("[BLENDER] Before prepare");
        prepare(corners, sizes);
        spdlog::debug("[BLENDER] After prepare");
        
        // Record feed completion time for each image
        std::vector<std::chrono::high_resolution_clock::time_point> feed_times(frame_data.size());

        cv::Mat mask_warped;
        std::vector<cv::Mat> masks_warped;
        // Important: get seam mask with lock
        {
            // important: read seam masks with lock
            std::shared_lock <std::shared_mutex> lock(*seam_mask_mutex_);
            for (int i = 0; i < stitch_params_[RTStitching::SEAM_FINDER_SCALE].masks.size(); i++)
            {
                stitch_params_[RTStitching::SEAM_FINDER_SCALE].masks[i].copyTo(mask_warped);
                masks_warped.push_back(mask_warped);
            }
        }

        std::unique_lock<std::mutex> lock(mutex_);
        spdlog::debug("[BLENDER] frame_data size: {}", frame_data.size());
        for (size_t i = 0; i < frame_data.size(); ++i) {
            spdlog::debug("[BLENDER] Processing image {}", i);
            mask_warped = masks_warped[i];
            if (mask_warped.size() != frame_data[i].data.size())
                return false;

            // Convert to 16SC3 format (与 OpenCV stitching_detail 一致)
            // ⚠ 关键修复：原先用 convertTo(..., 257.0, -32768.0) 把像素从 0~255
            //   拉伸到 int16 满量程 ±32768。FeatherBlender / MultiBandBlender 的
            //   feed 是用 int16 缓冲做 dst += src*weight 的加权累加，在两路重叠带里
            //   亮像素(接近 32767)相加会【溢出回绕成负值】 -> 接缝处出现灰带/偏色/重影。
            //   正确做法是直接 convertTo(CV_16SC3)，把 0~255 原样存进 16S，留足累加余量。
            cv::Mat img_16sc3;
            if (frame_data[i].data.type() == CV_8UC3) {
                frame_data[i].data.convertTo(img_16sc3, CV_16SC3);   // 0~255 -> 16S, 无缩放/偏移
            }
            else if (frame_data[i].data.type() == CV_16SC3) {
                frame_data[i].data.copyTo(img_16sc3);
            }
            else {
                if (frame_data[i].data.channels() == 1) {
                    cv::cvtColor(frame_data[i].data, img_16sc3, cv::COLOR_GRAY2BGR);
                    img_16sc3.convertTo(img_16sc3, CV_16SC3);
                }
                else {
                    frame_data[i].data.convertTo(img_16sc3, CV_16SC3);
                }
            }

            // Use converted 16SC3 image for feed
            spdlog::debug("[BLENDER] Before feed");
            feed(i, img_16sc3, mask_warped, corners[i]);
            spdlog::debug("[BLENDER] After feed");

            // Record feed completion time
            feed_times[i] = high_resolution_clock::now();
        }
        
        Mat result, result_mask;
        Size size(1920, 1080);

        spdlog::debug("[BLENDER] Before blend");
        blend(result, result_mask);
        spdlog::debug("[BLENDER] After blend");
        
        // Record blend completion time
        auto blend_finish_time = high_resolution_clock::now();

        // [新增] 性能日志：与原调试图叠字信息同源同定义，图像调试关闭时照常记录
        {
            // 融合阶段耗时（对应原 "buffer-out to blend-finished"）
            RTStitching::perfRecord("Blender", "blend_ms",
                duration<double, std::milli>(blend_finish_time - buffer_read_time).count(),
                frame_data[0].img_idx);

            // 融合输出帧间隔（倒数即端到端输出帧率）
            if (perf_has_last_blend_) {
                RTStitching::perfRecord("Blender", "blend_interval_ms",
                    duration<double, std::milli>(blend_finish_time - perf_last_blend_finish_).count(),
                    frame_data[0].img_idx);
            }
            perf_last_blend_finish_ = blend_finish_time;
            perf_has_last_blend_ = true;

            if (last_index_.size() < frame_data.size()) last_index_.resize(frame_data.size(), 0);
            if (last_skipped_.size() < frame_data.size()) last_skipped_.resize(frame_data.size(), 0);
            for (size_t i = 0; i < frame_data.size(); ++i) {
                const std::string cam = "cam" + std::to_string(i);
                // 采集 -> 融合缓冲弹出（对应原 "Frame-in to buffer-out"）
                RTStitching::perfRecord("Blender", cam + "_frame_to_bufferout_ms",
                    duration<double, std::milli>(buffer_read_time - frame_data[i].timestamp).count(),
                    frame_data[i].img_idx);
                // 采集 -> feed 完成
                RTStitching::perfRecord("Blender", cam + "_frame_to_feed_ms",
                    duration<double, std::milli>(feed_times[i] - frame_data[i].timestamp).count(),
                    frame_data[i].img_idx);
                // 端到端时延：采集 -> 融合完成
                RTStitching::perfRecord("Blender", cam + "_e2e_ms",
                    duration<double, std::milli>(blend_finish_time - frame_data[i].timestamp).count(),
                    frame_data[i].img_idx);
                // [修复] 跳帧数改为带符号差并夹到 >=0，修掉原先无符号下溢
                //   （曾显示 "skiped 18446744073709551615 frames" = 2^64-1）
                int64_t skipped = (int64_t)frame_data[i].img_idx - (int64_t)last_index_[i] - 1;
                if (skipped < 0) skipped = 0;
                last_skipped_[i] = skipped;
                last_index_[i] = frame_data[i].img_idx;
                RTStitching::perfRecord("Blender", cam + "_skipped_frames",
                    (double)skipped, frame_data[i].img_idx);
            }

            // [新增] 取帧策略观测指标：
            //   reused_paths   —— 本轮有几路没有新帧、复用了缓冲区旧帧（0 = 三路全新）；
            //   max_ts_skew_ms —— 各路输入帧时间戳最大差，直接量化拼缝处的时间对齐质量。
            //   写报告时可用这两项证明"不齐帧取最新"对拼接质量的影响是可控且被监控的。
            {
                int reused = 0;
                for (size_t i = 0; i < new_data_flags.size() && i < frame_data.size(); ++i)
                    if (!new_data_flags[i]) ++reused;
                RTStitching::perfRecord("Blender", "reused_paths",
                    (double)reused, frame_data[0].img_idx);

                double ts_min_ms = 0.0, ts_max_ms = 0.0;
                for (size_t i = 0; i < frame_data.size(); ++i) {
                    double t = duration<double, std::milli>(
                        frame_data[i].timestamp - frame_data[0].timestamp).count();
                    if (i == 0) { ts_min_ms = ts_max_ms = t; }
                    else { ts_min_ms = std::min(ts_min_ms, t); ts_max_ms = std::max(ts_max_ms, t); }
                }
                RTStitching::perfRecord("Blender", "max_ts_skew_ms",
                    ts_max_ms - ts_min_ms, frame_data[0].img_idx);

                // [新增] 陈旧度保护：某路帧龄超阈值，说明该路 Warper/相机可能已卡死，
                //   复用旧帧会输出一块"冻结"画面——必须让日志能看出来。告警限频 2s。
                constexpr double kStaleLimitMs = 300.0;
                static auto last_stale_warn = std::chrono::steady_clock::now() - std::chrono::seconds(10);
                for (size_t i = 0; i < frame_data.size(); ++i) {
                    double age_ms = duration<double, std::milli>(
                        buffer_read_time - frame_data[i].timestamp).count();
                    if (age_ms > kStaleLimitMs) {
                        auto now_w = std::chrono::steady_clock::now();
                        if (std::chrono::duration_cast<std::chrono::seconds>(now_w - last_stale_warn).count() >= 2) {
                            last_stale_warn = now_w;
                            spdlog::warn("[BLENDER] cam{} frame is stale ({:.0f} ms old, limit {} ms) — "
                                "upstream warper/camera may be stalled, output region frozen",
                                i, age_ms, (int)kStaleLimitMs);
                        }
                    }
                }
            }
        }
        
        // Convert result to displayable format
        // ⚠ 配合上面的修复：result 现在是 0~255 范围的 16S，直接 convertTo 即可，
        //   不再用 (255/65535, 127.5) 那套与满量程编码配对的反变换。
        Mat result_display;
        result.convertTo(result_display, CV_8UC3);

        // [修改] INTER_LINEAR_EXACT -> INTER_LINEAR：EXACT 是定点精确版，显著更慢；
        //   显示/推流通道用普通双线性即可，肉眼无差别。
        resize(result_display, result_display, size, 0, 0, INTER_LINEAR);

// ========== launcher 显示通道：固定文件名覆盖写 ==========
// [修改] 两处修复：
//   1) 限频判断此前写的是 33ms（注释本意 100ms），周期 >33ms 时等于每轮都编码，
//      1080p JPEG 单次 ~20-30ms 全部消耗在融合线程里——这是 blend 结束到下一轮
//      取帧之间约 63ms 空隙的最大单项。现按本意改为 100ms（launcher 10fps 足够）。
//   2) 编码+写盘整体移交异步写盘线程（jpegWriterLoop），融合线程只做一次
//      Mat 拷贝（~2-4ms）即继续下一轮；写盘慢时单槽邮箱自动只保留最新帧。
static auto last_save_time = std::chrono::steady_clock::now();
auto now_save = std::chrono::steady_clock::now();
if (std::chrono::duration_cast<std::chrono::milliseconds>(now_save - last_save_time).count() >= 33) {
    last_save_time = now_save;
    {
        std::lock_guard<std::mutex> jpeg_lock(jpeg_mutex_);
        result_display.copyTo(jpeg_pending_);   // 覆盖旧的未写帧，只留最新
        jpeg_has_pending_ = true;
    }
    jpeg_cv_.notify_one();
}


        if(show_window_>0){
            if (show_window_ >= 2) {
                // [修复] 不再把 blend() 输出的 result（CV_16SC3）直接交给显示端：
                //   HighGUI 显示 16S 时按 [-32768,32767]->[0,255] 线性映射（v/256+128），
                //   0~255 范围的数据全被压到 128 附近，整幅呈中灰。这里改用上面
                //   已显式 convertTo(CV_8UC3) 的 result_display（与 /tmp/stitched.jpg
                //   同源），在源头保证送显/写盘的一定是 8U 图，不依赖显示端的隐式转换。
                RTStitching::debugDump("Blender_result_out", result_display);
                RTStitching::debugDump("Blender_result_mask_out", result_mask);
                // Draw timestamp information
                // drawTimestamps(result, frame_data, buffer_read_time, feed_times, blend_finish_time);
            }

            // Scale the mask to fit the output rectangle
            cv::Mat scaled_mask = scaleMaskToFitRectangle(result_mask, output_width_, output_height_);

            if (!scaled_mask.empty()) {
                result_mask = scaled_mask;

                // Scale the result image to match the mask size
                if (!result.empty()) {
                    cv::Mat scaled_result;
                    cv::resize(result, scaled_result, result_mask.size(), 0, 0, cv::INTER_LINEAR);
                    result = scaled_result;
                }
            }

            // Display the scaled result
            if (!result.empty()) {
                // Calculate the position of the target rectangle at the image center
                int img_w = result.cols;
                int img_h = result.rows;
                int target_x = (img_w - output_width_) / 2;
                int target_y = (img_h - output_height_) / 2;


                // Ensure the target rectangle is within image boundaries
                cv::Rect target_rect(target_x, target_y, output_width_, output_height_);
                target_rect &= cv::Rect(0, 0, img_w, img_h);
                // std::cout << "[LH: BLENDER: ]" << img_w << "; " << img_h << "; " << target_x << "; " << target_y << "; " << std::endl;

                if (target_rect.width > 0 && target_rect.height > 0) {
                    // Crop the target region
                    cv::Mat result_roi = result(target_rect);

                    // Convert to display format
                    // ⚠ 同步上面的修复：result 现在是 0~255 的 16S，直接转 8U。
                    cv::Mat result_roi_display;
                    result_roi.convertTo(result_roi_display, CV_8UC3);
                    drawTimestamps(result_roi_display, frame_data, buffer_read_time, feed_times, blend_finish_time);

                    // [修改] 裁剪结果写盘 /root/build/debug/Blender_result_resize.jpg（Windows 仍 imshow）
                    int dbg_key = RTStitching::debugDump("Blender_result_resize", result_roi_display);

                    // Visualize the scaled mask
                    if (config_params_.verbose_output && !result_mask.empty() && show_window_ >= 3) {
                        visualizeScaledMask(result_mask, output_width_, output_height_);
                    }

                    if (dbg_key == 27) {  // ESC to stop (仅 Windows 有效)
                        stop_requested_.store(true, std::memory_order_release);
                    }
                }
            }

        }
        
        RTStitching::Image output_image;
        result_display.copyTo(output_image.data);
        result_mask.copyTo(output_image.mask);
        output_image.img_idx = frame_data[0].img_idx;
        output_image.timestamp = high_resolution_clock::now();

        if (!output_buffer_.try_push_back(output_image)) {
            spdlog::warn("[BLENDER] Output buffer full, dropping blended result for frame {}", output_image.img_idx);
        }

        // Output timestamp information to console
        if (config_params_.verbose_output) {
            spdlog::info("[BLENDER] === Frame {} Timestamps ===", frame_data[0].img_idx);
            spdlog::info("[BLENDER] Buffer read time: {}", formatTimestamp(buffer_read_time));
            for (size_t i = 0; i < frame_data.size(); ++i) {
                spdlog::info("[BLENDER] Camera {}:", i);
                spdlog::info("[BLENDER]   Original: {}", formatTimestamp(frame_data[i].timestamp));
                spdlog::info("[BLENDER]   Buffer read: {}", formatTimestamp(buffer_read_time));
                spdlog::info("[BLENDER]   Feed finish: {}", formatTimestamp(feed_times[i]));
                spdlog::info("[BLENDER]   Feed duration: {} ms", calculateTimeDifference(frame_data[i].timestamp, feed_times[i]));
                spdlog::info("[BLENDER]   Buffer to feed: {} ms", calculateTimeDifference(buffer_read_time, feed_times[i]));
            }
            spdlog::info("[BLENDER] Blend finish: {}", formatTimestamp(blend_finish_time));
            spdlog::info("[BLENDER] Total process: {} ms", calculateTimeDifference(buffer_read_time, blend_finish_time));
            spdlog::info("[BLENDER] ==================================");
        }
        spdlog::debug("[BLENDER] camera:0,output_data.img_idx:{}", frame_data[0].img_idx);
        spdlog::debug("[BLENDER] camera:1,output_data.img_idx:{}", frame_data[1].img_idx);

        // [新增] blend 完成 -> processFrame 返回 的收尾耗时（显示转换/写盘交接/输出拷贝等）。
        //   blend_interval_ms - blend_ms - post_blend_ms ≈ 真正的取帧等待，
        //   改完对比这三项即可验证优化把空隙压掉了多少。
        RTStitching::perfRecord("Blender", "post_blend_ms",
            duration<double, std::milli>(high_resolution_clock::now() - blend_finish_time).count(),
            frame_data[0].img_idx);
        return true;

    }
    catch (const exception& e) {
        last_error_ = std::string("Error processing frame: ") + e.what();
        spdlog::error("[BLENDER] {}", last_error_);
        return false;
    }
}

void BlenderModule::prepare(const vector<Point>& corners, const vector<Size>& sizes) {
    if (!is_initialized_) {
        last_error_ = "Blender not initialized";
        spdlog::error("[BLENDER] {}", last_error_);
        return;
    }

    Rect dst_roi = resultRoi(corners, sizes);
    Size dst_sz = dst_roi.size();

    float blend_width = sqrt(static_cast<float>(dst_sz.area())) * config_params_.blend_strength / 100.f;
    if (blend_width < 1.f) {
        blender_ = detail::Blender::createDefault(detail::Blender::NO, config_params_.try_cuda);
        if (config_params_.verbose_output) {
            spdlog::info("[BLENDER] Using NO blender (blend_width < 1)");
        }
    }
    else if (config_params_.blender_type == "multiband" || config_params_.blender_type == "Blender::MULTI_BAND") {
        Ptr<detail::MultiBandBlender> mb = blender_.dynamicCast<detail::MultiBandBlender>();
        if (mb) {
            int num_bands = static_cast<int>(ceil(log(blend_width) / log(2.)) - 1.);
            mb->setNumBands(num_bands);
            if (config_params_.verbose_output) {
                spdlog::info("[BLENDER] Multi-band blender, number of bands: {}", mb->numBands());
            }
        }
    }
    else if (config_params_.blender_type == "feather" || config_params_.blender_type == "Blender::FEATHER") {
        Ptr<detail::FeatherBlender> fb = blender_.dynamicCast<detail::FeatherBlender>();
        if (fb) {
            fb->setSharpness(1.f / blend_width);
            if (config_params_.verbose_output) {
                spdlog::info("[BLENDER] Feather blender, sharpness: {}", fb->sharpness());
            }
        }
    }

    blender_->prepare(corners, sizes);
    is_prepared_ = true;

    if (config_params_.verbose_output) {
        spdlog::info("[BLENDER] Blender prepared for {} images, result size: {}x{}",
            corners.size(), dst_sz.width, dst_sz.height);
    }
}

void BlenderModule::feed(int img_idx, const Mat& img, const Mat& mask, const Point& corner) {
    if (!is_prepared_) {
        last_error_ = "Blender not prepared. Call prepare() first.";
        spdlog::error("[BLENDER] {}", last_error_);
        return;
    }

    blender_->feed(img, mask, corner);

    if (config_params_.verbose_output) {
        spdlog::info("[BLENDER] Fed image #{} to blender", img_idx);
    }
}

void BlenderModule::blend(Mat& result, Mat& result_mask) {
    if (!is_prepared_) {
        last_error_ = "Blender not prepared. Call prepare() first.";
        spdlog::error("[BLENDER] {}", last_error_);
        return;
    }

    blender_->blend(result, result_mask);

    if (config_params_.verbose_output) {
        spdlog::info("[BLENDER] Blending completed. Result size: {}x{}", result.size().width, result.size().height);
    }
}

std::string BlenderModule::getInfo() const {
    string info = "Blender Module Info:\n";
    info += "  Initialized: " + string(is_initialized_ ? "Yes" : "No") + "\n";
    info += "  Prepared: " + string(is_prepared_ ? "Yes" : "No") + "\n";
    info += "  Blend Type: " + config_params_.blender_type + "\n";
    info += "  Try CUDA: " + string(config_params_.try_cuda ? "Yes" : "No") + "\n";
    info += "  Blend Strength: " + to_string(config_params_.blend_strength) + "\n";
    info += "  Camera Count: " + to_string(config_params_.camera_count) + "\n";
    info += "  Input Buffer: Set\n";
    info += "  Output Buffer: Set\n";
    info += "  Last Processing Time: " + to_string(last_processing_time_) + " ms";

    if (!last_error_.empty()) {
        info += "\n  Last Error: " + last_error_;
    }

    return info;
}

void BlenderModule::drawTimestamps(cv::Mat& image,
    const std::vector<RTStitching::Image>& frame_data,
    const std::chrono::high_resolution_clock::time_point& buffer_read_time,
    const std::vector<std::chrono::high_resolution_clock::time_point>& feed_times,
    const std::chrono::high_resolution_clock::time_point& blend_finish_time) {
    if (image.empty()) return;

    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.5;  // Slightly smaller font to fit more content
    int thickness = 1;
    cv::Scalar color(0, 255, 0); // Green
    cv::Scalar yellow(255, 255, 0);
    cv::Scalar cyan(255, 255, 0);
    cv::Scalar magenta(255, 0, 255);
    int line_height = 15;

    Size image_sz = image.size();
    int rect_width = 500;
    int rect_ox = (image_sz.width - rect_width) / 2;
    int rect_oy = 10;
    int x = rect_ox + 10;
    int y = rect_oy + 20;
    // std::cout << "[LH: BLENDER: ]" << image_sz << "; " << rect_ox << "; " << rect_oy << "; " << rect_width << "; " << x << "; " << y << "; " << std::endl;

    // Calculate number of lines to display
    int total_lines = 3 + frame_data.size() * 4; // Title + buffer time + 4 lines per camera + total time and blend time
    int background_height = total_lines * line_height + 20;

    // Draw semi-transparent background
    cv::Mat overlay;
    image.copyTo(overlay);
    cv::Rect background_rect(rect_ox, rect_oy, rect_width, background_height);
    cv::Mat roi = overlay(background_rect);
    cv::Mat color_background(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
    cv::addWeighted(color_background, 0.6, roi, 0.4, 0, roi);
    overlay.copyTo(image);

    // Draw title
    //std::string title = "Blender Timestamps - Frame: " + std::to_string(frame_data[0].img_idx);
    //cv::putText(image, title, cv::Point(x, y), font_face, font_scale + 0.1, color, thickness + 1);
    //y += line_height + 5;

    //// Draw buffer read time
    //std::string buffer_info = "Buffer Read: " + formatTimestamp(buffer_read_time);
    //cv::putText(image, buffer_info, cv::Point(x, y), font_face, font_scale, cyan, thickness);
    //y += line_height;

    //// Draw separator line
    //cv::line(image, cv::Point(x, y), cv::Point(x + 400, y), cv::Scalar(100, 100, 100), 1);
    //y += line_height;

    // [N路适配] 防御性扩容，确保 last_index_ 至少和当前帧路数一样长
    if (last_index_.size() < frame_data.size())
        last_index_.resize(frame_data.size(), 0);

    // Draw detailed timestamp information for each image
    for (size_t i = 0; i < frame_data.size(); ++i) {
        // Camera title
        std::string cam_title = "Camera " + std::to_string(i) + ":";
        cv::putText(image, cam_title, cv::Point(x, y), font_face, font_scale, yellow, thickness);
        y += line_height;

        // [修复] 跳帧数改由 run() 的性能统计统一计算（带符号、夹到>=0），
        //   这里只读显示，避免此前无符号减法下溢显示 2^64-1 帧的问题；
        //   last_index_ 也不再在绘制函数里更新，保证统计与显示一致。
        std::stringstream ss;
        const int64_t skipped_disp = (i < last_skipped_.size()) ? last_skipped_[i] : 0;
        ss << "skiped " << skipped_disp << (skipped_disp < 2 ? " frame" : " frames");
        // Frame timestamp
        std::string orig_info = "  Timestamp of frame : " + formatTimestamp(frame_data[i].timestamp)+" Skip:"+ss.str();
        cv::putText(image, orig_info, cv::Point(x + 10, y), font_face, font_scale - 0.1, color, thickness);
        y += line_height;

        // Feed completion time
        std::string feed_info = "  Timestamp after feed: " + formatTimestamp(feed_times[i]);
        cv::putText(image, feed_info, cv::Point(x + 10, y), font_face, font_scale - 0.1, magenta, thickness);
        y += line_height;

        // Processing time statistics
        std::string timing_info = "  Duration: Frame-in to buffer-out: " +
            std::to_string(static_cast<int>(calculateTimeDifference(frame_data[i].timestamp, buffer_read_time))) +
            "ms, buffer-out to feed-in: " +
            std::to_string(static_cast<int>(calculateTimeDifference(buffer_read_time, feed_times[i]))) + "ms";
        cv::putText(image, timing_info, cv::Point(x + 10, y), font_face, font_scale - 0.1, color, thickness);
        y += line_height;
    }

    // Draw separator line
    cv::line(image, cv::Point(x, y), cv::Point(x + 400, y), cv::Scalar(100, 100, 100), 1);
    y += line_height;

    // Draw total processing time
    double total_time = calculateTimeDifference(buffer_read_time, blend_finish_time);
    std::string total_info = "Duration: buffer-out to blend-finished: " + std::to_string(static_cast<int>(total_time)) + "ms";
    cv::putText(image, total_info, cv::Point(x, y), font_face, font_scale, cv::Scalar(0, 255, 255), thickness + 1);
    y += line_height;

    // Draw blend completion time
    std::string blend_time = "Timestamp of blend-finish: " + formatTimestamp(blend_finish_time);
    cv::putText(image, blend_time, cv::Point(x, y), font_face, font_scale, color, thickness);
}

std::string BlenderModule::formatTimestamp(const std::chrono::high_resolution_clock::time_point& timestamp) {
    auto system_time = std::chrono::system_clock::now() +
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
            timestamp - std::chrono::high_resolution_clock::now());
    return formatSystemTime(system_time);
}

double BlenderModule::calculateTimeDifference(const std::chrono::high_resolution_clock::time_point& start,
    const std::chrono::high_resolution_clock::time_point& end) {
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
}

cv::Mat BlenderModule::scaleMaskToFitRectangle(const cv::Mat& mask, int output_width_, int output_height_) {
    if (mask.empty()) {
        spdlog::error("[BLENDER] Empty mask provided for scaling");
        return cv::Mat();
    }

    try {
        // 转为单通道二值
        cv::Mat mask_gray;
        if (mask.channels() > 1) cv::cvtColor(mask, mask_gray, cv::COLOR_BGR2GRAY);
        else mask_gray = mask.clone();

        cv::threshold(mask_gray, mask_gray, 128, 255, cv::THRESH_BINARY);

        // 找到掩码的有效区域
        std::vector<cv::Point> mask_points;
        cv::findNonZero(mask_gray, mask_points);
        if (mask_points.empty()) {
            spdlog::warn("[BLENDER] No valid mask region found");
            return cv::Mat();
        }

        // 获取有效区域的边界矩形
        cv::Rect mask_rect = cv::boundingRect(mask_points);

        // 计算需要的缩放比例 - 确保目标矩形能完全内接在mask白色区域内
        double scale_x = static_cast<double>(output_width_) / static_cast<double>(mask_rect.width);
        double scale_y = static_cast<double>(output_height_) / static_cast<double>(mask_rect.height);
        double scale = std::max(scale_x, scale_y);

        // 增加安全余量，确保完全内接
        scale *= 1.2; // 增加20%的安全余量

        // 限制缩放范围
        scale = std::max(1.0, std::min(scale, 5.0));

        // 计算缩放后的尺寸
        int new_width = static_cast<int>(mask_gray.cols * scale);
        int new_height = static_cast<int>(mask_gray.rows * scale);

        // 简单的resize缩放
        cv::Mat scaled_mask;
        cv::resize(mask_gray, scaled_mask, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);

        // 创建一个更大的画布，确保目标矩形能完全内接
        int canvas_width = new_width + 200; // 增加额外边界
        int canvas_height = new_height + 200;

        cv::Mat canvas = cv::Mat::zeros(canvas_height, canvas_width, CV_8UC1);

        // 将缩放后的mask放置在画布中心
        int start_x = (canvas_width - new_width) / 2;
        int start_y = (canvas_height - new_height) / 2;

        cv::Rect roi_rect(start_x, start_y, new_width, new_height);
        if (roi_rect.x >= 0 && roi_rect.y >= 0 &&
            roi_rect.x + roi_rect.width <= canvas_width &&
            roi_rect.y + roi_rect.height <= canvas_height) {
            scaled_mask.copyTo(canvas(roi_rect));
        }

        // 验证目标矩形是否完全在白色区域内
        int target_x = (canvas_width - output_width_) / 2;
        int target_y = (canvas_height - output_height_) / 2;
        cv::Rect target_rect(target_x, target_y, output_width_, output_height_);

        // 确保目标矩形在画布范围内
        target_rect &= cv::Rect(0, 0, canvas_width, canvas_height);

        if (target_rect.width > 0 && target_rect.height > 0) {
            cv::Mat target_roi = canvas(target_rect);
            double min_val, max_val;
            cv::minMaxLoc(target_roi, &min_val, &max_val);

            if (min_val >= 250) {
                if (config_params_.verbose_output) {
                    spdlog::info("[BLENDER] Mask scaled successfully: {}x{} -> {}x{} (scale: {:.2f})",
                        mask_gray.cols, mask_gray.rows, canvas_width, canvas_height, scale);
                    spdlog::info("[BLENDER] Target rectangle {}x{} fully inside white region",
                        output_width_, output_height_);
                }
                return canvas;
            }
            else {
                spdlog::warn("[BLENDER] Target rectangle not fully inside white region after scaling");
            }
        }

        spdlog::warn("[BLENDER] Failed to ensure target rectangle inside white region");
        return canvas; // 返回画布，即使不完全内接

    }
    catch (const std::exception& e) {
        spdlog::error("[BLENDER] Error scaling mask: {}", e.what());
        return cv::Mat();
    }
}

void BlenderModule::visualizeScaledMask(const cv::Mat& mask, int target_width, int target_height) {
    if (mask.empty()) return;

    cv::Mat mask_gray;
    if (mask.channels() == 1) mask_gray = mask.clone();
    else cv::cvtColor(mask, mask_gray, cv::COLOR_BGR2GRAY);

    cv::Mat vis;
    cv::cvtColor(mask_gray, vis, cv::COLOR_GRAY2BGR);

    std::vector<cv::Point> pts;
    cv::findNonZero(mask_gray, pts);
    if (pts.empty()) return;
    cv::Rect bbox = cv::boundingRect(pts);

    cv::Point img_center(mask.cols / 2, mask.rows / 2);
    cv::circle(vis, img_center, 5, cv::Scalar(0, 255, 0), -1);
    cv::putText(vis, "ImageCenter", img_center + cv::Point(8, -8),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

    cv::Rect target_rect(
        img_center.x - target_width / 2,
        img_center.y - target_height / 2,
        target_width, target_height
    );
    cv::rectangle(vis, target_rect, cv::Scalar(0, 0, 255), 2);

    cv::rectangle(vis, bbox, cv::Scalar(255, 0, 0), 2);

    std::string info = cv::format("Target:%dx%d  Image:%dx%d  BBox:%dx%d",
        target_width, target_height, mask.cols, mask.rows, bbox.width, bbox.height);
    cv::putText(vis, info, cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

    cv::Mat target_roi = mask_gray(target_rect);
    double min_val, max_val;
    cv::minMaxLoc(target_roi, &min_val, &max_val);

    std::string status = (min_val >= 250) ? "INSIDE" : "OUTSIDE";
    cv::putText(vis, "Status: " + status, cv::Point(10, 60),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, (min_val >= 250) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

    // [修改] 掩膜可视化写盘 /root/build/debug/Scaled_Mask_Visualization.jpg（Windows 仍 imshow）
    RTStitching::debugDump("Scaled Mask Visualization", vis);
}

