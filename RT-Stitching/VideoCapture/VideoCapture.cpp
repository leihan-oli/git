// VideoCapture.cpp
#include "VideoCapture.hpp"   // 已 include Platform.hpp
#include <iostream>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <memory>
#include <sstream>
#include <iomanip>
#include <ctime>

#include <spdlog/spdlog.h>
#include "../Utility/DebugDump.hpp"   // [新增] 调试图像统一写盘 /root/build/debug/
#include "../Utility/PerfLog.hpp"     // [新增] 性能日志（CSV + 周期摘要）
#include "../Utility/ThreadAffinity.hpp" // [新增] 线程绑核（RK3588 大小核）

// [新增] 旁路(特征/曝光/接缝)抽帧比例：每 N 帧去畸变并推送一帧。
//   30fps 下 N=5 即旁路 6fps，远高于接缝 1Hz 的消费速率。
#ifndef RT_SIDECHANNEL_DECIMATE
#define RT_SIDECHANNEL_DECIMATE 5
#endif

// ============================================================================
// [新增] 旁路异步工人：把 1/N 抽帧的"去畸变 + 特征/曝光/接缝推送"移出采集节拍。
//   实测(perf.csv 按 idx%5 分组)内联执行时，每第 5 帧的采集间隔被拉到
//   ~134ms，整路帧率被压到 ~19fps。改为工人线程后，采集线程只做一次
//   Mat 浅拷贝交接（微秒级），节拍完全由相机供帧决定。
//   利用"每路相机独占一个采集线程"的事实，用 thread_local 做到每路一个
//   工人，随采集线程退出自动 join，无需修改头文件。
// ============================================================================
namespace {
struct SideChannelWorker {
    std::thread th;
    std::mutex m;
    std::condition_variable cv;
    RTStitching::Image pending;
    bool has = false;
    bool quit = false;
    ~SideChannelWorker() {
        { std::lock_guard<std::mutex> lk(m); quit = true; }
        cv.notify_all();
        if (th.joinable()) th.join();
    }
};
} // namespace



// ---------- Static Dummy objects for safe initialization of invalid references ----------
static CircularBuffer<RTStitching::Image> dummy_output_buffer(1);
static CircularBufferSync<RTStitching::Image> dummy_sync_buffer(1, 1);

// ---------- Constructor ----------
VideoCapture::VideoCapture(
    bool show_window,
    bool record_video,
    const std::string& module_name)
    : camera_info_(),
    output_buffer_(dummy_output_buffer),
    sync_buffer_to_camest_(dummy_sync_buffer),
    sync_buffer_to_camest_index_(0),
    sync_buffer_to_expest_(dummy_sync_buffer),
    sync_buffer_to_expest_index_(0),
    sync_buffer_to_seamfinder_(dummy_sync_buffer),
    sync_buffer_to_seamfinder_index_(0),
    worker_thread_(),
    is_running_(false),
    is_paused_(false),
    stop_requested_(false),
    show_window_(show_window),
    record_video_(record_video),
    module_name_(module_name),
    cap_(),
    need_resize_(false),
    image_frame_(),
    frame_index_(0),
    cm_dev_(nullptr),
    cm_dll_handle_(nullptr),
    rtsp_cx_initialized_(false),
    recording_initialized_(false),
    recorded_frames_(0),
    stop_record_(false),
    target_interval_ms_(1000 / camera_info_.fps),
    first_frame_(true)
{
    if (module_name_.empty())
        module_name_ = "VideoCapture_NoOutput";
}

VideoCapture::VideoCapture(
    CircularBuffer<RTStitching::Image>& output_buffer,
    bool show_window,
    bool record_video,
    const std::string& module_name)
    : camera_info_(),
    output_buffer_(output_buffer),
    sync_buffer_to_camest_(dummy_sync_buffer),
    sync_buffer_to_camest_index_(0),
    sync_buffer_to_expest_(dummy_sync_buffer),
    sync_buffer_to_expest_index_(0),
    sync_buffer_to_seamfinder_(dummy_sync_buffer),
    sync_buffer_to_seamfinder_index_(0),
    worker_thread_(),
    is_running_(false),
    is_paused_(false),
    stop_requested_(false),
    show_window_(show_window),
    record_video_(record_video),
    module_name_(module_name),
    cap_(),
    need_resize_(false),
    image_frame_(),
    frame_index_(0),
    cm_dev_(nullptr),
    cm_dll_handle_(nullptr),
    rtsp_cx_initialized_(false),
    recording_initialized_(false),
    recorded_frames_(0),
    stop_record_(false),
    target_interval_ms_(1000 / camera_info_.fps),
    first_frame_(true)
{
    if (module_name_.empty())
        module_name_ = "VideoCapture_NoOutput";
}

VideoCapture::VideoCapture(
    RTStitching::CameraInfo camera_info,
    const RTStitching::CameraParams& camera_params,   // [新增]
    CircularBuffer<RTStitching::Image>& output_buffer,
    CircularBufferSync<RTStitching::Image>& sync_buffer_to_camest,
    size_t sync_buffer_to_camest_index,
    CircularBufferSync<RTStitching::Image>& sync_buffer_to_expest,
    size_t sync_buffer_to_expest_index,
    CircularBufferSync<RTStitching::Image>& sync_buffer_to_seamfinder,
    size_t sync_buffer_to_seamfinder_index,
    bool show_window,
    bool record_video,
    const std::string& module_name)
    : camera_info_(camera_info),
    camera_params_(camera_params),                    // [新增]
    output_buffer_(output_buffer),
    sync_buffer_to_camest_(sync_buffer_to_camest),
    sync_buffer_to_camest_index_(sync_buffer_to_camest_index),
    sync_buffer_to_expest_(sync_buffer_to_expest),
    sync_buffer_to_expest_index_(sync_buffer_to_expest_index),
    sync_buffer_to_seamfinder_(sync_buffer_to_seamfinder),
    sync_buffer_to_seamfinder_index_(sync_buffer_to_seamfinder_index),
    worker_thread_(),
    is_running_(false),
    is_paused_(false),
    stop_requested_(false),
    show_window_(show_window),
    record_video_(record_video),
    module_name_(module_name),
    cap_(),
    need_resize_(false),
    image_frame_(),
    frame_index_(0),
    cm_dev_(nullptr),
    cm_dll_handle_(nullptr),
    rtsp_cx_initialized_(false),
    recording_initialized_(false),
    recorded_frames_(0),
    stop_record_(false),
    target_interval_ms_(1000 / camera_info_.fps),
    first_frame_(true)
{
    if (module_name_.empty())
        module_name_ = "VideoCapture_" + std::to_string(camera_info_.video_index);

    // [新增] 判断是否需要去畸变：开关打开 且 D 非空非全零
    undistort_enabled_ = false;
    if (camera_info_.undistort && !camera_params_.D.empty()) {
        if (cv::countNonZero(camera_params_.D != 0.0) > 0 &&
            !camera_params_.K.empty()) {
            undistort_enabled_ = true;
            spdlog::info("[VIDEOCAPTURE] [{}] Undistortion ENABLED.", module_name_);
        }
    }
    if (!undistort_enabled_) {
        spdlog::info("[VIDEOCAPTURE] [{}] Undistortion disabled (flag off or D is zero).",
                     module_name_);
    }
}

VideoCapture::~VideoCapture() {
    stop();
    cleanupRtspCx();
}

// =====================================================================
// RTSP_CX (CmPlay.dll) 相关实现
//   Windows: 通过 LoadLibrary 加载 CmPlay.dll
//   Linux:   该 SDK 不可用，自动 fallback 到 cv::VideoCapture + FFMPEG
// =====================================================================
bool VideoCapture::initRtspCx() {
#ifdef _WIN32
    // ----- Windows: 走原有 CmPlay.dll 路径 -----
    cm_dll_handle_ = LoadLibraryA("CmPlay.dll");
    if (!cm_dll_handle_) {
        spdlog::error("[VIDEOCAPTURE] [{}] Cannot load CmPlay.dll", module_name_);
        return false;
    }
    std::string ip_port = camera_info_.url.substr(7, camera_info_.url.find_last_of('/') - 7);
    int channel = std::stoi(camera_info_.url.substr(camera_info_.url.find_last_of('/') + 3));

    spdlog::info("[VIDEOCAPTURE] [{}] Extracted IP:Port: {}, Channel: {}", module_name_, ip_port, channel);

    CMP_InitDev = (void* (*)(const char*, int))GetProcAddress(cm_dll_handle_, "CMP_InitDev");
    CMP_UnInitDev = (void* (*)(void*))GetProcAddress(cm_dll_handle_, "CMP_UnInitDev");
    CMP_OpenDevMedia = (int (*)(void*, int, void*))GetProcAddress(cm_dll_handle_, "CMP_OpenDevMedia");
    CMP_CloseDevMedia = (int (*)(void*, int))GetProcAddress(cm_dll_handle_, "CMP_CloseDevMedia");
    CMP_SetMediaCallbackFunc = (int (*)(void*, CMPStruct*))GetProcAddress(cm_dll_handle_, "CMP_SetMediaCallbackFunc");

    if (!CMP_InitDev || !CMP_UnInitDev || !CMP_OpenDevMedia || !CMP_CloseDevMedia || !CMP_SetMediaCallbackFunc) {
        spdlog::error("[VIDEOCAPTURE] [{}] Cannot get all DLL functions", module_name_);
        FreeLibrary(cm_dll_handle_);
        cm_dll_handle_ = nullptr;
        return false;
    }

    cm_dev_ = CMP_InitDev(ip_port.c_str(), 1);
    if (!cm_dev_) {
        spdlog::error("[VIDEOCAPTURE] [{}] CMP_InitDev failed for {}", module_name_, camera_info_.url);
        FreeLibrary(cm_dll_handle_);
        cm_dll_handle_ = nullptr;
        return false;
    }

    CMPStruct callback_struct = { 2, 1, this, &VideoCapture::rtspCxDataCallback };
    if (CMP_SetMediaCallbackFunc(cm_dev_, &callback_struct) != 0) {
        spdlog::error("[VIDEOCAPTURE] [{}] CMP_SetMediaCallbackFunc failed", module_name_);
        CMP_UnInitDev(cm_dev_);
        cm_dev_ = nullptr;
        FreeLibrary(cm_dll_handle_);
        cm_dll_handle_ = nullptr;
        return false;
    }

    if (CMP_OpenDevMedia(cm_dev_, channel, nullptr) != 0) {
        spdlog::error("[VIDEOCAPTURE] [{}] CMP_OpenDevMedia failed for channel 0", module_name_);
        CMP_UnInitDev(cm_dev_);
        cm_dev_ = nullptr;
        FreeLibrary(cm_dll_handle_);
        cm_dll_handle_ = nullptr;
        return false;
    }

    rtsp_cx_initialized_ = true;
    spdlog::info("[VIDEOCAPTURE] [{}] RTSP_CX initialized: {}", module_name_, camera_info_.url);
    return true;
#else
    // ----- Linux: CmPlay.dll 不可用，fallback 到标准 RTSP -----
    spdlog::warn("[VIDEOCAPTURE] [{}] rtsp_cx (CmPlay.dll) not supported on Linux. "
                 "Falling back to FFMPEG-backed RTSP: {}",
                 module_name_, camera_info_.url);
    bool ok = cap_.open(camera_info_.url, cv::CAP_FFMPEG);
    if (!ok) {
        // 退一步：让 OpenCV 自己挑后端
        ok = cap_.open(camera_info_.url);
    }
    if (ok) {
        // 注意：fallback 之后这一路其实走的是普通 cap_ 的 grab/retrieve，
        // 而不是 processRtspCxFrame()。下面 run() 里有判断兼容。
        rtsp_cx_initialized_ = true;
        need_resize_ = true;
        spdlog::info("[VIDEOCAPTURE] [{}] RTSP_CX fallback opened via OpenCV: {}",
                     module_name_, camera_info_.url);
    }
    return ok;
#endif
}

void VideoCapture::cleanupRtspCx() {
#ifdef _WIN32
    if (cm_dev_) {
        if (CMP_CloseDevMedia) {
            try {
                int channel = std::stoi(camera_info_.url.substr(camera_info_.url.find_last_of('/') + 3));
                CMP_CloseDevMedia(cm_dev_, channel);
            } catch (...) {}
        }
        if (CMP_UnInitDev) {
            CMP_UnInitDev(cm_dev_);
        }
        cm_dev_ = nullptr;
    }

    if (cm_dll_handle_) {
        FreeLibrary(cm_dll_handle_);
        cm_dll_handle_ = nullptr;
    }
#else
    // Linux: 没有 DLL 句柄要释放；如果是 fallback 模式 cap_ 由后面的 release 负责
    cm_dev_ = nullptr;
    cm_dll_handle_ = nullptr;
#endif

    rtsp_cx_initialized_ = false;

    CMP_InitDev = nullptr;
    CMP_UnInitDev = nullptr;
    CMP_OpenDevMedia = nullptr;
    CMP_CloseDevMedia = nullptr;
    CMP_SetMediaCallbackFunc = nullptr;
}

void CALLBACK VideoCapture::rtspCxDataCallback(int iCh, int iFlag, void** data, int iW, int iH, void* user) {
    // [说明] 该回调只会被 CmPlay.dll 调用（仅 Windows 下 initRtspCx 成功时）
    // Linux 下永远不会触发，但函数体仍需存在以满足类的 vtable / 链接需求
    VideoCapture* capture = static_cast<VideoCapture*>(user);
    if (capture) {
        capture->onRtspCxFrameReceived(iCh, iFlag, data, iW, iH);
    }
}

void VideoCapture::onRtspCxFrameReceived(int iCh, int iFlag, void** data, int iW, int iH) {
    if (!data || !data[0] || iW <= 0 || iH <= 0) return;

    try {
        std::lock_guard<std::mutex> lock(cm_mtx_);
        cv::Mat rgb_frame(iH, iW, CV_8UC3, data[0]);
        cv::cvtColor(rgb_frame, cm_frame_, cv::COLOR_RGB2BGR);
    }
    catch (const std::exception& e) {
        spdlog::error("[VIDEOCAPTURE] [{}] RTSP_CX frame processing error: {}", module_name_, e.what());
    }
    catch (...) {
        spdlog::error("[VIDEOCAPTURE] [{}] RTSP_CX frame processing unknown error", module_name_);
    }
}

void VideoCapture::processRtspCxFrame() {
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(cm_mtx_);
        if (cm_frame_.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (first_frame_) {
                last_frame_time_ = std::chrono::steady_clock::now();
                first_frame_ = false;
            }
            return;
        }

        if (cm_frame_.cols != camera_info_.width || cm_frame_.rows != camera_info_.height) {
            cv::resize(cm_frame_, frame, cv::Size(camera_info_.width, camera_info_.height));
        }
        else {
            frame = cm_frame_.clone();
        }
    }

    RTStitching::Image img;
    img.data = frame;
    img.timestamp = std::chrono::high_resolution_clock::now();
    img.img_idx = frame_index_++;

    // [修改] 热路径(->Warper)推原始帧：去畸变已并入 Warper 联合映射表，
    //   采集线程不再做逐帧 remap。
    try { output_buffer_.push_back(img); } catch (...) {}

    // [修改] 特征/曝光/接缝旁路仍需要与 Warper 输出同几何的去畸变图像，
    //   但均为低频消费者（接缝~1Hz、特征一次性），按 1/RT_SIDECHANNEL_DECIMATE
    //   抽帧去畸变即可；其余帧不推旁路（CircularBufferSync 消费端取最新，无影响）。
    if (!undistort_enabled_) {
        try { sync_buffer_to_camest_.push_back(sync_buffer_to_camest_index_, img); } catch (...) {}
        try { sync_buffer_to_expest_.push_back(sync_buffer_to_expest_index_, img); } catch (...) {}
        try { sync_buffer_to_seamfinder_.push_back(sync_buffer_to_seamfinder_index_, img); } catch (...) {}
    }
    else if ((img.img_idx % RT_SIDECHANNEL_DECIMATE) == 0) {
        RTStitching::Image side = img;
        side.data = img.data.clone();
        applyUndistort(side.data);
        try { sync_buffer_to_camest_.push_back(sync_buffer_to_camest_index_, side); } catch (...) {}
        try { sync_buffer_to_expest_.push_back(sync_buffer_to_expest_index_, side); } catch (...) {}
        try { sync_buffer_to_seamfinder_.push_back(sync_buffer_to_seamfinder_index_, side); } catch (...) {}
    }

    if (record_video_ && !stop_record_) {
        if (!recording_initialized_) {
            initializeRecording(frame);
        }
        writeVideoFrame(frame);
    }

    if (show_window_) {
        std::string timestamp_str = formatTimestamp(img.timestamp);
        cv::putText(frame, timestamp_str,
            cv::Point(15, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2);
        // [修改] Linux 上 imshow 与 Qt 抢占 GUI 资源，改为限频原子写盘到
        //   /root/build/debug/<module_name_>.jpg（Windows 仍走 imshow）
        if (RTStitching::debugDump(module_name_, frame) == 27) {
            stop_requested_.store(true);
        }
    }
}


void VideoCapture::start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (is_running_) return;

    stop_requested_.store(false, std::memory_order_release);
    is_paused_.store(false, std::memory_order_release);

    first_frame_ = true;
    target_interval_ms_ = 1000 / camera_info_.fps;

    spdlog::info("[VIDEOCAPTURE] [{}] Target FPS: {}, Interval: {}ms",
                 module_name_, camera_info_.fps, target_interval_ms_);

    bool ok = false;

    if (camera_info_.type.empty() || camera_info_.type == "WITHOUT_INIT") {
        camera_info_.type = "camera";
        camera_info_.url = "0";
        camera_info_.width = 640;
        camera_info_.height = 480;
    }

    if (camera_info_.type == "camera") {
        int dev_id = 0;
        try {
            dev_id = std::stoi(camera_info_.url);
        }
        catch (...) {
            spdlog::error("[VIDEOCAPTURE] [{}] Invalid camera URL: {}", module_name_, camera_info_.url);
            is_running_.store(false);
            return;
        }
#if defined(_WIN32)
        ok = cap_.open(dev_id, cv::CAP_MSMF);
#else
        // [修改] Linux/RK3588 上 USB 摄像头走 V4L2
        ok = cap_.open(dev_id, cv::CAP_V4L2);
        if (!ok) {
            // 兜底：让 OpenCV 自己选后端
            ok = cap_.open(dev_id);
        }
#endif
    }
    else if (camera_info_.type == "video" || camera_info_.type == "rtsp") {
        ok = cap_.open(camera_info_.url, cv::CAP_FFMPEG);
        if (!ok) ok = cap_.open(camera_info_.url);
    }
    else if (camera_info_.type == "image") {
        image_frame_ = cv::imread(camera_info_.url, cv::IMREAD_COLOR);
        if (image_frame_.empty()) {
            spdlog::error("[VIDEOCAPTURE] [{}] Cannot load image: {}", module_name_, camera_info_.url);
            is_running_.store(false);
            return;
        }
        if (image_frame_.cols != camera_info_.width || image_frame_.rows != camera_info_.height) {
            cv::resize(image_frame_, image_frame_, cv::Size(camera_info_.width, camera_info_.height));
        }
        ok = true;
        spdlog::info("[VIDEOCAPTURE] [{}] Loaded image: {} ({})",
                     module_name_, camera_info_.url, image_frame_.cols, image_frame_.rows);
    }
    else if (camera_info_.type == "rtsp_cx") {
        ok = initRtspCx();
        if (ok) {
            need_resize_ = true;
            spdlog::info("[VIDEOCAPTURE] [{}] Opened rtsp_cx: {}", module_name_, camera_info_.url);
        }
    }
    else {
        spdlog::error("[VIDEOCAPTURE] [{}] Unsupported type: {}", module_name_, camera_info_.type);
        is_running_.store(false);
        return;
    }

    if (!ok) {
        spdlog::error("[VIDEOCAPTURE] [{}] Cannot open video source: {}", module_name_, camera_info_.url);
        is_running_.store(false);
        return;
    }
    if (camera_info_.type != "image" && cap_.isOpened()) {
        // 强制使用 MJPEG 格式（必须最先设置）
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    // 再设置分辨率
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_info_.width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_info_.height);
        // [新增] 显式请求帧率 + 缩小驱动环形缓冲（降低陈旧帧时延）
        cap_.set(cv::CAP_PROP_FPS, camera_info_.fps > 0 ? camera_info_.fps : 30);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        int actual_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int actual_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        need_resize_ = (actual_w != camera_info_.width || actual_h != camera_info_.height);

        // [新增] 回读驱动实际生效的格式与帧率 —— cap_.set() 失败是静默的，
        //   若这里打印的 FOURCC 仍为 YUYV，说明 MJPG 没生效，USB2 带宽
        //   会把 720p 压到 ~8fps；FPS 同理，以驱动回读值为准。
        const int fcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
        const char fcc_str[5] = { (char)(fcc & 0xFF), (char)((fcc >> 8) & 0xFF),
                                  (char)((fcc >> 16) & 0xFF), (char)((fcc >> 24) & 0xFF), '\0' };
        spdlog::info("[VIDEOCAPTURE] [{}] Opened source: {} ({}x{}), driver FOURCC={}, FPS={}",
                     module_name_, camera_info_.url, actual_w, actual_h,
                     fcc_str, cap_.get(cv::CAP_PROP_FPS));
    }

    try {
        worker_thread_ = std::thread(&VideoCapture::runImpl, this);
    }
    catch (const std::exception& e) {
        spdlog::error("[VIDEOCAPTURE] [{}] Failed to start thread: {}", module_name_, e.what());
        stop_requested_.store(true, std::memory_order_release);
        is_running_.store(false);
        if (cap_.isOpened()) cap_.release();
    }
}

void VideoCapture::stop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_running_) return;

    stop_requested_.store(true, std::memory_order_release);
    cv_.notify_all();

    if (worker_thread_.joinable()) worker_thread_.join();

    stopRecording();

    is_running_ = false;
    is_paused_ = false;
    stop_requested_ = false;
}

void VideoCapture::pause() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_paused_.load()) is_paused_.store(true, std::memory_order_release);
}

void VideoCapture::resume() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_paused_.load()) {
        is_paused_.store(false, std::memory_order_release);
        last_frame_time_ = std::chrono::steady_clock::now();
        cv_.notify_one();
    }
}

bool VideoCapture::isRunning() const { return is_running_; }
bool VideoCapture::isPaused() const { return is_paused_.load(); }

bool VideoCapture::stopRequested() const noexcept {
    return stop_requested_.load(std::memory_order_acquire);
}

// =====================================================================
// [新增] 去畸变实现
//   - buildUndistortMaps: 用 K、D 预计算一次 remap 映射表
//       newCameraMatrix = K（保持内参不变），确保下游 Warper 用同一个 K 时几何严格对齐
//   - applyUndistort:     每帧用预计算映射表做 remap（实时高效）
// =====================================================================
void VideoCapture::buildUndistortMaps(const cv::Size& frame_size) {
    try {
        cv::Mat K, D;
        camera_params_.K.convertTo(K, CV_64F);
        camera_params_.D.convertTo(D, CV_64F);

        // newCameraMatrix 取 K 本身：去畸变后图像保持与标定一致的内参，
        // 与 Warper::doWarp 里使用的 K 完全一致，避免引入新的错位。
        cv::initUndistortRectifyMap(
            K, D, cv::Mat(),          // R = I（不做立体校正旋转）
            K,                        // newCameraMatrix = K
            frame_size,
            CV_16SC2,                 // 定点映射，remap 更快
            undistort_map1_, undistort_map2_);

        undistort_size_  = frame_size;
        undistort_ready_ = true;
        spdlog::info("[VIDEOCAPTURE] [{}] Undistort maps built for {}x{}",
                     module_name_, frame_size.width, frame_size.height);
    }
    catch (const cv::Exception& e) {
        undistort_ready_   = false;
        undistort_enabled_ = false;   // 构建失败则永久关闭，避免每帧重试
        spdlog::error("[VIDEOCAPTURE] [{}] Failed to build undistort maps: {}",
                      module_name_, e.what());
    }
}

void VideoCapture::applyUndistort(cv::Mat& frame) {
    if (!undistort_enabled_ || frame.empty()) return;

    // 若尺寸变化或首次调用，则（重新）构建映射表
    if (!undistort_ready_ || frame.size() != undistort_size_) {
        buildUndistortMaps(frame.size());
        if (!undistort_ready_) return;   // 构建失败，放弃去畸变
    }

    cv::Mat undistorted;
    cv::remap(frame, undistorted, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
    frame = undistorted;
}

void VideoCapture::runImpl() {
    // [撤销绑小核] 720p MJPEG 软解码在 A55 上约 35ms/帧，超过 33ms 帧周期，
    //   绑小核反而限死采集；交给调度器在空闲核间迁移更优。
    // RTStitching::bindToLittleCores(module_name_);
    is_running_.store(true, std::memory_order_release);

    while (!stopRequested()) {
        run();
    }

    stopRecording();
    is_running_.store(false, std::memory_order_release);
}

void VideoCapture::run() {
    auto now = std::chrono::steady_clock::now();

    // [修复] 软件节流只用于 image/video 文件源；实时相机(type=="camera")
    //   由 V4L2 驱动按帧率阻塞供帧，grab() 会自然对齐相机节奏。
    //   原实现要求"距上一帧【处理结束】>= 1000/fps ms"才开始下一帧，
    //   实际周期 = 节流33ms + grab等待 + 解码/去畸变/分发耗时（串联而非流水），
    //   并与相机 30fps 节奏产生拍频 —— 这正是实测 ~8.5fps、间隔 55~220ms
    //   发散分布的根因，MJPG 设置正确与否都会被它压住。
    if (camera_info_.type != "camera" && !first_frame_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time_);
        if (elapsed.count() < target_interval_ms_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return;
        }
    }

    cv::Mat frame;

    if (is_paused_.load()) {
        waitForWork();
    }

    if (camera_info_.type == "image") {
        if (image_frame_.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (first_frame_) {
                last_frame_time_ = std::chrono::steady_clock::now();
                first_frame_ = false;
            }
            return;
        }

        RTStitching::Image img;
        img.data = image_frame_.clone();
        img.timestamp = std::chrono::high_resolution_clock::now();
        img.img_idx = frame_index_++;

        // [修改] 热路径(->Warper)推原始帧：去畸变已并入 Warper 联合映射表，
        //   采集线程不再做逐帧 remap。
        try { output_buffer_.push_back(img); } catch (...) {}

        // [修改] 特征/曝光/接缝旁路仍需要与 Warper 输出同几何的去畸变图像，
        //   但均为低频消费者（接缝~1Hz、特征一次性），按 1/RT_SIDECHANNEL_DECIMATE
        //   抽帧去畸变即可；其余帧不推旁路（CircularBufferSync 消费端取最新，无影响）。
        if (!undistort_enabled_) {
            try { sync_buffer_to_camest_.push_back(sync_buffer_to_camest_index_, img); } catch (...) {}
            try { sync_buffer_to_expest_.push_back(sync_buffer_to_expest_index_, img); } catch (...) {}
            try { sync_buffer_to_seamfinder_.push_back(sync_buffer_to_seamfinder_index_, img); } catch (...) {}
        }
        else if ((img.img_idx % RT_SIDECHANNEL_DECIMATE) == 0) {
            RTStitching::Image side = img;
            side.data = img.data.clone();
            applyUndistort(side.data);
            try { sync_buffer_to_camest_.push_back(sync_buffer_to_camest_index_, side); } catch (...) {}
            try { sync_buffer_to_expest_.push_back(sync_buffer_to_expest_index_, side); } catch (...) {}
            try { sync_buffer_to_seamfinder_.push_back(sync_buffer_to_seamfinder_index_, side); } catch (...) {}
        }

        if (record_video_ && !stop_record_) {
            if (!recording_initialized_) initializeRecording(image_frame_);
            writeVideoFrame(image_frame_);
        }

        if (show_window_) {
            std::string timestamp_str = formatTimestamp(img.timestamp);
            cv::putText(img.data, timestamp_str,
                cv::Point(15, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 0),
                2);
            // [修改] 调试帧写盘 /root/build/debug/（Windows 仍 imshow）
            if (RTStitching::debugDump(module_name_, img.data) == 27) stop_requested_.store(true);
        }

        // [新增] 性能日志：采集帧间隔（倒数即该路实际采集帧率）
        if (!first_frame_) {
            RTStitching::perfRecord(module_name_, "capture_interval_ms",
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - last_frame_time_).count(),
                img.img_idx);
        }
        last_frame_time_ = std::chrono::steady_clock::now();
        first_frame_ = false;
        return;
    }

    // [修改] rtsp_cx 在 Linux 下走的是 OpenCV fallback（cap_ 已打开），
    //        因此只有 Windows 且 CmPlay.dll 真正初始化成功时才走 processRtspCxFrame()
    if (camera_info_.type == "rtsp_cx") {
#ifdef _WIN32
        if (rtsp_cx_initialized_ && cm_dll_handle_ != nullptr) {
            processRtspCxFrame();
            last_frame_time_ = std::chrono::steady_clock::now();
            first_frame_ = false;
            return;
        }
#endif
        // Linux fallback：继续往下走通用 cap_ 路径
    }

    if (!cap_.isOpened()) {
        spdlog::error("[VIDEOCAPTURE] [{}] Camera not opened, retrying...", module_name_);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (first_frame_) {
            last_frame_time_ = std::chrono::steady_clock::now();
            first_frame_ = false;
        }
        return;
    }

    const auto perf_read_start = std::chrono::steady_clock::now();   // [新增] 读帧计时起点
    bool grab_ok = cap_.grab();
    if (!grab_ok) {
        spdlog::error("[VIDEOCAPTURE] [{}] Frame grab failed.", module_name_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (first_frame_) {
            last_frame_time_ = std::chrono::steady_clock::now();
            first_frame_ = false;
        }
        else {
            last_frame_time_ += std::chrono::milliseconds(target_interval_ms_);
        }
        return;
    }

    bool retrieve_ok = cap_.retrieve(frame);
    if (!retrieve_ok || frame.empty()) {
        spdlog::error("[VIDEOCAPTURE] [{}] Empty frame or read failed.", module_name_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (first_frame_) {
            last_frame_time_ = std::chrono::steady_clock::now();
            first_frame_ = false;
        }
        else {
            last_frame_time_ += std::chrono::milliseconds(target_interval_ms_);
        }
        return;
    }

    const auto perf_read_end = std::chrono::steady_clock::now();     // [新增] grab+retrieve(MJPEG解码) 结束

    if (need_resize_) {
        cv::resize(frame, frame, cv::Size(camera_info_.width, camera_info_.height));
    }

    // [修改] 逐帧去畸变已并入 Warper 联合映射表，此处不再执行；
    //   applyUndistort 仅在下方旁路抽帧时按 1/N 频率调用。
    const auto perf_prep_end = std::chrono::steady_clock::now();      // [新增] resize 结束

// ========== 新增：保存第一帧到文件 ==========
    static bool saved_orig = false;
    if (!saved_orig && !frame.empty()) {
        cv::imwrite("/tmp/original_frame.jpg", frame);
        std::cout << "[VIDEOCAPTURE] Saved original frame to /tmp/original_frame.jpg, size=" 
                  << frame.cols << "x" << frame.rows << std::endl;
        saved_orig = true;
    }
    // ==========================================

    RTStitching::Image img;
    img.data = frame.clone();
    img.timestamp = std::chrono::high_resolution_clock::now();
    img.img_idx = frame_index_++;

    // [修改] 热路径(->Warper)推原始帧：去畸变已并入 Warper 联合映射表，
    //   采集线程不再做逐帧 remap。
    try { output_buffer_.push_back(img); } catch (...) {}

    // [修改] 特征/曝光/接缝旁路仍需要与 Warper 输出同几何的去畸变图像，
    //   但均为低频消费者（接缝~1Hz、特征一次性），按 1/RT_SIDECHANNEL_DECIMATE
    //   抽帧去畸变即可；其余帧不推旁路（CircularBufferSync 消费端取最新，无影响）。
    if (!undistort_enabled_) {
        try { sync_buffer_to_camest_.push_back(sync_buffer_to_camest_index_, img); } catch (...) {}
        try { sync_buffer_to_expest_.push_back(sync_buffer_to_expest_index_, img); } catch (...) {}
        try { sync_buffer_to_seamfinder_.push_back(sync_buffer_to_seamfinder_index_, img); } catch (...) {}
    }
    else if ((img.img_idx % RT_SIDECHANNEL_DECIMATE) == 0) {
        // [修改] 异步交接：去畸变与三路 sync push 移入每路独立的工人线程，
        //   采集节拍零阻塞（此前该内联块使每第 5 帧间隔冲到 ~134ms）。
        thread_local std::unique_ptr<SideChannelWorker> side_worker;
        if (!side_worker) {
            side_worker = std::make_unique<SideChannelWorker>();
            SideChannelWorker* w = side_worker.get();
            w->th = std::thread([this, w]() {
                RTStitching::Image job;
                while (true) {
                    {
                        std::unique_lock<std::mutex> lk(w->m);
                        w->cv.wait(lk, [w] { return w->has || w->quit; });
                        if (w->quit && !w->has) return;
                        job = w->pending;          // Mat 浅拷贝（引用计数）
                        w->has = false;
                    }
                    // applyUndistort 内部 remap 到新 Mat 后替换头部，
                    // 不改写原像素，无需 clone；原始帧仍安全共享给 Warper。
                    applyUndistort(job.data);
                    try { sync_buffer_to_camest_.push_back(sync_buffer_to_camest_index_, job); } catch (...) {}
                    try { sync_buffer_to_expest_.push_back(sync_buffer_to_expest_index_, job); } catch (...) {}
                    try { sync_buffer_to_seamfinder_.push_back(sync_buffer_to_seamfinder_index_, job); } catch (...) {}
                }
            });
        }
        {
            std::lock_guard<std::mutex> lk(side_worker->m);
            side_worker->pending = img;            // 浅拷贝交接，微秒级
            side_worker->has = true;               // 工人忙时直接覆盖为最新帧
        }
        side_worker->cv.notify_one();
    }

    if (record_video_ && !stop_record_) {
        if (!recording_initialized_) initializeRecording(frame);
        writeVideoFrame(frame);
    }

    if (show_window_) {
        std::string timestamp_str = formatTimestamp(img.timestamp);
        cv::putText(frame, timestamp_str,
            cv::Point(15, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2);
        // [修改] 调试帧写盘 /root/build/debug/（Windows 仍 imshow）
        if (RTStitching::debugDump(module_name_, frame) == 27) stop_requested_.store(true, std::memory_order_release);
    }

    // [新增] 性能日志：采集帧间隔（倒数即该路实际采集帧率）
    if (!first_frame_) {
        RTStitching::perfRecord(module_name_, "capture_interval_ms",
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - last_frame_time_).count(),
            img.img_idx);
    }
    // [新增] 分段耗时：read_ms = grab等待 + retrieve(MJPEG解码)；
    //        preprocess_ms = resize + 去畸变 remap。
    //   下一版 CSV 里这两项 + capture_interval 即可判定瓶颈在
    //   相机供帧(read大) 还是本线程 CPU 处理(preprocess大)。
    RTStitching::perfRecord(module_name_, "read_ms",
        std::chrono::duration<double, std::milli>(perf_read_end - perf_read_start).count(),
        img.img_idx);
    RTStitching::perfRecord(module_name_, "preprocess_ms",
        std::chrono::duration<double, std::milli>(perf_prep_end - perf_read_end).count(),
        img.img_idx);
    last_frame_time_ = std::chrono::steady_clock::now();
    first_frame_ = false;
}

bool VideoCapture::waitForWork() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return !is_paused_.load() || stopRequested(); });
    return stopRequested();
}

bool VideoCapture::initializeRecording(const cv::Mat& frame) {
    if (!recording_initialized_) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        // [修改] Platform.hpp 在 Linux 下把 localtime_s 重定向到 localtime_r
        localtime_s(&tm_buf, &time_t);

        std::stringstream ss;
        ss << "video_" << std::to_string(camera_info_.video_index) << "_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".mp4";
        video_filename_ = ss.str();

        std::vector<int> codecs = {
            cv::VideoWriter::fourcc('H', '2', '6', '4'),
            cv::VideoWriter::fourcc('X', '2', '6', '4'),
            cv::VideoWriter::fourcc('M', 'P', '4', 'V'),
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G')
        };
        std::vector<std::string> codec_names = { "H264", "X264", "MP4V", "MJPEG" };

        double fps = 25.0;
        cv::Size frame_size(frame.cols, frame.rows);
        bool success = false;

        for (size_t i = 0; i < codecs.size(); i++) {
            success = video_writer_.open(video_filename_, codecs[i], fps, frame_size, true);
            if (success) {
                recording_initialized_ = true;
                recording_start_time_ = std::chrono::steady_clock::now();
                recorded_frames_ = 0;
                spdlog::info("[VIDEOCAPTURE] [{}] Started recording with {}: {} ({})",
                             module_name_, codec_names[i], video_filename_,
                             frame_size.width, frame_size.height);
                break;
            }
        }

        if (!success) {
            spdlog::error("[VIDEOCAPTURE] [{}] All recording codecs failed", module_name_);
        }
        return success;
    }
    return true;
}

void VideoCapture::writeVideoFrame(const cv::Mat& frame) {
    if (recording_initialized_ && !frame.empty()) {
        try {
            video_writer_.write(frame);
            recorded_frames_++;

            if (recorded_frames_ % 100 == 0) {
                auto current_time = std::chrono::steady_clock::now();
                auto recording_duration = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - recording_start_time_);
                double actual_fps = static_cast<double>(recorded_frames_) /
                    std::max(1.0, static_cast<double>(recording_duration.count()));

                spdlog::info("[VIDEOCAPTURE] [{}] Recorded {} frames, duration: {}s, FPS: {:.1f}",
                             module_name_, recorded_frames_, recording_duration.count(), actual_fps);
            }
        }
        catch (const std::exception& e) {
            spdlog::error("[VIDEOCAPTURE] [{}] Error writing video frame: {}", module_name_, e.what());
        }
    }
}

void VideoCapture::stopRecording() {
    if (recording_initialized_) {
        video_writer_.release();
        auto recording_duration = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - recording_start_time_);

        spdlog::info("[VIDEOCAPTURE] [{}] Stopped recording: {} ({} frames, {} seconds)",
                     module_name_, video_filename_, recorded_frames_, recording_duration.count());

        recording_initialized_ = false;
        recorded_frames_ = 0;
        stop_record_ = true;
    }
}

void VideoCapture::pauseRecording()  { stop_record_ = true; }
void VideoCapture::resumeRecording() { stop_record_ = false; }

std::string VideoCapture::formatTimestamp(const std::chrono::high_resolution_clock::time_point& tp) {
    auto now = std::chrono::system_clock::now();
    auto tp_sys = now + std::chrono::duration_cast<std::chrono::system_clock::duration>(
        tp - std::chrono::high_resolution_clock::now());

    std::time_t t = std::chrono::system_clock::to_time_t(tp_sys);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        tp_sys.time_since_epoch()) % 1000;

    std::tm tm_buf{};
    localtime_s(&tm_buf, &t);    // Linux 下宏重定向到 localtime_r

    std::stringstream ss;
    ss << std::put_time(&tm_buf, "%H:%M:%S")
       << "." << std::setw(3) << std::setfill('0') << ms.count();
    return ss.str();
}

std::string VideoCapture::getModuleName() const {
    return module_name_;
}
