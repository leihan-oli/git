#include "RTStitcher.hpp"
#include <atomic>
#include <csignal>
#include <filesystem>
#ifndef _WIN32
#include <unistd.h>
#endif


// [新增] 解析配置文件路径：
//   1) 先按给定路径找（绝对路径，或相对当前工作目录）
//   2) 找不到则从可执行文件目录逐级向上搜 config.yaml
//   这样无论从 VS 调试、bin 目录双击、还是 Linux 终端启动都能定位到配置。
static std::string resolveConfigPath(const std::string& given) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (fs::exists(given, ec)) return given;

    fs::path dir = getExecutableDir();
    for (int i = 0; i < 8 && !dir.empty(); ++i) {
        fs::path cand = dir / "config.yaml";
        if (fs::exists(cand, ec)) return cand.string();
        if (dir == dir.root_path()) break;
        dir = dir.parent_path();
    }
    return given;  // 兜底，交给 loadFromFile 报错
}

static void printUsage(char** argv)
{
    std::cout <<
        "Real-time video stitching application.\n\n"
        << argv[0] << " [flags]\n\n"
        "Flags:\n"
        "  -c, --config <config_file>\n"
        "      Path to configuration YAML file.\n"
        "  -h, --help\n"
        "      Print this help message.\n";
}

static int parseCmdArgs(int argc, char** argv, std::string& config_file)
{
    if (argc == 1) return 0;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h" || arg == "/?")
        {
            printUsage(argv);
            return -1;
        }
        else if ((arg == "--config" || arg == "-c") && i + 1 < argc)
        {
            config_file = argv[i + 1];
            i++;
        }
        else
        {
            std::cout << "Bad argument: " << arg << std::endl;
            printUsage(argv);
            return -1;
        }
    }
    return 0;
}

// ========== 新增：全局退出标志和信号处理器 ==========
static std::atomic<bool> g_should_exit{false};

void signalHandler(int sig)
{
    if (sig == SIGTERM || sig == SIGINT) {
        spdlog::info("[RTSTITCHER] Received signal {}, exiting gracefully...", sig);
        g_should_exit.store(true, std::memory_order_relaxed);
    }
}
// ====================================================

int main(int argc, char* argv[]) {
    // ========== 注册信号处理器 ==========
#ifdef _WIN32
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);
#else
    struct sigaction sa;
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGINT, &sa, nullptr);
#endif

    // ===================================

    spdlog::set_level(spdlog::level::LOG_LEVEL);

    std::string config_file = CONFIG_FILE;
    int retval = parseCmdArgs(argc, argv, config_file);
    if (retval)
        return retval;
    config_file = resolveConfigPath(config_file);   // [新增] 自动定位 config.yaml
    std::cout << "config_file:" << config_file << std::endl;

    spdlog::info("[RTSTITCHER] Using config file: {}", config_file);

    SetConsoleOutputCP(CP_UTF8);
    if (!getConfigParams(config_file))
        return 0;

    stitchingParamsInitialize();
    bufferInitialize();
    threadInitialize();

    spdlog::info("[RTSTITCHER] All camera captures started.");
    std::cout << std::endl;
    std::cout << "========== RT-Stitching started! ==========" << std::endl;
    std::cout << "========== Operation Instruction ==========" << std::endl;
    std::cout << "[RTSTITCHER] All camera captures started." << std::endl;
    std::cout << "[RTSTITCHER] Press '1' to pause USB camera" << std::endl;
    std::cout << "[RTSTITCHER] Press '2' to resume USB camera" << std::endl;
    std::cout << "[RTSTITCHER] Press '3' to pause RTSP camera" << std::endl;
    std::cout << "[RTSTITCHER] Press '4' to resume RTSP camera" << std::endl;
    std::cout << "[RTSTITCHER] Press '5' to stop camera record" << std::endl;
    std::cout << "[RTSTITCHER] Press '6' to pause camera record" << std::endl;
    std::cout << "[RTSTITCHER] Press '7' to resume camera record" << std::endl;
    std::cout << "[RTSTITCHER] Press 'ESC' to exit." << std::endl;
    std::cout << "===========================================" << std::endl;
    std::cout << std::endl;

    #ifdef _WIN32
        cv::namedWindow("Stitching Result", cv::WINDOW_NORMAL);
        cv::namedWindow("Seam Alpha Comparison", cv::WINDOW_NORMAL);
    #endif

    // ========== 主循环：检测退出标志和键盘输入 ==========
    while (!g_should_exit.load(std::memory_order_relaxed)) {
        if (kbHit()) {
            int key = getKey();
            if (key == 27) {  // ESC 键
                spdlog::info("[RTSTITCHER] ESC pressed, stopping...");
                g_should_exit.store(true, std::memory_order_relaxed);
                break;  // 立即跳出循环，执行清理
            }
            else if (key == '1') {
                spdlog::info("[RTSTITCHER] Pausing USB camera...");
                if (captures.size() > 0 && captures[0]) captures[0]->pause();
            }
            else if (key == '2') {
                spdlog::info("[RTSTITCHER] Resuming USB camera...");
                if (captures.size() > 0 && captures[0]) captures[0]->resume();
            }
            else if (key == '3' && captures.size() > 1) {
                spdlog::info("[RTSTITCHER] Pausing RTSP camera...");
                captures[1]->pause();
            }
            else if (key == '4' && captures.size() > 1) {
                spdlog::info("[RTSTITCHER] Resuming RTSP camera...");
                captures[1]->resume();
            }
#if MODULE_VIDEOCAPTURE_RECORD
            else if (key == '5') {
                if (captures.size() > 0) captures[0]->stopRecording();
                spdlog::info("[RTSTITCHER] Stop camera0 record...");
                if (captures.size() > 1) captures[1]->stopRecording();
                spdlog::info("[RTSTITCHER] Stop camera1 record...");
            }
            else if (key == '6') {
                if (captures.size() > 0) captures[0]->pauseRecording();
                if (captures.size() > 1) captures[1]->pauseRecording();
                spdlog::info("[RTSTITCHER] Pause camera record...");
            }
            else if (key == '7') {
                if (captures.size() > 0) captures[0]->resumeRecording();
                if (captures.size() > 1) captures[1]->resumeRecording();
                spdlog::info("[RTSTITCHER] Resume camera record...");
            }
#endif
        }
#ifdef _WIN32
        if (displayer) {
            cv::Mat disp_frame;
            if (displayer->getLatestFrame(disp_frame) && !disp_frame.empty())
                cv::imshow("Stitching Result", disp_frame);
        }
        if (seam_finder) {                                   
            cv::Mat seam_frame;
            if (seam_finder->getLatestSeamDebugFrame(seam_frame) && !seam_frame.empty())
                cv::imshow("Seam Alpha Comparison", seam_frame);
        }
#endif
        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // ========== 退出循环后，停止所有模块并释放资源 ==========
    spdlog::info("[RTSTITCHER] Stopping all modules and threads...");

    spdlog::info("[RTSTITCHER] Stopping camera capture threads...");
    for (auto& cap : captures) {
        if (cap && cap->isRunning()) cap->stop();
    }

    spdlog::info("[RTSTITCHER] Stopping warper threads...");
    for (auto& warper : warpers) {
        if (warper && warper->isRunning()) warper->stop();
    }

    spdlog::info("[RTSTITCHER] Stopping feature finder thread...");
    if (feature_finder && feature_finder->isRunning()) feature_finder->stop();

    spdlog::info("[RTSTITCHER] Stopping camera parameter estimation thread...");
    if (camera_param_est && camera_param_est->isRunning()) camera_param_est->stop();

    spdlog::info("[RTSTITCHER] Stopping seam finder thread...");
    if (seam_finder && seam_finder->isRunning()) seam_finder->stop();

    spdlog::info("[RTSTITCHER] Stopping image blending thread...");
    if (blender && blender->isRunning()) blender->stop();

    spdlog::info("[RTSTITCHER] Stopping display thread...");
    if (displayer && displayer->isRunning()) displayer->stop();

    spdlog::info("[RTSTITCHER] All threads stopped successfully.");

    spdlog::info("[RTSTITCHER] Releasing resources...");
    captures.clear();
    warpers.clear();
    feature_finder.reset();
    camera_param_est.reset();
    seam_finder.reset();
    fused_saliency.reset();   // 必须在 seam_finder 之后释放（seam_finder 持有它的裸指针）
    blender.reset();
    displayer.reset();

    spdlog::info("[RTSTITCHER] ===== Program exited normally =====");
    return 0;
}

bool getConfigParams(const std::string& config_file) {
    auto config = std::make_unique<Config>();
    if (!config->loadFromFile(config_file)) {
        spdlog::error("[RTSTITCHER] Failed to load config file: {}", config_file);
        return false;
    }
    config_params = std::make_shared<RTStitching::ConfigParams>(config->getParams());
    spdlog::info("[RTSTITCHER] Success to load config file: {}", config_file);
    return true;
}

void stitchingParamsInitialize(void) {
    stitching_params = std::make_shared<std::vector<RTStitching::CameraStitchParams>>(RTStitching::SCALE_LENGTH);
    for (int scale_cnt = RTStitching::INPUT_SCALE; scale_cnt < RTStitching::SCALE_LENGTH; scale_cnt++) {
        (*stitching_params)[scale_cnt].csp_ver = 1;
        (*stitching_params)[scale_cnt].sm_ver = 1;
        spdlog::info("[RTSTITCHER] ##-- Start processing scale:{}", scale_cnt);

        if (config_params->scale_megapix[scale_cnt] < 0) {
            (*stitching_params)[scale_cnt].scale_ratio = 1.0;
        }
        else {
            (*stitching_params)[scale_cnt].scale_ratio = cv::min(
                1.0,
                sqrt(config_params->scale_megapix[scale_cnt] * 1e6 /
                     (config_params->camera_info[0].width * config_params->camera_info[0].height))
            );
        }
    }

    update_stitching_params(
        *config_params,
        *stitching_params,
        RTStitching::INPUT_SCALE
    );
}

void threadInitialize(void)
{
    auto seam_mask_mutex = std::make_shared<std::shared_mutex>();

    /********** VideoCapture Module **********/
#if MODULE_VIDEOCAPTURE
    captures.reserve(config_params->camera_count);
    for (size_t cam_idx = 0; cam_idx < config_params->camera_count; ++cam_idx) {
        captures.emplace_back(std::make_unique<VideoCapture>(
            config_params->camera_info[cam_idx],
            config_params->camera_params[cam_idx],   // [新增] 传入 K/D 用于去畸变
            *videocapture_to_warper_buffers[cam_idx],
            *videocapture_to_featurefinder,
            cam_idx,
            *videocapture_to_exposcompest,
            cam_idx,
            *videocapture_to_seamfinder,
            cam_idx,
            MODULE_VIDEOCAPTURE_DEBUG,
            MODULE_VIDEOCAPTURE_RECORD,
            "Cam_" + std::to_string(cam_idx)
        ));
#if MODULE_VIDEOCAPTURE_START
        captures.back()->start();
#endif
    }
#endif

    /********** FeatureFinder Module **********/
#if MODULE_FEATURE_FINEDER
    feature_finder = std::make_unique<FeatureFinder>(
        *config_params,
        *videocapture_to_featurefinder,
        *featurefinder_to_estimator,
        MODULE_FEATURE_FINEDER_DEBUG,
        "FeatureFinder"
    );
#if MODULE_FEATURE_FINEDER_START
    feature_finder->start();
#endif
#endif

    /********** Warper Module **********/
#if MODULE_WARPER
    warpers.reserve(config_params->camera_count);

#if MODULE_WARPER_TO_EXPOSCOMP
    for (size_t cam_idx = 0; cam_idx < config_params->camera_count; ++cam_idx) {
        warpers.emplace_back(std::make_unique<WarperModule>(
            *videocapture_to_warper_buffers[cam_idx],
            *warper_to_exposcomp_buffers[cam_idx],
            *config_params,
            *stitching_params,
            cam_idx,
            MODULE_WARPER_DEBUG,
            "Warper_" + std::to_string(cam_idx)
        ));
        if (!warpers.back()->initialize(*config_params)) {
            spdlog::error("[RTSTITCHER] Warper {} initialization failed!", cam_idx);
            return;
        }
#if MODULE_WARPER_START
        warpers.back()->start();
#endif
    }
#elif MODULE_WARPER_TO_BLENDER
    for (size_t cam_idx = 0; cam_idx < config_params->camera_count; ++cam_idx) {
        warpers.emplace_back(std::make_unique<WarperModule>(
            *videocapture_to_warper_buffers[cam_idx],
            *exposcomp_to_blender,
            *config_params,
            *stitching_params,
            cam_idx,
            MODULE_WARPER_DEBUG,
            "Warper_" + std::to_string(cam_idx)
        ));
        if (!warpers.back()->initialize(*config_params)) {
            spdlog::error("[RTSTITCHER] Warper {} initialization failed!", cam_idx);
        }
#if MODULE_WARPER_START
        warpers.back()->start();
#endif
    }
#endif
#endif

    /********** CameraParamEst Module **********/
#if MODULE_CAMERA_PARAM_EST
    camera_param_est = std::make_unique<CameraParamEst>(
        *config_params,
        *stitching_params,
        *featurefinder_to_estimator,
        MODULE_CAMERA_PARAM_EST_DEBUG   // [新增] 接入调试开关，输出到 /root/build/debug/
    );
#if MODULE_CAMERA_PARAM_EST_START
    camera_param_est->start();
#endif
#endif

    /********** SeamFinder Module **********/
#if MODULE_SEAM_FINDER
    // ============================================================
    // 显著性来源装配（3.4 / 3.5）
    //   gaze_reader / saliency_reader 都实现 ISaliencySource 接口，
    //   统一用 FusedSaliencySource 包住两路，由 config.yaml 的
    //   saliency.{use_gaze,use_u2net,fusion_mode,...} 决定最终行为：
    //     - use_gaze=1, use_u2net=0  -> 仅眼动（回归原行为）
    //     - use_gaze=0, use_u2net=1  -> 仅 U2-Net（3.4 验证）
    //     - use_gaze=1, use_u2net=1  -> 两路融合（3.5）
    //   最终把统一的 ISaliencySource* 注入 SeamFinder，下游零改动。
    // ============================================================
    ISaliencySource* gaze_src  = nullptr;   // 眼动来源（可空）
    ISaliencySource* u2net_src = nullptr;   // U2-Net 来源（可空）

#if MODULE_GAZE_AWARE
    gaze_reader = std::make_unique<GazeDataReader>(
        config_params->gaze_data_path,   // [修改] 路径来自 config.yaml（已锚定到配置文件目录）
        50
    );
    // [新增] socket 模式：板子上接收 Windows(Tobii) 经 TCP 发来的注视数据
    if (config_params->gaze_transport == "socket") {
        gaze_reader->useSocket(config_params->gaze_socket_port);
        spdlog::error("[DIAG][RTSTITCHER] Gaze transport = SOCKET, port {}", config_params->gaze_socket_port);
    } else {
        spdlog::error("[DIAG][RTSTITCHER] Gaze transport = FILE ({})", config_params->gaze_data_path);
    }
    gaze_reader->start();
    gaze_src = gaze_reader.get();
    spdlog::info("[RTSTITCHER] GazeDataReader started");
#else
    spdlog::info("[RTSTITCHER] GazeDataReader disabled (MODULE_GAZE_AWARE=0).");
#endif

#if MODULE_SALIENCY_U2NET
    saliency_reader = std::make_unique<SaliencyMapReader>(
        config_params->sal_u2net_dir, 30);     // 与 Python saliency_writer.py 的 --out_dir 一致
    saliency_reader->start();
    u2net_src = saliency_reader.get();
    spdlog::info("[RTSTITCHER] SaliencyMapReader started, dir={}", config_params->sal_u2net_dir);
#else
    spdlog::info("[RTSTITCHER] SaliencyMapReader disabled (MODULE_SALIENCY_U2NET=0).");
#endif

    // 融合源：根据 yaml 配置决定输出哪一路 / 如何融合
    fused_saliency = std::make_unique<FusedSaliencySource>(
        gaze_src,
        u2net_src,
        config_params->sal_use_gaze,
        config_params->sal_use_u2net,
        FusedSaliencySource::parseMode(config_params->sal_fusion_mode),
        config_params->sal_gaze_weight,
        config_params->sal_u2net_weight
    );
    spdlog::info("[RTSTITCHER] FusedSaliencySource: use_gaze={}, use_u2net={}, mode={}",
                 config_params->sal_use_gaze,
                 config_params->sal_use_u2net,
                 config_params->sal_fusion_mode);

    seam_finder = std::make_unique<SeamFinder>(
        *videocapture_to_seamfinder,
        *config_params,
        *stitching_params,
        seam_mask_mutex,
        MODULE_SEAM_FINDER_DEBUG,
        "SeamFinder",
        fused_saliency.get()            // ← 注入统一融合源
    );
#if MODULE_SEAM_FINDER_START
    if (seam_finder->initialize(*config_params)) {
        seam_finder->start();
        spdlog::info("[RTSTITCHER] Seam finder started successfully (using preloaded parameters)");
    }
    else {
        spdlog::error("[RTSTITCHER] Seam finder initialization failed: {}", seam_finder->getLastError());
    }
#endif
#endif

    /********** Blender Module **********/
#if MODULE_BLENDER
    blender = std::make_unique<BlenderModule>(
        *stitching_params,
        *exposcomp_to_blender,
        *blender_to_display,
        *config_params,
        seam_mask_mutex,
        MODULE_BLENDER_DEBUG,
        "Blender"
    );
#if MODULE_BLENDER_START
    if (!blender->start()) {
        spdlog::error("[RTSTITCHER] Image blending module startup failed: {}", blender->getLastError());
    }
#endif
#endif

    /********** Displayer Module **********/
    // [优化] Displayer 仅在 Windows 上用于 imshow 预览；开发板(Linux)上
    //   结果通过 Blender 写出的 /tmp/stitched.jpg 供 GUI 读取，无需 Displayer，
    //   故在板子上直接弃用，省去每帧的 scaleMask/resize/写盘开销。
    //   blender_to_display 缓冲无人消费时 Blender 的 try_push_back 会安全失败
    //   并跳过（warn 级日志在 LOG_LEVEL=err 下不打印），不影响主链路。
#if defined(_WIN32) && MODULE_DISPLAYER
    displayer = std::make_unique<Displayer>(
        *blender_to_display,
        *config_params,
        nullptr,
        "Stitching Result",
        true
    );
#if MODULE_DISPLAYER_START
    displayer->start();
#endif
#else
    spdlog::info("[RTSTITCHER] Displayer disabled on this platform (board uses /tmp/stitched.jpg).");
#endif
}

void bufferInitialize(void)
{
    videocapture_to_featurefinder = std::make_shared<CircularBufferSync<RTStitching::Image>>(
        config_params->camera_count, RTSTITCHING_BUFFER_CAPACITY
    );
    videocapture_to_exposcompest = std::make_shared<CircularBufferSync<RTStitching::Image>>(
        config_params->camera_count, RTSTITCHING_BUFFER_CAPACITY
    );
    videocapture_to_seamfinder = std::make_shared<CircularBufferSync<RTStitching::Image>>(
        config_params->camera_count, RTSTITCHING_BUFFER_CAPACITY
    );
    exposcomp_to_blender = std::make_shared<CircularBufferSync<RTStitching::Image>>(
        config_params->camera_count, RTSTITCHING_BUFFER_CAPACITY
    );

    videocapture_to_warper_buffers.reserve(config_params->camera_count);
    warper_to_exposcomp_buffers.reserve(config_params->camera_count);

    for (size_t i = 0; i < config_params->camera_count; ++i) {
        videocapture_to_warper_buffers.emplace_back(
            std::make_shared<CircularBuffer<RTStitching::Image>>(RTSTITCHING_BUFFER_CAPACITY)
        );
        warper_to_exposcomp_buffers.emplace_back(
            std::make_shared<CircularBuffer<RTStitching::Image>>(RTSTITCHING_BUFFER_CAPACITY)
        );
    }
    featurefinder_to_estimator = std::make_shared<CircularBuffer<std::vector<ImageFeatures>>>(
        RTSTITCHING_BUFFER_CAPACITY
    );
    blender_to_display = std::make_shared<CircularBuffer<RTStitching::Image>>(
        RTSTITCHING_BUFFER_CAPACITY
    );
}