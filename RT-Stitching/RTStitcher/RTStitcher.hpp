#ifndef RTSTITCHER_HPP
#define RTSTITCHER_HPP

// 1. 系统头文件
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <shared_mutex>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <spdlog/spdlog.h>

// [新增] 跨平台兼容头（必须在 windows.h / dlfcn.h 等之前）
#include "Platform.hpp"

// 2. 自定义模块头文件
#include "Types.hpp"
#include "CircularBuffer.hpp"
#include "CircularBufferSync.hpp"
#include "FeatureFinder.hpp"
#include "Config.hpp"
#include "VideoCapture.hpp"
#include "Blender.hpp"
#include "Displayer.hpp"
#include "SeamFinder.hpp"
#include "Warper.hpp"
#include "CameraParamEst.hpp"
#include "Utility.hpp"
#include "GazeDataReader.hpp"
#include "SaliencyMapReader.hpp"
#include "FusedSaliencySource.hpp"

// 3. 宏定义
// [修改] 默认配置文件改为相对路径，跨平台通用；可被命令行 -c 覆盖
#ifndef CONFIG_FILE
#define CONFIG_FILE "config.yaml"   // 相对路径；找不到会自动从 exe 目录向上搜，亦可用 -c 覆盖
#endif

#define RTSTITCHING_BUFFER_CAPACITY  5
#define LOG_LEVEL  err

// [新增] 是否启用眼动仪注视感知缝合
//   Windows：默认开启
//   Linux/RK3588：默认关闭（先让主流水线跑通）
//   也可通过 -DMODULE_GAZE_AWARE=1/0 在 CMake 命令行覆盖
#ifndef MODULE_GAZE_AWARE
    #ifdef _WIN32
        #define MODULE_GAZE_AWARE 1
    #else
        #define MODULE_GAZE_AWARE 0
    #endif
#endif

// [新增] 是否启用 U2-Net 深度学习显著性
//   通常由 CMake 的 -DMODULE_SALIENCY_U2NET=ON/OFF 定义；
//   这里给一个兜底，未定义时按关闭处理。
#ifndef MODULE_SALIENCY_U2NET
    #define MODULE_SALIENCY_U2NET 0
#endif

#define MODE_STATIC_CSP 1
#define MODE_PARAM_EST  0
#define MODE_VIDEO_RECORD 0

#if MODE_STATIC_CSP
    #define MODULE_VIDEOCAPTURE 1
    #define MODULE_VIDEOCAPTURE_START 1
    #define MODULE_VIDEOCAPTURE_RECORD false
    #define MODULE_FEATURE_FINEDER 1
    #define MODULE_FEATURE_FINEDER_START 0
    #define MODULE_CAMERA_PARAM_EST 1
    #define MODULE_CAMERA_PARAM_EST_START 0
    #define MODULE_EXPOS_EST 0
    #define MODULE_EXPOS_EST_START 0
    #define MODULE_SEAM_FINDER 1
    #define MODULE_SEAM_FINDER_START 1
    #define MODULE_WARPER 1
    #define MODULE_WARPER_START 1
    #define MODULE_WARPER_TO_BLENDER 1
    #define MODULE_WARPER_TO_EXPOSCOMP 0
    #define MODULE_BLENDER 1
    #define MODULE_BLENDER_START 1
    #define MODULE_DISPLAYER 1
    #define MODULE_DISPLAYER_START 1
#elif MODE_PARAM_EST
    #define MODULE_VIDEOCAPTURE 1
    #define MODULE_VIDEOCAPTURE_START 1
    #define MODULE_VIDEOCAPTURE_RECORD false
    #define MODULE_FEATURE_FINEDER 1
    #define MODULE_FEATURE_FINEDER_START 1
    #define MODULE_CAMERA_PARAM_EST 1
    #define MODULE_CAMERA_PARAM_EST_START 1
    #define MODULE_EXPOS_EST 0
    #define MODULE_EXPOS_EST_START 0
    #define MODULE_SEAM_FINDER 1
    #define MODULE_SEAM_FINDER_START 0
    #define MODULE_WARPER 1
    #define MODULE_WARPER_START 1
    #define MODULE_WARPER_TO_BLENDER 1
    #define MODULE_WARPER_TO_EXPOSCOMP 0
    #define MODULE_BLENDER 1
    #define MODULE_BLENDER_START 0
    #define MODULE_DISPLAYER 1
    #define MODULE_DISPLAYER_START 0
#elif MODE_VIDEO_RECORD
    #define MODULE_VIDEOCAPTURE 1
    #define MODULE_VIDEOCAPTURE_START 1
    #define MODULE_VIDEOCAPTURE_RECORD true
    #define MODULE_FEATURE_FINEDER 0
    #define MODULE_FEATURE_FINEDER_START 0
    #define MODULE_CAMERA_PARAM_EST 0
    #define MODULE_CAMERA_PARAM_EST_START 0
    #define MODULE_EXPOS_EST 0
    #define MODULE_EXPOS_EST_START 0
    #define MODULE_SEAM_FINDER 0
    #define MODULE_SEAM_FINDER_START 0
    #define MODULE_WARPER 0
    #define MODULE_WARPER_START 0
    #define MODULE_WARPER_TO_BLENDER 0
    #define MODULE_WARPER_TO_EXPOSCOMP 0
    #define MODULE_BLENDER 0
    #define MODULE_BLENDER_START 0
    #define MODULE_DISPLAYER 0
    #define MODULE_DISPLAYER_START 0
#else
    // user defined
#endif

// ============================================================================
// [修改] 模块调试开关说明：
//   开启后，各模块的调试可视化在 Windows 上仍走 imshow；在板子(Linux)上
//   统一改为限频原子写盘到 /root/build/debug/（见 Utility/DebugDump.hpp），
//   避免工作线程 imshow 与 launcher 的 Qt 抢占 GUI 资源导致卡死。
//   各开关对应的调试输出文件（Linux）：
//     MODULE_VIDEOCAPTURE_DEBUG     -> /root/build/debug/Cam_<idx>.jpg（带时间戳水印的采集帧）
//     MODULE_FEATURE_FINEDER_DEBUG  -> /root/build/debug/FeatureFinder_Keypoints_Comparison.jpg
//     MODULE_CAMERA_PARAM_EST_DEBUG -> /root/build/debug/Stitching_Masks_Overview.jpg
//     MODULE_EXPOS_EST_DEBUG        -> （预留：当前流水线曝光模块无可视化输出）
//     MODULE_SEAM_FINDER_DEBUG      -> /tmp/seam_alpha_compare.jpg（launcher 小窗显示通道，保持原路径）
//     MODULE_WARPER_DEBUG           -> /root/build/debug/Warper_<idx>_info.jpg
//     MODULE_BLENDER_DEBUG (>=1)    -> /root/build/debug/Blender_result_resize.jpg
//                          (>=2)    -> 另加 Blender_result_out.jpg / Blender_result_mask_out.jpg
//                          (>=3)    -> 另加 Scaled_Mask_Visualization.jpg（需 verbose_output）
//
// [新增] 性能日志（Utility/PerfLog.hpp）与上述图像开关完全解耦、始终开启：
//   - 逐条明细追加写 /root/build/debug/perf.csv（wall_ms,module,metric,frame_idx,value_ms）
//   - 每 5 秒对各指标输出一次 avg/min/max 摘要到运行日志（[PERF] 前缀），
//     测报告表 3.1 时保持下面所有图像开关为 false/0，直接抄 [PERF] 摘要即可。
//   指标一览：Cam_i/capture_interval_ms；Warper_i/{input_buffer_delay_ms,warp_ms,
//   total_latency_ms}；Blender/{blend_ms,blend_interval_ms,camN_frame_to_bufferout_ms,
//   camN_frame_to_feed_ms,camN_e2e_ms,camN_skipped_frames}；SeamFinder/seam_find_ms。
//   如需彻底零开销可在 PerfLog.hpp 中将 RT_PERF_LOG_ENABLED 置 0。
// ============================================================================
#define MODULE_VIDEOCAPTURE_DEBUG false
#define MODULE_FEATURE_FINEDER_DEBUG false
#define MODULE_CAMERA_PARAM_EST_DEBUG false
#define MODULE_EXPOS_EST_DEBUG false
#define MODULE_SEAM_FINDER_DEBUG true   // [修改] 测性能时关闭；需要 launcher 小窗接缝可视化再打开
#define MODULE_WARPER_DEBUG false
#define MODULE_BLENDER_DEBUG 0

// 4. Global parameters
std::shared_ptr<RTStitching::ConfigParams> config_params;
std::shared_ptr<std::vector<RTStitching::CameraStitchParams>> stitching_params;

// 5. 缓冲区
std::shared_ptr<CircularBufferSync<RTStitching::Image>> videocapture_to_featurefinder;
std::shared_ptr<CircularBufferSync<RTStitching::Image>> videocapture_to_exposcompest;
std::shared_ptr<CircularBufferSync<RTStitching::Image>> videocapture_to_seamfinder;
std::shared_ptr<CircularBufferSync<RTStitching::Image>> exposcomp_to_blender;

std::vector<std::shared_ptr<CircularBuffer<RTStitching::Image>>> videocapture_to_warper_buffers;
std::vector<std::shared_ptr<CircularBuffer<RTStitching::Image>>> warper_to_exposcomp_buffers;

std::shared_ptr<CircularBuffer<std::vector<ImageFeatures>>> featurefinder_to_estimator;
std::shared_ptr<CircularBuffer<RTStitching::Image>> blender_to_display;

// 6. 各模块实例
std::vector<std::unique_ptr<VideoCapture>> captures;
std::vector<std::unique_ptr<WarperModule>> warpers;

std::unique_ptr<FeatureFinder> feature_finder;
std::unique_ptr<CameraParamEst> camera_param_est;
std::unique_ptr<SeamFinder> seam_finder;
std::unique_ptr<BlenderModule> blender;
std::unique_ptr<Displayer> displayer;

// 7. 眼动仪/u2net
std::unique_ptr<GazeDataReader> gaze_reader;
std::unique_ptr<SaliencyMapReader> saliency_reader;
std::unique_ptr<FusedSaliencySource> fused_saliency;   // 融合源，注入 SeamFinder


bool getConfigParams(const std::string& config_file = CONFIG_FILE);
void stitchingParamsInitialize(void);
void threadInitialize(void);
void bufferInitialize(void);
#endif // RTSTITCHER_HPP
