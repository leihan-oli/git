#include "Config.hpp"
#include <filesystem>
#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>  // 添加spdlog头文件

using namespace std;

cv::Mat Config::readMat(const YAML::Node& node, int rows, int cols) {
    if (!node || node.size() != rows * cols) {
        spdlog::error("[CONFIG] Matrix size mismatch, returning default matrix [{}x{}]", rows, cols);
        return cv::Mat::eye(rows, cols, CV_64F);
    }

    cv::Mat M(rows, cols, CV_64F);
    for (int i = 0; i < rows * cols; ++i) {
        M.at<double>(i / cols, i % cols) = node[i].as<double>();
    }
    return M;
}

Config::Config() {
    spdlog::info("[CONFIG] Create config");
}

Config::Config(const std::string& filename) {
    spdlog::info("[CONFIG] Create config and load from: {}", filename);
    if (!loadFromFile(filename)) {
        throw std::runtime_error("Failed to load config from: " + filename);
    }
}

Config::~Config() {
    spdlog::info("[CONFIG] Destroy config");
}

bool Config::loadFromFile(const std::string& filename) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filename);
    }
    catch (const std::exception& e) {
        spdlog::error("[CONFIG] Failed to read YAML file: {}", e.what());
        return false;
    }

    auto s = root["stitching"];
    if (!s) {
        spdlog::error("[CONFIG] YAML missing 'stitching' section");
        return false;
    }

    // Basic configuration
    params_.preview = s["preview"].as<bool>();
    params_.try_cuda = s["try_cuda"].as<bool>();
    params_.log_level = s["log_level"].as<int>();
    params_.verbose_output = s["verbose_output"].as<bool>();
    params_.camera_count = s["camera_count"].as<int>();
    params_.output_width = s["output_width"].as<int>();
    params_.output_height = s["output_height"].as<int>();

    // Image scaling parameters
    params_.scale_megapix[RTStitching::INPUT_SCALE] = s["input_scale_megapix"].as<double>();
    params_.scale_megapix[RTStitching::HOMO_EST_SCALE] = s["homo_est_scale_megapix"].as<double>();
    params_.scale_megapix[RTStitching::EXP_EST_SCALE] = s["exp_est_scale_megapix"].as<double>();
    params_.scale_megapix[RTStitching::SEAM_FINDER_SCALE] = s["seam_finder_scale_megapix"].as<double>();
    params_.scale_megapix[RTStitching::BLENDER_SCALE] = s["blender_scale_megapix"].as<double>();

    // Feature detection and matching
    params_.features_type = s["features_type"].as<std::string>();
    params_.matcher_type = s["matcher_type"].as<std::string>();
    params_.match_conf = s["match_conf"].as<float>();
    params_.max_features = s["max_features"].as<int>();
    params_.matching_thresh = s["matching_thresh"].as<double>();
    params_.match_rw = s["match_rw"].as<int>();

    // Camera pose and geometric correction
    params_.adjuster_type = s["adjuster_type"].as<std::string>();
    params_.wave_correction = s["wave_correction"].as<std::string>();
    params_.warper_type = s["warper_type"].as<std::string>();
    params_.estimator_type = s["estimator_type"].as<std::string>();
    // [新增] 近景视差补偿工作距离（可选字段；缺省=0 关闭，兼容旧配置）
    params_.parallax_d0 = s["parallax_d0"] ? s["parallax_d0"].as<double>() : 0.0;
    if (params_.parallax_d0 > 0.0) {
        spdlog::info("[CONFIG]   - Parallax compensation: d0 = {} m (spherical only)",
                     params_.parallax_d0);
        if (params_.warper_type != "spherical") {
            spdlog::warn("[CONFIG]   - parallax_d0 > 0 but warper_type = '{}' "
                         "(only 'spherical' supported) -> compensation DISABLED",
                         params_.warper_type);
            params_.parallax_d0 = 0.0;
        }
    }
    // Exposure compensation
    params_.exp_type = s["exp_type"].as<std::string>();
    params_.exp_nr_feeds = s["exp_nr_feeds"].as<int>();
    params_.exp_nr_filtering = s["exp_nr_filtering"].as<int>();
    params_.exp_block_size = s["exp_block_size"].as<int>();

    // Seam and blending
    params_.seam_find_type = s["seam_find_type"].as<std::string>();
    params_.crop = s["crop"].as<bool>();
    params_.blender_type = s["blender_type"].as<std::string>();
    params_.blend_strength = s["blend_strength"].as<int>();
    if (s["gaze_sigma_ratio"])
        params_.gaze_sigma_ratio = s["gaze_sigma_ratio"].as<float>();
    //if (s["gaze_saliency_threshold"])
     //   params_.gaze_saliency_threshold = s["gaze_saliency_threshold"].as<float>();
    if (s["gaze_alpha"])                                        // ← 新增
        params_.gaze_alpha = s["gaze_alpha"].as<float>();

    if (s["gaze_data_path"])                                    // [新增] 眼动数据文件路径
        params_.gaze_data_path = s["gaze_data_path"].as<std::string>();

    if (s["gaze_transport"])                                    // [新增] 传输方式 file/socket
        params_.gaze_transport = s["gaze_transport"].as<std::string>();
    if (s["gaze_socket_port"])                                  // [新增] socket 端口
        params_.gaze_socket_port = s["gaze_socket_port"].as<int>();

    // --- 显著性来源融合（眼动 + U2-Net）---
    if (auto sal = s["saliency"]) {
        if (sal["use_gaze"])     params_.sal_use_gaze     = sal["use_gaze"].as<bool>();
        if (sal["use_u2net"])    params_.sal_use_u2net    = sal["use_u2net"].as<bool>();
        if (sal["fusion_mode"])  params_.sal_fusion_mode  = sal["fusion_mode"].as<std::string>();
        if (sal["gaze_weight"])  params_.sal_gaze_weight  = sal["gaze_weight"].as<float>();
        if (sal["u2net_weight"]) params_.sal_u2net_weight = sal["u2net_weight"].as<float>();
        if (sal["u2net_dir"])    params_.sal_u2net_dir    = sal["u2net_dir"].as<std::string>();
        if (sal["display_mode"]) params_.sal_display_mode = sal["display_mode"].as<std::string>();
    }
    // Output
    params_.result_name = s["result_name"].as<std::string>();
    
    // Camera information
    params_.camera_info.clear();
    for (const auto& node : root["camera_info"]) {
        RTStitching::CameraInfo ci;
        ci.video_index = node["video_index"].as<int>();
        ci.description = node["description"].as<std::string>();
        ci.type = node["type"].as<std::string>();
        ci.url = node["url"].as<std::string>();
        ci.width = node["width"].as<int>();
        ci.height = node["height"].as<int>();
        ci.undistort = node["undistort"].as<bool>();
        ci.fps = node["fps"].as<int>();
        params_.camera_info.push_back(ci);
    }

    // ====== 路径锚定：把 config 内的相对路径解析为「相对 config.yaml 所在目录」======
    //   绝对路径保持不变；网络源(rtsp/http)不解析。整个工程换机器/换平台都无需改路径。
    {
        namespace fs = std::filesystem;
        std::error_code _ec;
        fs::path cfg_dir = fs::absolute(fs::path(filename), _ec).parent_path();
        auto resolve = [&](std::string& pth) {
            if (pth.empty()) return;
            fs::path pp(pth);
            if (pp.is_absolute()) return;
            pth = (cfg_dir / pp).lexically_normal().string();
        };
        resolve(params_.sal_u2net_dir);
        resolve(params_.gaze_data_path);
        for (auto& ci : params_.camera_info) {
            if (ci.type == "video" || ci.type == "image" || ci.type == "file")
                resolve(ci.url);   // rtsp/http 等网络源跳过
        }
    }

    // Camera parameters
    params_.camera_params.clear();
    for (const auto& node : root["camera_params"]) {
        RTStitching::CameraParams cp;
        cp.K = readMat(node["K"], 3, 3);
        cp.R = readMat(node["R"], 3, 3);
        cp.T = readMat(node["T"], 3, 1);
        cp.D = readMat(node["D"], 5, 1);
        cp.ppx = node["ppx"].as<double>();
        cp.ppy = node["ppy"].as<double>();
        cp.focal = node["focal"].as<double>();
        cp.aspect = node["aspect"].as<double>();
        params_.camera_params.push_back(cp);
    }

    return validate();
}

bool Config::validate() const {
    bool ok = true;

    if (params_.camera_count <= 0) {
        spdlog::error("[CONFIG] camera_count <= 0");
        ok = false;
    }

    if (params_.camera_info.size() != params_.camera_count) {
        spdlog::error("[CONFIG] Number of camera_info does not match camera_count");
        ok = false;
    }

    if (params_.camera_params.size() != params_.camera_count) {
        spdlog::error("[CONFIG] Number of camera_params does not match camera_count");
        ok = false;
    }

    // Validate scale parameters
    for (int i = 0; i < RTStitching::SCALE_LENGTH; ++i) {
        if (params_.scale_megapix[i] < -1.0 || params_.scale_megapix[i] > 10.0) {
            spdlog::error("[CONFIG] scale_megapix[{}] out of reasonable range: {}", i, params_.scale_megapix[i]);
            ok = false;
        }
    }

    return ok;
}

void Config::print() const {
    spdlog::info("[CONFIG] ==========================================");
    spdlog::info("[CONFIG]         Stitching Config Summary");
    spdlog::info("[CONFIG] ==========================================");

    // Basic information
    spdlog::info("[CONFIG] Basic configuration:");
    spdlog::info("[CONFIG]   - Number of cameras: {}", params_.camera_count);
    spdlog::info("[CONFIG]   - Preview mode: {}", params_.preview ? "On" : "Off");
    spdlog::info("[CONFIG]   - CUDA acceleration: {}", params_.try_cuda ? "On" : "Off");
    spdlog::info("[CONFIG]   - Log level: {}", params_.log_level);
    spdlog::info("[CONFIG]   - Verbose output: {}", params_.verbose_output ? "Yes" : "No");

    // Scale parameters
    spdlog::info("[CONFIG] Scale parameters (megapixels):");
    spdlog::info("[CONFIG]   - Input scale: {}", params_.scale_megapix[RTStitching::INPUT_SCALE]);
    spdlog::info("[CONFIG]   - Homography estimation: {}", params_.scale_megapix[RTStitching::HOMO_EST_SCALE]);
    spdlog::info("[CONFIG]   - Exposure estimation: {}", params_.scale_megapix[RTStitching::EXP_EST_SCALE]);
    spdlog::info("[CONFIG]   - Seam finding: {}", params_.scale_megapix[RTStitching::SEAM_FINDER_SCALE]);
    spdlog::info("[CONFIG]   - Blender: {}", params_.scale_megapix[RTStitching::BLENDER_SCALE]);

    // Algorithm parameters
    spdlog::info("[CONFIG] Algorithm parameters:");
    spdlog::info("[CONFIG]   - Feature type: {}", params_.features_type);
    spdlog::info("[CONFIG]   - Matcher type: {}", params_.matcher_type);
    spdlog::info("[CONFIG]   - Maximum features: {}", params_.max_features);
    spdlog::info("[CONFIG]   - Warper type: {}", params_.warper_type);
    spdlog::info("[CONFIG]   - Blender type: {}", params_.blender_type);

    // Camera information
    spdlog::info("[CONFIG] Camera information:");
    for (size_t i = 0; i < params_.camera_info.size(); ++i) {
        const auto& ci = params_.camera_info[i];
        spdlog::info("[CONFIG]   [{}] {} ({})", i, ci.description, ci.type);
        spdlog::info("[CONFIG]       Source: {}", ci.url);
        spdlog::info("[CONFIG]       Resolution: {}x{}", ci.width, ci.height);
        spdlog::info("[CONFIG]       Undistort: {}", ci.undistort ? "Yes" : "No");

        // Display camera parameters summary
        if (i < params_.camera_params.size()) {
            const auto& cp = params_.camera_params[i];
            spdlog::info("[CONFIG]       Focal length: {}, Principal point: ({}, {})", cp.focal, cp.ppx, cp.ppy);
        }
    }

    spdlog::info("[CONFIG] Output file: {}", params_.result_name);
    spdlog::info("[CONFIG] ==========================================");
}
