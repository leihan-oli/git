#include "AllConfig.h"
#include <fstream>
#include <QSaveFile>
#include <QDebug>

namespace YAML {

// ========== CameraInfo ==========
Node convert<CameraInfo>::encode(const CameraInfo& rhs) {
    Node node;
    node["video_index"] = rhs.video_index;
    node["description"] = rhs.description;
    node["type"] = rhs.type;
    node["url"] = rhs.url;
    node["width"] = rhs.width;
    node["height"] = rhs.height;
    node["undistort"] = rhs.undistort;
    node["model"] = rhs.model;
    node["fps"] = rhs.fps;
    return node;
}

bool convert<CameraInfo>::decode(const Node& node, CameraInfo& rhs) {
    if (!node.IsMap()) return false;
    rhs.video_index = node["video_index"].as<int>();
    rhs.description = node["description"].as<std::string>();
    rhs.type = node["type"].as<std::string>();
    rhs.url = node["url"].as<std::string>();
    rhs.width = node["width"].as<int>();
    rhs.height = node["height"].as<int>();
    rhs.undistort = node["undistort"].as<bool>();
    if (node["model"]) rhs.model = node["model"].as<std::string>();
    rhs.fps = node["fps"].as<int>();
    return true;
}

// ========== CameraParams ==========
Node convert<CameraParams>::encode(const CameraParams& rhs) {
    Node node;
    node["K"] = rhs.K;
    node["R"] = rhs.R;
    node["T"] = rhs.T;
    node["D"] = rhs.D;
    node["ppx"] = rhs.ppx;
    node["ppy"] = rhs.ppy;
    node["focal"] = rhs.focal;
    node["aspect"] = rhs.aspect;
    return node;
}

bool convert<CameraParams>::decode(const Node& node, CameraParams& rhs) {
    if (!node.IsMap()) return false;
    rhs.K = node["K"].as<std::vector<double>>();
    rhs.R = node["R"].as<std::vector<double>>();
    rhs.T = node["T"].as<std::vector<double>>();
    rhs.D = node["D"].as<std::vector<double>>();
    rhs.ppx = node["ppx"].as<double>();
    rhs.ppy = node["ppy"].as<double>();
    rhs.focal = node["focal"].as<double>();
    rhs.aspect = node["aspect"].as<double>();
    return true;
}

// ========== Basic ==========
Node convert<AllConfig::Basic>::encode(const AllConfig::Basic& rhs) {
    Node node;
    node["preview"] = rhs.preview;
    node["try_cuda"] = rhs.try_cuda;
    node["log_level"] = rhs.log_level;
    node["verbose_output"] = rhs.verbose_output;
    node["camera_count"] = rhs.camera_count;
    node["output_width"] = rhs.output_width;
    node["output_height"] = rhs.output_height;
    return node;
}

bool convert<AllConfig::Basic>::decode(const Node& node, AllConfig::Basic& rhs) {
    if (!node.IsMap()) return false;
    rhs.preview = node["preview"].as<bool>();
    rhs.try_cuda = node["try_cuda"].as<bool>();
    rhs.log_level = node["log_level"].as<int>();
    rhs.verbose_output = node["verbose_output"].as<bool>();
    rhs.camera_count = node["camera_count"].as<int>();
    rhs.output_width = node["output_width"].as<int>();
    rhs.output_height = node["output_height"].as<int>();
    return true;
}

// ========== Scale ==========
Node convert<AllConfig::Scale>::encode(const AllConfig::Scale& rhs) {
    Node node;
    node["input_scale_megapix"] = rhs.input_scale_megapix;
    node["homo_est_scale_megapix"] = rhs.homo_est_scale_megapix;
    node["exp_est_scale_megapix"] = rhs.exp_est_scale_megapix;
    node["seam_finder_scale_megapix"] = rhs.seam_finder_scale_megapix;
    node["blender_scale_megapix"] = rhs.blender_scale_megapix;
    return node;
}

bool convert<AllConfig::Scale>::decode(const Node& node, AllConfig::Scale& rhs) {
    if (!node.IsMap()) return false;
    rhs.input_scale_megapix = node["input_scale_megapix"].as<double>();
    rhs.homo_est_scale_megapix = node["homo_est_scale_megapix"].as<double>();
    rhs.exp_est_scale_megapix = node["exp_est_scale_megapix"].as<double>();
    rhs.seam_finder_scale_megapix = node["seam_finder_scale_megapix"].as<double>();
    rhs.blender_scale_megapix = node["blender_scale_megapix"].as<double>();
    return true;
}

// ========== Feature ==========
Node convert<AllConfig::Feature>::encode(const AllConfig::Feature& rhs) {
    Node node;
    node["features_type"] = rhs.features_type;
    node["matcher_type"] = rhs.matcher_type;
    node["match_conf"] = rhs.match_conf;
    node["max_features"] = rhs.max_features;
    node["matching_thresh"] = rhs.matching_thresh;
    node["match_rw"] = rhs.match_rw;
    return node;
}

bool convert<AllConfig::Feature>::decode(const Node& node, AllConfig::Feature& rhs) {
    if (!node.IsMap()) return false;
    rhs.features_type = node["features_type"].as<std::string>();
    rhs.matcher_type = node["matcher_type"].as<std::string>();
    rhs.match_conf = node["match_conf"].as<double>();
    rhs.max_features = node["max_features"].as<int>();
    rhs.matching_thresh = node["matching_thresh"].as<double>();
    rhs.match_rw = node["match_rw"].as<int>();
    return true;
}

// ========== CameraAdjust ==========
Node convert<AllConfig::CameraAdjust>::encode(const AllConfig::CameraAdjust& rhs) {
    Node node;
    node["adjuster_type"] = rhs.adjuster_type;
    node["wave_correction"] = rhs.wave_correction;
    node["warper_type"] = rhs.warper_type;
    node["estimator_type"] = rhs.estimator_type;
    node["parallax_d0"] = rhs.parallax_d0;
    node["align_depth"] = rhs.align_depth;
    return node;
}

bool convert<AllConfig::CameraAdjust>::decode(const Node& node, AllConfig::CameraAdjust& rhs) {
    if (!node.IsMap()) return false;
    rhs.adjuster_type = node["adjuster_type"].as<std::string>();
    rhs.wave_correction = node["wave_correction"].as<std::string>();
    rhs.warper_type = node["warper_type"].as<std::string>();
    rhs.estimator_type = node["estimator_type"].as<std::string>();
    if (node["parallax_d0"]) rhs.parallax_d0 = node["parallax_d0"].as<double>();
    if (node["align_depth"]) rhs.align_depth = node["align_depth"].as<int>();
    return true;
}

// ========== Exposure ==========
Node convert<AllConfig::Exposure>::encode(const AllConfig::Exposure& rhs) {
    Node node;
    node["exp_type"] = rhs.exp_type;
    node["exp_nr_feeds"] = rhs.exp_nr_feeds;
    node["exp_nr_filtering"] = rhs.exp_nr_filtering;
    node["exp_block_size"] = rhs.exp_block_size;
    return node;
}

bool convert<AllConfig::Exposure>::decode(const Node& node, AllConfig::Exposure& rhs) {
    if (!node.IsMap()) return false;
    rhs.exp_type = node["exp_type"].as<std::string>();
    rhs.exp_nr_feeds = node["exp_nr_feeds"].as<int>();
    rhs.exp_nr_filtering = node["exp_nr_filtering"].as<int>();
    rhs.exp_block_size = node["exp_block_size"].as<int>();
    return true;
}

// ========== Seam ==========
Node convert<AllConfig::Seam>::encode(const AllConfig::Seam& rhs) {
    Node node;
    node["seam_find_type"] = rhs.seam_find_type;
    node["crop"] = rhs.crop;
    node["blender_type"] = rhs.blender_type;
    node["blend_strength"] = rhs.blend_strength;
    return node;
}

bool convert<AllConfig::Seam>::decode(const Node& node, AllConfig::Seam& rhs) {
    if (!node.IsMap()) return false;
    rhs.seam_find_type = node["seam_find_type"].as<std::string>();
    rhs.crop = node["crop"].as<bool>();
    rhs.blender_type = node["blender_type"].as<std::string>();
    rhs.blend_strength = node["blend_strength"].as<int>();
    return true;
}

// ========== Gaze ==========
Node convert<AllConfig::Gaze>::encode(const AllConfig::Gaze& rhs) {
    Node node;
    node["gaze_sigma_ratio"] = rhs.gaze_sigma_ratio;
    node["gaze_alpha"] = rhs.gaze_alpha;
    node["gaze_data_path"] = rhs.gaze_data_path;
    node["gaze_transport"] = rhs.gaze_transport;
    node["gaze_socket_port"] = rhs.gaze_socket_port;
    return node;
}

bool convert<AllConfig::Gaze>::decode(const Node& node, AllConfig::Gaze& rhs) {
    if (!node.IsMap()) return false;
    rhs.gaze_sigma_ratio = node["gaze_sigma_ratio"].as<double>();
    rhs.gaze_alpha = node["gaze_alpha"].as<double>();
    if (node["gaze_data_path"]) rhs.gaze_data_path = node["gaze_data_path"].as<std::string>();
    if (node["gaze_transport"]) rhs.gaze_transport = node["gaze_transport"].as<std::string>();
    if (node["gaze_socket_port"]) rhs.gaze_socket_port = node["gaze_socket_port"].as<int>();
    return true;
}

// ========== Saliency ==========
Node convert<AllConfig::Saliency>::encode(const AllConfig::Saliency& rhs) {
    Node node;
    node["use_gaze"] = rhs.use_gaze;
    node["use_u2net"] = rhs.use_u2net;
    node["fusion_mode"] = rhs.fusion_mode;
    node["gaze_weight"] = rhs.gaze_weight;
    node["u2net_weight"] = rhs.u2net_weight;
    node["u2net_dir"] = rhs.u2net_dir;
    node["display_mode"] = rhs.display_mode;
    return node;
}

bool convert<AllConfig::Saliency>::decode(const Node& node, AllConfig::Saliency& rhs) {
    if (!node.IsMap()) return false;
    rhs.use_gaze = node["use_gaze"].as<bool>();
    rhs.use_u2net = node["use_u2net"].as<bool>();
    rhs.fusion_mode = node["fusion_mode"].as<std::string>();
    rhs.gaze_weight = node["gaze_weight"].as<double>();
    rhs.u2net_weight = node["u2net_weight"].as<double>();
    rhs.u2net_dir = node["u2net_dir"].as<std::string>();
    rhs.display_mode = node["display_mode"].as<std::string>();
    return true;
}

} // namespace YAML

// ========== AllConfig 成员函数 ==========

bool AllConfig::loadFromFile(const std::string& path) {
    try {
        YAML::Node root = YAML::LoadFile(path);
        if (root["stitching"]) {
            auto st = root["stitching"];
            basic = st.as<Basic>();
            scale = st.as<Scale>();
            feature = st.as<Feature>();
            cameraAdjust = st.as<CameraAdjust>();
            exposure = st.as<Exposure>();
            seam = st.as<Seam>();
            gaze = st.as<Gaze>();
            if (st["saliency"]) {
                saliency = st["saliency"].as<Saliency>();
            }
            if (st["result_name"]) {
                result_name = st["result_name"].as<std::string>();
            }
        }
        if (root["camera_info"]) {
            cameraInfos = root["camera_info"].as<std::vector<CameraInfo>>();
        }
        if (root["camera_params"]) {
            cameraParamsList = root["camera_params"].as<std::vector<CameraParams>>();
        }
        return true;
    } catch (const std::exception& e) {
        qDebug() << "Failed to load YAML:" << e.what();
        setDefaults();
        return false;
    }
}

bool AllConfig::saveToFile(const std::string& path) const {
    YAML::Node root;

    // 构建 stitching 节点
    YAML::Node stitching;
    stitching["preview"] = basic.preview;
    stitching["try_cuda"] = basic.try_cuda;
    stitching["log_level"] = basic.log_level;
    stitching["verbose_output"] = basic.verbose_output;
    stitching["camera_count"] = basic.camera_count;
    stitching["output_width"] = basic.output_width;
    stitching["output_height"] = basic.output_height;

    stitching["input_scale_megapix"] = scale.input_scale_megapix;
    stitching["homo_est_scale_megapix"] = scale.homo_est_scale_megapix;
    stitching["exp_est_scale_megapix"] = scale.exp_est_scale_megapix;
    stitching["seam_finder_scale_megapix"] = scale.seam_finder_scale_megapix;
    stitching["blender_scale_megapix"] = scale.blender_scale_megapix;

    stitching["features_type"] = feature.features_type;
    stitching["matcher_type"] = feature.matcher_type;
    stitching["match_conf"] = feature.match_conf;
    stitching["max_features"] = feature.max_features;
    stitching["matching_thresh"] = feature.matching_thresh;
    stitching["match_rw"] = feature.match_rw;

    stitching["adjuster_type"] = cameraAdjust.adjuster_type;
    stitching["wave_correction"] = cameraAdjust.wave_correction;
    stitching["warper_type"] = cameraAdjust.warper_type;
    stitching["estimator_type"] = cameraAdjust.estimator_type;
    stitching["parallax_d0"] = cameraAdjust.parallax_d0;

    stitching["exp_type"] = exposure.exp_type;
    stitching["exp_nr_feeds"] = exposure.exp_nr_feeds;
    stitching["exp_nr_filtering"] = exposure.exp_nr_filtering;
    stitching["exp_block_size"] = exposure.exp_block_size;

    stitching["seam_find_type"] = seam.seam_find_type;
    stitching["crop"] = seam.crop;
    stitching["blender_type"] = seam.blender_type;
    stitching["blend_strength"] = seam.blend_strength;

    stitching["gaze_sigma_ratio"] = gaze.gaze_sigma_ratio;
    stitching["gaze_alpha"] = gaze.gaze_alpha;
    stitching["gaze_data_path"] = gaze.gaze_data_path;
    stitching["gaze_transport"] = gaze.gaze_transport;
    stitching["gaze_socket_port"] = gaze.gaze_socket_port;

    stitching["saliency"]["use_gaze"] = saliency.use_gaze;
    stitching["saliency"]["use_u2net"] = saliency.use_u2net;
    stitching["saliency"]["fusion_mode"] = saliency.fusion_mode;
    stitching["saliency"]["gaze_weight"] = saliency.gaze_weight;
    stitching["saliency"]["u2net_weight"] = saliency.u2net_weight;
    stitching["saliency"]["u2net_dir"] = saliency.u2net_dir;
    stitching["saliency"]["display_mode"] = saliency.display_mode;

    stitching["result_name"] = result_name;

    root["stitching"] = stitching;

    // camera_info
    root["camera_info"] = cameraInfos;

    // camera_params — 矩阵使用流式序列，映射块风格
    YAML::Node cameraParamsNode;
    for (const auto& cp : cameraParamsList) {
        YAML::Node paramNode;
        paramNode["K"] = cp.K;
        paramNode["K"].SetStyle(YAML::EmitterStyle::Flow);
        paramNode["R"] = cp.R;
        paramNode["R"].SetStyle(YAML::EmitterStyle::Flow);
        paramNode["T"] = cp.T;
        paramNode["T"].SetStyle(YAML::EmitterStyle::Flow);
        paramNode["D"] = cp.D;
        paramNode["D"].SetStyle(YAML::EmitterStyle::Flow);
        paramNode["ppx"] = cp.ppx;
        paramNode["ppy"] = cp.ppy;
        paramNode["focal"] = cp.focal;
        paramNode["aspect"] = cp.aspect;
        cameraParamsNode.push_back(paramNode);
    }
    root["camera_params"] = cameraParamsNode;

    // 使用 QSaveFile 原子写入
    QSaveFile file(QString::fromStdString(path));
    if (!file.open(QIODevice::WriteOnly)) {
        qDebug() << "Failed to open file for writing:" << path.c_str();
        return false;
    }

    YAML::Emitter emitter;
    emitter << root;
    file.write(emitter.c_str());
    if (!file.commit()) {
        qDebug() << "Failed to commit file:" << path.c_str();
        return false;
    }
    return true;
}

void AllConfig::setDefaults() {
    // ========== stitching ==========
    basic.preview = false;
    basic.try_cuda = false;
    basic.log_level = 2;
    basic.verbose_output = true;
    basic.camera_count = 3;
    basic.output_width = 1600;
    basic.output_height = 720;

    scale.input_scale_megapix = -1.0;
    scale.homo_est_scale_megapix = 0.6;
    scale.exp_est_scale_megapix = 0.6;
    scale.seam_finder_scale_megapix = 0.1;
    scale.blender_scale_megapix = -1.0;

    feature.features_type = "surf";
    feature.matcher_type = "homography";
    feature.match_conf = 0.3;
    feature.max_features = 1000;
    feature.matching_thresh = 1.0;
    feature.match_rw = -1;

    cameraAdjust.adjuster_type = "ray";
    cameraAdjust.wave_correction = "horiz";
    cameraAdjust.warper_type = "spherical";
    cameraAdjust.estimator_type = "homography";
    cameraAdjust.parallax_d0 = 0.0;   // <=0 关闭近景视差补偿
    cameraAdjust.align_depth = -1;

    exposure.exp_type = "gain_blocks";
    exposure.exp_nr_feeds = 1;
    exposure.exp_nr_filtering = 2;
    exposure.exp_block_size = 32;

    seam.seam_find_type = "gc_color";
    seam.crop = false;
    seam.blender_type = "Blender::FEATHER";
    seam.blend_strength = 5;

    gaze.gaze_sigma_ratio = 0.10;
    gaze.gaze_alpha = 8.0;
    gaze.gaze_data_path = "out/build/x64-Release/gaze_data.bin";
    gaze.gaze_transport = "socket";
    gaze.gaze_socket_port = 5599;

    saliency.use_gaze = true;
    saliency.use_u2net = true;
    saliency.fusion_mode = "max";
    saliency.gaze_weight = 0.7;
    saliency.u2net_weight = 0.3;
    saliency.u2net_dir = "/root/build/SaliencyDetector/saliency_out";
    saliency.display_mode = "combined";

    result_name = "result.jpg";

    // ========== camera_info ==========
    cameraInfos.clear();

    CameraInfo cam0;
    cam0.video_index = 0;
    cam0.description = "left";
    cam0.type = "camera";
    cam0.url = "41";
    cam0.width = 1280;
    cam0.height = 720;
    cam0.undistort = true;
    cam0.model = "pinhole";
    cam0.fps = 30;

    CameraInfo cam1;
    cam1.video_index = 1;
    cam1.description = "middle";
    cam1.type = "camera";
    cam1.url = "45";
    cam1.width = 1280;
    cam1.height = 720;
    cam1.undistort = true;
    cam1.model = "pinhole";
    cam1.fps = 30;

    CameraInfo cam2;
    cam2.video_index = 2;
    cam2.description = "right";
    cam2.type = "camera";
    cam2.url = "43";
    cam2.width = 1280;
    cam2.height = 720;
    cam2.undistort = true;
    cam2.model = "pinhole";
    cam2.fps = 30;

    cameraInfos.push_back(cam0);
    cameraInfos.push_back(cam1);
    cameraInfos.push_back(cam2);

    // ========== camera_params ==========
    cameraParamsList.clear();

    CameraParams p0;
    p0.K = {481.162671, 0.000000, 627.329662,
            0.000000, 480.580224, 350.210243,
            0.000000, 0.000000, 1.000000};
    p0.R = {0.469593566, 0.039565276, -0.881995732,
            0.002510982, 0.998931468, 0.046147775,
            0.882879141, -0.023885374, 0.468992442};
    p0.T = {-0.050951, 0.001201, -0.029803};
    p0.D = {-0.018983, -0.026611, -0.001317, 0.004129, 0.007104};
    p0.ppx = 627.329662;
    p0.ppy = 350.210243;
    p0.focal = 480.871448;
    p0.aspect = 1.0;

    CameraParams p1;
    p1.K = {474.141471, 0.000000, 641.456032,
            0.000000, 471.291605, 337.480734,
            0.000000, 0.000000, 1.000000};
    p1.R = {1.000000000, 0.000000000, 0.000000000,
            0.000000000, 1.000000000, 0.000000000,
            0.000000000, 0.000000000, 1.000000000};
    p1.T = {0.000000, 0.000000, 0.000000};
    p1.D = {-0.020518, -0.021940, -0.001342, 0.000980, 0.005833};
    p1.ppx = 641.456032;
    p1.ppy = 337.480734;
    p1.focal = 472.716538;
    p1.aspect = 1.0;

    CameraParams p2;
    p2.K = {486.292866, 0.000000, 612.388852,
            0.000000, 485.734645, 358.840483,
            0.000000, 0.000000, 1.000000};
    p2.R = {0.519843846, -0.037656216, 0.853430949,
            -0.007583255, 0.998785207, 0.048688860,
            -0.854227645, -0.031782389, 0.518926787};
    p2.T = {0.050675, 0.001867, -0.026109};
    p2.D = {-0.013560, -0.034703, -0.002946, -0.002513, 0.010398};
    p2.ppx = 612.388852;
    p2.ppy = 358.840483;
    p2.focal = 486.013755;
    p2.aspect = 1.0;

    cameraParamsList.push_back(p0);
    cameraParamsList.push_back(p1);
    cameraParamsList.push_back(p2);
}
