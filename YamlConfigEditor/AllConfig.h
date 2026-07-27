#ifndef ALLCONFIG_H
#define ALLCONFIG_H

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

// 相机信息结构
struct CameraInfo {
    int video_index = 0;
    std::string description;
    std::string type;       // "video"、"camera" 或 "rtsp"
    std::string url;
    int width = 1280;
    int height = 720;
    bool undistort = false;
    std::string model = "pinhole";
    int fps = 30;
};

// 相机内参结构
struct CameraParams {
    std::vector<double> K;   // 9个double (内参矩阵)
    std::vector<double> R;   // 9个double (旋转矩阵)
    std::vector<double> T;   // 3个double (平移向量)
    std::vector<double> D;   // 5个double (畸变系数)
    double ppx = 640;
    double ppy = 480;
    double focal = 531.649477;
    double aspect = 1.0;
};

// 全局配置
struct AllConfig {
    // 1. 基础设置
    struct Basic {
        bool preview = false;
        bool try_cuda = false;
        int log_level = 2;
        bool verbose_output = true;
        int camera_count = 3;          // ★ 改为 3，默认三路
        int output_width = 1600;
        int output_height = 720;
    } basic;

    // 2. 缩放参数
    struct Scale {
        double input_scale_megapix = -1.0;
        double homo_est_scale_megapix = 0.6;
        double exp_est_scale_megapix = 0.6;
        double seam_finder_scale_megapix = 0.1;
        double blender_scale_megapix = -1.0;
    } scale;

    // 3. 特征
    struct Feature {
        std::string features_type = "surf";
        std::string matcher_type = "homography";
        double match_conf = 0.3;
        int max_features = 1000;
        double matching_thresh = 1.0;
        int match_rw = -1;
    } feature;

    // 4. 相机调整
    struct CameraAdjust {
        std::string adjuster_type = "ray";
        std::string wave_correction = "horiz";
        std::string warper_type = "spherical";
        std::string estimator_type = "homography";
        double parallax_d0 = 0.0;      // 新增：近景视差补偿默认关闭，不在界面中修改
        int align_depth = -1;
    } cameraAdjust;

    // 5. 曝光
    struct Exposure {
        std::string exp_type = "gain_blocks";
        int exp_nr_feeds = 1;
        int exp_nr_filtering = 2;
        int exp_block_size = 32;
    } exposure;

    // 6. 接缝
    struct Seam {
        std::string seam_find_type = "gc_color";
        bool crop = false;
        std::string blender_type = "Blender::FEATHER";
        int blend_strength = 5;
    } seam;

    // 7. 注视感知
    struct Gaze {
        double gaze_sigma_ratio = 0.10;
        double gaze_alpha = 8.0;
        std::string gaze_data_path = "out/build/x64-Release/gaze_data.bin";
        std::string gaze_transport = "socket";
        int gaze_socket_port = 5599;
    } gaze;

    // 8. 显著性融合
    struct Saliency {
        bool use_gaze = true;
        bool use_u2net = true;
        std::string fusion_mode = "max";
        double gaze_weight = 0.7;
        double u2net_weight = 0.3;
        std::string u2net_dir = "/root/build/SaliencyDetector/saliency_out";
        std::string display_mode = "combined";
    } saliency;

    // 9. 输出文件名
    std::string result_name = "result.jpg";

    // 10. 相机信息列表
    std::vector<CameraInfo> cameraInfos;

    // 11. 相机内参列表
    std::vector<CameraParams> cameraParamsList;

    // 文件操作
    bool loadFromFile(const std::string& path);
    bool saveToFile(const std::string& path) const;
    void setDefaults();
};

// YAML转换特化声明
namespace YAML {
    template<> struct convert<CameraInfo> {
        static Node encode(const CameraInfo& rhs);
        static bool decode(const Node& node, CameraInfo& rhs);
    };
    template<> struct convert<CameraParams> {
        static Node encode(const CameraParams& rhs);
        static bool decode(const Node& node, CameraParams& rhs);
    };
    template<> struct convert<AllConfig::Basic> {
        static Node encode(const AllConfig::Basic& rhs);
        static bool decode(const Node& node, AllConfig::Basic& rhs);
    };
    template<> struct convert<AllConfig::Scale> {
        static Node encode(const AllConfig::Scale& rhs);
        static bool decode(const Node& node, AllConfig::Scale& rhs);
    };
    template<> struct convert<AllConfig::Feature> {
        static Node encode(const AllConfig::Feature& rhs);
        static bool decode(const Node& node, AllConfig::Feature& rhs);
    };
    template<> struct convert<AllConfig::CameraAdjust> {
        static Node encode(const AllConfig::CameraAdjust& rhs);
        static bool decode(const Node& node, AllConfig::CameraAdjust& rhs);
    };
    template<> struct convert<AllConfig::Exposure> {
        static Node encode(const AllConfig::Exposure& rhs);
        static bool decode(const Node& node, AllConfig::Exposure& rhs);
    };
    template<> struct convert<AllConfig::Seam> {
        static Node encode(const AllConfig::Seam& rhs);
        static bool decode(const Node& node, AllConfig::Seam& rhs);
    };
    template<> struct convert<AllConfig::Gaze> {
        static Node encode(const AllConfig::Gaze& rhs);
        static bool decode(const Node& node, AllConfig::Gaze& rhs);
    };
    template<> struct convert<AllConfig::Saliency> {
        static Node encode(const AllConfig::Saliency& rhs);
        static bool decode(const Node& node, AllConfig::Saliency& rhs);
    };
}

#endif // ALLCONFIG_H
