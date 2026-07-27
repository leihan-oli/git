#ifndef TYPES_H
#define TYPES_H

#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/core/mat.hpp>  // 依赖OpenCV的矩阵类型存储图像及参数
#include <chrono>                // 用于高精度时间戳记录
#include <cstdint>               // 提供uint64_t等固定宽度整数类型
#include <vector>                // 用于存储多相机/多帧数据
#include <string>                // 用于路径、类型等字符串描述
#include <iostream>
#include <shared_mutex>

/**
 * @namespace RTStitching
 * @brief 实时图像拼接模块的命名空间，隔离自定义类型与其他库冲突
 */
namespace RTStitching {

    /**
     * @enum scale_t
     * @brief 图像拼接流程中各阶段的缩放尺度枚举，对应不同处理阶段的分辨率需求
     * @note 枚举值顺序与StitchingParams::scale_megapix数组索引一一对应
     */
    typedef enum
    {
        INPUT_SCALE,            // 输入图像原始分辨率（full_img.size().area()）
        HOMO_EST_SCALE,         // 特征提取与单应矩阵估计的中等分辨率
        EXP_EST_SCALE,          // 曝光补偿计算的低分辨率
        SEAM_FINDER_SCALE,      // 接缝查找的低分辨率（兼顾速度与精度）
        BLENDER_SCALE,          // 图像融合阶段的分辨率（决定最终输出精度）
        SCALE_LENGTH            // 缩放尺度的总数量（用于数组大小定义）
    } scale_t;

    /**
     * @struct CameraParams
     * @brief 相机内参与外参结构体，存储相机光学特性及位姿信息
     * @details 用于图像畸变矫正、坐标转换（像素→世界坐标）及多相机位姿对齐
     */
    struct CameraParams {
        cv::Mat K;         // 3x3内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]，焦距与主点坐标
        cv::Mat R;         // 3x3旋转矩阵，描述相机相对世界坐标系的旋转
        cv::Mat T;         // 3x1平移向量，描述相机相对世界坐标系的平移
        cv::Mat D;         // 5x1畸变系数向量 [k1, k2, p1, p2, k3]，用于径向/切向畸变矫正
        double ppx;        // 主点x坐标（像素），等效于K.at<double>(0,2)
        double ppy;        // 主点y坐标（像素），等效于K.at<double>(1,2)
        double focal;      // 焦距（像素），通常取(fx+fy)/2
        double aspect;     // 宽高比（图像宽度/高度），用于分辨率适配

        /**
         * @brief 构造函数，初始化参数为默认值（无畸变、单位位姿）
         */
        CameraParams() :
            K(cv::Mat::eye(3, 3, CV_64F)),  // 内参初始化为单位矩阵（无畸变）
            R(cv::Mat::eye(3, 3, CV_64F)),  // 旋转矩阵初始化为单位矩阵（无旋转）
            T(cv::Mat::zeros(3, 1, CV_64F)), // 平移向量初始化为零（原点位置）
            D(cv::Mat::zeros(5, 1, CV_64F)), // 畸变系数初始化为零（无畸变）
            ppx(0.0),                       // 主点x默认值
            ppy(0.0),                       // 主点y默认值
            focal(0.0),                     // 焦距默认值
            aspect(1.0)                      // 宽高比默认1:1
        {
        }
    };

    /**
     * @struct CameraStitchParams
     * @brief 相机拼接区域参数结构体，定义单路相机图像在全景图中的位置与有效区域
     * @details 用于拼接时的图像映射、区域裁剪及重叠区域计算
     */
    struct CameraStitchParams {
        int csp_ver;                         // CameraStitchParams版本号，初始化时为1
        int sm_ver;                         // seam_mask版本号，初始化时为1
        float scale_ratio                    // 尺度比例，原始画面尺度为1.0
            = 1.0f;                         // （用于多尺度处理时的缩放比例）
		float mid_focal = 0.0f;               // 中值焦距，作为Warper的参数
        std::vector<cv::UMat> masks;         // 单通道二值掩码（1=有效区域，0=无效/边缘区域）
        std::vector<cv::Point> corners;      // 拼接区域四顶点（顺时针排列），定义在全景图中的位置
        std::vector<cv::Size> sizes;         // 拼接后该相机对应区域的分辨率（宽x高）

        /**
         * @brief 构造函数，初始化参数为默认值（vector元素数量均为2）
         */
        /**
         * @brief 构造函数，默认分配 2 个相机的槽位
         * @note  这里只是默认值；真正的相机数由 ConfigParams::camera_count 决定。
         *        update_stitching_params() 会在运行时按 camera_count 把
         *        masks/corners/sizes 动态扩容到 N 路（支持双路/三路/N路拼接）。
         */
        CameraStitchParams() {
            // 1. 初始化masks：默认2个空的单通道掩码（运行时按 camera_count 扩容）
            masks.resize(2);

            // 2. 初始化corners：默认2组角点（运行时按 camera_count 扩容）
            corners.resize(2);

            // 3. 初始化sizes：默认2个分辨率槽位（运行时按 camera_count 扩容）
            sizes.resize(2);
        }
    };

    /**
     * @struct CameraInfo
     * @brief 相机配置信息结构体，存储相机数据源及基础属性（从配置文件解析）
     * @details 用于初始化相机输入（本地设备/视频文件/RTSP流）及预处理参数
     */
    struct CameraInfo {
        int video_index;               // 相机索引（0-based，用于多相机区分，如0=左相机）
        std::string description;       // 相机描述（如"left_cam"、"front_cam"）
        std::string type;              // 数据源类型（枚举值："camera"/"video"/"image"/"rtsp"）
        std::string url;               // 数据源路径（设备号/文件路径/RTSP URL）
        int width;                     // 输出图像宽度（像素）
        int height;                    // 输出图像高度（像素）
        bool undistort;                // 畸变矫正开关（true=启用，使用CameraParams中的D矫正）
        int fps;

        /**
         * @brief 构造函数，初始化参数为默认值（未初始化状态）
         */
        CameraInfo() :
            video_index(0),                  // 默认索引0
            description("WITHOUT_INIT"),     // 未初始化描述
            type("WITHOUT_INIT"),            // 未初始化类型
            url("WITHOUT_INIT"),             // 未初始化路径
            width(1920),                     // 默认宽度1920
            height(1080),                    // 默认高度1080
            undistort(false),                 // 默认不启用畸变矫正
            fps(1)
        {
        }
    };

    /**
     * @struct Image
     * @brief 图像数据结构体，封装图像帧数据及元信息（时间戳/序号）
     * @details 用于多相机数据同步、帧序管理及拼接流程中的数据传递
     */
    struct Image {
        cv::Mat data;                 // 图像数据（支持CV_8UC3/RGB、CV_8UC1/灰度等格式）
        cv::Mat mask;
        std::chrono::high_resolution_clock::time_point timestamp;  // 采集时间戳（ns级精度）
        uint64_t img_idx;             // 帧序号（单调递增，用于帧丢失检测）

        /**
         * @brief 构造函数，初始化参数为默认值（空图像）
         */
        Image() :
            data(cv::Mat()),                                  // 图像初始化为空
            timestamp(std::chrono::high_resolution_clock::now()),  // 时间戳初始化为当前时间
            img_idx(0)                                    // 帧序号初始化为0
        {
        }
    };

    /**
     * @struct ConfigParams
     * @brief 图像拼接算法总配置参数结构体，控制拼接全流程的行为
     * @details 包含从输入到输出的所有可配置参数，支持从配置文件加载或代码中修改
     */
    struct ConfigParams {
        // 1. 基础配置
        bool preview;            // 是否启用实时预览窗口（调试用）
        bool try_cuda;           // 是否尝试使用CUDA加速（需OpenCV编译时支持CUDA）
        int log_level;           // 日志级别（0=无日志，1=错误，2=警告，3=信息，4=调试）
        bool verbose_output;     // 是否输出详细运行信息（如特征点数量、匹配耗时等）
        int camera_count;        // 参与拼接的相机数量
        int output_width;
        int output_height;
        // 2. 图像缩放尺度参数（与scale_t枚举对应）
        double scale_megapix[SCALE_LENGTH];  // 各阶段缩放至目标百万像素（-1=不缩放）

        // 3. 特征检测与匹配参数
        std::string features_type;    // 特征检测器类型（"ORB"/"SIFT"/"SURF"/"AKAZE"）
        std::string matcher_type;     // 匹配器类型（"homography"/"affine"等）
        float match_conf;             // 匹配对筛选置信度阈值（如ORB默认0.3）
        int max_features;             // 单幅图像最大特征点数量（控制计算量）
        double matching_thresh;       // 特征匹配距离阈值（小于该值视为有效匹配）
        int match_rw;                 // 匹配范围宽度（用于RangeMatcher的区域限制，-1=无限制）

        // 4. 相机姿态与几何校正参数
        std::string adjuster_type;    // 相机参数调整器类型（"ray"/"reproj"等，优化内外参）
        std::string wave_correction;  // 波形校正模式（"horiz"/"vert"/"no"，消除拼接波动）
        std::string warper_type;      // 图像扭曲器类型（"spherical"/"cylindrical"/"plane"等）
        std::string estimator_type;
        // [新增] 近景视差补偿工作距离（米）。<=0 = 关闭（纯旋转，原行为）。
        //   >0 时球面 warp 假设场景在半径 d0 的球面上，并用 camera_params[i].T
        //   （相机光心在 world 系的位置，由标定外参导出）补偿视差：
        //   d0 处物体严格对齐，偏离 d0 的深度错位为 f·b·|1/d − 1/d0|。
        //   仅 warper_type == "spherical" 时生效。
        double parallax_d0 = 0.0;
        
        // 5. 曝光补偿参数
        std::string exp_type;         // 曝光补偿类型（"gain"/"gain_blocks"/"no"）
        int exp_nr_feeds;             // 曝光补偿迭代次数（增益传播次数）
        int exp_nr_filtering;         // 曝光补偿过滤迭代次数（平滑增益）
        int exp_block_size;           // 分块曝光补偿的块大小（像素，如32x32）

        // 6. 接缝计算与融合参数
        std::string seam_find_type;   // 接缝查找器类型（"gc_color"/"gc_colorgrad"/"no"）
        bool crop;                    // 是否自动裁剪拼接后的黑边区域
        std::string blender_type;     // 融合器类型（"multiband"/"feather"/"no"）
        int blend_strength;           // 融合强度（用于计算融合带宽，值越大过渡越平滑）

        float gaze_sigma_ratio;       //避让范围
		//float gaze_saliency_threshold;//避让显著性阈值
        float gaze_alpha;             //注视感知图割能量调制强度，0=禁用，4~6 推荐

        // --- 显著性来源融合（眼动 + U2-Net）---
        bool        sal_use_gaze;     // 启用眼动注视显著性
        bool        sal_use_u2net;    // 启用 U2-Net 深度学习显著性
        std::string sal_fusion_mode;  // 两路都开时融合方式: "max"/"weighted"/"mul"
        float       sal_gaze_weight;  // weighted 模式眼动权重
        float       sal_u2net_weight; // weighted 模式 U2-Net 权重
        std::string sal_u2net_dir;    // Python saliency_writer.py 写出目录（C++ 读取）
        std::string sal_display_mode; // "separate"=分窗对比 / "combined"=融合后单窗

        // 眼动数据文件路径（相对 config.yaml 目录解析；Tobii 端写、C++ 端读）
        std::string gaze_data_path;

        // 眼动数据传输方式：file=本地文件(Windows测试) / socket=TCP接收(RK3588)
        std::string gaze_transport;   // "file" / "socket"
        int         gaze_socket_port; // socket 模式 TCP 监听端口

		bool camera_param_est_flag; // 相机参数估计标志（位掩码，控制估计行为）
        bool seamfinder_flag; // 相机参数估计标志（位掩码，控制估计行为）

        // 7. 输出参数
        std::string result_name;      // 拼接结果图像保存路径（如"result.jpg"）

        // 动态参数列表（与相机数量关联）
        std::vector<CameraParams> camera_params;         // 每个相机的内参/外参
        std::vector<CameraInfo> camera_info;             // 每个相机的配置信息

        /**
         * @brief 构造函数，初始化所有参数为默认值
         */
        ConfigParams() :
            preview(false),
            try_cuda(false),
            log_level(0),
            verbose_output(false),
            camera_count(2),  // 默认2路相机拼接
            output_width(669),
            output_height(749),
            // 缩放尺度：输入不缩放，特征提取0.6MP，曝光/接缝0.1MP，融合不缩放
            scale_megapix{ -1, 0.6, 0.1, 0.1, -1 },
            max_features(1000),
            matching_thresh(1.0),
            match_conf(0.3f),
            match_rw(-1),
            exp_nr_feeds(1),
            exp_nr_filtering(2),
            exp_block_size(32),
            features_type("surf"),
            matcher_type("homography"),
            estimator_type("affine"),
            adjuster_type("ray"),
            wave_correction("horiz"),
            warper_type("spherical"),
            exp_type("GAIN_BLOCKS"),
            seam_find_type("gc_color"),
            crop(false),
            blender_type("Blender::MULTI_BAND"),
            blend_strength(5),
            gaze_sigma_ratio(0.1),
            gaze_alpha(4.0f),
            sal_use_gaze(true),
            sal_use_u2net(false),
            sal_fusion_mode("max"),
            sal_gaze_weight(0.5f),
            sal_u2net_weight(0.5f),
            sal_u2net_dir("./saliency_out"),
            sal_display_mode("combined"),
            gaze_data_path("out/build/x64-Release/gaze_data.bin"),
            gaze_transport("file"),
            gaze_socket_port(5599),
            camera_param_est_flag(false),
            seamfinder_flag(false),
            result_name("result.jpg")
        {
            // 初始化与相机数量关联的参数列表
            camera_params.resize(camera_count);
            camera_info.resize(camera_count);
        }
    };

    // 新增：曝光补偿器数据包装结构
    struct ExposureCompensatorWrapper {
        cv::Ptr<cv::detail::ExposureCompensator> compensator;  // OpenCV智能指针，自动管理生命周期
        std::vector<cv::Point> corners;                        // 角点信息，用于补偿应用
        std::vector<cv::UMat> masks;                           // 掩码信息，用于补偿应用
        bool is_estimated;                                     // 标记是否已完成估计
        uint64_t frame_id;                                     // 帧ID，用于数据同步
        std::chrono::high_resolution_clock::time_point timestamp; // 时间戳

        ExposureCompensatorWrapper() : is_estimated(false), frame_id(0) {
            timestamp = std::chrono::high_resolution_clock::now();
        }
    };
}  // namespace RTStitching

#endif  // TYPES_H