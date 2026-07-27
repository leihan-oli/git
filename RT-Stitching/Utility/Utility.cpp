#include "Utility.hpp"

// 跨平台键盘按键检测函数（兼容Windows/Linux/Unix）
// 返回值：1=有按键按下，0=无按键按下
int kbHit() {
#ifdef _WIN32
    return _kbhit(); // Windows平台：直接调用系统API检测按键
#else
    struct termios oldt, newt;  // 终端属性结构体（保存原始/修改后属性）
    int ch;                     // 读取的按键值
    int oldf;                   // 文件描述符原始状态

    tcgetattr(STDIN_FILENO, &oldt); // 获取标准输入终端原始属性
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 关闭规范模式（无需回车）和回显（不显示按键）
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // 立即应用新终端属性
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); // 设置非阻塞读取（避免阻塞程序）

    ch = getchar(); // 尝试读取按键

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 恢复终端原始属性
    fcntl(STDIN_FILENO, F_SETFL, oldf);      // 恢复文件描述符状态

    if (ch != EOF) {
        ungetc(ch, stdin); // 若读取到按键，放回输入流（供后续getKey获取）
        return 1;
    }
    return 0;
#endif
}

// 跨平台键盘按键获取函数（兼容Windows/Linux/Unix）
// 返回值：按下的按键ASCII码（ESC键为27）
int getKey() {
#ifdef _WIN32
    return _getch(); // Windows平台：直接调用系统API获取按键
#else
    return getchar(); // Linux/Unix平台：从输入流读取按键
#endif
}

/*
    功能：根据基础尺度的相机参数，批量更新所有尺度的相机拼接参数
    核心逻辑：
        1. 计算所有相机焦距中值（变形器初始化关键参数）
        2. 基于基础尺度掩码，通过变形器计算各尺度下的图像尺寸、角点、掩码
        3. 存储计算结果到拼接参数结构体，供后续拼接流程使用
    输入参数：
        config_params：全局配置参数（含变形器类型、相机数量等）
        stitching_params：待更新的各尺度拼接参数（输出参数，存储尺寸/角点/掩码）
        camera_params：基础尺度下的相机参数（内参focal、外参等）
        base_scale：基础尺度标识（参数基准，通常为INPUT_SCALE）
*/
// ============================================================
// [新增] 近景视差补偿的球面映射实现（声明见 Utility.hpp）
//   数学与 OpenCV SphericalProjector 完全同式，仅在 d0>0 时把
//   "世界方向 = R·K⁻¹·u" 替换为 "射线与半径 d0 球面交点的方向"：
//     前向:  r̂w = normalize(R·K⁻¹·u)
//            t  = -(C·r̂w) + sqrt((C·r̂w)² - |C|² + d0²)
//            dir = (C + t·r̂w) / d0
//            u = scale·atan2(dir.x, dir.z),  v = scale·(π - acos(dir.y/|dir|))
//     后向:  dir = 球面反投影(u,v)
//            Xc = Rᵀ·(d0·dir − C)   （d0<=0 时 Xc = Rᵀ·dir）
//            像素 = K·Xc/Xc.z（Xc.z<=0 判无效，映射填 -1e5 → remap 出黑边）
//   该实现已与离线验证脚本(verify_stitch_offline.py)逐点比对到 1e-6 像素。
// ============================================================
namespace RTStitching {

cv::Rect buildSphericalMapsD0(
    cv::Size src_size,
    const cv::Mat& K_in, const cv::Mat& R_in, const cv::Mat& T_in,
    float scale, double d0,
    cv::Mat& xmap, cv::Mat& ymap)
{
    cv::Mat K64, R64;
    K_in.convertTo(K64, CV_64F);
    R_in.convertTo(R64, CV_64F);
    const double fx = K64.at<double>(0, 0), cx = K64.at<double>(0, 2);
    const double fy = K64.at<double>(1, 1), cy = K64.at<double>(1, 2);
    cv::Matx33d R(R64);
    cv::Matx33d Rt = R.t();

    cv::Vec3d C(0.0, 0.0, 0.0);
    if (!T_in.empty()) {
        cv::Mat T64;
        T_in.convertTo(T64, CV_64F);
        if (T64.total() >= 3) {
            const double* p = T64.ptr<double>();
            C = cv::Vec3d(p[0], p[1], p[2]);
        }
    }
    const bool comp = (d0 > 1e-9);
    const double C2 = C.dot(C);
    const double s = static_cast<double>(scale);

    // ---- 前向投影源图像边界，确定 pano ROI ----
    double umin = DBL_MAX, vmin = DBL_MAX, umax = -DBL_MAX, vmax = -DBL_MAX;
    auto fwd = [&](double px, double py) {
        cv::Vec3d ray((px - cx) / fx, (py - cy) / fy, 1.0);
        cv::Vec3d rw = R * ray;
        rw /= cv::norm(rw);
        cv::Vec3d dir = rw;
        if (comp) {
            const double cr = C.dot(rw);
            const double t = -cr + std::sqrt(std::max(cr * cr - C2 + d0 * d0, 0.0));
            dir = (C + t * rw) * (1.0 / d0);
        }
        const double n = cv::norm(dir);
        const double u = s * std::atan2(dir[0], dir[2]);
        const double v = s * (CV_PI - std::acos(std::min(1.0, std::max(-1.0, dir[1] / n))));
        umin = std::min(umin, u); umax = std::max(umax, u);
        vmin = std::min(vmin, v); vmax = std::max(vmax, v);
    };
    for (int x = 0; x < src_size.width; ++x) { fwd(x, 0.0); fwd(x, src_size.height - 1.0); }
    for (int y = 0; y < src_size.height; ++y) { fwd(0.0, y); fwd(src_size.width - 1.0, y); }

    cv::Rect roi(static_cast<int>(std::floor(umin)),
                 static_cast<int>(std::floor(vmin)),
                 static_cast<int>(std::ceil(umax)) - static_cast<int>(std::floor(umin)) + 1,
                 static_cast<int>(std::ceil(vmax)) - static_cast<int>(std::floor(vmin)) + 1);

    // ---- 后向映射表（尺寸约定与 RotationWarperBase::buildMaps 一致：+1）----
    xmap.create(roi.height + 1, roi.width + 1, CV_32FC1);
    ymap.create(roi.height + 1, roi.width + 1, CV_32FC1);
    for (int y = 0; y < xmap.rows; ++y) {
        float* mx = xmap.ptr<float>(y);
        float* my = ymap.ptr<float>(y);
        const double vv = (roi.y + y) / s;
        const double sinv = std::sin(CV_PI - vv);
        const double wy = std::cos(CV_PI - vv);
        for (int x = 0; x < xmap.cols; ++x) {
            const double uu = (roi.x + x) / s;
            cv::Vec3d dir(sinv * std::sin(uu), wy, sinv * std::cos(uu));
            cv::Vec3d Xc = comp ? (Rt * (d0 * dir - C)) : (Rt * dir);
            if (Xc[2] > 1e-9) {
                mx[x] = static_cast<float>(fx * Xc[0] / Xc[2] + cx);
                my[x] = static_cast<float>(fy * Xc[1] / Xc[2] + cy);
            }
            else {
                mx[x] = -1e5f;
                my[x] = -1e5f;
            }
        }
    }
    return roi;
}

cv::Point warpSphericalD0(
    cv::InputArray src,
    const cv::Mat& K, const cv::Mat& R, const cv::Mat& T_center,
    float scale, double d0,
    int interp_mode, int border_mode,
    cv::OutputArray dst)
{
    cv::Mat xmap, ymap;
    const cv::Rect roi = buildSphericalMapsD0(src.size(), K, R, T_center,
                                              scale, d0, xmap, ymap);
    dst.create(roi.height + 1, roi.width + 1, src.type());
    cv::remap(src, dst, xmap, ymap, interp_mode, border_mode);
    return roi.tl();
}

} // namespace RTStitching


void update_stitching_params(
    RTStitching::ConfigParams& config_params,
    std::vector<RTStitching::CameraStitchParams>& stitching_params,
    RTStitching::scale_t base_scale)
{
    

    // 步骤1：计算所有相机焦距的中值（用于统一变形器的焦距基准）
    float mid_focal;                // 焦距中值（最终结果）
    std::vector<double> focals;     // 存储所有相机的焦距

    for (size_t i = 0; i < config_params.camera_params.size(); ++i)
    {
        focals.push_back(config_params.camera_params[i].focal); // 收集每个相机的焦距
    }
    sort(focals.begin(), focals.end()); // 焦距排序（升序）
    if (focals.size() % 2 == 1)
        // 奇数个相机：取中间位置的焦距值
        mid_focal = static_cast<float>(focals[focals.size() / 2]);
    else
        // 偶数个相机：取中间两个焦距的平均值
        mid_focal = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;
    //std::cout << "LH: [update_stitching_params]: 步骤1完成 - 焦距中值:" << mid_focal << std::endl;

    // 步骤2：计算基础尺度下的图像尺寸（原始尺寸 × 基础尺度比例）
    cv::Size base_size = cv::Size(
        config_params.camera_info[0].width * stitching_params[base_scale].scale_ratio,
        config_params.camera_info[0].height * stitching_params[base_scale].scale_ratio
    );
    cv::UMat base_mask;    // 基础尺度掩码（全白，用于变形计算）
    cv::UMat warped_mask;  // 变形后的掩码（各尺度输出）
    cv::Point warped_corner; // 变形后图像的左上角角点坐标
    base_mask.create(base_size, CV_8U); // 创建基础掩码（尺寸=基础尺度图像尺寸，类型=单通道8位）
    base_mask.setTo(cv::Scalar::all(255)); // 掩码设为全白（255表示有效区域）
    
    // 步骤3：遍历所有尺度，更新各尺度拼接参数
    for (int scale_cnt = RTStitching::INPUT_SCALE; scale_cnt < RTStitching::SCALE_LENGTH; scale_cnt++)
    {
        //std::cout << "LH: [update_stitching_params]: 开始处理尺度:" << scale_cnt << std::endl;
        // 计算当前尺度的焦距中值（基础中值 × 当前尺度比例）
        stitching_params[scale_cnt].mid_focal = mid_focal * stitching_params[scale_cnt].scale_ratio /
                stitching_params[base_scale].scale_ratio;
        // 根据配置创建对应的图像变形器（旋转投影类型）

        
    // warper related
        cv::Ptr<cv::WarperCreator> warper_creator;
        cv::Ptr<cv::detail::RotationWarper> warper;
        warper_creator = WarperModule::initWarperCreator(config_params.warper_type, false);
        warper = warper_creator->create(stitching_params[scale_cnt].mid_focal);

        stitching_params[scale_cnt].csp_ver++;

        // [N路适配] CameraStitchParams 构造时默认只分配了 2 个相机的槽位，
        //   当 camera_count > 2（如三路拼接）时，下面按 i 写入 sizes[i]/corners[i]/masks[i]
        //   会越界。这里按实际相机数动态扩容，保证 2/3/N 路通用。
        if (stitching_params[scale_cnt].sizes.size() < static_cast<size_t>(config_params.camera_count)) {
            stitching_params[scale_cnt].sizes.resize(config_params.camera_count);
            stitching_params[scale_cnt].corners.resize(config_params.camera_count);
            stitching_params[scale_cnt].masks.resize(config_params.camera_count);
        }

        // 遍历所有相机，计算当前尺度下的变形参数
        for (size_t i = 0; i < config_params.camera_count; ++i) {
            // 对基础尺度掩码执行变形：
            // 输入：基础掩码、相机参数；输出：当前尺度变形掩码、角点坐标
            // [新增] 传入 parallax_d0 与本尺度 warper 焦距 —— 掩码/角点必须与
            //   Warper 热路径用同一套（补偿后的）几何，否则 Blender 放置错位。
            WarperModule::doWarp(
                warper,                  // 变形器实例
                base_mask,               // 基础尺度掩码（输入）
                config_params.camera_params[i],        // 第i个相机的参数
                // 尺度比例系数（当前尺度/基础尺度）
                stitching_params[scale_cnt].scale_ratio / stitching_params[base_scale].scale_ratio,
                cv::INTER_LINEAR,        // 插值方式（线性插值，平衡速度与精度）
                cv::BORDER_CONSTANT,     // 边界填充方式（常量填充）
                warped_mask,             // 变形后的掩码（输出）
                warped_corner,           // 变形后图像的角点（输出）
                config_params.parallax_d0,                       // [新增] 视差补偿工作距离
                stitching_params[scale_cnt].mid_focal            // [新增] 本尺度 warper 焦距
            );
            // 保存当前尺度下第i个相机的拼接参数
            stitching_params[scale_cnt].sizes[i] = warped_mask.size();   // 变形后图像尺寸
            stitching_params[scale_cnt].corners[i] = warped_corner;     // 变形后角点坐标
            
            if (scale_cnt != RTStitching::SEAM_FINDER_SCALE) {
                stitching_params[scale_cnt].masks[i] = warped_mask;         // 变形后掩码
            }
            
            // 打印调试信息（验证参数正确性）
            //std::cout << "LH: [update_stitching_params]: 相机" << i << " - stitching_params版本: " << stitching_params[scale_cnt].csp_ver << std::endl;
            //std::cout << "LH: [update_stitching_params]: 相机" << i << " - 基础掩码尺寸: " << base_mask.size() << std::endl;
            //std::cout << "LH: [update_stitching_params]: 相机" << i << " - 变形角点: " << warped_corner << std::endl;
            //std::cout << "LH: [update_stitching_params]: 相机" << i << " - 变形后掩码尺寸: " << warped_mask.size() << std::endl;
            //std::cout << "LH: [update_stitching_params]: 相机" << i << " - 当前尺度焦距中值: " << stitching_params[scale_cnt].mid_focal << std::endl;
        }
        warper.release(); // 释放变形器资源（避免内存泄漏）
        
        // 步骤4：若相机数量≥2，可视化前两个相机的变形掩码（调试用）
        //if (config_params.camera_count >= 2) {
        //    CameraParamEst::visualize_stitching_params(stitching_params);
        //}
    }
}
// Type check before blend code
std::string typeToString_tmp(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

std::string matToString(const cv::Mat& mat) {
    std::stringstream ss;
    ss << mat;
    return ss.str();
}
// 将相机参数转换为字符串的函数
std::string cameraParamsToString(const cv::Mat& K, const cv::Mat& R) {
    cv::Vec3d euler_rad = matrixToEuler(R);

    std::ostringstream oss;

    // 格式化K矩阵
    oss << "K:[" << K.at<double>(0, 0) << "," << K.at<double>(0, 1) << "," << K.at<double>(0, 2) << "|"
        << K.at<double>(1, 0) << "," << K.at<double>(1, 1) << "," << K.at<double>(1, 2) << "|"
        << K.at<double>(2, 0) << "," << K.at<double>(2, 1) << "," << K.at<double>(2, 2) << "] ";

    // 格式化欧拉角（度）
    oss << "Euler(deg):[Yaw:" << rad2deg(euler_rad[2])
        << ",Pitch:" << rad2deg(euler_rad[1])
        << ",Roll:" << rad2deg(euler_rad[0]) << "]";

    return oss.str();
}
// 将旋转矩阵转换为欧拉角的函数
cv::Vec3d matrixToEuler(const cv::Mat& R) {
    CV_Assert(R.rows == 3 && R.cols == 3 && R.type() == CV_64F);

    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;

    double x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }

    return cv::Vec3d(x, y, z); // 返回弧度值 [roll, pitch, yaw]
}
// 弧度转角度函数
inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

cv::Mat eulerToMatrix(double roll, double pitch, double yaw) {
    cv::Mat rotation_matrix;
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;


    // 使用Rodrigues变换或分别计算每个轴的旋转
    cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(roll_rad), -sin(roll_rad),
        0, sin(roll_rad), cos(roll_rad));

    cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
        cos(pitch_rad), 0, sin(pitch_rad),
        0, 1, 0,
        -sin(pitch_rad), 0, cos(pitch_rad));

    cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
        cos(yaw_rad), -sin(yaw_rad), 0,
        sin(yaw_rad), cos(yaw_rad), 0,
        0, 0, 1);

    rotation_matrix = R_z * R_y * R_x;
    return rotation_matrix;
}


