#include "CameraParamEst.hpp"
#include <opencv2/stitching/detail/autocalib.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>
#include "Utility.hpp"

using namespace cv;
using namespace std;

#include <cmath>



CircularBuffer<std::vector<RTStitching::CameraParams>> CameraParams_filter_(10);

CameraParamEst::CameraParamEst(
    RTStitching::ConfigParams& config_params,
    std::vector<RTStitching::CameraStitchParams>& stitch_params,
    CircularBuffer<std::vector<cv::detail::ImageFeatures>>& input_buffer,
    bool show_window   // [新增] 调试可视化开关
) :
    last_processing_time_(0.0),
    show_window_(show_window),
    is_running_(false),
    is_paused_(false),
    stop_requested_(false),
    try_cuda_(config_params.try_cuda),
    match_conf_(config_params.match_conf),
    conf_thresh_(config_params.match_conf),
    range_width_(config_params.match_rw),
    matcher_type_(config_params.matcher_type),
    estimator_type_(config_params.estimator_type),
    adjuster_type_(config_params.adjuster_type),
    warper_type_(config_params.warper_type),
    wave_correction_(config_params.wave_correction),
    config_params_(config_params),
    stitch_params_(stitch_params),
    input_buffer_(input_buffer)
{
    calculate_RT_camera_params_.reserve(config_params_.camera_count);
    RT_camera_params_.reserve(config_params_.camera_count);
    if (!initializeFeatureMatcher(matcher_type_, try_cuda_)) {
        last_error_ = "Failed to initialize feature matcher with type: " + matcher_type_;
    }

    if (!initializeMotionEstimator(estimator_type_)) {
        last_error_ = "Failed to initialize motion estimator with type: " + estimator_type_;
    }

    if (!initializeBundleAdjuster(adjuster_type_)) {
        last_error_ = "Failed to initialize bundle adjuster with type: " + adjuster_type_;
    }
}

CameraParamEst::~CameraParamEst() {
    stop();
}

bool CameraParamEst::start() {
    lock_guard<mutex> lock(mutex_);

    if (is_running_) return false;

    is_running_.store(true);
    is_paused_.store(false);
    stop_requested_.store(false);

    worker_thread_ = thread(&CameraParamEst::runImpl, this);

    spdlog::info("[CAMERAPARAMEST] FeatureToWarp thread started");
    return true;
}

void CameraParamEst::stop() {
    if (!is_running_.load()) return;

    stop_requested_.store(true);
    condition_.notify_all();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    is_running_.store(false);
    is_paused_.store(false);

    spdlog::info("[CAMERAPARAMEST] FeatureToWarp thread stopped");
}

void CameraParamEst::pause() {
    is_paused_.store(true);
    spdlog::info("[CAMERAPARAMEST] FeatureToWarp thread paused");
}

void CameraParamEst::resume() {
    if (is_paused_.load()) {
        is_paused_.store(false);
        condition_.notify_one();
        spdlog::info("[CAMERAPARAMEST] FeatureToWarp thread resumed");
    }
}

bool CameraParamEst::isRunning() const {
    return is_running_.load(memory_order_acquire);
}

bool CameraParamEst::isPaused() const {
    return is_paused_.load(memory_order_acquire);
}


void CameraParamEst::runImpl() {
    spdlog::info("[CAMERAPARAMEST] FeatureToWarp thread main loop started");

    while (!stop_requested_.load()) {
        {
            unique_lock<mutex> lock(mutex_);
            condition_.wait(lock, [this] {
                return stop_requested_.load() || !is_paused_.load();
                });
        }

        if (stop_requested_.load()) {
            break;
        }

        if (!input_buffer_.empty()) {
            try {
                vector<detail::ImageFeatures> features;
                
                if (input_buffer_.try_pop_front(features)) {
                    auto start_time = chrono::high_resolution_clock::now();
                    
                    if (!FeatureMatching(features)) {
                        spdlog::error("[CAMERAPARAMEST] FeatureMatching failed: {}", last_error_);
                        continue;
                    }
                    
                    if (!CameraEstimation(features)) {
                        spdlog::error("[CAMERAPARAMEST] CameraEstimation failed: {}", last_error_);
                        continue;
                    }
                    
                    if (!BundleAdjustment(features)) {
                        spdlog::error("[CAMERAPARAMEST] BundleAdjustment failed: {}", last_error_);
                        continue;
                    }
                    
                    if (!WaveCorrection()) {
                        spdlog::error("[CAMERAPARAMEST] WaveCorrection failed: {}", last_error_);
                        continue;
                    }
                    
                    update_camera_params();
                    update_stitching_params(config_params_, stitch_params_, RTStitching::HOMO_EST_SCALE);
                    config_params_.camera_param_est_flag = true;
                    config_params_.seamfinder_flag = false;
                    // [修改] 由 MODULE_CAMERA_PARAM_EST_DEBUG 控制是否输出掩膜总览调试图
                    if (show_window_ && config_params_.camera_count >= 2) {
                        CameraParamEst::visualize_stitching_params(stitch_params_);
                    }
                    auto end_time = chrono::high_resolution_clock::now();
                    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
                    last_processing_time_ = duration.count();

                    int camera_index_ = 0;
                    std::string camera_info = cameraParamsToString(camera_params_[camera_index_].K(), camera_params_[camera_index_].R);
                    std::cout << "[LH: Camera " << camera_index_ << " Params V_" << stitch_params_[RTStitching::HOMO_EST_SCALE].csp_ver << ": " << camera_info << std::endl;
                    camera_index_ = 1;
                    camera_info = cameraParamsToString(camera_params_[camera_index_].K(), camera_params_[camera_index_].R);
                    std::cout << "[LH: Camera " << camera_index_ << " Params V_" << stitch_params_[RTStitching::HOMO_EST_SCALE].csp_ver << ": " << camera_info << std::endl;


                    spdlog::info("[CAMERAPARAMEST] FeatureToWarp processing completed successfully in {}ms", last_processing_time_);
                }
            }
            catch (const exception& e) {
                last_error_ = string("Processing exception: ") + e.what();
                spdlog::error("[CAMERAPARAMEST] {}", last_error_);
            }
        }

        this_thread::sleep_for(chrono::milliseconds(10));
    }

    spdlog::info("[CAMERAPARAMEST] FeatureToWarp thread main loop ended");
}

bool CameraParamEst::FeatureMatching(const vector<detail::ImageFeatures>& features) {
    if (!validateInputFeatures(features)) {
        return false;
    }

    if (!feature_matcher_) {
        last_error_ = "Feature matcher not initialized";
        return false;
    }

    try {
        pairwise_matches_.clear();
        (*feature_matcher_)(features, pairwise_matches_);
        feature_matcher_->collectGarbage();

        if (pairwise_matches_.empty()) {
            last_error_ = "No feature matches found";
            return false;
        }

        // 检查所有匹配对的内点数量和有效性
        for (size_t i = 0; i < pairwise_matches_.size(); ++i) {
            if (pairwise_matches_[1].num_inliers < 10) {
                last_error_ = "Insufficient inliers in match " + std::to_string(i);
                return false;
            }
            // 可选：检查匹配点坐标是否在图像范围内
            const auto& img1 = features[pairwise_matches_[i].src_img_idx];
            const auto& img2 = features[pairwise_matches_[i].dst_img_idx];
            for (const auto& m : pairwise_matches_[i].matches) {
                const auto& kp1 = img1.keypoints[m.queryIdx];
                const auto& kp2 = img2.keypoints[m.trainIdx];
                if (kp1.pt.x < 0 || kp1.pt.x >= img1.img_size.width ||
                    kp1.pt.y < 0 || kp1.pt.y >= img1.img_size.height ||
                    kp2.pt.x < 0 || kp2.pt.x >= img2.img_size.width ||
                    kp2.pt.y < 0 || kp2.pt.y >= img2.img_size.height) {
                    last_error_ = "Invalid match coordinates in pair " + std::to_string(i);
                    return false;
                }
            }
        }

        spdlog::info("[CAMERAPARAMEST] Feature matching completed: {} matches", pairwise_matches_.size());
        return true;
    }
    catch (const cv::Exception& e) {
        last_error_ = string("OpenCV exception in feature matching: ") + e.what();
        return false;
    }
}

bool CameraParamEst::CameraEstimation(const vector<detail::ImageFeatures>& features) {
    if (!motion_estimator_) {
        last_error_ = "Motion estimator not initialized";
        return false;
    }

    try {
        camera_params_.clear();
        if (!(*motion_estimator_)(features, pairwise_matches_, camera_params_)) {
            last_error_ = "Camera estimation failed";
            return false;
        }
        spdlog::info("[CAMERAPARAMEST] [[[Camera estimation:]]]");
        for (size_t i = 0; i < pairwise_matches_.size(); i++)
        {
            spdlog::info("[CAMERAPARAMEST] pairwise_matches_ matches count: {}", pairwise_matches_[i].matches.size());
            spdlog::info("[CAMERAPARAMEST] pairwise_matches_ inliers count: {}", pairwise_matches_[i].num_inliers);
        }
        spdlog::info("[CAMERAPARAMEST] camera_params_[0].K: {}", matToString(camera_params_[0].K()));
        spdlog::info("[CAMERAPARAMEST] camera_params_[0].R: {}", matToString(camera_params_[0].R));
        spdlog::info("[CAMERAPARAMEST] camera_params_[1].K: {}", matToString(camera_params_[1].K()));
        spdlog::info("[CAMERAPARAMEST] camera_params_[1].R: {}", matToString(camera_params_[1].R));
        spdlog::info("[CAMERAPARAMEST] Camera estimation completed: {} cameras", camera_params_.size());
        return true;
    }
    catch (const cv::Exception& e) {
        last_error_ = string("OpenCV exception in camera estimation: ") + e.what();
        return false;
    }
}

bool CameraParamEst::BundleAdjustment(const vector<detail::ImageFeatures>& features) {
    if (!bundle_adjuster_) {
        last_error_ = "Bundle adjuster not initialized";
        return false;
    }

    try {
        vector<cv::Mat> original_R;
        
        for (auto& cam : camera_params_) {
            original_R.push_back(cam.R.clone());
            if (cam.R.type() != CV_32F) {
                cam.R.convertTo(cam.R, CV_32F);
            }
        }
        
        bundle_adjuster_->setConfThresh(conf_thresh_);
        bool success = (*bundle_adjuster_)(features, pairwise_matches_, camera_params_);

        spdlog::info("[CAMERAPARAMEST] Camera BundleAdjustment:");
        spdlog::info("[CAMERAPARAMEST] camera_params_[0].K: {}", matToString(camera_params_[0].K()));
        spdlog::info("[CAMERAPARAMEST] camera_params_[0].R: {}", matToString(camera_params_[0].R));
        spdlog::info("[CAMERAPARAMEST] camera_params_[1].K: {}", matToString(camera_params_[1].K()));
        spdlog::info("[CAMERAPARAMEST] camera_params_[1].R: {}", matToString(camera_params_[1].R));
        
        for (size_t i = 0; i < camera_params_.size(); ++i) {
                camera_params_[i].R.copyTo(camera_params_[i].R);
        }

        if (!success) {
            last_error_ = "Bundle adjustment failed";
            return false;
        }
        spdlog::info("[CAMERAPARAMEST] Bundle adjustment success!");
        return true;
    }
    catch (const cv::Exception& e) {
        last_error_ = string("OpenCV exception in bundle adjustment: ") + e.what();
        return false;
    }
}

bool CameraParamEst::WaveCorrection() {
    if (wave_correction_ == "no") {
        spdlog::info("[CAMERAPARAMEST] Wave correction skipped");
        return true;
    }

    try {
        vector<cv::Mat> rmats;
        for (const auto& camera : camera_params_) {
            rmats.push_back(camera.R);
        }


        detail::WaveCorrectKind wave_kind = getWaveCorrectKind(wave_correction_);
        detail::waveCorrect(rmats, wave_kind);

        for (size_t i = 0; i < camera_params_.size(); ++i) {
                rmats[i].convertTo(camera_params_[i].R, CV_64F);

        }

        spdlog::info("[CAMERAPARAMEST] Wave correction completed: {}", wave_correction_);
        spdlog::info("[CAMERAPARAMEST] {}", matToString(rmats[0]));
        spdlog::info("[CAMERAPARAMEST] {}", matToString(rmats[1]));
        return true;
    }
    catch (const cv::Exception& e) {
        last_error_ = string("OpenCV exception in wave correction: ") + e.what();
        return false;
    }
}

bool CameraParamEst::initializeFeatureMatcher(const string& matcher_type, bool try_cuda) {
    if (matcher_type == "affine") {
        feature_matcher_ = makePtr<detail::AffineBestOf2NearestMatcher>(false, try_cuda, match_conf_);
    }
    else if (range_width_ == -1) {
        feature_matcher_ = makePtr<detail::BestOf2NearestMatcher>(try_cuda, match_conf_);
    }
    else {
        feature_matcher_ = makePtr<detail::BestOf2NearestRangeMatcher>(range_width_, try_cuda, match_conf_);
    }
    return !feature_matcher_.empty();
}

bool CameraParamEst::initializeMotionEstimator(const string& estimator_type) {
    if (estimator_type == "affine") {
        motion_estimator_ = makePtr<detail::AffineBasedEstimator>();
    }
    else {
        motion_estimator_ = makePtr<detail::HomographyBasedEstimator>();
    }
    return !motion_estimator_.empty();
}

bool CameraParamEst::initializeBundleAdjuster(const string& adjuster_type, const string& refine_mask) {
    if (adjuster_type == "reproj") {
        bundle_adjuster_ = makePtr<detail::BundleAdjusterReproj>();
    }
    else if (adjuster_type == "ray") {
        bundle_adjuster_ = makePtr<detail::BundleAdjusterRay>();
    }
    else if (adjuster_type == "affine") {
        bundle_adjuster_ = makePtr<detail::BundleAdjusterAffinePartial>();
    }
    else {
        last_error_ = "Unknown bundle adjuster type: " + adjuster_type;
        return false;
    }

    if (bundle_adjuster_ && refine_mask.size() == 5) {
        Mat_<uchar> refine_mask_mat = Mat::zeros(3, 3, CV_8U);
        if (refine_mask[0] == 'x') refine_mask_mat(0, 0) = 1;
        if (refine_mask[1] == 'x') refine_mask_mat(0, 1) = 1;
        if (refine_mask[2] == 'x') refine_mask_mat(0, 2) = 1;
        if (refine_mask[3] == 'x') refine_mask_mat(1, 1) = 1;
        if (refine_mask[4] == 'x') refine_mask_mat(1, 2) = 1;
        bundle_adjuster_->setRefinementMask(refine_mask_mat);
    }

    return !bundle_adjuster_.empty();
}

detail::WaveCorrectKind CameraParamEst::getWaveCorrectKind(const string& wave_correction) {
    if (wave_correction == "horiz") {
        return detail::WAVE_CORRECT_HORIZ;
    }
    else if (wave_correction == "vert") {
        return detail::WAVE_CORRECT_VERT;
    }
    else {
        return detail::WAVE_CORRECT_HORIZ;
    }
}

bool CameraParamEst::validateInputFeatures(const vector<detail::ImageFeatures>& features) {
    if (features.empty()) {
        last_error_ = "No features to process";
        return false;
    }

    for (const auto& feat : features) {
        if (feat.keypoints.empty()) {
            last_error_ = "Empty keypoints in features";
            return false;
        }
        // 检查特征点坐标是否在图像尺寸范围内
        for (const auto& kp : feat.keypoints) {
            if (kp.pt.x < 0 || kp.pt.x >= feat.img_size.width ||
                kp.pt.y < 0 || kp.pt.y >= feat.img_size.height) {
                last_error_ = "Keypoint out of image bounds";
                return false;
            }
        }
    }

    return true;
}


void CameraParamEst::update_camera_params(void)   //主要实现camera_params滤波、输出决策逻辑
{
    unique_lock<mutex> lock(mutex_);

#define EST_FILTER
#ifdef  EST_FILTER

    // [N路适配] 原代码写死只回写 camera 0 / camera 1，三路及以上时
    //   第 3 个及之后相机的 K/R/focal 不会被估计结果更新。改为按实际相机数循环。
    const int n_cam = std::min(
        static_cast<int>(camera_params_.size()),
        static_cast<int>(config_params_.camera_params.size()));
    for (int i = 0; i < n_cam; ++i) {
        camera_params_[i].K().copyTo(config_params_.camera_params[i].K);
        camera_params_[i].R.copyTo(config_params_.camera_params[i].R);
        config_params_.camera_params[i].focal = config_params_.camera_params[i].K.at<double>(0, 0);
    }
#else
    camera_params_[0].K().copyTo(RT_camera_params_[0].K);
    camera_params_[0].R.copyTo(RT_camera_params_[0].R);
    camera_params_[1].K().copyTo(RT_camera_params_[1].K);
    camera_params_[1].R.copyTo(RT_camera_params_[1].R);

    CameraParams_filter_.push_back(RT_camera_params_);

    cv::Mat K0_sum = cv::Mat::zeros(RT_camera_params_[0].K.size(), RT_camera_params_[0].K.type());
    cv::Mat K1_sum = cv::Mat::zeros(RT_camera_params_[1].K.size(), RT_camera_params_[1].K.type());

    cv::Vec3d euler0_sum(0, 0, 0);
    cv::Vec3d euler1_sum(0, 0, 0);
    int valid_frame_count = 0;

    for (size_t i = 0; i < 10; ++i)
    {
        if (CameraParams_filter_.get_i(i, RT_camera_params_))
        {
            spdlog::info("[CAMERAPARAMEST] CameraParams_filter_ size: {}", CameraParams_filter_.size());
            valid_frame_count++;

            cv::add(K0_sum, RT_camera_params_[0].K, K0_sum);
            cv::add(K1_sum, RT_camera_params_[1].K, K1_sum);

            cv::Mat rvec0, rvec1;
            cv::Rodrigues(RT_camera_params_[0].R, rvec0);
            cv::Rodrigues(RT_camera_params_[1].R, rvec1);

            euler0_sum += cv::Vec3d(rvec0);
            euler1_sum += cv::Vec3d(rvec1);
        }
    }

    if (valid_frame_count > 0)
    {
        if (K0_sum.type() != CV_64F) {
            K0_sum.convertTo(K0_sum, CV_64F);
        }
        if (K1_sum.type() != CV_64F) {
            K1_sum.convertTo(K1_sum, CV_64F);
        }

        cv::Mat K0_avg, K1_avg;
        cv::divide(K0_sum, valid_frame_count, K0_avg);
        cv::divide(K1_sum, valid_frame_count, K1_avg);

        cv::Vec3d rvec0_avg = euler0_sum / valid_frame_count;
        cv::Vec3d rvec1_avg = euler1_sum / valid_frame_count;
        cv::Mat R0_avg, R1_avg;
        cv::Rodrigues(rvec0_avg, R0_avg);
        cv::Rodrigues(rvec1_avg, R1_avg);

        cv::Mat U, W, Vt;
        cv::SVD::compute(R0_avg, W, U, Vt);
        R0_avg = U * Vt;
        if (cv::determinant(R0_avg) < 0) Vt.row(2) *= -1;
        R0_avg = U * Vt;

        cv::SVD::compute(R1_avg, W, U, Vt);
        R1_avg = U * Vt;
        if (cv::determinant(R1_avg) < 0) Vt.row(2) *= -1;
        R1_avg = U * Vt;

        K0_avg.copyTo(calculate_RT_camera_params_[0].K);
        R0_avg.copyTo(calculate_RT_camera_params_[0].R);
        K1_avg.copyTo(calculate_RT_camera_params_[1].K);
        R1_avg.copyTo(calculate_RT_camera_params_[1].R);
    }
    else
    {
        camera_params_[0].K().copyTo(calculate_RT_camera_params_[0].K);
        camera_params_[0].R.copyTo(calculate_RT_camera_params_[0].R);
        camera_params_[1].K().copyTo(calculate_RT_camera_params_[1].K);
        camera_params_[1].R.copyTo(calculate_RT_camera_params_[1].R);
    }
    while (1);
    calculate_RT_camera_params_[0].K.copyTo(config_params_.camera_params[0].K);
    calculate_RT_camera_params_[0].R.copyTo(config_params_.camera_params[0].R);
    config_params_.camera_params[0].focal = config_params_.camera_params[0].K.at<double>(0, 0);
    calculate_RT_camera_params_[1].K.copyTo(config_params_.camera_params[1].K);
    calculate_RT_camera_params_[1].R.copyTo(config_params_.camera_params[1].R);
    config_params_.camera_params[1].focal = config_params_.camera_params[1].K.at<double>(0, 0);
#endif //  EST_FILTER
    spdlog::info("[CAMERAPARAMEST] ---------- First camera parameters ----------");
    spdlog::info("[CAMERAPARAMEST] Intrinsic matrix K: {}",matToString(config_params_.camera_params[0].K));
    spdlog::info("[CAMERAPARAMEST] Rotation matrix R: {}", matToString(config_params_.camera_params[0].R));

    spdlog::info("[CAMERAPARAMEST] ---------- Second camera parameters ----------");
    spdlog::info("[CAMERAPARAMEST] Intrinsic matrix K: {}", matToString(config_params_.camera_params[1].K));
    spdlog::info("[CAMERAPARAMEST] Rotation matrix R: {}", matToString(config_params_.camera_params[1].R));

}






