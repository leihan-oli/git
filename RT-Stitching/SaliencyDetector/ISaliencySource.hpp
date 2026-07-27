#ifndef ISALIENCY_SOURCE_HPP
#define ISALIENCY_SOURCE_HPP

#include <opencv2/core.hpp>
#include <cstdint>

/**
 * @brief 显著性来源统一抽象接口
 *
 * 让 SeamFinder 与“显著性从哪来”解耦：
 *   - GazeDataReader        眼动注视显著性（让其 implement 本接口）
 *   - SaliencyMapReader     U2-Net 文件版（Windows 阶段一）
 *   - FusedSaliencySource   两路融合
 *   - RknnSaliencyDetector  RK3588 板端直推版（阶段二，未来）
 *
 * 约定：generateSaliencyMap 返回 canvas 尺寸的 CV_32F 单通道图，值域 [0,1]。
 *      sigma / max_age_ms 对部分来源无意义（如 U2-Net），实现可忽略，
 *      保留签名是为了与既有 GazeDataReader::generateSaliencyMap 完全对齐。
 */
class ISaliencySource {
public:
    virtual ~ISaliencySource() = default;

    virtual cv::Mat generateSaliencyMap(int width, int height,
                                        double sigma = 0.0,
                                        int64_t max_age_ms = 0) const = 0;

    /// 当前是否有可用的显著性数据（无数据时 SeamFinder 可跳过注入）
    virtual bool hasSaliency() const = 0;
};

#endif // ISALIENCY_SOURCE_HPP
