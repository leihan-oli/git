// =============================================================================
// GazeAwareGraphCutSeamFinder.cpp
// =============================================================================
#include "GazeAwareGraphCutSeamFinder.hpp"
#include "GCGraph.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/util.hpp>

#include <algorithm>
#include <cmath>

namespace gaze_seam {

using cv::ACCESS_READ;
using cv::ACCESS_RW;
using cv::Mat;
using cv::Point;
using cv::Rect;
using cv::Size;
using cv::UMat;
using cv::Vec3f;

// ---------------------------------------------------------------------------
// Construction / saliency injection
// ---------------------------------------------------------------------------

GazeAwareGraphCutSeamFinder::GazeAwareGraphCutSeamFinder(
    int cost_type, float terminal_cost, float bad_region_penalty)
    : cost_type_(cost_type)
    , terminal_cost_(terminal_cost)
    , bad_region_penalty_(bad_region_penalty)
    , canvas_origin_(0, 0)
    , alpha_(0.f)
    , saliency_active_(false)
{}

GazeAwareGraphCutSeamFinder::~GazeAwareGraphCutSeamFinder() = default;

void GazeAwareGraphCutSeamFinder::setSaliencyMap(
    const Mat& saliency, Point canvas_origin, float alpha)
{
    CV_Assert(!saliency.empty());
    CV_Assert(saliency.type() == CV_32F);
    saliency_        = saliency;     // shallow copy, ref-counted
    canvas_origin_   = canvas_origin;
    alpha_           = alpha;
    saliency_active_ = (alpha > 0.f);
}

void GazeAwareGraphCutSeamFinder::clearSaliencyMap()
{
    saliency_        = Mat();
    saliency_active_ = false;
    alpha_           = 0.f;
}

// ---------------------------------------------------------------------------
// Pixel-level helpers (anonymous namespace)
// ---------------------------------------------------------------------------
namespace {

/// L2 colour distance between corresponding pixels of two CV_32FC3 images.
inline float colorDiff(const Mat& img1, const Mat& img2, int y, int x) {
    const Vec3f& a = img1.at<Vec3f>(y, x);
    const Vec3f& b = img2.at<Vec3f>(y, x);
    const float dB = a[0] - b[0];
    const float dG = a[1] - b[1];
    const float dR = a[2] - b[2];
    return std::sqrt(dB * dB + dG * dG + dR * dR);
}

/// L2 norm of the gradient vector at (y,x), summed over BGR channels.
inline float gradMag(const Mat& dx, const Mat& dy, int y, int x) {
    const Vec3f& gx = dx.at<Vec3f>(y, x);
    const Vec3f& gy = dy.at<Vec3f>(y, x);
    return std::sqrt(gx[0] * gx[0] + gx[1] * gx[1] + gx[2] * gx[2] +
                     gy[0] * gy[0] + gy[1] * gy[1] + gy[2] * gy[2]);
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// find(): convert to float, then iterate over image pairs.
// Mirrors PairwiseSeamFinder::run() but with our own pixel format conversion.
// ---------------------------------------------------------------------------
void GazeAwareGraphCutSeamFinder::find(
    const std::vector<UMat>& src,
    const std::vector<Point>& corners,
    std::vector<UMat>& masks)
{
    if (src.empty()) return;
    CV_Assert(src.size() == corners.size());
    CV_Assert(src.size() == masks.size());

    images_.resize(src.size());
    sizes_.resize(src.size());
    for (size_t i = 0; i < src.size(); ++i) {
        CV_Assert(src[i].channels() == 3);
        src[i].convertTo(images_[i], CV_32F);  // store as CV_32FC3 UMat
        sizes_[i] = images_[i].size();
    }
    corners_.assign(corners.begin(), corners.end());
    masks_.assign(masks.begin(), masks.end());  // shallow refs; modified in-place

    // Pairwise iteration (identical to cv::detail::PairwiseSeamFinder::run).
    for (size_t i = 0; i + 1 < sizes_.size(); ++i) {
        for (size_t j = i + 1; j < sizes_.size(); ++j) {
            Rect roi;
            if (cv::detail::overlapRoi(corners_[i], corners_[j],
                                        sizes_[i], sizes_[j], roi))
            {
                findInPair(i, j, roi);
            }
        }
    }

    // masks_ is shallow-copied from masks, so in-place updates already
    // propagated. Re-assign to be explicit (and to support future refactoring).
    masks.assign(masks_.begin(), masks_.end());
}

// ---------------------------------------------------------------------------
// findInPair(): build s-t graph for the overlap, run BK max-flow, update masks.
// ---------------------------------------------------------------------------
void GazeAwareGraphCutSeamFinder::findInPair(size_t first, size_t second, Rect roi)
{
    Mat img1  = images_[first ].getMat(ACCESS_READ);
    Mat img2  = images_[second].getMat(ACCESS_READ);
    Mat mask1 = masks_ [first ].getMat(ACCESS_RW);
    Mat mask2 = masks_ [second].getMat(ACCESS_RW);
    const Point tl1 = corners_[first ];
    const Point tl2 = corners_[second];

    // -----------------------------------------------------------------------
    // [修复] 与 OpenCV GraphCutSeamFinder 保持一致：在重叠 ROI 四周扩一圈
    //   gap 像素的边带一起建图。边带从完整图像/掩码采样，ROI 之外只被单张
    //   图覆盖的像素会成为"只属于 img1 / 只属于 img2"的源汇锚点。
    //   没有这圈锚点时，凡是整行(列)都同时被两张掩码覆盖的区域（非相邻
    //   相机对的细长矩形交集、相邻对重叠区的上下两端）会变成无约束的
    //   中性块，最小割会把它整块划给一侧——割缝退化成贴着 ROI 矩形边缘
    //   的竖直直线。这正是"部分接缝是直线而非锯齿"的根源。
    // -----------------------------------------------------------------------
    const int gap = 10;
    const int H  = roi.height;
    const int W  = roi.width;
    const int PH = H + 2 * gap;   // padded height
    const int PW = W + 2 * gap;   // padded width

    // 1. 提取带边带的子图/子掩码；落在某张图矩形之外的像素填 0（掩码为 0，
    //    只会通过 bad_region_penalty 影响边权，同 OpenCV 行为）。
    Mat subimg1   = Mat::zeros(PH, PW, CV_32FC3);
    Mat subimg2   = Mat::zeros(PH, PW, CV_32FC3);
    Mat sub_mask1 = Mat::zeros(PH, PW, CV_8U);
    Mat sub_mask2 = Mat::zeros(PH, PW, CV_8U);

    for (int py = 0; py < PH; ++py) {
        const int y  = py - gap;                 // ROI 坐标
        const int y1 = y + roi.y - tl1.y;        // img1 坐标
        const int y2 = y + roi.y - tl2.y;        // img2 坐标
        for (int px = 0; px < PW; ++px) {
            const int x  = px - gap;
            const int x1 = x + roi.x - tl1.x;
            const int x2 = x + roi.x - tl2.x;
            if (y1 >= 0 && y1 < img1.rows && x1 >= 0 && x1 < img1.cols) {
                subimg1.at<Vec3f>(py, px)   = img1.at<Vec3f>(y1, x1);
                sub_mask1.at<uchar>(py, px) = mask1.at<uchar>(y1, x1);
            }
            if (y2 >= 0 && y2 < img2.rows && x2 >= 0 && x2 < img2.cols) {
                subimg2.at<Vec3f>(py, px)   = img2.at<Vec3f>(y2, x2);
                sub_mask2.at<uchar>(py, px) = mask2.at<uchar>(y2, x2);
            }
        }
    }

    // 2. COST_COLOR_GRAD 的梯度预计算（在带边带的子图上做，gap>=Sobel 半径，
    //    边界效应不会波及内层 ROI）。
    Mat dx1, dx2, dy1, dy2;
    if (cost_type_ == COST_COLOR_GRAD) {
        cv::Sobel(subimg1, dx1, CV_32F, 1, 0);
        cv::Sobel(subimg1, dy1, CV_32F, 0, 1);
        cv::Sobel(subimg2, dx2, CV_32F, 1, 0);
        cv::Sobel(subimg2, dy2, CV_32F, 0, 1);
    }

    // 3. 在扩边后的网格上建图。
    const int       N          = PW * PH;
    const float     weight_eps = 1.f;
    GCGraph<float>  graph(N, 2 * N);

    // 3a. 顶点 + 终端权重。
    for (int py = 0; py < PH; ++py) {
        for (int px = 0; px < PW; ++px) {
            int v = graph.addVtx();
            const float src_w  = sub_mask1.at<uchar>(py, px) ? terminal_cost_ : 0.f;
            const float sink_w = sub_mask2.at<uchar>(py, px) ? terminal_cost_ : 0.f;
            graph.addTermWeights(v, src_w, sink_w);
        }
    }

    // 3b. 邻接边 + 可选显著性调制。
    const bool has_sal = saliency_active_ && !saliency_.empty();
    const int  sal_w   = has_sal ? saliency_.cols : 0;
    const int  sal_h   = has_sal ? saliency_.rows : 0;

    auto sampleSal = [&](int cy, int cx) -> float {
        if (!has_sal) return 0.f;
        if (cx < 0 || cy < 0 || cx >= sal_w || cy >= sal_h) return 0.f;
        return saliency_.at<float>(cy, cx);
    };

    auto modulate = [&](float w, float s_avg) {
        return has_sal ? w * (1.f + alpha_ * s_avg) : w;
    };

    auto baseColor = [&](int yp, int xp, int yq, int xq) {
        return colorDiff(subimg1, subimg2, yp, xp)
             + colorDiff(subimg1, subimg2, yq, xq)
             + weight_eps;
    };
    auto baseColorGrad = [&](int yp, int xp, int yq, int xq) {
        const float c = colorDiff(subimg1, subimg2, yp, xp)
                      + colorDiff(subimg1, subimg2, yq, xq);
        const float g = gradMag(dx1, dy1, yp, xp) + gradMag(dx2, dy2, yp, xp)
                      + gradMag(dx1, dy1, yq, xq) + gradMag(dx2, dy2, yq, xq);
        return c / (g + weight_eps) + weight_eps;
    };

    for (int py = 0; py < PH; ++py) {
        // 画布坐标（显著性图坐标系）：先去掉 gap 偏移回到 ROI 坐标，再平移。
        const int cy = (py - gap) + roi.y - canvas_origin_.y;
        for (int px = 0; px < PW; ++px) {
            const int v  = py * PW + px;
            const int cx = (px - gap) + roi.x - canvas_origin_.x;

            // ---- 水平边 (px, px+1) ----
            if (px + 1 < PW) {
                float w = (cost_type_ == COST_COLOR)
                        ? baseColor    (py, px, py, px + 1)
                        : baseColorGrad(py, px, py, px + 1);

                const bool bad = !sub_mask1.at<uchar>(py, px)     ||
                                 !sub_mask1.at<uchar>(py, px + 1) ||
                                 !sub_mask2.at<uchar>(py, px)     ||
                                 !sub_mask2.at<uchar>(py, px + 1);
                if (bad) w += bad_region_penalty_;

                w = modulate(w, 0.5f * (sampleSal(cy, cx) + sampleSal(cy, cx + 1)));
                graph.addEdges(v, v + 1, w, w);
            }

            // ---- 垂直边 (py, py+1) ----
            if (py + 1 < PH) {
                float w = (cost_type_ == COST_COLOR)
                        ? baseColor    (py, px, py + 1, px)
                        : baseColorGrad(py, px, py + 1, px);

                const bool bad = !sub_mask1.at<uchar>(py, px)     ||
                                 !sub_mask1.at<uchar>(py + 1, px) ||
                                 !sub_mask2.at<uchar>(py, px)     ||
                                 !sub_mask2.at<uchar>(py + 1, px);
                if (bad) w += bad_region_penalty_;

                w = modulate(w, 0.5f * (sampleSal(cy, cx) + sampleSal(cy + 1, cx)));
                graph.addEdges(v, v + PW, w, w);
            }
        }
    }

    // 4. 求最小割。
    graph.maxFlow();

    // -----------------------------------------------------------------------
    // 5. 回写掩码——只更新内层 ROI（边带只参与建图，不回写）。
    //    [修复] 加上与 OpenCV 相同的保护条件：源侧只在 mask1 有效处清 mask2，
    //    汇侧只在 mask2 有效处清 mask1。否则中性区（两掩码同为有效或同为
    //    无效、标签本质任意）的任意标签会把有效区域沿直线边界成片抹掉。
    //    注：roi = rect1 ∩ rect2，内层像素在两张图内必然越界安全。
    // -----------------------------------------------------------------------
    for (int y = 0; y < H; ++y) {
        const int y1 = y + roi.y - tl1.y;
        const int y2 = y + roi.y - tl2.y;
        for (int x = 0; x < W; ++x) {
            const int v  = (y + gap) * PW + (x + gap);
            const int x1 = x + roi.x - tl1.x;
            const int x2 = x + roi.x - tl2.x;

            if (graph.inSourceSegment(v)) {
                if (mask1.at<uchar>(y1, x1))
                    mask2.at<uchar>(y2, x2) = 0;
            } else {
                if (mask2.at<uchar>(y2, x2))
                    mask1.at<uchar>(y1, x1) = 0;
            }
        }
    }
}

}  // namespace gaze_seam
