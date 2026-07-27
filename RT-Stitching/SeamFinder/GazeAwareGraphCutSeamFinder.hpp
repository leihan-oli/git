// =============================================================================
// GazeAwareGraphCutSeamFinder.hpp
// -----------------------------------------------------------------------------
// Drop-in replacement for cv::detail::GraphCutSeamFinder that bakes a
// per-pixel saliency map directly into the min-cut energy function, so that
// seams naturally avoid regions the user is looking at.
//
// Edge weight modulation:
//
//   w_modulated = (1 + alpha * mean(S(p), S(q))) * w_base
//
//   - S(.) in [0,1] is the gaze saliency map (canvas frame)
//   - alpha controls avoidance strength (0 = disabled, 4~6 typical, 10 = strong)
//   - w_base is the standard COST_COLOR or COST_COLOR_GRAD edge weight
//
// Because this happens *inside* the graph cut, no morphological post-processing
// is required afterwards; the produced masks are already smooth and avoid the
// gaze region by construction.
//
// Thread-safety:
//   Not thread-safe by itself. setSaliencyMap() must not be called while
//   find() is running. (One-thread-at-a-time per instance is fine.)
// =============================================================================
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <vector>

namespace gaze_seam {

class GazeAwareGraphCutSeamFinder : public cv::detail::PairwiseSeamFinder {
public:
    enum CostType { COST_COLOR, COST_COLOR_GRAD };

    /// @param cost_type            COST_COLOR or COST_COLOR_GRAD
    /// @param terminal_cost        s/t terminal capacity (default 10000, same as OpenCV)
    /// @param bad_region_penalty   added to weight when either pixel is outside a mask
    GazeAwareGraphCutSeamFinder(int cost_type = COST_COLOR,
                                float terminal_cost      = 10000.f,
                                float bad_region_penalty = 1000.f);
    ~GazeAwareGraphCutSeamFinder() override;

    // ---- Gaze injection ----------------------------------------------------

    /// Inject a saliency map for the *next* find() call.
    /// The map is consumed but not cleared, so successive find() calls reuse
    /// it until you overwrite or clear it.
    ///
    /// @param saliency       CV_32F single-channel image, values in [0, 1].
    ///                       Resolution must equal the full panorama canvas
    ///                       (canvas_w x canvas_h) at the SEAM_FINDER_SCALE.
    /// @param canvas_origin  Top-left of the saliency map expressed in the
    ///                       same coordinate frame as the `corners` you will
    ///                       pass to find(). For a layout where canvas is
    ///                       built by (min_x, min_y) over corners, pass
    ///                       cv::Point(min_x, min_y).
    /// @param alpha          Avoidance strength. 0 disables modulation
    ///                       (graph cut behaves identically to OpenCV's).
    void setSaliencyMap(const cv::Mat& saliency,
                        cv::Point canvas_origin,
                        float alpha = 4.0f);

    /// Disable saliency modulation for subsequent find() calls.
    void clearSaliencyMap();

    /// True if a non-empty saliency map with alpha > 0 is currently armed.
    bool isSaliencyActive() const { return saliency_active_; }

    // ---- SeamFinder interface ----------------------------------------------

    void find(const std::vector<cv::UMat>& src,
              const std::vector<cv::Point>& corners,
              std::vector<cv::UMat>& masks) override;

protected:
    void findInPair(size_t first, size_t second, cv::Rect roi) override;

private:
    int   cost_type_;
    float terminal_cost_;
    float bad_region_penalty_;

    cv::Mat   saliency_;          // canvas frame, CV_32F, [0,1]
    cv::Point canvas_origin_;     // canvas TL in corners' frame
    float     alpha_;
    bool      saliency_active_;
};

}  // namespace gaze_seam
