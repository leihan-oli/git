#ifndef FUSED_SALIENCY_SOURCE_HPP
#define FUSED_SALIENCY_SOURCE_HPP

#include "ISaliencySource.hpp"
#include <opencv2/imgproc.hpp>
#include <string>

class FusedSaliencySource : public ISaliencySource {
public:
    enum FusionMode { MAX, WEIGHTED, MUL };

    FusedSaliencySource(ISaliencySource* gaze, ISaliencySource* u2net,
        bool use_gaze, bool use_u2net,
        FusionMode mode = MAX,
        float gaze_w = 0.5f, float u2net_w = 0.5f)
        : gaze_(gaze), u2net_(u2net)
        , use_gaze_(use_gaze), use_u2net_(use_u2net)
        , mode_(mode), gaze_w_(gaze_w), u2net_w_(u2net_w) {
    }

    cv::Mat generateSaliencyMap(int width, int height,
        double sigma = 0.0,
        int64_t max_age_ms = 3000) const override {
        cv::Mat sg, su;
        if (use_gaze_ && gaze_)
            sg = gaze_->generateSaliencyMap(width, height, sigma, max_age_ms);
        if (use_u2net_ && u2net_)
            su = u2net_->generateSaliencyMap(width, height);

        const bool hg = !sg.empty(), hu = !su.empty();
        if (!hg && !hu) return cv::Mat();
        if (hg && !hu) return sg;
        if (!hg && hu) return su;

        // 两路都有 -> 融合
        cv::Mat out;
        switch (mode_) {
        case MAX:      cv::max(sg, su, out); break;
        case MUL:      cv::multiply(sg, su, out); break;
        case WEIGHTED: cv::addWeighted(sg, gaze_w_, su, u2net_w_, 0.0, out); break;
        }
        // weighted 后重新归一化，保证 alpha 物理意义不变
        double mx = 0.0; cv::minMaxLoc(out, nullptr, &mx);
        if (mx > 1e-6) out /= static_cast<float>(mx);
        return out;
    }

    bool hasSaliency() const override {
        return (use_gaze_ && gaze_ && gaze_->hasSaliency()) ||
            (use_u2net_ && u2net_ && u2net_->hasSaliency());
    }

    static FusionMode parseMode(const std::string& s) {
        if (s == "mul")      return MUL;
        if (s == "weighted") return WEIGHTED;
        return MAX;
    }

private:
    ISaliencySource* gaze_;
    ISaliencySource* u2net_;
    bool use_gaze_, use_u2net_;
    FusionMode mode_;
    float gaze_w_, u2net_w_;
};
#endif