// 读取 Python 端 U2-Net 写出的显著性图 (saliency.png + saliency.seq),
// 生成 canvas 尺寸的显著性图供 SeamFinder 使用。
// 结构对照 GazeDataReader.cpp。

#include "SaliencyMapReader.hpp"

#include <fstream>
#include <chrono>
#include <spdlog/spdlog.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

SaliencyMapReader::SaliencyMapReader(const std::string& dir, int poll_interval_ms)
    : dir_(dir)
    , poll_interval_ms_(poll_interval_ms)
    , last_seq_(0)
    , has_data_(false)
    , running_(false)
{
    // 与 Python saliency_writer.py 的文件名约定一致
    const char sep =
#ifdef _WIN32
        '\\';
#else
        '/';
#endif
    png_path_ = dir_ + sep + "saliency.bmp";   // .png -> .bmp
    seq_path_ = dir_ + sep + "saliency.seq";
}

SaliencyMapReader::~SaliencyMapReader() {
    stop();
}

bool SaliencyMapReader::start() {
    if (running_.load()) return true;
    running_.store(true);
    poll_thread_ = std::thread(&SaliencyMapReader::pollLoop, this);
    spdlog::info("[SALIENCYREADER] Started polling '{}' every {}ms", dir_, poll_interval_ms_);
    return true;
}

void SaliencyMapReader::stop() {
    running_.store(false);
    if (poll_thread_.joinable()) {
        poll_thread_.join();
    }
    spdlog::info("[SALIENCYREADER] Stopped");
}

void SaliencyMapReader::pollLoop() {
    while (running_.load()) {
        readOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms_));
    }
}

bool SaliencyMapReader::readOnce() {
    // 1) 先读序列号文件，未变化则跳过（轻量，避免每次都解码 png）
    std::ifstream seq_file(seq_path_);
    if (!seq_file.is_open()) {
        return false;  // Python 端尚未启动
    }
    uint64_t seq = 0;
    seq_file >> seq;
    if (!seq_file.good() && seq == 0) {
        return false;  // 半写状态，下次再读
    }
    if (seq == last_seq_.load()) {
        return false;  // 无新数据
    }

    // 2) 序列号有更新，读取 png
    cv::Mat img = cv::imread(png_path_, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        spdlog::debug("[SALIENCYREADER] png not ready (seq={})", seq);
        return false;  // png 可能正在被 rename，下次重试
    }

    // 转 CV_32F [0,1]
    cv::Mat f32;
    img.convertTo(f32, CV_32F, 1.0 / 255.0);

    {
        std::unique_lock<std::shared_mutex> lock(map_mutex_);
        latest_ = std::move(f32);
        last_seq_.store(seq);
        has_data_.store(true, std::memory_order_release);
    }

    spdlog::debug("[SALIENCYREADER] Read saliency map {}x{}, seq={}",
                  latest_.cols, latest_.rows, seq);
    return true;
}

cv::Mat SaliencyMapReader::generateSaliencyMap(int width, int height,
                                               double /*sigma*/,
                                               int64_t /*max_age_ms*/) const {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    if (latest_.empty() || width <= 0 || height <= 0) {
        return cv::Mat();
    }

    cv::Mat out;
    if (latest_.cols == width && latest_.rows == height) {
        out = latest_.clone();
    } else {
        cv::resize(latest_, out, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
    }

    // 缩放后再归一化一次，确保严格 [0,1]
    double mx = 0.0;
    cv::minMaxLoc(out, nullptr, &mx);
    if (mx > 1e-6) out /= static_cast<float>(mx);

    return out;
}
