#ifndef SALIENCY_MAP_READER_HPP
#define SALIENCY_MAP_READER_HPP

#include <string>
#include <atomic>
#include <thread>
#include <shared_mutex>
#include <cstdint>
#include <opencv2/core.hpp>

#include "ISaliencySource.hpp"

/**
 * @brief U2-Net 显著性图读取器（文件版，落地阶段一）
 *
 * 后台轮询 Python 端 saliency_writer.py 写出的共享目录：
 *   <dir>/saliency.seq   纯文本递增序列号（变化才重读）
 *   <dir>/saliency.png   8 位单通道显著性图（0~255 即 [0,1]*255）
 *
 * 结构与 GazeDataReader 一致：start() 起后台轮询线程，
 * generateSaliencyMap() 供 SeamFinder 调用，按 canvas 尺寸缩放并归一化。
 */
class SaliencyMapReader : public ISaliencySource {
public:
    explicit SaliencyMapReader(const std::string& dir,
                               int poll_interval_ms = 30);
    ~SaliencyMapReader() override;

    SaliencyMapReader(const SaliencyMapReader&) = delete;
    SaliencyMapReader& operator=(const SaliencyMapReader&) = delete;

    bool start();
    void stop();

    // ---- ISaliencySource ----
    cv::Mat generateSaliencyMap(int width, int height,
                                double sigma = 0.0,
                                int64_t max_age_ms = 0) const override;

    bool hasSaliency() const override {
        return has_data_.load(std::memory_order_acquire);
    }

    uint64_t getLastSeq() const { return last_seq_.load(); }

private:
    std::string dir_;
    std::string png_path_;
    std::string seq_path_;
    int poll_interval_ms_;

    // 最新显著性图缓存（原始网络输出尺寸，CV_32F [0,1]）
    mutable std::shared_mutex map_mutex_;
    cv::Mat latest_;
    std::atomic<uint64_t> last_seq_;
    std::atomic<bool> has_data_;

    std::atomic<bool> running_;
    std::thread poll_thread_;

    bool readOnce();   // 读 seq，变化则读 png
    void pollLoop();
};

#endif // SALIENCY_MAP_READER_HPP
