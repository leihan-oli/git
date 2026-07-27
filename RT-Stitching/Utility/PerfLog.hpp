#ifndef RTSTITCHING_PERF_LOG_HPP
#define RTSTITCHING_PERF_LOG_HPP

// ============================================================================
// [新增] 轻量性能日志（替代调试图像里的叠字时序信息，用于填写报告性能表）
//
// 特点：
//   - 与 MODULE_*_DEBUG 图像开关完全解耦，图像调试关闭时照常记录；
//   - 每条记录一行 CSV，追加写入 RT_PERF_LOG_DIR/perf.csv
//     （列：wall_ms,module,metric,frame_idx,value_ms）；
//   - 每 5 秒对各 (module,metric) 输出一次 avg/min/max 摘要到 spdlog，
//     现场看日志即可直接抄数，不必事后处理 CSV；
//   - 开销：每条记录一次互斥锁 + 一行格式化写入（缓冲），微秒级，
//     远小于原来每帧多次 JPEG 编码的调试写盘；
//   - 编译期总开关 RT_PERF_LOG_ENABLED（默认 1），置 0 时 record() 为空函数。
//
// 用法：
//   RTStitching::perfRecord(module_name_, "warp_ms", warp_ms, img_idx);
//
// CSV 汇总示例（板子或 PC 上）：
//   python3 -c "import pandas as pd;d=pd.read_csv('/root/build/debug/perf.csv');\
//     print(d.groupby(['module','metric'])['value_ms'].describe())"
// ============================================================================

#ifndef RT_PERF_LOG_ENABLED
#define RT_PERF_LOG_ENABLED 1
#endif

#ifndef RT_PERF_LOG_DIR
#define RT_PERF_LOG_DIR "/root/build/debug"
#endif

#include <spdlog/spdlog.h>

#include <string>
#include <map>
#include <mutex>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <cstdint>

namespace RTStitching {

class PerfLog {
public:
    static PerfLog& instance() {
        static PerfLog inst;
        return inst;
    }

    // module    : 模块名（如 "Cam_0" / "Warper_1" / "Blender" / "SeamFinder"）
    // metric    : 指标名（如 "warp_ms" / "blend_ms" / "e2e_ms"）
    // value_ms  : 数值（约定单位 ms；帧计数类指标直接填个数）
    // frame_idx : 关联帧号（无则填 0）
    void record(const std::string& module, const std::string& metric,
                double value_ms, uint64_t frame_idx = 0) {
#if RT_PERF_LOG_ENABLED
        std::lock_guard<std::mutex> lk(mtx_);
        ensureOpen();

        const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        csv_ << wall_ms << ',' << module << ',' << metric << ','
             << frame_idx << ',' << value_ms << '\n';
        if (++lines_since_flush_ >= 64) {   // 批量刷盘，降低 IO 次数
            csv_.flush();
            lines_since_flush_ = 0;
        }

        // 累计窗口统计
        Stat& s = stats_[module + "/" + metric];
        s.n += 1;
        s.sum += value_ms;
        if (value_ms < s.mn) s.mn = value_ms;
        if (value_ms > s.mx) s.mx = value_ms;

        // 每 5 秒输出一次窗口摘要（avg/min/max），随后清零重新累计
        const auto now = std::chrono::steady_clock::now();
        if (now - last_summary_ >= std::chrono::seconds(5)) {
            last_summary_ = now;
            for (auto& kv : stats_) {
                Stat& st = kv.second;
                if (st.n == 0) continue;
                spdlog::info("[PERF] {}: n={} avg={:.2f} min={:.2f} max={:.2f} (ms)",
                             kv.first, st.n, st.sum / st.n, st.mn, st.mx);
                st = Stat{};
            }
            csv_.flush();
            lines_since_flush_ = 0;
        }
#else
        (void)module; (void)metric; (void)value_ms; (void)frame_idx;
#endif
    }

private:
    struct Stat {
        uint64_t n = 0;
        double sum = 0.0;
        double mn = 1e300;
        double mx = -1e300;
    };

    PerfLog() = default;
    PerfLog(const PerfLog&) = delete;
    PerfLog& operator=(const PerfLog&) = delete;

    void ensureOpen() {
        if (opened_) return;
        std::error_code ec;
        std::filesystem::create_directories(RT_PERF_LOG_DIR, ec);
        const std::string path = std::string(RT_PERF_LOG_DIR) + "/perf.csv";
        const bool need_header = !std::filesystem::exists(path, ec) ||
                                 std::filesystem::file_size(path, ec) == 0;
        csv_.open(path, std::ios::app);
        if (csv_.is_open() && need_header) {
            csv_ << "wall_ms,module,metric,frame_idx,value_ms\n";
        }
        if (!csv_.is_open()) {
            spdlog::warn("[PERF] failed to open {}", path);
        }
        opened_ = true;
    }

    std::mutex mtx_;
    std::ofstream csv_;
    bool opened_ = false;
    int lines_since_flush_ = 0;
    std::map<std::string, Stat> stats_;
    std::chrono::steady_clock::time_point last_summary_ = std::chrono::steady_clock::now();
};

inline void perfRecord(const std::string& module, const std::string& metric,
                       double value_ms, uint64_t frame_idx = 0) {
    PerfLog::instance().record(module, metric, value_ms, frame_idx);
}

} // namespace RTStitching

#endif // RTSTITCHING_PERF_LOG_HPP
