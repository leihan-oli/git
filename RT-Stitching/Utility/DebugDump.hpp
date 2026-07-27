#ifndef RTSTITCHING_DEBUG_DUMP_HPP
#define RTSTITCHING_DEBUG_DUMP_HPP

// ============================================================================
// [新增] 统一调试图像输出辅助
//
// 背景：板子(Linux/RK3588)上 OpenCV HighGUI(imshow/waitKey) 与 launcher 的
//   Qt 事件循环会互相抢占 GUI 资源，从工作线程 imshow 会导致线程卡死、
//   stop() 的 join() 永久阻塞。因此仿照 SeamFinder 的做法：
//     - Windows：保留 imshow + waitKey(1)（开发机调试不变）
//     - Linux  ：限频 + 原子写盘（先写 .tmp 再 rename），输出到
//                RT_DEBUG_DUMP_DIR（默认 /root/build/debug/）
//
// 用法（各模块的 MODULE_*_DEBUG 开关仍经构造函数传入 show_window_，
//       只把原来的 cv::imshow(...)/cv::waitKey(1) 换成本函数）：
//
//   if (show_window_) {
//       if (RTStitching::debugDump(module_name_, frame) == 27)
//           stop_requested_.store(true);   // Windows 下 ESC 仍可退出
//   }
//
// 返回值：Windows 返回 cv::waitKey(1) 的键值（ESC=27）；Linux 恒返回 -1。
// 线程安全：每个 name 独立限频，内部用互斥锁保护，可从任意工作线程调用。
// ============================================================================

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#ifdef _WIN32
#include <opencv2/highgui.hpp>
#endif
#include <spdlog/spdlog.h>

#include <string>
#include <map>
#include <mutex>
#include <chrono>
#include <cstdio>
#include <filesystem>

#ifndef RT_DEBUG_DUMP_DIR
#define RT_DEBUG_DUMP_DIR "/root/build/debug"
#endif

namespace RTStitching {

// 把窗口名清洗成安全文件名（空格/冒号/斜杠 -> 下划线）
inline std::string debugDumpSanitize(const std::string& name) {
    std::string s = name.empty() ? std::string("unnamed") : name;
    for (auto& c : s) {
        if (c == ' ' || c == ':' || c == '/' || c == '\\' ||
            c == '*' || c == '?' || c == '"' || c == '<' || c == '>' || c == '|')
            c = '_';
    }
    return s;
}

// name        : 原窗口名（同时作为输出文件名 <name>.jpg）
// img         : 待输出图像（非 CV_8U 深度会自动 convertTo 后写盘）
// interval_ms : Linux 写盘限频周期，默认 33ms（约 30fps），与 SeamFinder 一致
inline int debugDump(const std::string& name, const cv::Mat& img, int interval_ms = 33) {
    if (img.empty()) return -1;

#ifdef _WIN32
    (void)interval_ms;
    cv::imshow(name, img);
    return cv::waitKey(1);
#else
    using clock = std::chrono::steady_clock;

    // 每个 name 独立限频（多模块/多路相机互不影响）
    static std::mutex mtx;
    static std::map<std::string, clock::time_point> last_save;

    const std::string key = debugDumpSanitize(name);
    {
        std::lock_guard<std::mutex> lk(mtx);
        auto now = clock::now();
        auto it = last_save.find(key);
        if (it != last_save.end() &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count() < interval_ms) {
            return -1;   // 限频窗口内，跳过本帧
        }
        last_save[key] = now;
    }

    // 目录不存在时自动创建（防止 imwrite 静默失败）
    static const std::string dir = RT_DEBUG_DUMP_DIR;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    // imwrite 只接受 8U（jpg），16S/32F 等深度先转换
    cv::Mat to_write;
    if (img.depth() != CV_8U) {
        img.convertTo(to_write, CV_8U);
    } else {
        to_write = img;
    }

    // 原子写盘：先写 .tmp 再 rename，避免读端读到半写文件
    const std::string tmp_path = dir + "/" + key + ".tmp.jpg";
    const std::string out_path = dir + "/" + key + ".jpg";
    if (cv::imwrite(tmp_path, to_write)) {
        std::rename(tmp_path.c_str(), out_path.c_str());
    } else {
        spdlog::warn("[DEBUGDUMP] imwrite failed ({})", tmp_path);
    }
    return -1;
#endif
}

} // namespace RTStitching

#endif // RTSTITCHING_DEBUG_DUMP_HPP
