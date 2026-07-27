#ifndef RTSTITCHING_THREAD_AFFINITY_HPP
#define RTSTITCHING_THREAD_AFFINITY_HPP

// ============================================================================
// [新增] 线程绑核辅助（RK3588 big.LITTLE）
//
// RK3588 拓扑：CPU0-3 = Cortex-A55（小核, ~1.8GHz），CPU4-7 = Cortex-A76（大核,
// ~2.2-2.4GHz）。上板前请用下面命令核实编号（不同板卡固件可能有差异）：
//   cat /sys/devices/system/cpu/cpu*/cpufreq/cpuinfo_max_freq
// 频率高的 4 个即大核；若编号不同，改下面 kBigCores/kLittleCores 即可。
//
// 用法：在各工作线程入口第一行调用
//   RTStitching::bindToBigCores(module_name_);     // 重计算线程(Warper/Blender/SeamFinder)
//   RTStitching::bindToLittleCores(module_name_);  // 轻负载线程(VideoCapture)
//
// 编译期总开关 RT_THREAD_AFFINITY_ENABLED（默认 1）；非 Linux 平台自动空实现。
// ============================================================================

#ifndef RT_THREAD_AFFINITY_ENABLED
#define RT_THREAD_AFFINITY_ENABLED 1
#endif

#include <string>
#include <spdlog/spdlog.h>

#if defined(__linux__) && RT_THREAD_AFFINITY_ENABLED
#include <pthread.h>
#include <sched.h>
#endif

namespace RTStitching {

#if defined(__linux__) && RT_THREAD_AFFINITY_ENABLED

inline bool bindCurrentThreadToCpus(std::initializer_list<int> cpus,
                                    const std::string& tag) {
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int c : cpus) CPU_SET(c, &set);
    const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (rc == 0) {
        spdlog::info("[AFFINITY] [{}] bound to cpus {{{}..}}", tag, *cpus.begin());
    } else {
        spdlog::warn("[AFFINITY] [{}] pthread_setaffinity_np failed rc={}", tag, rc);
    }
    return rc == 0;
}

// RK3588 默认拓扑，如与实测不符请修改这两行
inline bool bindToBigCores(const std::string& tag)    { return bindCurrentThreadToCpus({4, 5, 6, 7}, tag); }
inline bool bindToLittleCores(const std::string& tag) { return bindCurrentThreadToCpus({0, 1, 2, 3}, tag); }

#else

inline bool bindToBigCores(const std::string&)    { return false; }
inline bool bindToLittleCores(const std::string&) { return false; }

#endif

} // namespace RTStitching

#endif // RTSTITCHING_THREAD_AFFINITY_HPP
