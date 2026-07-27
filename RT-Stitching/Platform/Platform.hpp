#ifndef RTSTITCHING_PLATFORM_HPP
#define RTSTITCHING_PLATFORM_HPP

// =====================================================================
// 跨平台兼容头：集中处理 Windows / Linux 差异
// 适配 Windows (MSVC) 与 Linux (gcc / aarch64-linux-gnu，包括 RK3588)
// =====================================================================
#include <ctime>
#include <string>
#include <filesystem>
#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>
    #include <winsock2.h>   // 必须在 windows.h 之后（已定义 WIN32_LEAN_AND_MEAN，无 winsock v1 冲突）
    #include <ws2tcpip.h>

    using DllHandle = HMODULE;

    // localtime_s 在 Windows 上参数顺序为 (out, in)
    inline void rt_localtime(std::tm* out, const std::time_t* in) {
        localtime_s(out, in);
    }
#else
    // ----- Linux / Unix -----
    #include <dlfcn.h>      // dlopen / dlsym / dlclose
    #include <unistd.h>
    #include <ctime>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <sys/select.h>
    #include <sys/time.h>

    using DllHandle = void*;

    // CALLBACK 在 Win32 是 __stdcall 调用约定，Linux 下用空宏即可
    #ifndef CALLBACK
        #define CALLBACK
    #endif

    // 在 Linux 下伪造一个 SetConsoleOutputCP，让主程序不用到处 #ifdef
    #ifndef CP_UTF8
        #define CP_UTF8 65001
    #endif
    inline int SetConsoleOutputCP(unsigned int) { return 1; }

    // localtime_r (POSIX) 参数顺序为 (in, out)，与 localtime_s 相反
    inline void rt_localtime(std::tm* out, const std::time_t* in) {
        localtime_r(in, out);
    }
#endif

// 把 localtime_s 直接重定向到 rt_localtime，避免在原代码里到处替换
#ifndef _WIN32
    #define localtime_s(out, in) rt_localtime((out), (in))
#endif


// =====================================================================
// 取「当前可执行文件所在目录」（跨平台）
//   用于在 CWD 不确定时（VS 调试、双击运行、Linux 终端）稳定定位资源。
//   Windows: GetModuleFileNameW ；Linux: readlink(/proc/self/exe)
// =====================================================================
inline std::filesystem::path getExecutableDir() {
#ifdef _WIN32
    wchar_t buf[MAX_PATH] = {0};
    DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    if (n > 0 && n < MAX_PATH)
        return std::filesystem::path(buf).parent_path();
#else
    char buf[4096] = {0};
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n > 0) { buf[n] = '\0'; return std::filesystem::path(buf).parent_path(); }
#endif
    return std::filesystem::current_path();
}


// =====================================================================
// 跨平台 socket 封装（Windows Winsock / Linux BSD socket）
//   gaze 数据从 Windows(Tobii) 经 TCP 发送到 RK3588 板子时使用。
// =====================================================================
#ifdef _WIN32
    using rt_socket_t = SOCKET;
    #define RT_INVALID_SOCKET INVALID_SOCKET
    inline int  rt_socket_startup() { WSADATA w; return WSAStartup(MAKEWORD(2, 2), &w); }
    inline void rt_socket_cleanup() { WSACleanup(); }
    inline void rt_socket_close(rt_socket_t s) { closesocket(s); }
#else
    using rt_socket_t = int;
    #define RT_INVALID_SOCKET (-1)
    inline int  rt_socket_startup() { return 0; }
    inline void rt_socket_cleanup() {}
    inline void rt_socket_close(rt_socket_t s) { ::close(s); }
#endif

#endif // RTSTITCHING_PLATFORM_HPP
