#ifndef GAZE_DATA_READER_HPP
#define GAZE_DATA_READER_HPP

#include <string>
#include <vector>
#include <mutex>
#include <cstdint>
#include <chrono>
#include <atomic>
#include <thread>
#include <opencv2/core.hpp>

#include "Platform.hpp"
#include "ISaliencySource.hpp"

/**
 * @brief 单个注视点数据
 */
struct GazePoint {
    int64_t  timestamp_ms;  // Unix 毫秒时间戳
    double   x;             // 归一化屏幕坐标 [0,1]
    double   y;             // 归一化屏幕坐标 [0,1]
};

/**
 * @brief 注视数据读取器
 * 
 * 周期性读取 Python 写入的二进制注视数据文件,
 * 提供注视点查询和显著性图生成接口。
 */
class GazeDataReader : public ISaliencySource {
public:
    /**
     * @param file_path   二进制注视数据文件路径 (与 Python 端配置一致)
     * @param poll_interval_ms  文件轮询间隔 (毫秒)
     */
    explicit GazeDataReader(const std::string& file_path = "gaze_data.bin",
                            int poll_interval_ms = 50);
    ~GazeDataReader();

    // 禁止拷贝
    GazeDataReader(const GazeDataReader&) = delete;
    GazeDataReader& operator=(const GazeDataReader&) = delete;

    /**
     * @brief 切换为 socket 接收模式（在 start() 之前调用）
     *
     * 文件模式(默认): 轮询本地 gaze_data.bin —— 适合 Windows 本机测试。
     * socket 模式: 作为 TCP 服务端监听 port, 接收 Windows(Tobii) 端发来的
     *              注视数据 —— 适合 RK3588 板子(Tobii 只能在 Windows 跑)。
     * 两种模式产生的 gaze_points_ 完全一致, 下游(显著性图)无差别。
     *
     * @param port  TCP 监听端口(与 Python 发送端一致)
     */
    void useSocket(int port) { transport_ = Transport::SOCKET; port_ = port; }

    /**
     * @brief 启动后台线程(文件轮询 或 socket 接收, 取决于 useSocket)
     */
    bool start();

    /**
     * @brief 停止轮询线程
     */
    void stop();

    /**
     * @brief 获取最新的注视点列表 (线程安全)
     * @param max_age_ms 仅返回此时间范围内的注视点 (0=全部)
     */
    std::vector<GazePoint> getGazePoints(int64_t max_age_ms = 3000) const;

    /**
     * @brief 获取最新的注视焦点 (所有点的加权平均)
     * @param[out] x  归一化 x 坐标
     * @param[out] y  归一化 y 坐标
     * @return true 如果有有效注视数据
     */
    bool getGazeFocus(double& x, double& y) const;

    /**
     * @brief 生成注视显著性图 (核心接口)
     *
     * 在给定尺寸的图像上, 以注视点为中心绘制高斯分布,
     * 形成显著性热力图。值越高 = 用户越关注 = 接缝应避开。
     *
     * @param width   输出图像宽度 (像素)
     * @param height  输出图像高度 (像素)
     * @param sigma   高斯核标准差 (像素), 控制影响范围
     * @param max_age_ms 仅使用此时间范围内的注视点
     * @return CV_32F 单通道图像, 值域 [0, 1]
     */
    cv::Mat generateSaliencyMap(int width, int height,
                                double sigma = 80.0,
                                int64_t max_age_ms = 3000) const override;

    /**
     * @brief ISaliencySource 接口: 当前是否有可用注视数据
     */
    bool hasSaliency() const override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return !gaze_points_.empty();
    }

    /**
     * @brief 将归一化注视坐标映射到全景 warped 图像坐标
     *
     * 注视点是屏幕归一化坐标 [0,1], 需要映射到全景拼接结果的像素坐标。
     * 映射方式: 假设显示器上展示的是裁剪后的全景结果, 做线性映射。
     *
     * @param gaze_x  归一化注视 x
     * @param gaze_y  归一化注视 y
     * @param pano_width   全景图宽度
     * @param pano_height  全景图高度
     * @param[out] px  全景图像素 x
     * @param[out] py  全景图像素 y
     */
    static void gazeToPixel(double gaze_x, double gaze_y,
                            int pano_width, int pano_height,
                            int& px, int& py);

    /**
     * @brief 是否有新数据 (自上次调用 getGazePoints 后)
     */
    bool hasNewData() const { return has_new_data_.load(std::memory_order_acquire); }

    /**
     * @brief 获取最后一次成功读取的写入序列号
     */
    uint32_t getLastSeq() const { return last_seq_; }

private:
    enum class Transport { FILE, SOCKET };
    Transport transport_ = Transport::FILE;   // 默认文件模式(Windows 测试)
    int port_ = 0;                             // socket 模式监听端口

    // 文件读取
    std::string file_path_;
    int poll_interval_ms_;

    // 注视数据缓存
    mutable std::mutex data_mutex_;
    std::vector<GazePoint> gaze_points_;
    uint32_t last_seq_;
    mutable std::atomic<bool> has_new_data_;

    // 轮询线程
    std::atomic<bool> running_;
    std::thread poll_thread_;

    // 文件格式常量
    static const uint32_t MAGIC   = 0x47415A45;  // "GAZE"
    static const uint32_t VERSION = 1;
    static const int HEADER_SIZE  = 16;           // 4 × uint32
    static const int POINT_SIZE   = 24;           // int64 + 2 × double

    /**
     * @brief 读取一次二进制文件
     * @return true 如果成功读取且序列号有更新
     */
    bool readFile();

    /**
     * @brief 轮询线程主循环(文件模式)
     */
    void pollLoop();

    /**
     * @brief 解析一段完整的 gaze 二进制 blob(header+points)
     *        文件模式与 socket 模式共用同一个解析器, 保证语义一致。
     * @return true 当成功解析且序列号有更新
     */
    bool ingestBlob(const uint8_t* data, size_t len);

    // ---- socket 模式(TCP 服务端) ----
    void socketLoop();                            // 监听+接受连接的主循环
    void handleClient(rt_socket_t client);        // 单连接收数据循环
    bool recvAll(rt_socket_t fd, void* dst, size_t len);  // 收满 len 字节(可被 stop 打断)
};

#endif 
