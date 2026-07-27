#ifndef BLENDER_MODULE_HPP
#define BLENDER_MODULE_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include "Types.hpp"
#include "../CircularBufferSync/CircularBufferSync.hpp"
#include "../CircularBuffer/CircularBuffer.hpp"

//using namespace RTStitching;

class BlenderModule {
public:
    BlenderModule(
        std::vector<RTStitching::CameraStitchParams>& stitch_params,
        CircularBufferSync<RTStitching::Image>& input_buffer,
        CircularBuffer<RTStitching::Image>& output_buffer,
        RTStitching::ConfigParams& config_params,
        std::shared_ptr<std::shared_mutex> seam_mask_mutex,
        int show_window = 1,
        const std::string& module_name = ""
    );
    ~BlenderModule();



    // 线程控制接口
    bool start();           // 启动线程
    void stop();            // 停止线程
    void pause();           // 暂停线程  
    void resume();          // 恢复线程
    bool isRunning() const; // 检查线程是否在运行
    bool isPaused() const;  // 检查线程是否暂停

    // 获取错误信息
    std::string getLastError() const { return last_error_; }
    // 获取处理时间
    double getLastProcessingTime() const { return last_processing_time_; }

private:
    void runImpl();  // 线程主循环实现
    // 准备工作，在feed之前调用
    void prepare(const std::vector<cv::Point>& corners,
        const std::vector<cv::Size>& sizes);

    // 添加图像到混合器
    void feed(int img_idx, const cv::Mat& img, const cv::Mat& mask, const cv::Point& corner);

    // 执行混合操作
    void blend(cv::Mat& result, cv::Mat& result_mask);

    // 获取混合器状态信息
    std::string getInfo() const;

    // 新增：绘制时间戳信息到图像
    void drawTimestamps(cv::Mat& image,
        const std::vector<RTStitching::Image>& frame_data,
        const std::chrono::high_resolution_clock::time_point& buffer_read_time,
        const std::vector<std::chrono::high_resolution_clock::time_point>& feed_times,
        const std::chrono::high_resolution_clock::time_point& blend_finish_time);

    // 新增：格式化时间戳为字符串
    std::string formatTimestamp(const std::chrono::high_resolution_clock::time_point& timestamp);

    // 新增：计算时间差（毫秒）
    double calculateTimeDifference(const std::chrono::high_resolution_clock::time_point& start,
        const std::chrono::high_resolution_clock::time_point& end);

    // 新增：缩放掩码以适配目标矩形
    cv::Mat scaleMaskToFitRectangle(const cv::Mat& mask, int target_width = 669, int target_height = 749);

    // 新增：可视化缩放后的掩码（用于调试）
    void visualizeScaledMask(const cv::Mat& mask, int target_width, int target_height);

    // 线程控制成员变量
    std::atomic<bool> is_running_;
    std::atomic<bool> is_paused_;
    std::atomic<bool> stop_requested_;
    std::thread worker_thread_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::shared_ptr<std::shared_mutex> seam_mask_mutex_;    // mutex to protect seam mask
    int show_window_;

    // 混合器实例
    cv::Ptr<cv::detail::Blender> blender_;
    std::string last_error_;
    double last_processing_time_;

    // 配置参数
    RTStitching::ConfigParams& config_params_;
    std::vector<RTStitching::CameraStitchParams>& stitch_params_;

    bool is_initialized_;
    bool is_prepared_;
    int output_width_;
    int output_height_;
    // [N路适配] 原为 int last_index_[2]，按相机索引 i 访问，三路及以上会越界。
    //   改为 vector，在构造函数里按 camera_count 初始化为全 0。
    std::vector<uint64_t> last_index_;
    // [新增] 最近一次各路跳帧数（run() 性能统计计算，drawTimestamps 只读显示）
    std::vector<int64_t> last_skipped_;
    // [新增] 上一次 blend 完成时刻（用于统计输出帧间隔 = 端到端帧率倒数）
    std::chrono::high_resolution_clock::time_point perf_last_blend_finish_;
    bool perf_has_last_blend_ = false;
    // 缓冲区引用
    CircularBufferSync<RTStitching::Image>& input_buffer_;
    CircularBuffer<RTStitching::Image>& output_buffer_;

    // [新增] 异步 JPEG 写盘线程（launcher 显示通道 /tmp/stitched.jpg）
    //   原先每轮在融合线程里同步 imwrite 1080p JPEG（约 20-30ms），
    //   是 blend 结束到下一轮取帧之间约 63ms 空隙的主要来源之一。
    //   改为"单槽邮箱"：融合线程只 copy 一次 Mat 即返回，编码+rename
    //   由独立低优先级线程完成；写盘慢时邮箱自动只保留最新一帧。
    void jpegWriterLoop();
    std::thread jpeg_writer_thread_;
    std::mutex jpeg_mutex_;
    std::condition_variable jpeg_cv_;
    cv::Mat jpeg_pending_;
    bool jpeg_has_pending_ = false;
    std::atomic<bool> jpeg_stop_{ false };

    // [修改] 处理帧数据；new_data_flags 来自 CircularBufferSync::back()，
    //   标记本轮哪些路是新帧（false = 该路无新帧、复用了缓冲区旧帧），
    //   用于 reused_paths / max_ts_skew_ms 统计与陈旧度保护。
    bool processFrame(const std::vector<RTStitching::Image>& frame_data,
        const std::vector<bool>& new_data_flags = std::vector<bool>());

    // 设置混合器类型
    bool setBlenderType(const std::string& blender_type, bool try_cuda = false, int blend_strength = 5);
};

// 计算结果ROI
cv::Rect resultRoi(const std::vector<cv::Point>& corners, const std::vector<cv::Size>& sizes);

#endif // BLENDER_MODULE_HPP