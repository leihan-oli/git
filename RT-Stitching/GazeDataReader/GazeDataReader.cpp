//读取 Python 端写入的二进制注视数据, 生成显著性图供 SeamFinder 使用

#include "ISaliencySource.hpp"
#include "GazeDataReader.hpp"
#include <fstream>
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>

//构造
GazeDataReader::GazeDataReader(const std::string& file_path, int poll_interval_ms)
    : file_path_(file_path)
    , poll_interval_ms_(poll_interval_ms)
    , last_seq_(0)
    , has_new_data_(false)
    , running_(false)
{
}

GazeDataReader::~GazeDataReader() {
    stop();
}


bool GazeDataReader::start() {
    if (running_.load()) return true;
    running_.store(true);
    if (transport_ == Transport::SOCKET) {
        poll_thread_ = std::thread(&GazeDataReader::socketLoop, this);
        spdlog::error("[DIAG][GAZEREADER] Started in SOCKET mode, listening on port {}", port_);
    } else {
        poll_thread_ = std::thread(&GazeDataReader::pollLoop, this);
        spdlog::info("[GAZEREADER] Started polling '{}' every {}ms", file_path_, poll_interval_ms_);
    }
    return true;
}

void GazeDataReader::stop() {
    running_.store(false);
    if (poll_thread_.joinable()) {
        poll_thread_.join();
    }
    spdlog::info("[GAZEREADER] Stopped");
}

void GazeDataReader::pollLoop() {
    while (running_.load()) {
        readFile();
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms_));
    }
}

//文件读取：整文件读入后交给共享解析器 ingestBlob
bool GazeDataReader::readFile() {
    std::ifstream file(file_path_, std::ios::binary);
    if (!file.is_open()) {
        return false;  // Python 端尚未启动, 文件不存在
    }
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(file)),
                              std::istreambuf_iterator<char>());
    if (buf.size() < static_cast<size_t>(HEADER_SIZE)) {
        return false;  // 半写/空文件, 下次再读
    }
    return ingestBlob(buf.data(), buf.size());
}

// 共享解析器：文件模式与 socket 模式都调用它
// blob 布局(小端, 与 Python 端 struct '<IIII' + '<qdd' 一致):
//   [magic u32][version u32][count u32][write_seq u32]  (16B)
//   count × ([timestamp i64][x f64][y f64])             (24B each)
// 注: x86 与 RK3588(ARM) 均为小端, 直接 memcpy 即可, 无需字节序转换。
bool GazeDataReader::ingestBlob(const uint8_t* data, size_t len) {
    if (data == nullptr || len < static_cast<size_t>(HEADER_SIZE)) {
        return false;
    }
    uint32_t magic, version, count, write_seq;
    std::memcpy(&magic,     data + 0,  4);
    std::memcpy(&version,   data + 4,  4);
    std::memcpy(&count,     data + 8,  4);
    std::memcpy(&write_seq, data + 12, 4);

    if (magic != MAGIC) {
        spdlog::warn("[GAZEREADER] Invalid magic: 0x{:08X}", magic);
        return false;
    }
    if (version != VERSION) {
        spdlog::warn("[GAZEREADER] Unsupported version: {}", version);
        return false;
    }
    if (write_seq == last_seq_) {
        return false;  // 序列号未变, 无新数据
    }
    if (count > 10000) {
        spdlog::warn("[GAZEREADER] Unreasonable point count: {}", count);
        return false;
    }
    size_t need = static_cast<size_t>(HEADER_SIZE) + static_cast<size_t>(count) * POINT_SIZE;
    if (len < need) {
        spdlog::warn("[GAZEREADER] Truncated blob: have {} need {}", len, need);
        return false;
    }

    std::vector<GazePoint> points(count);
    const uint8_t* p = data + HEADER_SIZE;
    for (uint32_t i = 0; i < count; ++i) {
        int64_t ts; double x, y;
        std::memcpy(&ts, p + 0,  8);
        std::memcpy(&x,  p + 8,  8);
        std::memcpy(&y,  p + 16, 8);
        p += POINT_SIZE;
        points[i] = { ts, x, y };
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        gaze_points_ = std::move(points);
        last_seq_ = write_seq;
        has_new_data_.store(true, std::memory_order_release);
    }
    spdlog::debug("[GAZEREADER] Ingested {} points, seq={}", count, write_seq);
    return true;
}

// ============================================================
// socket 模式：作为 TCP 服务端，接收 Windows(Tobii) 端推送的注视数据
// ============================================================
bool GazeDataReader::recvAll(rt_socket_t fd, void* dst, size_t len) {
    uint8_t* out = static_cast<uint8_t*>(dst);
    size_t got = 0;
    while (got < len) {
        if (!running_.load()) return false;
        // 用 select 加超时, 让线程能周期性检查 running_ 以便干净退出
        fd_set rfds; FD_ZERO(&rfds); FD_SET(fd, &rfds);
        timeval tv; tv.tv_sec = 0; tv.tv_usec = 200 * 1000;  // 200ms
        int sel = select(static_cast<int>(fd) + 1, &rfds, nullptr, nullptr, &tv);
        if (sel == 0) continue;       // 超时, 回去检查 running_
        if (sel < 0)  return false;   // select 出错
        int n = ::recv(fd, reinterpret_cast<char*>(out + got),
                       static_cast<int>(len - got), 0);
        if (n <= 0) return false;     // 对端关闭或出错
        got += static_cast<size_t>(n);
    }
    return true;
}

void GazeDataReader::handleClient(rt_socket_t client) {
    std::vector<uint8_t> buf;
    uint64_t diag_n = 0;   // [DIAG] 收到的帧计数
    while (running_.load()) {
        // 自描述帧: 先收 16B header, 从中取 count, 再收 count*24B
        uint8_t header[HEADER_SIZE];
        if (!recvAll(client, header, HEADER_SIZE)) return;

        uint32_t count;
        std::memcpy(&count, header + 8, 4);
        if (count > 10000) {
            spdlog::warn("[GAZEREADER] socket: bad count {}, drop connection", count);
            return;  // 帧错位, 断开让对端重连
        }
        size_t payload = static_cast<size_t>(count) * POINT_SIZE;
        buf.resize(static_cast<size_t>(HEADER_SIZE) + payload);
        std::memcpy(buf.data(), header, HEADER_SIZE);
        if (payload > 0 && !recvAll(client, buf.data() + HEADER_SIZE, payload)) return;

        bool ok = ingestBlob(buf.data(), buf.size());
        if (diag_n < 3 || (diag_n % 60 == 0))   // [DIAG] 前3帧 + 之后每60帧
            spdlog::error("[DIAG][GAZEREADER] socket frame #{}: count={}, ingest_ok={}",
                          diag_n, count, ok);
        ++diag_n;
    }
}

void GazeDataReader::socketLoop() {
    if (rt_socket_startup() != 0) {
        spdlog::error("[GAZEREADER] socket startup failed");
        return;
    }
    rt_socket_t listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd == RT_INVALID_SOCKET) {
        spdlog::error("[GAZEREADER] create socket failed");
        rt_socket_cleanup();
        return;
    }
    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&yes), sizeof(yes));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;                  // 监听所有网卡
    addr.sin_port        = htons(static_cast<uint16_t>(port_));

    if (bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        spdlog::error("[DIAG][GAZEREADER] bind port {} FAILED (errno/WSA)", port_);
        rt_socket_close(listen_fd);
        rt_socket_cleanup();
        return;
    }
    listen(listen_fd, 1);
    spdlog::error("[DIAG][GAZEREADER] TCP server listening on port {}", port_);

    while (running_.load()) {
        // accept 也用 select 加超时, 便于干净退出
        fd_set rfds; FD_ZERO(&rfds); FD_SET(listen_fd, &rfds);
        timeval tv; tv.tv_sec = 0; tv.tv_usec = 200 * 1000;
        int sel = select(static_cast<int>(listen_fd) + 1, &rfds, nullptr, nullptr, &tv);
        if (sel <= 0) continue;

        rt_socket_t client = accept(listen_fd, nullptr, nullptr);
        if (client == RT_INVALID_SOCKET) continue;
        spdlog::error("[DIAG][GAZEREADER] client connected");
        handleClient(client);
        rt_socket_close(client);
        spdlog::info("[GAZEREADER] client disconnected, waiting for reconnect...");
    }

    rt_socket_close(listen_fd);
    rt_socket_cleanup();
    spdlog::info("[GAZEREADER] socket server stopped");
}

// 获取注视点列表
std::vector<GazePoint> GazeDataReader::getGazePoints(int64_t max_age_ms) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    has_new_data_.store(false, std::memory_order_release);

    if (max_age_ms <= 0) return gaze_points_;

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    int64_t cutoff = now_ms - max_age_ms;

    std::vector<GazePoint> result;
    result.reserve(gaze_points_.size());
    for (const auto& pt : gaze_points_) {
        if (pt.timestamp_ms >= cutoff) result.push_back(pt);
    }
    return result;
}

bool GazeDataReader::getGazeFocus(double& x, double& y) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (gaze_points_.empty()) return false;

    double sum_x = 0, sum_y = 0, sum_w = 0;
    int64_t newest_ts = gaze_points_.back().timestamp_ms;

    for (const auto& pt : gaze_points_) {
        double age_ms = static_cast<double>(newest_ts - pt.timestamp_ms);
        double w = std::exp(-age_ms / 500.0);  // 半衰期 500ms
        sum_x += pt.x * w;
        sum_y += pt.y * w;
        sum_w += w;
    }
    if (sum_w < 1e-9) return false;
    x = sum_x / sum_w;
    y = sum_y / sum_w;
    return true;
}
// 注视坐标映射
void GazeDataReader::gazeToPixel(double gaze_x, double gaze_y,
                                  int pano_width, int pano_height,
                                  int& px, int& py) {
    px = std::max(0, std::min(pano_width  - 1,
         static_cast<int>(std::round(gaze_x * (pano_width  - 1)))));
    py = std::max(0, std::min(pano_height - 1,
         static_cast<int>(std::round(gaze_y * (pano_height - 1)))));
}

// 生成显著性图（高斯核4）
cv::Mat GazeDataReader::generateSaliencyMap(int width, int height,
                                             double sigma,
                                             int64_t max_age_ms) const {
    cv::Mat saliency = cv::Mat::zeros(height, width, CV_32F);

    auto points = getGazePoints(max_age_ms);
    if (points.empty()) return saliency;

    int64_t newest_ts = points.back().timestamp_ms;
    int radius = static_cast<int>(3.0 * sigma);
    double inv_2sigma2 = 1.0 / (2.0 * sigma * sigma);

    for (const auto& pt : points) {
        int px, py;
        gazeToPixel(pt.x, pt.y, width, height, px, py);

        // 时间衰减: 越新的注视点权重越大, 半衰期 1000ms
        double age_ms = static_cast<double>(newest_ts - pt.timestamp_ms);
        double time_weight = std::exp(-age_ms / 1000.0);

        int x0 = std::max(0, px - radius);
        int x1 = std::min(width  - 1, px + radius);
        int y0 = std::max(0, py - radius);
        int y1 = std::min(height - 1, py + radius);

        for (int row = y0; row <= y1; ++row) {
            float* ptr = saliency.ptr<float>(row);
            for (int col = x0; col <= x1; ++col) {
                double dx = col - px, dy = row - py;
                ptr[col] += static_cast<float>(
                    std::exp(-(dx * dx + dy * dy) * inv_2sigma2) * time_weight);
            }
        }
    }

    // 归一化到 [0, 1]
    double max_val;
    cv::minMaxLoc(saliency, nullptr, &max_val);
    if (max_val > 1e-6) saliency /= static_cast<float>(max_val);

    return saliency;
}
