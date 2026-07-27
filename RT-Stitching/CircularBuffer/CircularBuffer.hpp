#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <boost/circular_buffer.hpp>
#include <stdexcept>
#include <string>
#include <opencv2/core/mat.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <mutex>  // 保留互斥锁支持
#include <condition_variable>

/**
 * 带基础锁机制的环形缓冲区
 * 特点：保留必要的线程安全支持，适用于轻量级多线程场景
 * 优化点：
 * 1. 保留std::mutex实现基础线程安全（区别于纯无锁版本）
 * 2. 结合性能与安全性，采用lock_guard实现自动加解锁
 * 3. 保留灵活的接口设计同时保证同步安全性
 * 4. 补充异常处理和日志输出，避免溢出/读空
 */
template<typename T>
class CircularBuffer {
public:
    std::condition_variable cv_;             // 条件变量（声明cv_）

    // 显式构造函数（带参数校验）
    explicit CircularBuffer(size_t capacity) : buffer_(capacity) {
        if (capacity == 0) {
            spdlog::error("[CIRCULARBUFFER] Constructor failed: capacity cannot be zero");
            throw std::invalid_argument("[CIRCULARBUFFER] Capacity cannot be zero");
        }
    }

    // 默认构造函数（委托构造）
    CircularBuffer() : CircularBuffer(100) {}

    /**
     * 写入元素（带锁，覆盖模式）
     * 返回值：true=正常插入，false=缓冲区满已覆盖旧元素
     */
    bool push_back(const T& item) {
        std::lock_guard<std::mutex> lock(mutex_);

        bool overwrote = buffer_.full();
        if (overwrote) {
            // 缓冲区满时输出警告，告知覆盖行为
            spdlog::warn("[CIRCULARBUFFER] Buffer full, overwriting oldest element (capacity: {}, current size: {})",
                buffer_.capacity(), buffer_.size());
            //buffer_.pop_front();
        }
        buffer_.push_back(item);
        return !overwrote;
    }

    /**
     * 尝试写入元素（非覆盖，带锁）
     * 返回值：true=写入成功，false=缓冲区满写入失败
     */
    bool try_push_back(const T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.full()) {
            // 缓冲区满时输出调试日志，告知写入失败
            spdlog::debug("[CIRCULARBUFFER] try_push_back failed: buffer is full (capacity: {}, current size: {})",
                buffer_.capacity(), buffer_.size());
            return false;
        }
        buffer_.push_back(item);
        return true;
    }

    /**
     * 读取并移除队首元素（带锁）
     * 异常：缓冲区空时抛出runtime_error
     */
    T pop_front() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            spdlog::error("[CIRCULARBUFFER] pop_front failed: buffer is empty");
            throw std::runtime_error("[CIRCULARBUFFER] Cannot pop from empty buffer");
        }
        T item = buffer_.front();
        buffer_.pop_front();
        return item;
    }

    /**
     * 获取队尾元素（不删除，带锁）
     * 异常：缓冲区空时抛出runtime_error
     */
    T back_without_clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            spdlog::error("[CIRCULARBUFFER] back failed: buffer is empty");
            throw std::runtime_error("[CIRCULARBUFFER] Cannot get back from empty buffer");
        }
        return buffer_.back();
    }

    /**
     * @brief 带锁读取循环缓冲区队尾元素，并清空整个缓冲区
     * @tparam T 缓冲区存储的数据类型
     * @param[out] item 接收队尾元素的引用：成功时为最新队尾值，缓冲区空时无效
     * @return bool 执行结果：true=读取并清空成功，false=缓冲区空（读取失败）
     * @note 1. 线程安全：通过std::lock_guard自动加/解锁（基于mutex_），避免并发冲突；
     *       2. 日志说明：缓冲区空时输出失败日志，元素数>1时输出“元素数超1且已清空”的提示日志
     */
    bool back(T& item) {
        std::lock_guard<std::mutex> lock(mutex_); // RAII锁：作用域内自动加锁，退出时解锁

        if (buffer_.empty()) { // 缓冲区空：输出日志并返回失败
            spdlog::info("[CIRCULARBUFFER] back failed: buffer is empty");
            return false;
        }

        if (buffer_.size() > 1) // 元素数>1：输出提示日志（说明后续会清空）
            spdlog::info("[CIRCULARBUFFER] num of items bigger than one, have cleared item");

        item = buffer_.back(); // 读取缓冲区队尾元素赋值给item

        buffer_.clear(); // 清空整个缓冲区（核心逻辑，与“读队尾”绑定）
        return true;
    }
    /**
     * 等待并读取队尾元素（阻塞，带锁）
     * 行为：缓冲区空时阻塞，直到有元素写入
     */
    T wait_pop_back() {
        std::unique_lock<std::mutex> lock(mutex_);

        // 阻塞等待，直到缓冲区非空（避免虚假唤醒）
        cv_.wait(lock, [this] { return !buffer_.empty(); });

        T item = buffer_.back();
        buffer_.pop_back();
        return item;
    }

    /**
     * 获取队首元素（不删除，带锁）
     * 异常：缓冲区空时抛出runtime_error
     */
    T front() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            spdlog::error("[CIRCULARBUFFER] front failed: buffer is empty");
            throw std::runtime_error("[CIRCULARBUFFER] Cannot get front from empty buffer");
        }
        return buffer_.front();
    }

    /**
     * 按索引获取元素（不删除，带锁）
     * 返回值：true=获取成功，false=缓冲区空或索引无效
     */
    bool get_i(int i, T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            spdlog::debug("[CIRCULARBUFFER] get_i failed: buffer is empty");
            return false;
        }
        // 修复索引判断逻辑：i应>=0且<i buffer_.size()（0-based索引）
        if (i < 0 || static_cast<size_t>(i) > buffer_.size()) {
            spdlog::warn("[CIRCULARBUFFER] get_i failed: invalid index {} (current size: {})",
                i, buffer_.size());
            return false;
        }
        item = buffer_.at(i);
        return true;
    }

    /**
     * 尝试读取队首元素（非阻塞，带锁）
     * 返回值：true=读取成功，false=缓冲区空
     */
    bool try_pop_front(T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            spdlog::debug("[CIRCULARBUFFER] try_pop_front failed: buffer is empty");
            return false;
        }
        item = buffer_.front();
        buffer_.pop_front();
        return true;
    }

    // 状态查询方法（均带锁保护，确保线程安全）
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.empty();
    }

    bool full() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.full();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }

    size_t capacity() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.capacity();
    }

    size_t available() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.capacity() - buffer_.size();
    }

    /**
     * 清空缓冲区（带锁）
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!buffer_.empty()) {
            spdlog::debug("[CIRCULARBUFFER] Clearing buffer (current size: {})", buffer_.size());
            buffer_.clear();
        }
    }

private:
    boost::circular_buffer<T> buffer_;  // 底层存储容器
    mutable std::mutex mutex_;          // 互斥锁（mutable允许const方法使用）
};

#endif // CIRCULAR_BUFFER_H