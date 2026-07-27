#ifndef CIRCULAR_BUFFER_SYNC_HPP
#define CIRCULAR_BUFFER_SYNC_HPP

#include <boost/circular_buffer.hpp>
#include <vector>
#include <set>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <atomic>
#include <spdlog/spdlog.h>
#include <stdexcept>

/**
 * @brief A thread-safe circular buffer container that synchronizes data across multiple buffers.
 *
 * This class manages a collection of circular buffers, each accessible via an index. It provides
 * thread-safe write operations to individual buffers and a synchronized read operation that waits
 * until all buffers have new data before returning.
 *
 * @tparam T The type of elements stored in the circular buffers.
 */
template<typename T>
class CircularBufferSync {
public:
    /**
     * @brief Construct a new Circular Buffer Sync object.
     *
     * @param num_of_buf Number of individual circular buffers to manage.
     * @param buf_cap Capacity of each individual circular buffer.
     */
    CircularBufferSync(size_t num_of_buf, size_t buf_cap)
        : num_buffers_(num_of_buf) {
        if (num_of_buf == 0) {
            spdlog::error("[CIRCULARBUFFERSYNC] Constructor failed: number of buffers cannot be zero");
            throw std::invalid_argument("[CIRCULARBUFFERSYNC] Number of buffers cannot be zero");
        }
        if (buf_cap == 0) {
            spdlog::error("[CIRCULARBUFFERSYNC] Constructor failed: buffer capacity cannot be zero");
            throw std::invalid_argument("[CIRCULARBUFFERSYNC] Buffer capacity cannot be zero");
        }

        buffers_.reserve(num_of_buf);
        new_data_flags_.reserve(num_of_buf);

        // Initialize each buffer and its corresponding atomic flag
        for (size_t i = 0; i < num_of_buf; ++i) {
            buffers_.emplace_back(buf_cap);
            new_data_flags_.push_back(new std::atomic<bool>(false));
        }
    }

    /**
     * @brief Destroy the Circular Buffer Sync object.
     *
     * Cleans up all dynamically allocated resources, including atomic flags.
     */
    ~CircularBufferSync() {
        // Clean up dynamically allocated atomic flags
        for (auto* flag : new_data_flags_) {
            delete flag;
        }
    }

    // Delete copy constructor and assignment operator to prevent thread-safety issues
    CircularBufferSync(const CircularBufferSync&) = delete;
    CircularBufferSync& operator=(const CircularBufferSync&) = delete;

    /**
     * @brief Write data to a specific circular buffer.
     *
     * Thread-safe operation that adds data to the specified buffer, marks it as updated,
     * and notifies waiting threads.
     *
     * @param buffer_index Index of the buffer to write to (0-based).
     * @param data The data element to add to the buffer.
     * @return true if the write was successful (valid buffer index).
     * @return false if the buffer index is out of range.
     */
    bool push_back(size_t buffer_index, const T& data) {
        // Validate buffer index
        if (buffer_index >= num_buffers_) {
            spdlog::warn("[CIRCULARBUFFERSYNC] push_back failed: buffer index {} out of range (max: {})",
                buffer_index, num_buffers_ - 1);
            return false;
        }

        // Lock for thread-safe modification of the buffer
        std::lock_guard<std::mutex> lock(mutex_);

        auto& target_buf = buffers_[buffer_index];
        // Check if buffer is full (boost::circular_buffer will overwrite by default, add warning)
        if (target_buf.full()) {
            //spdlog::warn("[CIRCULARBUFFERSYNC] push_back: buffer {} is full, overwriting oldest data (capacity: {})",
            //    buffer_index, target_buf.capacity());
        }

        // Add data to the specified circular buffer (boost::circular_buffer handles overflow safely)
        target_buf.push_back(data);

        // Mark buffer as having new data and track the updated buffer
        new_data_flags_[buffer_index]->store(true, std::memory_order_release);
        updated_producers_.insert(buffer_index);

        // Notify waiting threads that new data is available
        cond_var_.notify_all();

        return true;
    }

    bool back(std::vector<T>& results, std::vector<bool>& new_data_flags,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),size_t wait_for_num=1) {
        // Lock with unique_lock to allow condition variable waiting
        std::unique_lock<std::mutex> lock(mutex_);
        if (wait_for_num > num_buffers_ || wait_for_num < 1)
            wait_for_num = num_buffers_;
        // Predicate checks if all buffers have new data
        auto predicate = [wait_for_num,this]() {
            return (updated_producers_.size() >= wait_for_num);
            };

        // Wait for all buffers to be updated or timeout
        if (!cond_var_.wait_for(lock, timeout, predicate)) {
            spdlog::debug("[CIRCULARBUFFERSYNC] back failed: timeout waiting for all buffers (updated: {}, need: {})",
                updated_producers_.size(), wait_for_num);
            return false;
        }

        // Prepare output vectors
        results.clear();
        new_data_flags.clear();
        results.reserve(num_buffers_);
        new_data_flags.reserve(num_buffers_);

        // Collect latest data and flags from each buffer
        for (size_t i = 0; i < num_buffers_; ++i) {
            auto& target_buf = buffers_[i];
            if (!target_buf.empty()) {
                results.push_back(target_buf.back()); // Get most recent element
            }
            else {
                // Buffer is empty (should not happen if updated flag is set, add warning)
                spdlog::warn("[CIRCULARBUFFERSYNC] back: buffer {} is marked as updated but empty", i);
                results.push_back(T()); // Default-constructed element if buffer is empty
                return false;
            }
            new_data_flags.push_back(new_data_flags_[i]->load(std::memory_order_acquire));
        }

        // Reset all new data flags and clear update tracking
        for (size_t i = 0; i < num_buffers_; ++i) {
            new_data_flags_[i]->store(false, std::memory_order_release);
        }
        updated_producers_.clear();

        return true;
    }

    /**
     * @brief ������л�������Ԫ�أ�����������������������䣩
     *
     * �̰߳�ȫ���������ÿ�� circular_buffer �е�Ԫ�أ�ͬʱ���������ݱ�־�͸��¸���״̬��
     * ������պ����״̬����ͬ���߼��쳣��
     */
    void clearAllBuffers() {
        std::lock_guard<std::mutex> lock(mutex_); // �̰߳�ȫ��

        // �������л����������Ԫ�أ�boost::circular_buffer::clear() ����������
        for (auto& buf : buffers_) {
            buf.clear();
        }

        // �������������ݱ�־��������պ��Ա��Ϊ�������ݣ�
        for (auto* flag : new_data_flags_) {
            flag->store(false, std::memory_order_release);
        }

        // ��ո��¸��ټ��ϣ�����������״̬��
        updated_producers_.clear();

        spdlog::debug("[CIRCULARBUFFERSYNC] clearAllBuffers: cleared all elements from {} buffers (capacity remains unchanged)",
            num_buffers_);
    }

    /**
     * @brief Get the total number of managed buffers.
     *
     * @return size_t Number of buffers.
     */
    size_t getNumBuffers() const {
        return num_buffers_;
    }

    /**
     * @brief Get the count of buffers that have new/updated data.
     *
     * @return size_t Number of updated buffers.
     */
    size_t getUpdatedProducerCount() const {
        std::lock_guard<std::mutex> lock(mutex_); // Ensure thread-safe access
        return updated_producers_.size();
    }

    /**
     * @brief Reset the state of all buffers (clear update flags and tracking).
     *
     * Resets all new data flags and clears the set of updated buffers.
     */
    void resetState() {
        std::lock_guard<std::mutex> lock(mutex_); // Ensure thread-safe modification

        // Reset all new data flags
        for (size_t i = 0; i < num_buffers_; ++i) {
            new_data_flags_[i]->store(false, std::memory_order_release);
        }
        // Clear tracking of updated buffers
        updated_producers_.clear();

        spdlog::debug("[CIRCULARBUFFERSYNC] resetState: cleared all update flags and tracking");
    }

private:
    std::vector<boost::circular_buffer<T>> buffers_; ///< Collection of circular buffers
    std::vector<std::atomic<bool>*> new_data_flags_; ///< Atomic flags tracking new data in each buffer
    std::set<size_t> updated_producers_; ///< Set tracking indices of buffers with new data

    mutable std::mutex mutex_; ///< Mutex for thread-safe operations
    std::condition_variable cond_var_; ///< Condition variable for synchronization

    const size_t num_buffers_; ///< Total number of managed buffers
};

#endif // CIRCULAR_BUFFER_SYNC_HPP