#ifndef LIGHTNING_LATEST_TASK_H
#define LIGHTNING_LATEST_TASK_H

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <utility>

namespace lightning {
// Bounded work queue: replace stale pending snapshots and interrupt waits on shutdown.
template <typename T>
class LatestTask {
 public:
    using Clock = std::chrono::steady_clock;
    bool Put(T value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_) return false;
            pending_ = std::move(value);
        }
        condition_.notify_one();
        return true;
    }
    bool Wait(T& value, Clock::time_point earliest) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stopped_ || pending_.has_value(); });
        if (stopped_) return false;
        condition_.wait_until(lock, earliest, [this] { return stopped_; });
        if (stopped_) return false;
        value = std::move(*pending_);
        pending_.reset();
        return true;
    }
    void Stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
            pending_.reset();
        }
        condition_.notify_all();
    }
 private:
    std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<T> pending_;
    bool stopped_ = false;
};
}  // namespace lightning
#endif
