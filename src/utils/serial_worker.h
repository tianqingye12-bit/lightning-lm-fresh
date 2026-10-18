#pragma once
#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <future>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <thread>

namespace lightning {
// FIFO admission order. Only this worker may touch the live SLAM core; no parallel ESKF calls.
class SerialWorker {
 public:
    enum Kind { imu = 0, lidar = 1, control = 2 };
    struct Snapshot {
        std::array<size_t, 3> queued{};
        std::array<size_t, 3> rejected{};
        size_t failures = 0;
    };
    explicit SerialWorker(std::array<size_t, 3> limits) : limits_(limits) {
        for (auto n : limits) if (!n) throw std::invalid_argument("worker capacities must be positive");
    }
    ~SerialWorker() { Stop(); }
    SerialWorker(const SerialWorker&) = delete;
    SerialWorker& operator=(const SerialWorker&) = delete;
    void Start() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (started_) throw std::logic_error("worker starts only once");
        started_ = true;
        thread_ = std::thread([this] { Run(); });
    }
    bool Post(Kind kind, std::function<void(double)> task) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_ || closed_ || state_.queued[kind] >= limits_[kind]) {
            ++state_.rejected[kind];
            return false;
        }
        jobs_.push_back({kind, std::move(task), Clock::now()});
        ++state_.queued[kind];
        cv_.notify_one();
        return true;
    }
    bool IsCurrent() const { return std::this_thread::get_id() == thread_.get_id(); }
    // Safe for services/public SaveMap; exceptions reach the caller and never strand a future.
    void Call(std::function<void()> fn) {
        if (std::this_thread::get_id() == thread_.get_id()) { fn(); return; }
        auto promise = std::make_shared<std::promise<void>>();
        auto result = promise->get_future();
        if (!Post(control, [fn = std::move(fn), promise](double) {
                try { fn(); promise->set_value(); }
                catch (...) { promise->set_exception(std::current_exception()); }
            })) throw std::runtime_error("SLAM control queue closed/full");
        result.get();
    }
    Snapshot State() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }
    // Drain admitted work before joining. Called after the receiver executor stops.
    void Stop() {
        { std::lock_guard<std::mutex> lock(mutex_); closed_ = true; }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }
 private:
    using Clock = std::chrono::steady_clock;
    struct Job { Kind kind; std::function<void(double)> fn; Clock::time_point arrived; };
    void Run() {
        for (;;) {
            Job job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return closed_ || !jobs_.empty(); });
                if (jobs_.empty()) return;
                job = std::move(jobs_.front()); jobs_.pop_front();
                --state_.queued[job.kind];
            }
            const double wait_ms = std::chrono::duration<double, std::milli>(Clock::now() - job.arrived).count();
            try { job.fn(wait_ms); }
            catch (...) { std::lock_guard<std::mutex> lock(mutex_); ++state_.failures; }
        }
    }
    std::array<size_t, 3> limits_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<Job> jobs_;
    Snapshot state_;
    bool started_ = false, closed_ = false;
    std::thread thread_;
};
}  // namespace lightning
