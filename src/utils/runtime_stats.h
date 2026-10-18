#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace lightning {
class RuntimeStats {
 public:
    enum Stage { preprocess, deskew, downsample, registration, map_update, ui_prepare,
                 frame_total, imu_wait, lidar_wait, imu_update, save_map, render_frame, matching, count };
    using Clock = std::chrono::steady_clock;
    void Observe(Stage stage, double ms) {
        if (!std::isfinite(ms) || ms < 0) return;
        std::lock_guard<std::mutex> lock(mutex_);
        auto& r = records_[stage];
        ++r.n; r.total += ms; r.maximum = std::max(r.maximum, ms);
        r.samples[r.next++ % r.samples.size()] = ms;
    }
    // Cumulative mean/max and p95 of the latest 256 samples, bounded memory.
    std::string Summary() const {
        static const char* names[] = {"preprocess", "deskew", "downsample", "registration", "map_update",
            "ui_prepare", "frame_total", "imu_wait", "lidar_wait", "imu_update", "save_map", "render_frame", "matching"};
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream out; out << std::fixed << std::setprecision(2);
        for (int i = 0; i < count; ++i) {
            const auto& r = records_[i]; if (!r.n) continue;
            std::vector<double> samples(r.samples.begin(), r.samples.begin() + std::min(r.n, r.samples.size()));
            std::sort(samples.begin(), samples.end());
            out << ' ' << names[i] << "_ms(mean/p95/max)=" << r.total/r.n << '/'
                << samples[static_cast<size_t>(std::ceil(samples.size()*.95)) - 1] << '/' << r.maximum << " n=" << r.n;
        }
        return out.str();
    }
    class Scope {
     public:
        Scope(RuntimeStats& stats, Stage stage) : stats_(stats), stage_(stage), started_(Clock::now()) {}
        ~Scope() { stats_.Observe(stage_, std::chrono::duration<double, std::milli>(Clock::now()-started_).count()); }
     private:
        RuntimeStats& stats_; Stage stage_; Clock::time_point started_;
    };
 private:
    struct Record { size_t n=0, next=0; double total=0, maximum=0; std::array<double,256> samples{}; };
    mutable std::mutex mutex_;
    std::array<Record, count> records_;
};
template<class F> decltype(auto) Measure(RuntimeStats& stats, RuntimeStats::Stage stage, F&& fn) {
    RuntimeStats::Scope scope(stats, stage);
    return std::forward<F>(fn)();
}
}  // namespace lightning
