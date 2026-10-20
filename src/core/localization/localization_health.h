#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

namespace lightning::loc {
// Uses steady wall time for outages, message time for processing lag. Never compare bag time to wall time.
class LocalizationHealth {
 public:
    struct Limits {
        double imu_timeout_s = 1.0;
        double lidar_timeout_s = 2.0;
        double result_timeout_s = 2.0;
        double max_result_lag_s = 1.0;
        double max_match_lag_s = 3.0;
        unsigned recovery_matches = 3;
    };
    struct Snapshot {
        std::string state, reason;
        bool valid = false;
        double imu_age = -1, lidar_age = -1, result_age = -1, result_lag = -1, match_lag = -1;
        unsigned good_matches = 0;
    };
    void Configure(const Limits& limits) {
        for (double x : {limits.imu_timeout_s, limits.lidar_timeout_s, limits.result_timeout_s,
                         limits.max_result_lag_s, limits.max_match_lag_s})
            if (!std::isfinite(x) || x <= 0) throw std::invalid_argument("invalid localization health timeout");
        if (!limits.recovery_matches) throw std::invalid_argument("recovery_matches must be positive");
        std::lock_guard<std::mutex> lock(mutex_);
        limits_ = limits;
    }
    // Call before handing a message to the estimator; regression latches until restart.
    bool Input(bool imu, double stamp, double now) {
        std::lock_guard<std::mutex> lock(mutex_);
        Evaluate(now);
        auto& stream = imu ? imu_ : lidar_;
        if (!std::isfinite(stamp) || stamp < 0 || (stream.seen && stamp < stream.stamp)) {
            clock_fault_ = true; good_ = 0; return false;
        }
        if (clock_fault_ || (stream.seen && stamp == stream.stamp)) return false;
        stream = {true, stamp, now};
        return true;
    }
    void Match(double stamp, bool good, double now) {
        std::lock_guard<std::mutex> lock(mutex_);
        Evaluate(now);
        // Reusing GetLocalizationResult() after a failed scan must not count as a new match.
        if (!good || !std::isfinite(stamp) || (match_seen_ && stamp <= match_stamp_) ||
            !imu_.seen || !lidar_.seen || stamp > Latest() + 0.5 || Latest() - stamp > limits_.max_match_lag_s) {
            match_good_ = false; good_ = 0; return;
        }
        match_stamp_ = stamp; match_seen_ = true; match_good_ = true;
        good_ = std::min(good_ + 1, limits_.recovery_matches);
    }
    void Result(double stamp, bool valid, double now) {
        std::lock_guard<std::mutex> lock(mutex_);
        Evaluate(now);
        if (!std::isfinite(stamp) || (result_.seen && stamp < result_.stamp) || stamp > Latest() + 0.5) {
            result_good_ = false; return;
        }
        // Duplicate output does not refresh the watchdog.
        if (!result_.seen || stamp > result_.stamp) result_ = {true, stamp, now};
        result_good_ = valid;
    }
    Snapshot Get(double now) {
        std::lock_guard<std::mutex> lock(mutex_);
        return Evaluate(now);
    }
 private:
    struct Stream { bool seen = false; double stamp = 0, wall = 0; };
    double Latest() const { return std::max(imu_.stamp, lidar_.stamp); }
    Snapshot Evaluate(double now) {
        Snapshot s;
        s.imu_age = imu_.seen ? std::max(0.0, now - imu_.wall) : -1;
        s.lidar_age = lidar_.seen ? std::max(0.0, now - lidar_.wall) : -1;
        s.result_age = result_.seen ? std::max(0.0, now - result_.wall) : -1;
        s.result_lag = result_.seen ? std::max(0.0, Latest() - result_.stamp) : -1;
        s.match_lag = match_seen_ ? std::max(0.0, Latest() - match_stamp_) : -1;
        s.state = "INITIALIZING"; s.reason = "waiting_for_sensors";
        if (clock_fault_) { s.state = "LOST"; s.reason = "timestamp_regression_restart_required"; }
        else if (s.imu_age > limits_.imu_timeout_s) { s.state = "LOST"; s.reason = "imu_timeout"; }
        else if (s.lidar_age > limits_.lidar_timeout_s) { s.state = "LOST"; s.reason = "lidar_timeout"; }
        else if (!imu_.seen || !lidar_.seen) { /* initializing */ }
        else if (s.match_lag > limits_.max_match_lag_s) { s.state = "LOST"; s.reason = "map_match_stale"; }
        else if (s.result_age > limits_.result_timeout_s || s.result_lag > limits_.max_result_lag_s) {
            s.state = "LOST"; s.reason = "result_stale";
        } else if (!match_seen_) { s.reason = "waiting_for_map_match"; }
        else if (!match_good_) { s.state = "DEGRADED"; s.reason = "map_match_rejected"; }
        else if (good_ < limits_.recovery_matches) { s.reason = "confirming_recovery"; }
        else if (!result_.seen || !result_good_) { s.state = "DEGRADED"; s.reason = "invalid_fused_result"; }
        else { s.state = "GOOD"; s.reason = "ok"; s.valid = true; }
        if (s.state == "LOST") good_ = 0;
        s.good_matches = good_;
        return s;
    }
    std::mutex mutex_;
    Limits limits_;
    Stream imu_, lidar_, result_;
    bool clock_fault_ = false, match_seen_ = false, match_good_ = false, result_good_ = false;
    double match_stamp_ = 0;
    unsigned good_ = 0;
};
}  // namespace lightning::loc
