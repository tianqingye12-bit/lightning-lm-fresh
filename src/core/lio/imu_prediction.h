#ifndef LIGHTNING_IMU_PREDICTION_H
#define LIGHTNING_IMU_PREDICTION_H

#include <cmath>

namespace lightning::prediction {

struct Options {
    double max_step_s = 0.10;
    double max_horizon_s = 0.30;
    int stats_interval = 200;
    bool Valid() const {
        return std::isfinite(max_step_s) && std::isfinite(max_horizon_s) &&
               max_step_s > 0 && max_horizon_s >= max_step_s && stats_interval >= 0;
    }
};

enum class Decision { predict, already_covered, invalid_time, step_gap, stale_lio };

// correction_time is the last accepted LiDAR update, not just a propagated LIO timestamp.
inline Decision CheckTime(double current_time, double target_time, double correction_time, const Options& options) {
    if (!options.Valid() || !std::isfinite(current_time) || !std::isfinite(target_time) ||
        !std::isfinite(correction_time) || current_time <= 0 || correction_time <= 0) {
        return Decision::invalid_time;
    }
    if (target_time <= current_time) return Decision::already_covered;
    if (target_time - correction_time > options.max_horizon_s + 1e-6) return Decision::stale_lio;
    if (target_time - current_time > options.max_step_s + 1e-6) return Decision::step_gap;
    return Decision::predict;
}

inline const char* Name(Decision decision) {
    switch (decision) {
        case Decision::predict: return "predict";
        case Decision::already_covered: return "already_covered";
        case Decision::invalid_time: return "invalid_time";
        case Decision::step_gap: return "step_gap";
        case Decision::stale_lio: return "stale_lio";
    }
    return "unknown";
}

// Keep UI timestamps monotonic across LiDAR rebases and skipped scans.
inline bool CanPublish(double timestamp, double last_published) {
    return std::isfinite(timestamp) && timestamp > last_published;
}

}  // namespace lightning::prediction
#endif
