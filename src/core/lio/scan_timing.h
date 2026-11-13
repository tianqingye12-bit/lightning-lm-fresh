#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace lightning::timing {
// Explicit opt-in: milliseconds are offsets from the ROS header, which can lie
// before, inside or after a scan. Preserve header+offset for every sample.
struct Options {
    bool enabled = false;
    double min_duration_s = .05, max_duration_s = .15, max_header_offset_s = .2;
    bool Valid() const {
        return std::isfinite(min_duration_s) && std::isfinite(max_duration_s) &&
               std::isfinite(max_header_offset_s) && min_duration_s > 0 &&
               min_duration_s < max_duration_s && max_duration_s <= 1 &&
               max_header_offset_s >= max_duration_s && max_header_offset_s <= 2;
    }
};
struct Interval {
    bool valid = false;
    double begin = 0, end = 0, minimum_s = 0, maximum_s = 0;
    const char* reason = "empty_scan";
};
template <class PointRange>
Interval HeaderInterval(const PointRange& points, double header, const Options& options) {
    Interval out;
    if (!options.Valid()) { out.reason = "invalid_options"; return out; }
    if (!std::isfinite(header)) { out.reason = "nonfinite_header"; return out; }
    if (points.size() < 2) return out;
    double low = std::numeric_limits<double>::infinity(), high = -low;
    for (const auto& point : points) {
        const double offset = point.time / 1000.0;
        if (!std::isfinite(offset)) { out.reason = "nonfinite_point_time"; return out; }
        low = std::min(low, offset); high = std::max(high, offset);
    }
    out.minimum_s = low; out.maximum_s = high;
    if (std::max(std::abs(low),std::abs(high)) > options.max_header_offset_s) {
        out.reason = "offset_outside_header_window"; return out;
    }
    const double duration = high-low;
    if (duration < options.min_duration_s || duration > options.max_duration_s) {
        out.reason = "scan_duration_outside_limits"; return out;
    }
    out.begin = header+low; out.end = header+high;
    if (!std::isfinite(out.begin) || !std::isfinite(out.end) || out.end <= out.begin) {
        out.reason = "unrepresentable_scan_interval"; return out;
    }
    out.valid = true; out.reason = "ok"; return out;
}
template <class PointRange>
Interval NormalizeHeaderOffsets(PointRange& points, double header, const Options& options) {
    const auto out = HeaderInterval(points, header, options);
    if (!out.valid) return out;  // Validation failure must not partially mutate a scan.
    double shift_ms = points.front().time;
    for (const auto& point : points) shift_ms = std::min(shift_ms, static_cast<double>(point.time));
    for (auto& point : points) point.time -= shift_ms;
    return out;
}
inline bool CanBackcast(double begin, double state_time, double tolerance = .002) {
    return std::isfinite(begin) && std::isfinite(state_time) &&
           state_time-begin <= tolerance+1e-9;
}
}  // namespace lightning::timing
