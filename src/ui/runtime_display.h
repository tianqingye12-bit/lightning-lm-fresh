#pragma once
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>
#include <utility>

namespace lightning::ui {
struct RuntimeDisplayOptions {
    double render_hz = 20, scan_hz = 5;
    double stale_wall_s = 1.0;
    size_t trajectory_max_points = 50000;
    size_t scan_max_points = 20000, history_max_points = 200000, history_scans = 20;
    bool Valid() const {
        return std::isfinite(render_hz) && render_hz >= 1 && render_hz <= 120 &&
               std::isfinite(scan_hz) && scan_hz > 0 && scan_hz <= render_hz &&
               std::isfinite(stale_wall_s) && stale_wall_s > 0 && trajectory_max_points >= 2 &&
               scan_max_points > 0 && history_max_points >= scan_max_points && history_scans > 0;
    }
};
// Segmentation is display-only: never connect samples across an invalid prediction interval.
class TrajectorySegments {
 public:
    void Break() { pending_ = true; }
    void Append(size_t index) {
        if (starts_.empty() || (pending_ && starts_.back() != index)) starts_.push_back(index);
        pending_ = false;
    }
    void Trim(size_t removed) {
        std::vector<size_t> next{0};
        for (auto index : starts_) if (index > removed) next.push_back(index - removed);
        starts_ = std::move(next);
    }
    void Clear() { starts_.clear(); pending_ = false; }
    const std::vector<size_t>& Starts() const { return starts_; }
 private:
    bool pending_ = false;
    std::vector<size_t> starts_;
};
}  // namespace lightning::ui
