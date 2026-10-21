#ifndef LIGHTNING_MAP_DETAIL_H
#define LIGHTNING_MAP_DETAIL_H

#include <cmath>
#include <cstddef>

namespace lightning {
namespace mapping {

// All defaults preserve the old dataset configuration's behavior.
struct DetailOptions {
    bool preserve_observed_points = false;  // Opt-in: representative raw points instead of centroids.
    int input_point_filter_num = 0;  // 0 inherits fasterlio.point_filter_num
    double keyframe_resolution = 0.0;  // 0 retains the preprocessed, deskewed cloud
    std::size_t keyframe_max_points = 0;  // 0 disables the additional cap
    double export_resolution = 0.1;
    double registration_min_ratio = 0.1;  // 0 disables density-based refinement

    bool local_fill_enabled = false;
    double local_fill_min_interval = 0.2;  // seconds, AND motion condition
    double local_fill_translation = 0.2;  // metres, OR rotation
    double local_fill_rotation_deg = 3.0;
    int local_fill_min_effective = 20;

    bool Valid() const {
        return input_point_filter_num >= 0 && std::isfinite(keyframe_resolution) && keyframe_resolution >= 0 &&
               (keyframe_resolution == 0 || (std::isfinite(static_cast<float>(keyframe_resolution)) &&
                                             static_cast<float>(keyframe_resolution) > 0)) &&
               std::isfinite(static_cast<float>(export_resolution)) && static_cast<float>(export_resolution) > 0 &&
               std::isfinite(registration_min_ratio) && registration_min_ratio >= 0 && registration_min_ratio <= 1 &&
               std::isfinite(local_fill_min_interval) && local_fill_min_interval > 0 &&
               std::isfinite(local_fill_translation) && local_fill_translation > 0 &&
               std::isfinite(local_fill_rotation_deg) && local_fill_rotation_deg > 0 &&
               local_fill_rotation_deg <= 180 && local_fill_min_effective > 0;
    }

    bool ShouldFill(bool accepted, int effective, double elapsed, double translation, double rotation_deg) const {
        return local_fill_enabled && accepted && effective >= local_fill_min_effective &&
               std::isfinite(elapsed) && std::isfinite(translation) && std::isfinite(rotation_deg) &&
               elapsed >= local_fill_min_interval && translation >= 0 && rotation_deg >= 0 &&
               (translation >= local_fill_translation || rotation_deg >= local_fill_rotation_deg);
    }
};

// Spread a point budget over the entire cloud, rather than truncating one scan sector.
inline std::size_t BudgetIndex(std::size_t index, std::size_t input_size, std::size_t output_size) {
    return (input_size / output_size) * index + ((input_size % output_size) * index) / output_size;
}

}  // namespace mapping
}  // namespace lightning
#endif
