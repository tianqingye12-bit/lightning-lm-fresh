#ifndef LIGHTNING_LOOP_DISPLAY_OPTIONS_H
#define LIGHTNING_LOOP_DISPLAY_OPTIONS_H

#include <cmath>
#include <cstddef>

namespace lightning::ui {
struct LoopDisplayOptions {
    bool enabled = true;
    double min_interval = 2.0;  // steady-clock seconds between background rebuilds
    std::size_t max_points = 200000;

    bool Valid() const {
        return std::isfinite(min_interval) && min_interval > 0 && min_interval <= 3600 && max_points > 0;
    }
};
}  // namespace lightning::ui
#endif
