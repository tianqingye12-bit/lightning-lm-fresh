#pragma once
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace lightning::navigation {
inline void CheckGridBudget(double blocks_x, double blocks_y, std::size_t block_width, std::size_t limit) {
    if (!std::isfinite(blocks_x) || !std::isfinite(blocks_y) || blocks_x <= 0 || blocks_y <= 0 ||
        blocks_x > std::numeric_limits<int>::max() || blocks_y > std::numeric_limits<int>::max())
        throw std::length_error("invalid navigation grid extent");
    const long double cells = static_cast<long double>(blocks_x)*blocks_y*block_width*block_width;
    // ToROS uses int for flattened indexing. Bound that even when the optional budget is disabled.
    if (cells > std::numeric_limits<int>::max() || (limit && cells > limit))
        throw std::length_error("navigation grid exceeds cell budget; use a coarser resolution or smaller map");
}
}  // namespace lightning::navigation
