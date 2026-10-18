#include "core/lio/map_detail.h"
#include <cassert>
#include <limits>
#include <iostream>

int main() {
    using lightning::mapping::DetailOptions;
    using lightning::mapping::BudgetIndex;
    DetailOptions options;
    assert(options.Valid());
    assert(!options.ShouldFill(true, 100, 1, 1, 10)); // legacy mode
    options.local_fill_enabled = true;
    assert(!options.ShouldFill(false, 100, 1, 1, 10)); // registration rejected
    assert(!options.ShouldFill(true, 19, 1, 1, 10)); // weak constraints
    assert(!options.ShouldFill(true, 100, 0.19, 1, 10)); // rate limit
    assert(!options.ShouldFill(true, 100, 10, 0, 0)); // stationary
    assert(!options.ShouldFill(true, 100, -1, 1, 10)); // timestamp reversal
    assert(options.ShouldFill(true, 20, 0.2, 0.2, 0)); // translation alone
    assert(options.ShouldFill(true, 20, 0.2, 0, 3)); // rotation alone
    double nan = std::numeric_limits<double>::quiet_NaN();
    assert(!options.ShouldFill(true, 100, nan, 1, 10));
    assert(!options.ShouldFill(true, 100, 1, nan, 10));
    assert(!options.ShouldFill(true, 100, 1, 1, nan));
    for (double bad : {nan, -0.1, 0.0, 1e300, 1e-300}) {
        auto invalid = options;
        invalid.export_resolution = bad;
        assert(!invalid.Valid());
        if (!std::isfinite(bad) || bad <= 0) {
            invalid = options;
            invalid.local_fill_min_interval = bad;
            assert(!invalid.Valid());
        }
    }
    auto invalid = options;
    invalid.keyframe_resolution = -1;
    assert(!invalid.Valid());
    invalid = options;
    invalid.input_point_filter_num = -1;
    assert(!invalid.Valid());
    invalid = options;
    invalid.registration_min_ratio = 1.1;
    assert(!invalid.Valid());
    // A budget must cover all scan sectors, have no duplicate indices, and stay in bounds.
    for (std::size_t size : {1u, 7u, 100u, 65537u}) {
        for (std::size_t budget = 1; budget <= size; budget = budget * 2 + 1) {
            std::size_t previous = 0;
            for (std::size_t i = 0; i < budget; ++i) {
                auto index = BudgetIndex(i, size, budget);
                assert(index < size);
                assert(i == 0 || index > previous);
                previous = index;
            }
            assert(size - previous <= (size + budget - 1) / budget);
        }
    }
    std::cout << "map detail policy and point budget tests passed\n";
}
