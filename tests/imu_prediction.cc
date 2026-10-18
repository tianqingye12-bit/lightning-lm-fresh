#include "core/lio/imu_prediction.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    namespace p = lightning::prediction;
    p::Options options;
    assert(options.Valid());
    // Hundreds of small steps must still stop at the accepted LiDAR horizon.
    const double correction = 1000.0;
    double current = correction;
    int integrated = 0;
    for (int i = 1; i <= 200; ++i) {
        const double target = correction + .005 * i;
        const auto decision = p::CheckTime(current, target, correction, options);
        if (decision == p::Decision::predict) { current = target; ++integrated; }
        else assert(decision == p::Decision::stale_lio);
    }
    assert(integrated >= 59 && integrated <= 60 && current - correction <= .300001);
    // A large gap must never be integrated in one step; a later LiDAR rebase can recover.
    assert(p::CheckTime(1000, 1000.2, 1000, options) == p::Decision::step_gap);
    assert(p::CheckTime(1000.2, 1000.205, 1000.2, options) == p::Decision::predict);
    assert(p::CheckTime(1000.2, 1001, 1000.2, options) == p::Decision::stale_lio);
    assert(p::CheckTime(1000.1, 1000.1, 1000, options) == p::Decision::already_covered);
    assert(p::CheckTime(1000.1, 999, 1000, options) == p::Decision::already_covered);
    // Rejected laser updates must NOT renew the correction time.
    assert(p::CheckTime(1000.4, 1000.405, 1000, options) == p::Decision::stale_lio);
    assert(p::CheckTime(1000, std::numeric_limits<double>::quiet_NaN(), 1000, options)
           == p::Decision::invalid_time);
    assert(p::CheckTime(0, 1000, 0, options) == p::Decision::invalid_time);
    assert(!p::CanPublish(10, 10) && !p::CanPublish(9, 10) && p::CanPublish(11, 10));
    assert(!p::CanPublish(std::numeric_limits<double>::infinity(), 10));
    // Main-filter clock plans must cover the WHOLE missing interval, not just relabel time.
    const auto gap = p::PlanPropagation(1000.2, 1000.45);
    assert(gap.valid && gap.gap && gap.steps == 13);
    assert(std::abs(gap.duration - .25) < 1e-10);
    assert(gap.duration / gap.steps <= .02);
    const auto normal = p::PlanPropagation(1000.45, 1000.455);
    assert(normal.valid && !normal.gap && normal.steps == 1);
    assert(!p::PlanPropagation(1000.5, 1000.4).valid);  // no absolute-value backward propagation
    assert(!p::PlanPropagation(1000.5, 1002).valid);    // bounded recovery work
    assert(!p::PlanPropagation(1000.5, std::numeric_limits<double>::infinity()).valid);
    const auto same = p::PlanPropagation(1000.5, 1000.5);
    assert(same.valid && same.steps == 0 && same.duration == 0);
    options.max_step_s = 0;
    assert(!options.Valid());
    options.max_step_s = .5;
    assert(!options.Valid());
    std::cout << "bounded IMU prediction timing tests passed\n";
}
