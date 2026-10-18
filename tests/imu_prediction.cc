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
    options.max_step_s = 0;
    assert(!options.Valid());
    options.max_step_s = .5;
    assert(!options.Valid());
    std::cout << "bounded IMU prediction timing tests passed\n";
}
