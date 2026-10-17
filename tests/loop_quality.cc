#include "core/loop_closing/loop_quality.h"
#include "ui/loop_display_options.h"
#include "utils/latest_task.h"
#include <cassert>
#include <chrono>
#include <cstring>
#include <future>
#include <iostream>
#include <limits>

int main() {
    using namespace lightning::loop_quality;
    Options options;
    assert(options.Valid());
    MatchMetrics good;
    good.converged = good.finite_transform = true;
    good.probability = 2;
    good.samples = 200;
    good.inliers = 100;
    good.inlier_rmse = 0.1;
    good.correction_translation = 0.2;
    good.correction_rotation_deg = 2;
    assert(options.Rejection(good, 1.3) == nullptr);
    auto bad = good;
    bad.converged = false;
    assert(std::strcmp(options.Rejection(bad, 1.3), "not_converged") == 0);
    bad = good; bad.finite_transform = false;
    assert(options.Rejection(bad, 1.3));
    bad = good; bad.probability = 1.3;
    assert(options.Rejection(bad, 1.3));
    bad = good; bad.inliers = 49;
    assert(options.Rejection(bad, 1.3));
    bad = good; bad.inliers = 59; // enough absolute points, insufficient overlap
    assert(std::strcmp(options.Rejection(bad, 1.3), "low_overlap") == 0);
    bad = good; bad.samples = 0;
    assert(options.Rejection(bad, 1.3));
    bad = good; bad.inliers = 201;
    assert(options.Rejection(bad, 1.3));
    bad = good; bad.inlier_rmse = 0.31;
    assert(options.Rejection(bad, 1.3));
    bad = good; bad.correction_translation = 5.1;
    assert(options.Rejection(bad, 1.3));
    bad = good; bad.correction_rotation_deg = 30.1;
    assert(options.Rejection(bad, 1.3));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (int field = 0; field < 4; ++field) {
        bad = good;
        if (field == 0) bad.probability = nan;
        if (field == 1) bad.inlier_rmse = nan;
        if (field == 2) bad.correction_translation = nan;
        if (field == 3) bad.correction_rotation_deg = nan;
        assert(options.Rejection(bad, 1.3));
    }
    assert(!options.RejectEdge(25));
    assert(options.RejectEdge(25.1) && options.RejectEdge(nan) && options.RejectEdge(-1));
    auto invalid = options;
    invalid.min_overlap = 1.1; assert(!invalid.Valid());
    invalid = options; invalid.min_inliers = invalid.max_samples + 1; assert(!invalid.Valid());
    invalid = options; invalid.max_loop_chi2 = nan; assert(!invalid.Valid());
    lightning::ui::LoopDisplayOptions display;
    assert(display.Valid());
    display.max_points = 0; assert(!display.Valid());
    display.max_points = 1; display.min_interval = nan; assert(!display.Valid());

    using Queue = lightning::LatestTask<int>;
    using namespace std::chrono_literals;
    Queue queue;
    queue.Put(1); queue.Put(2); queue.Put(3);
    int value = 0;
    auto earliest = Queue::Clock::now() + 50ms;
    assert(queue.Wait(value, earliest) && value == 3);
    assert(Queue::Clock::now() >= earliest);
    queue.Put(4);
    assert(queue.Wait(value, Queue::Clock::now()) && value == 4);
    // Shutdown interrupts both empty and rate-limited waits without processing stale work.
    auto blocked = std::async(std::launch::async, [&] { int v; return queue.Wait(v, Queue::Clock::now() + 1h); });
    queue.Stop();
    assert(blocked.wait_for(1s) == std::future_status::ready && !blocked.get());
    assert(!queue.Put(5));
    Queue delayed;
    delayed.Put(1);
    auto waiting = std::async(std::launch::async, [&] { int v; return delayed.Wait(v, Queue::Clock::now() + 1h); });
    delayed.Stop();
    assert(waiting.wait_for(1s) == std::future_status::ready && !waiting.get());
    std::cout << "loop match gates, chi2 gates, display settings and latest-task shutdown tests passed\n";
}
