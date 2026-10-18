#include "utils/serial_worker.h"
#include "utils/runtime_stats.h"
#include "ui/runtime_display.h"
#include <atomic>
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <vector>

int main() {
    using W = lightning::SerialWorker;
    W worker({2, 1, 8});
    worker.Start();
    std::promise<void> entered, release;
    auto gate = release.get_future().share();
    std::vector<int> order;
    std::atomic<int> active{0};
    assert(worker.Post(W::control, [&](double) {
        entered.set_value(); gate.wait(); order.push_back(0);
    }));
    assert(entered.get_future().wait_for(std::chrono::seconds(2)) == std::future_status::ready);
    auto task = [&](int id) {
        return [&, id](double wait) {
            assert(wait >= 0 && active.fetch_add(1) == 0);
            order.push_back(id); --active;
        };
    };
    assert(worker.Post(W::imu, task(1)));
    assert(worker.Post(W::lidar, task(2)));
    assert(worker.Post(W::imu, task(3)));
    assert(!worker.Post(W::imu, task(99)));
    assert(!worker.Post(W::lidar, task(99)));
    auto queued = worker.State();
    assert(queued.queued[W::imu] == 2 && queued.queued[W::lidar] == 1);
    assert(queued.rejected[W::imu] == 1 && queued.rejected[W::lidar] == 1);
    release.set_value();
    worker.Call([&] { worker.Call([&] { order.push_back(4); }); });
    assert((order == std::vector<int>{0,1,2,3,4}));
    bool caught = false;
    try { worker.Call([] { throw std::runtime_error("save failure"); }); }
    catch (const std::runtime_error&) { caught = true; }
    assert(caught);
    worker.Call([&] { order.push_back(5); });
    // Shutdown must drain work already admitted, rather than lose pending IMU/control futures.
    assert(worker.Post(W::control, task(6)));
    worker.Stop();
    assert(order.back() == 6 && worker.State().queued[W::control] == 0);
    assert(!worker.Post(W::imu, task(99)));
    try { worker.Call([] {}); assert(false); } catch (const std::runtime_error&) {}
    worker.Stop();

    lightning::RuntimeStats stats;
    for (int i=1; i<=100; ++i) stats.Observe(lightning::RuntimeStats::registration, i);
    const auto summary = stats.Summary();
    assert(summary.find("registration_ms(mean/p95/max)=50.50/95.00/100.00") != std::string::npos);
    std::thread first([&] { for (int i=0; i<1000; ++i) stats.Observe(lightning::RuntimeStats::imu_wait, 1); });
    std::thread second([&] { for (int i=0; i<1000; ++i) stats.Observe(lightning::RuntimeStats::imu_wait, 3); });
    first.join(); second.join();
    assert(stats.Summary().find("imu_wait_ms(mean/p95/max)=2.00/") != std::string::npos);
    assert(lightning::Measure(stats, lightning::RuntimeStats::preprocess, [] { return 42; }) == 42);

    lightning::ui::TrajectorySegments segments;
    segments.Append(0); segments.Append(1);
    segments.Break(); segments.Break(); segments.Append(2); segments.Append(3);
    assert((segments.Starts() == std::vector<size_t>{0,2}));
    segments.Trim(1);
    assert((segments.Starts() == std::vector<size_t>{0,1}));
    segments.Trim(1);
    assert((segments.Starts() == std::vector<size_t>{0}));
    segments.Clear(); segments.Break(); segments.Append(0);
    assert((segments.Starts() == std::vector<size_t>{0}));
    lightning::ui::RuntimeDisplayOptions display;
    assert(display.Valid());
    display.history_max_points = display.scan_max_points - 1;
    assert(!display.Valid());
    display.history_max_points = 200000; display.scan_hz = 100;
    assert(!display.Valid());
    std::cout << "runtime FIFO, bounded admission, save serialization, drain, timing and trajectory gap tests passed\n";
}
