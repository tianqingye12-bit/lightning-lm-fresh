#include "core/system/async_message_process.h"
#include <atomic>
#include <cassert>
#include <chrono>
#include <future>
#include <iostream>

int main() {
    using lightning::sys::AsyncMessageProcess;
    using namespace std::chrono_literals;
    { AsyncMessageProcess<int> never_started; never_started.Quit(); }
    AsyncMessageProcess<int> worker;
    bool rejected = false;
    try { worker.SetSkipParam(true, 0); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    worker.SetMaxSize(1);
    std::promise<void> entered, release;
    auto ready = entered.get_future();
    auto gate = release.get_future().share();
    std::atomic_int processed{0};
    worker.SetProcFunc([&](int value) {
        assert(value == 1);
        ++processed; entered.set_value(); gate.wait();
    });
    worker.Start(); worker.AddMessage(1);
    assert(ready.wait_for(2s) == std::future_status::ready);
    for (int i = 2; i < 100; ++i) worker.AddMessage(i);
    assert(worker.Dropped() == 97);
    auto quit = std::async(std::launch::async, [&] { worker.Quit(); });
    assert(quit.wait_for(20ms) == std::future_status::timeout); // must join active callback
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (worker.Running() && std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
    assert(!worker.Running());
    release.set_value();
    assert(quit.wait_for(2s) == std::future_status::ready);
    worker.AddMessage(100);
    worker.Quit();
    assert(processed.load() == 1); // pending work is discarded before resource destruction
    std::cout << "bounded async queue and orderly shutdown passed\n";
}
