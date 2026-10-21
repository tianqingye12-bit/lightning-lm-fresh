#include "core/maps/map_transaction.h"
#include "core/localization/localization_health.h"
#include <atomic>
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>

namespace fs = std::filesystem;
using lightning::loc::LocalizationHealth;
using lightning::maps::MapTransaction;

static void Write(const fs::path& p, const std::string& text) {
    std::ofstream out(p);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out << text;
    out.flush();
    out.close();
}
static std::string Read(const fs::path& p) {
    std::ifstream in(p);
    return {std::istreambuf_iterator<char>(in), {}};
}
template<class F> static void Fails(F action) {
    bool failed = false;
    try { action(); } catch (const std::exception&) { failed = true; }
    assert(failed);
}
static void SaveTests(const fs::path& root) {
    using lightning::maps::ValidMapId;
    assert(ValidMapId("street_06") && ValidMapId("街道_06"));
    for (auto id : {"", ".", "..", "../old", "a/b", "a\\b", ".hidden", "a\nb"}) assert(!ValidMapId(id));
    const auto target = root / "street";
    { MapTransaction t(target); Write(t.Staging()/"cloud.pcd", "old"); assert(t.Commit()); }
    fs::path staging;
    // Fault injection: partial new map + ENOSPC-style writer failure, no commit.
    Fails([&] {
        MapTransaction t(target); staging = t.Staging();
        Write(staging/"cloud.pcd", "partial");
        throw std::runtime_error("injected ENOSPC");
    });
    assert(Read(target/"cloud.pcd") == "old" && !fs::exists(staging));
    // Real stream failure and failed fsync validation cannot replace the old map.
    Fails([&] { MapTransaction t(target); Write("/dev/full", "disk full"); t.Commit(); });
    Fails([&] { MapTransaction t(target); fs::create_symlink("/dev/null", t.Staging()/"bad"); t.Commit(); });
    assert(Read(target/"cloud.pcd") == "old");
    Fails([&] { MapTransaction t(target, std::numeric_limits<std::uintmax_t>::max()); });
    { MapTransaction writer(target); Fails([&] { MapTransaction concurrent(target); }); }
    const auto alias = root / "alias";
    fs::create_directory_symlink(target, alias);
    Fails([&] { MapTransaction t(alias); });
    assert(Read(target/"cloud.pcd") == "old");
    fs::path backup;
    { MapTransaction t(target.string()+"/"); Write(t.Staging()/"cloud.pcd", "new");
      assert(t.Commit()); backup = t.Previous(); Fails([&] { t.Commit(); }); }
    assert(Read(target/"cloud.pcd") == "new" && Read(backup/"cloud.pcd") == "old");
    // A reader reopening the canonical path always sees a complete version.
    std::atomic_bool stop{false}, invalid{false};
    std::thread reader([&] {
        while (!stop.load()) {
            const auto value = Read(target/"cloud.pcd");
            if (value != "new" && value != "next") invalid.store(true);
        }
    });
    for (int i = 0; i < 8; ++i) {
        MapTransaction t(target); Write(t.Staging()/"cloud.pcd", i % 2 ? "new" : "next");
        assert(t.Commit());
    }
    stop.store(true); reader.join(); assert(!invalid.load());
}
static void Feed(LocalizationHealth& h, double stamp, double wall, bool match = true) {
    assert(h.Input(true, stamp, wall));
    assert(h.Input(false, stamp, wall));
    if (match) h.Match(stamp, true, wall);
    h.Result(stamp, true, wall);
}
static void HealthTests() {
    LocalizationHealth h;
    assert(h.Get(0).state == "INITIALIZING");
    // Bag epoch is deliberately unrelated to monotonic wall time.
    Feed(h, 1700000000, 0);
    Feed(h, 1700000000.1, .1);
    assert(!h.Get(.1).valid);
    Feed(h, 1700000000.2, .2);
    assert(h.Get(.2).valid);
    assert(!h.Input(true, 1700000000.2, .3)); // duplicate
    // IMU continues while LiDAR drops out. DR output alone cannot keep localization valid.
    for (int i = 3; i <= 24; ++i) {
        h.Input(true, 1700000000 + i*.1, i*.1);
        h.Result(1700000000 + i*.1, true, i*.1);
    }
    assert(h.Get(2.4).reason == "lidar_timeout");
    Feed(h, 1700000002.5, 2.5);
    Feed(h, 1700000002.6, 2.6);
    assert(!h.Get(2.6).valid);
    Feed(h, 1700000002.7, 2.7);
    assert(h.Get(2.7).valid);
    h.Match(1700000002.7, true, 2.71); // cached map match must not count
    assert(h.Get(2.71).state == "DEGRADED");
    Feed(h, 1700000002.8, 2.8);
    Feed(h, 1700000002.9, 2.9);
    Feed(h, 1700000003.0, 3.0);
    assert(h.Get(3.0).valid);
    h.Result(1700000003.0, false, 3.01);
    assert(!h.Get(3.01).valid);
    h.Result(1700000003.0, true, 3.02);
    assert(h.Get(3.02).valid);
    assert(!h.Input(true, 1700000002.0, 3.1));
    assert(h.Get(3.1).reason == "timestamp_regression_restart_required");
    assert(!h.Input(true, 1700000004.0, 4.0));
    LocalizationHealth stale;
    for (int i = 0; i < 3; ++i) Feed(stale, 10+i*.1, i*.1);
    assert(stale.Get(.2).valid);
    for (int i = 3; i < 40; ++i) Feed(stale, 10+i*.1, i*.1, false);
    assert(stale.Get(3.9).reason == "map_match_stale");
    LocalizationHealth result;
    for (int i = 0; i < 3; ++i) Feed(result, 10+i*.1, i*.1);
    for (int i = 3; i <= 20; ++i) {
        result.Input(true, 10+i*.1, i*.1); result.Input(false, 10+i*.1, i*.1);
        result.Match(10+i*.1, true, i*.1);
    }
    assert(result.Get(2).reason == "result_stale");
    LocalizationHealth nan;
    assert(!nan.Input(true, std::numeric_limits<double>::quiet_NaN(), 0));
    assert(!nan.Get(0).valid);
    LocalizationHealth::Limits limits;
    limits.imu_timeout_s = 0;
    Fails([&] { nan.Configure(limits); });
}
int main(int argc, char** argv) {
    assert(argc == 2);
    SaveTests(argv[1]); HealthTests();
    std::cout << "transaction failures, atomic replacement, backups and localization outage/recovery passed\n";
}
