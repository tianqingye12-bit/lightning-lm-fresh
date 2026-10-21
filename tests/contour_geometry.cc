#include "core/lio/observed_voxel.h"
#include "core/lio/surface_support.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <tuple>

struct Point { double x, y, z; int intensity; double time; };
using lightning::mapping::ObservedVoxelSampler;
using lightning::mapping::NeedsObservedVoxelPoint;

static auto Geometry(const std::vector<Point>& points) {
    std::vector<std::tuple<double,double,double>> result;
    for (const auto& p : points) result.emplace_back(p.x,p.y,p.z);
    std::sort(result.begin(), result.end());
    return result;
}
static void Sampling() {
    // Two physical walls in one voxel. Their centroid lies on neither wall.
    const std::vector<Point> corner{{0,.06,.05,11,.01},{.06,0,.05,22,.02}};
    ObservedVoxelSampler<Point> sample(.1);
    for (const auto& p : corner) sample.Add(p);
    assert(sample.Points().size() == 1);
    const auto chosen = sample.Points().front();
    assert(chosen.x == 0 || chosen.y == 0);
    assert(chosen.intensity == 11 && chosen.time == .01); // tie rule + complete record
    const double centroid_x = (corner[0].x+corner[1].x)/2;
    const double centroid_y = (corner[0].y+corner[1].y)/2;
    const auto wall_error = [](double x, double y) { return std::min(std::abs(x),std::abs(y)); };
    assert(wall_error(centroid_x,centroid_y) > .02);
    assert(wall_error(chosen.x,chosen.y) == 0);
    // Distinct negative cells and exact boundary points must not be merged by truncation.
    ObservedVoxelSampler<Point> boundary(.1);
    for (double x : {-.101,-.099,-.001,0.,.099,.1}) boundary.Add({x,0,0,1,0});
    assert(boundary.Points().size() == 4);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    assert(!boundary.Add({nan,0,0,1,0}));
    assert(!boundary.Add({std::numeric_limits<double>::infinity(),0,0,1,0}));
    assert(!boundary.Add({1e100,0,0,1,0}));
    assert(boundary.Invalid() == 3);
    bool rejected = false;
    try { ObservedVoxelSampler<Point> bad(0); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    // Same selected geometry regardless of scan block boundaries or point order.
    std::vector<Point> input;
    for (int i = 0; i < 20000; ++i)
        input.push_back({(i%100)*.013, ((i/100)%20)*.013, (i%3)*.01, i, i*.001});
    ObservedVoxelSampler<Point> reference(.05), streamed(.05), shuffled(.05);
    for (const auto& p : input) reference.Add(p);
    for (size_t begin = 0; begin < input.size(); begin += 37)
        for (size_t i = begin; i < std::min(input.size(), begin+37); ++i) streamed.Add(input[i]);
    std::shuffle(input.begin(), input.end(), std::mt19937(42));
    for (const auto& p : input) shuffled.Add(p);
    assert(Geometry(reference.Points()) == Geometry(streamed.Points()));
    assert(Geometry(reference.Points()) == Geometry(shuffled.Points()));
    assert(reference.Points().size() < input.size()/10);
    for (const auto& p : reference.Points()) {
        const auto source = std::find_if(input.begin(),input.end(),[&](const Point& q) { return q.intensity == p.intensity; });
        assert(source != input.end());
        assert(source->x == p.x && source->y == p.y && source->z == p.z && source->time == p.time);
    }
    // An adjacent voxel's center-near point must not erase a contour in an unoccupied cell.
    const Point edge{.99,.99,.5,0,0};
    assert(NeedsObservedVoxelPoint(edge, std::vector<Point>{{1.01,.5,.5,0,0}}, 1.));
    assert(!NeedsObservedVoxelPoint(edge, std::vector<Point>{{.5,.5,.5,0,0}}, 1.));
    assert(!NeedsObservedVoxelPoint(edge, std::vector<Point>{edge}, 1.));
}
static void SurfaceSupport() {
    using lightning::features::HasSurfaceSupport;
    lightning::features::SurfaceSupportOptions options;
    options.enabled = true;
    const std::vector<Point> patch{{-.2,-.2,0,0,0},{.2,-.2,0,0,0},{.2,.2,0,0,0},
                                   {-.2,.2,0,0,0},{0,0,0,0,0}};
    const std::array<double,3> normal{0,0,1};
    assert(HasSurfaceSupport({.1,.1,.05},normal,patch,options));
    assert(!HasSurfaceSupport({.1,.1,.2},normal,patch,options));
    assert(!HasSurfaceSupport({.45,0,0},normal,patch,options)); // on infinite plane, outside patch
    assert(!HasSurfaceSupport({4,0,0},normal,patch,options));
    assert(!HasSurfaceSupport({0,0,0},{0,0,0},patch,options));
    auto bad = patch;
    bad.front().x = std::numeric_limits<double>::quiet_NaN();
    assert(!HasSurfaceSupport({0,0,0},normal,bad,options));
    // A rotated plane in a distant world frame must make the same acceptance decision.
    const auto rotate = [](const std::array<double,3>& p) {
        const double c = std::cos(.7), s = std::sin(.7);
        return std::array<double,3>{c*p[0]+s*p[2],p[1],-s*p[0]+c*p[2]};
    };
    const auto transform = [&](const std::array<double,3>& p) {
        auto q = rotate(p); q[0] += 100000; q[1] -= 200000; q[2] += 30000; return q;
    };
    std::vector<Point> moved;
    for (const auto& p : patch) { const auto q=transform({p.x,p.y,p.z}); moved.push_back({q[0],q[1],q[2],0,0}); }
    assert(HasSurfaceSupport(transform({.1,.1,.05}),rotate(normal),moved,options));
    assert(!HasSurfaceSupport(transform({.45,0,0}),rotate(normal),moved,options));
    options.enabled = false;
    assert(HasSurfaceSupport({4,0,0},normal,patch,options)); // legacy path remains opt-out
}
int main() {
    Sampling(); SurfaceSupport();
    std::cout << "observed-point geometry, streaming, metadata, voxel boundaries and finite-patch matching passed\n";
}
