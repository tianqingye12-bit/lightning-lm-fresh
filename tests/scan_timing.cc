#include "core/lio/scan_timing.h"
#include <cassert>
#include <iostream>
#include <vector>

struct Point { double time; };
using namespace lightning::timing;

int main() {
    Options options; assert(options.Valid());
    const double header=1628081061.36510754;
    // Captured M2DGR statistics: last point is near the header, not scan duration.
    for (auto points : {std::vector<Point>{{-99.49707794189454},{.6428160071373},{-20.}},
                        std::vector<Point>{{.6428160071373},{-20.},{-99.49707794189454}},
                        std::vector<Point>{{0.},{100.},{40.}},
                        std::vector<Point>{{-100.},{-20.},{0.}}}) {
        const auto original=points;
        const auto result=NormalizeHeaderOffsets(points,header,options);
        assert(result.valid && result.end>result.begin);
        double low=points.front().time,high=low;
        for (size_t i=0;i<points.size();++i) {
            assert(std::abs(header+original[i].time/1000.-(result.begin+points[i].time/1000.))<1e-6);
            low=std::min(low,points[i].time);high=std::max(high,points[i].time);
        }
        assert(low==0 && std::abs(result.end-result.begin-high/1000.)<1e-6);
    }
    for (auto points : {std::vector<Point>{},std::vector<Point>{{0.}},
                        std::vector<Point>{{0.},{.642816}},std::vector<Point>{{0.},{200.}},
                        std::vector<Point>{{1000.},{1100.}},
                        std::vector<Point>{{0.},{std::numeric_limits<double>::quiet_NaN()}}}) {
        const auto original=points;
        assert(!NormalizeHeaderOffsets(points,header,options).valid);
        for (size_t i=0;i<points.size();++i)
            assert(points[i].time==original[i].time || (std::isnan(points[i].time)&&std::isnan(original[i].time)));
    }
    auto invalid=options;invalid.min_duration_s=0;assert(!invalid.Valid());
    invalid=options;invalid.max_header_offset_s=.01;assert(!invalid.Valid());
    assert(CanBackcast(10.,10.0006));
    assert(!CanBackcast(10.,10.003));
    assert(!CanBackcast(10.,std::numeric_limits<double>::quiet_NaN()));
    std::cout<<"scan_timing: signed/header offsets, arbitrary order, timestamp invariance, malformed timing and bounded overlap passed\n";
}
