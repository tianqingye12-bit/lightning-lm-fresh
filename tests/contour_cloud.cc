#include <pcl/point_types.h>
#include "core/lio/map_detail_cloud.h"
#include <cassert>
#include <iostream>

int main() {
    using P = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<P>;
    Cloud::Ptr input(new Cloud());
    for (int i = 0; i < 100; ++i) {
        P p; p.x = (i%2)*.06f; p.y = ((i+1)%2)*.06f; p.z = .05f; p.intensity = i;
        input->push_back(p);
    }
    lightning::mapping::DetailOptions options;
    options.keyframe_resolution = .1;
    auto centroid = lightning::mapping::RetainCloud<P>(input,options);
    options.preserve_observed_points = true;
    auto observed = lightning::mapping::RetainCloud<P>(input,options);
    assert(input->size() == 100 && observed != input && observed->size() == 1);
    assert(centroid->front().x > 0 && centroid->front().y > 0);
    assert(observed->front().x == 0 || observed->front().y == 0);
    bool original = false;
    for (const auto& p : input->points)
        original |= p.x == observed->front().x && p.y == observed->front().y && p.intensity == observed->front().intensity;
    assert(original);
    Cloud::Ptr empty(new Cloud());
    assert(lightning::mapping::ObservedVoxelCloud<P>(empty,.1)->empty());
    std::cout << "PCL contour adapters preserve observed surfaces and input clouds\n";
}
