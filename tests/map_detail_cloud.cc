#include <pcl/point_types.h>
#include "core/lio/map_detail_cloud.h"
#include <cassert>
#include <iostream>

int main() {
    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;
    Cloud::Ptr input(new Cloud());
    for (int i = 0; i < 100; ++i) {
        Point p;
        p.x = 0.001f + i * 0.01f;
        p.y = 0.01f;
        p.z = 0.01f;
        p.intensity = 42;
        input->push_back(p);
    }
    lightning::mapping::DetailOptions options;
    auto legacy = lightning::mapping::RetainCloud<Point>(input, options);
    assert(legacy == input && input->size() == 100);
    options.keyframe_resolution = 0.1;
    auto dense = lightning::mapping::RetainCloud<Point>(input, options);
    assert(dense->size() == 10 && input->size() == 100);
    options.keyframe_resolution = 0.5;
    auto coarse = lightning::mapping::RetainCloud<Point>(input, options);
    assert(coarse->size() == 2);
    assert(dense->size() > coarse->size());
    options.keyframe_resolution = 0;
    options.keyframe_max_points = 10;
    auto limited = lightning::mapping::RetainCloud<Point>(input, options);
    assert(limited->size() == 10 && input->size() == 100);
    assert(limited->front().x < 0.1 && limited->back().x > 0.8);
    assert(limited->front().intensity == 42);
    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
    extrinsic.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    extrinsic.block<3, 1>(0, 3) = Eigen::Vector3d(1, -2, 0.5);
    const auto original = input->front();
    auto body = lightning::mapping::CloudInBodyFrame<Point>(input, extrinsic);
    assert(body != input && body->size() == input->size());
    Eigen::Matrix4d world = Eigen::Matrix4d::Identity();
    world.block<3, 3>(0, 0) = Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
    world.block<3, 1>(0, 3) = Eigen::Vector3d(10, 20, 30);
    Eigen::Vector4d lidar_point(original.x, original.y, original.z, 1);
    Eigen::Vector4d body_point(body->front().x, body->front().y, body->front().z, 1);
    assert((world * body_point - world * extrinsic * lidar_point).norm() < 1e-5);
    assert(input->front().x == original.x && body->front().intensity == original.intensity);
    auto identity = lightning::mapping::CloudInBodyFrame<Point>(input, Eigen::Matrix4d::Identity());
    assert(identity != input && identity->front().x == original.x);
    Cloud::Ptr empty(new Cloud());
    assert(lightning::mapping::RetainCloud<Point>(empty, options)->empty());
    std::cout << "map detail PCL retention tests passed\n";
}
