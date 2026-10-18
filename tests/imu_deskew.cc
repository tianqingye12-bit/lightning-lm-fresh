#include "core/lio/imu_processing.hpp"
#include <iostream>
using namespace lightning;
IMUPtr sample(double t, double acc_norm) {
    auto imu = std::make_shared<IMU>();
    imu->timestamp = t;
    imu->linear_acceleration = Vec3d(0,0,acc_norm);
    imu->angular_velocity = Vec3d(0,0,1);
    return imu;
}
int check(double speed, double acc_norm) {
    ImuProcess proc;
    proc.SetUseIMUFilter(false);
    proc.SetAccCov(Vec3d::Constant(.1));
    proc.SetGyrCov(Vec3d::Constant(.1));
    ESKF filter;
    MeasureGroup init;
    init.lidar_begin_time_ = 0;
    init.lidar_end_time_ = .2;
    for (int i=0; i<=20; ++i) init.imu_.push_back(sample(i*.01, acc_norm));
    CloudPtr out(new PointCloudType);
    proc.Process(init, filter, out);
    if (!proc.IsIMUInited()) return 1;
    auto state = filter.GetX();
    state.bg_.setZero(); // Test known 1 rad/s rotation, not the initialized gyro bias.
    state.vel_ = Vec3d(speed,0,0);
    filter.ChangeX(state);
    MeasureGroup scan;
    scan.lidar_begin_time_ = .2;
    scan.lidar_end_time_ = .3;
    scan.scan_.reset(new PointCloudType);
    for (int i=1; i<=10; ++i) scan.imu_.push_back(sample(.2+i*.01, acc_norm));
    const Vec3d world(10,0,2);
    for (int i=0; i<=20; ++i) {
        double t=i*.005;
        auto p=SO3::exp(Vec3d(0,0,-t))*(world-Vec3d(speed*t,0,0));
        PointType pt;
        pt.x=p.x(); pt.y=p.y(); pt.z=p.z(); pt.time=t*1000.; pt.intensity=1;
        scan.scan_->push_back(pt);
    }
    proc.Process(scan, filter, out);
    if (std::abs(filter.GetX().timestamp_ - scan.lidar_end_time_) > 1e-10) return 3;
    const Vec3d expected=SO3::exp(Vec3d(0,0,-.1))*(world-Vec3d(speed*.1,0,0));
    double max_error=0;
    for (const auto& pt : *out) max_error=std::max(max_error,(pt.getVector3fMap().cast<double>()-expected).norm());
    std::cout << "constant rotation deskew max error: " << max_error << " m\n";
    return max_error < 1e-5 ? 0 : 2;
}

int check_gap_and_recovery() {
    ImuProcess proc;
    proc.SetUseIMUFilter(false);
    proc.SetAccCov(Vec3d::Constant(.1));
    proc.SetGyrCov(Vec3d::Constant(.1));
    ESKF filter;
    MeasureGroup init;
    init.lidar_begin_time_ = 0;
    init.lidar_end_time_ = .2;
    for (int i = 0; i <= 20; ++i) init.imu_.push_back(sample(i * .01, 9.81));
    CloudPtr out(new PointCloudType);
    proc.Process(init, filter, out);
    auto state = filter.GetX();
    state.vel_ = Vec3d(2, 0, 0);
    // Initialization estimates the known constant gyro as bias: this test isolates time/translation.
    filter.ChangeX(state);
    MeasureGroup scan;
    scan.lidar_begin_time_ = .2;
    scan.lidar_end_time_ = .5;
    scan.scan_.reset(new PointCloudType);
    PointType point;
    point.x = 10; point.y = 0; point.z = 1; point.time = 0; point.intensity = 42;
    scan.scan_->push_back(point);
    for (int i = 45; i <= 50; ++i) scan.imu_.push_back(sample(i * .01, 9.81));
    proc.Process(scan, filter, out);
    if (!out->empty() || !proc.LastScanHadImuGap() || proc.HasPropagationFault()) return 10;
    // Missing .2 -> .45 interval must advance PHYSICAL state by .25s too.
    if (std::abs(filter.GetX().timestamp_ - .5) > 1e-10 ||
        std::abs(filter.GetX().pos_.x() - .6) > 1e-8) return 11;
    if (scan.scan_->front().x != 10) return 12;  // skipped cloud not modified
    scan.lidar_begin_time_ = .5;
    scan.lidar_end_time_ = .6;
    scan.imu_.clear();
    for (int i = 51; i <= 60; ++i) scan.imu_.push_back(sample(i * .01, 9.81));
    proc.Process(scan, filter, out);
    if (out->empty() || proc.LastScanHadImuGap() || proc.HasPropagationFault() ||
        std::abs(filter.GetX().timestamp_ - .6) > 1e-10 ||
        std::abs(filter.GetX().pos_.x() - .8) > 1e-8) return 13;
    // Too large an interval must not fabricate a timestamp or retain the last scan as output.
    scan.lidar_begin_time_ = 1.9;
    scan.lidar_end_time_ = 2.;
    scan.imu_.clear();
    scan.imu_.push_back(sample(2., 9.81));
    proc.Process(scan, filter, out);
    if (!out->empty() || !proc.HasPropagationFault() ||
        std::abs(filter.GetX().timestamp_ - .6) > 1e-10) return 14;
    std::cout << "PASS: gap clock coverage, unreliable scan rejection, recovery and fault latch\n";
    return 0;
}

int main() {
    return check(0., 9.81) || check(2., 9.81) || check(0., 10.02) || check(0., 1.02) ||
           check_gap_and_recovery();
}
