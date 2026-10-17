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
    const Vec3d expected=SO3::exp(Vec3d(0,0,-.1))*(world-Vec3d(speed*.1,0,0));
    double max_error=0;
    for (const auto& pt : *out) max_error=std::max(max_error,(pt.getVector3fMap().cast<double>()-expected).norm());
    std::cout << "constant rotation deskew max error: " << max_error << " m\n";
    return max_error < 1e-5 ? 0 : 2;
}

int main() { return check(0., 9.81) || check(2., 9.81) || check(0., 10.02) || check(0., 1.02); }
