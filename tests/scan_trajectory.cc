#include "core/lio/scan_trajectory.h"
#include <cassert>
#include <iostream>
#include <limits>

using namespace lightning::trajectory;
Eigen::Isometry3d Pose(double x, double y = 0, double angle = 0) {
    auto pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x,y,0);
    pose.linear() = Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return pose;
}

int main(int argc, char** argv) {
    struct Point { double time; };
    const std::vector<Point> points{{0},{100},{30},{std::numeric_limits<double>::quiet_NaN()},{-1},{50}};
    const auto timing = InspectPointTiming(points);
    assert(timing.count == 6 && timing.nonfinite == 1 && timing.negative == 1);
    assert(timing.minimum_s == -.001 && timing.maximum_s == .1 && timing.last_s == .05);
    assert(InspectPointTiming(std::vector<Point>{}).count == 0);
    Options options; assert(options.Valid() && !options.enabled);
    auto invalid=options; invalid.max_records=99; assert(!invalid.Valid());
    Recorder disabled; disabled.Add(Scan{}); assert(disabled.Scans().empty());

    const auto ca=Pose(4,0,.2), cb=Pose(6,0,.6);
    std::vector<Anchor> anchors{{0,10,Pose(0),ca*Pose(0)},{1,12,Pose(2),cb*Pose(2)}};
    const auto middle=Correct(11,Pose(1,1),anchors);
    assert((middle.pose.matrix()-(Pose(5,0,.4)*Pose(1,1)).matrix()).norm()<1e-10);
    assert((Correct(10,Pose(0),anchors).pose.matrix()-anchors[0].optimized.matrix()).norm()<1e-10);
    assert((Correct(12,Pose(2),anchors).pose.matrix()-anchors[1].optimized.matrix()).norm()<1e-10);
    assert(std::string(Correct(9,Pose(0),anchors).mode)=="head_hold");
    assert(std::string(Correct(13,Pose(2),anchors).mode)=="tail_hold");
    const auto global=Pose(-10,3,-.7);
    auto moved=anchors;
    for(auto& a:moved) a.optimized=global*a.optimized;
    assert((Correct(11,Pose(1,1),moved).pose.matrix()-(global*middle.pose).matrix()).norm()<1e-10);
    const std::vector<Anchor> identity{{0,10,Pose(0),Pose(0)},{1,12,Pose(2),Pose(2)}};
    assert((Correct(11,Pose(1,1,.3),identity).pose.matrix()-Pose(1,1,.3).matrix()).norm()<1e-10);

    Recorder recorder; options.enabled=true; recorder.Configure(options);
    for(int index=-1;index<=21;++index) {
        const double t=10+index*.1;
        const double a=(t-10)/2;
        Scan scan; scan.begin=t-.1; scan.timestamp=scan.state_timestamp=t;
        scan.raw=Pack(Pose(2*a,4*a*(1-a)));
        scan.pose_valid=true; scan.status=index==13 ? Status::Rejected : Status::Accepted;
        scan.points=InspectPointTiming(std::vector<Point>{{0},{100}});
        scan.imu_count=20; scan.imu_first=t-.095; scan.imu_last=t-.005; scan.imu_latest=t+.005;
        scan.propagation_begin=t-.1;
        recorder.Add(scan);
    }
    assert(recorder.Scans().size()==23 && recorder.Faults()==0);
    auto duplicate=recorder.Scans().back();
    Recorder limited; options.max_records=100;limited.Configure(options);
    for(int i=0;i<101;++i) { duplicate.begin=20+i*.1;duplicate.timestamp=duplicate.begin+.1;limited.Add(duplicate); }
    assert(limited.Scans().size()==100 && limited.Faults()==1);
    limited.Add(duplicate); assert(limited.Faults()==2);
    if(argc>1) {
        const std::filesystem::path directory(argv[1]);
        std::filesystem::create_directories(directory);
        recorder.Export(directory,anchors);
        std::ofstream keyframes(directory/"keyframes.txt");keyframes<<std::setprecision(18);
        for(const auto& a:anchors) {
            keyframes<<a.id<<' '<<a.timestamp;
            for(auto value:Pack(a.optimized))keyframes<<' '<<value;
            keyframes<<'\n';
        }
        keyframes.close();
        std::ofstream frames(directory/"map_frames.yaml");
        frames<<"format_version: 2\nkeyframe_pose_frame: imu\nkeyframe_cloud_frame: imu\n"
              <<"T_imu_lidar: [1,0,0,0.2, 0,1,0,-0.1, 0,0,1,0.05, 0,0,0,1]\n";
    }
    std::cout<<"scan_trajectory: nonlinear motion, correction anchors, frame invariance, point timing and bounds passed\n";
}
