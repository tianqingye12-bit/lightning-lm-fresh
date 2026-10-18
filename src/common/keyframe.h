//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_KEYFRAME_H
#define LIGHTNING_KEYFRAME_H

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "common/std_types.h"

namespace lightning {

/// 关键帧描述
/// NOTE: 在添加后端后，需要加锁
class Keyframe {
   public:
    using Ptr = std::shared_ptr<Keyframe>;

    Keyframe() {}
    Keyframe(unsigned long id, CloudPtr cloud, NavState state, const SE3& T_imu_lidar = SE3())
        : id_(id), cloud_(cloud), T_imu_lidar_(T_imu_lidar), pose_lio_(state.GetPose()), state_(state) {
        timestamp_ = state_.timestamp_;
        pose_opt_ = pose_lio_;
    }

    // Serialize whole-graph commits with pose snapshots; individual getters keep their own locks.
    static std::mutex& PoseGraphMutex() {
        static std::mutex mutex;
        return mutex;
    }
    static std::vector<SE3> SnapshotPoses(const std::vector<Ptr>& keyframes, bool use_lio_pose = false) {
        UL graph_lock(PoseGraphMutex());
        std::vector<SE3> poses;
        poses.reserve(keyframes.size());
        for (const auto& kf : keyframes) poses.push_back(use_lio_pose ? kf->GetLIOPose() : kf->GetOptPose());
        return poses;
    }

    unsigned long GetID() const { return id_; }
    CloudPtr GetCloud() const { return cloud_; }
    const SE3& GetTImuLidar() const { return T_imu_lidar_; }

    // Sensor-origin algorithms (G2P5 rays/ground heights) still require LiDAR coordinates.
    CloudPtr GetLidarCloud() const {
        CloudPtr lidar(new PointCloudType(*cloud_));
        const SE3 T_lidar_imu = T_imu_lidar_.inverse();
        for (auto& point : lidar->points) {
            const Vec3d xyz = T_lidar_imu * point.getVector3fMap().cast<double>();
            point.getVector3fMap() = xyz.cast<float>();
        }
        return lidar;
    }

    SE3 GetLIOPose() {
        UL lock(data_mutex_);
        return pose_lio_;
    }

    void SetLIOPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_lio_ = pose;

        // also set opt
        pose_opt_ = pose_lio_;
    }

    SE3 GetOptPose() {
        UL lock(data_mutex_);
        return pose_opt_;
    }

    void SetOptPose(const SE3& pose) {
        UL lock(data_mutex_);
        pose_opt_ = pose;
    }

    void SetState(NavState s) {
        UL lock(data_mutex_);
        state_ = s;
    }

    NavState GetState() {
        UL lock(data_mutex_);
        return state_;
    }

   protected:
    unsigned long id_ = 0;

    double timestamp_ = 0;
    CloudPtr cloud_ = nullptr;  /// IMU-frame cloud; pose_lio_/pose_opt_ are T_world_imu

    SE3 T_imu_lidar_;  // immutable calibration accompanying this IMU-frame cloud

    std::mutex data_mutex_;
    SE3 pose_lio_;  // 前端的pose
    SE3 pose_opt_;  // 后端优化后的pose

    NavState state_;  // 卡尔曼滤波器状态
};

}  // namespace lightning

#endif  // LIGHTNING_KEYFRAME_H
