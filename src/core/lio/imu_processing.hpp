#pragma once

#ifndef FASTER_LIO_IMU_PROCESSING_H
#define FASTER_LIO_IMU_PROCESSING_H

#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>

#include "common/eigen_types.h"
#include "common/measure_group.h"
#include "common/point_def.h"
#include "common/options.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_filter.h"
#include "core/lio/imu_prediction.h"
#include "core/lio/pose6d.h"
#include "utils/timer.h"

namespace lightning {

/// IMU处理类
class ImuProcess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void SetExtrinsic(const Vec3d &transl, const Mat3d &rot);
    void SetGyrCov(const Vec3d &scaler);
    void SetAccCov(const Vec3d &scaler);
    void SetGyrBiasCov(const Vec3d &b_g);
    void SetAccBiasCov(const Vec3d &b_a);

    void Process(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &scan);

    bool HasPropagationFault() const { return propagation_fault_; }
    bool LastScanHadImuGap() const { return last_scan_had_gap_; }
    bool IsIMUInited() const { return imu_need_init_ == false; }
    void SetUseIMUFilter(bool b) { use_imu_filter_ = b; }

    double GetMeanAccNorm() const { return mean_acc_.norm(); }
    double GetAccScaleFactor() const { return acc_scale_factor_; }

    Eigen::Matrix<double, 12, 12> Q_;
    Vec3d cov_acc_;
    Vec3d cov_gyr_;
    Vec3d cov_acc_scale_;
    Vec3d cov_gyr_scale_;
    Vec3d cov_bias_gyr_;
    Vec3d cov_bias_acc_;

   private:
    void IMUInit(const MeasureGroup &meas, ESKF &kf_state, int &N);
    void UndistortPcl(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &pcl_out);

    static inline constexpr int max_init_count_ = 20;

    PointCloudType::Ptr cur_pcl_un_ = nullptr;
    lightning::IMUPtr last_imu_ = nullptr;
    std::deque<lightning::IMUPtr> imu_queue_;

    std::vector<Pose6D> imu_pose_;
    Mat3d R_lidar_imu_ = Mat3d ::Identity();
    Vec3d t_lidar_mu_ = Vec3d ::Zero();
    Vec3d mean_acc_ = Vec3d::Zero();
    Vec3d mean_gyr_ = Vec3d::Zero();
    Vec3d angvel_last_ = Vec3d ::Zero();
    Vec3d acc_s_last_ = Vec3d ::Zero();
    double acc_scale_factor_ = 1.0;

    double last_lidar_end_time_ = 0;
    int init_iter_num_ = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;

    bool propagation_fault_ = false;
    bool last_scan_had_gap_ = false;
    bool use_imu_filter_ = true;
    IMUFilter filter_;
};

inline ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num_ = 1;
    Q_.setZero();
    Q_.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5, 0.0, 0.0, 0.0;
    cov_acc_ = Vec3d(0.1, 0.1, 0.1);
    cov_gyr_ = Vec3d(0.1, 0.1, 0.1);
    cov_bias_gyr_ = Vec3d(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = Vec3d(0.0001, 0.0001, 0.0001);
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    last_imu_.reset(new lightning::IMU());
}

inline ImuProcess::~ImuProcess() {}

inline void ImuProcess::Reset() {
    mean_acc_ = Vec3d(0, 0, -1.0);
    mean_gyr_ = Vec3d(0, 0, 0);
    angvel_last_.setZero();
    acc_scale_factor_ = 1.0;

    imu_need_init_ = true;
    propagation_fault_ = false;
    last_scan_had_gap_ = false;
    last_lidar_end_time_ = 0;
    init_iter_num_ = 1;
    imu_queue_.clear();
    imu_pose_.clear();
    last_imu_.reset(new lightning::IMU());
    cur_pcl_un_.reset(new PointCloudType());
}

inline void ImuProcess::SetExtrinsic(const Vec3d &transl, const Mat3d &rot) {
    t_lidar_mu_ = transl;
    R_lidar_imu_ = rot;
}

inline void ImuProcess::SetGyrCov(const Vec3d &scaler) { cov_gyr_scale_ = scaler; }

inline void ImuProcess::SetAccCov(const Vec3d &scaler) { cov_acc_scale_ = scaler; }

inline void ImuProcess::SetGyrBiasCov(const Vec3d &b_g) { cov_bias_gyr_ = b_g; }

inline void ImuProcess::SetAccBiasCov(const Vec3d &b_a) { cov_bias_acc_ = b_a; }

inline void ImuProcess::IMUInit(const MeasureGroup &meas, ESKF &kf_state, int &N) {
    /** 1. initializing the gravity_, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity_ **/

    Vec3d cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu_.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu_.front()->angular_velocity;
        mean_acc_ = imu_acc;
        mean_gyr_ = gyr_acc;
    }

    for (const auto &imu : meas.imu_) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc = imu_acc;
        cur_gyr = gyr_acc;

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        cov_acc_ =
            cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ =
            cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        N++;
    }

    auto init_state = kf_state.GetX();
    init_state.timestamp_ = meas.imu_.back()->timestamp;
    init_state.grav_ = -mean_acc_ / mean_acc_.norm() * G_m_s2;
    init_state.bg_ = mean_gyr_;
    kf_state.ChangeX(init_state);

    auto init_P = kf_state.GetP();
    init_P.setIdentity();
    init_P.block<NavState::kBlockDim, NavState::kBlockDim>(NavState::kBgIdx, NavState::kBgIdx) =
        0.0001 * Mat3d::Identity();
    kf_state.ChangeP(init_P);

    // LOG(INFO) << "P diag: " << init_P.diagonal().transpose();

    last_imu_ = meas.imu_.back();
}

inline void ImuProcess::UndistortPcl(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &pcl_out) {
    auto v_imu = meas.imu_;
    v_imu.push_front(last_imu_);
    const double pcl_beg_time = meas.lidar_begin_time_;
    const double pcl_end_time = meas.lidar_end_time_;
    const ESKF before = kf_state;
    const auto fail = [&](const char* reason) {
        kf_state = before;
        propagation_fault_ = true;
        pcl_out.reset(new PointCloudType());
        LOG(ERROR) << "[imu_clock] propagation stopped: " << reason
                   << " state_time=" << std::setprecision(16) << before.GetX().timestamp_
                   << " scan_end=" << pcl_end_time << "; restart SLAM and replay slower";
    };
    if (!std::isfinite(pcl_beg_time) || !std::isfinite(pcl_end_time) || pcl_beg_time > pcl_end_time ||
        !prediction::PlanPropagation(kf_state.GetX().timestamp_, pcl_end_time).valid) {
        fail("invalid frame interval or gap longer than 1 second");
        return;
    }
    double previous_stamp = v_imu.front()->timestamp;
    for (const auto& imu : v_imu) {
        if (!std::isfinite(imu->timestamp) || imu->timestamp < previous_stamp ||
            imu->timestamp > pcl_end_time + 1e-6 || !imu->angular_velocity.allFinite() ||
            !imu->linear_acceleration.allFinite()) {
            fail("invalid or unordered IMU samples");
            return;
        }
        previous_stamp = imu->timestamp;
    }

    auto imu_state = kf_state.GetX();
    imu_pose_.clear();
    imu_pose_.emplace_back(imu_state.timestamp_ - pcl_beg_time, acc_s_last_, angvel_last_,
                           imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix());
    Vec3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    Mat3d R_imu;
    double dt = 0;
    Vec3d acc = v_imu.back()->linear_acceleration * acc_scale_factor_;
    Vec3d gyro = v_imu.back()->angular_velocity;

    if (use_imu_filter_) {
        // Never overwrite input samples shared with prediction replay or future scan groups.
        for (auto& imu : v_imu) {
            imu = std::make_shared<IMU>(filter_.Filter(*imu));
        }
    }
    Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
    Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
    Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;

    const auto propagate_to = [&](double target, const Vec3d& w, const Vec3d& a) {
        const auto plan = prediction::PlanPropagation(kf_state.GetX().timestamp_, target);
        if (!plan.valid) return false;
        last_scan_had_gap_ = last_scan_had_gap_ || plan.gap;
        const double step = plan.steps > 0 ? plan.duration / plan.steps : 0;
        for (int n = 0; n < plan.steps; ++n) {
            // Preserve the original interval's dt^2 noise scale across subdivisions;
            // extra predict covariance inflation makes missing-data recovery conservative.
            kf_state.Predict(step, Q_ * plan.steps, w, a);
        }
        // Only normalize rounding AFTER the full physical interval was propagated.
        kf_state.SetTime(target);
        const auto& state = kf_state.GetX();
        return state.pos_.allFinite() && state.vel_.allFinite() && state.rot_.matrix().allFinite() &&
               kf_state.GetP().allFinite();
    };

    for (auto it = v_imu.begin(); it + 1 != v_imu.end(); ++it) {
        const auto& head = *it;
        const auto& tail = *(it + 1);
        if (tail->timestamp <= kf_state.GetX().timestamp_) continue;
        angvel_avr = 0.5 * (head->angular_velocity + tail->angular_velocity);
        acc_avr = 0.5 * (head->linear_acceleration + tail->linear_acceleration) * acc_scale_factor_;
        acc = acc_avr;
        gyro = angvel_avr;
        // Check raw sample coverage too, even if the current frame starts inside the gap.
        last_scan_had_gap_ = last_scan_had_gap_ || tail->timestamp - head->timestamp > 0.10 + 1e-6;
        if (!propagate_to(tail->timestamp, gyro, acc)) {
            fail("invalid IMU propagation interval or nonfinite state");
            return;
        }
        imu_state = kf_state.GetX();
        angvel_last_ = angvel_avr - imu_state.bg_;
        acc_s_last_ = imu_state.rot_ * acc_avr + imu_state.grav_;
        const double offset = tail->timestamp - pcl_beg_time;
        imu_pose_.emplace_back(offset, acc_s_last_, angvel_last_, imu_state.vel_, imu_state.pos_,
                               imu_state.rot_.matrix());
    }

    // Use the actual state time, not abs(scan_end - last_sample), which can hide an early break.
    if (!propagate_to(pcl_end_time, gyro, acc)) {
        fail("invalid frame-tail propagation interval or nonfinite state");
        return;
    }
    imu_state = kf_state.GetX();
    last_imu_ = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;
    if (last_scan_had_gap_) {
        pcl_out.reset(new PointCloudType());
        LOG(WARNING) << "[imu_clock] missing IMU coverage (>0.10s): held-input prediction only; "
                     << "skip this scan's registration/map insertion; clock_error_s="
                     << kf_state.GetX().timestamp_ - pcl_end_time;
        return;
    }

    /*** sort point clouds by offset time ***/
    pcl_out = meas.scan_;
    std::sort(pcl_out->points.begin(), pcl_out->points.end(),
              [](const PointType &p1, const PointType &p2) { return p1.time < p2.time; });

    /*** undistort each lidar point (backward propagation) ***/
    if (pcl_out->empty()) {
        return;
    }

    std::ptrdiff_t point_index = static_cast<std::ptrdiff_t>(pcl_out->size()) - 1;
    for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = (head->rot);
        vel_imu = (head->vel);
        pos_imu = (head->pos);
        acc_imu = (tail->acc);
        angvel_avr = (tail->gyr);

        for (; point_index >= 0; --point_index) {
            auto it_pcl = pcl_out->points.begin() + point_index;
            const double point_time = it_pcl->time / 1000.0;
            if (point_time < head->offset_time) {
                break;
            }
            dt = point_time - head->offset_time;

            /// dt 有时候存在非法数据
            if (dt < 0 || dt > lo::lidar_time_interval) {
                // LOG(WARNING) << "find abnormal dt in cloud: " << dt;
                continue;
            }

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * p_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            // SO3::exp takes the full rotation vector; math::exp uses quaternion half angles.
            Mat3d R_i(R_imu * SO3::exp(angvel_avr * dt).matrix());

            Vec3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            Vec3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos_);
            Vec3d p_compensate = R_lidar_imu_.transpose() *
                                 (imu_state.rot_.inverse() * (R_i * (R_lidar_imu_ * P_i + t_lidar_mu_) + T_ei) -
                                  t_lidar_mu_);  // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            // if (it_pcl == pcl_out->points.begin()) {
            //     break;
            // }
        }
    }
}

inline void ImuProcess::Process(const MeasureGroup &meas, ESKF &kf_state, CloudPtr &scan) {
    scan.reset(new PointCloudType());
    last_scan_had_gap_ = false;
    if (propagation_fault_ || meas.imu_.empty()) {
        return;
    }

    if (imu_need_init_) {
        /// The very first lidar frame
        IMUInit(meas, kf_state, init_iter_num_);

        imu_need_init_ = true;

        last_imu_ = meas.imu_.back();

        auto imu_state = kf_state.GetX();
        if (init_iter_num_ > max_init_count_) {
            imu_need_init_ = false;

            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;
            const double mean_acc_norm = mean_acc_.norm();

            if ((mean_acc_norm > 0.5 && mean_acc_norm < 1.5) ||
                (mean_acc_norm > 7.0 && mean_acc_norm < 12.0)) {
                // The initial stationary acceleration and estimated gravity must cancel.
                // This handles both g and m/s^2 inputs and the measured scale error.
                acc_scale_factor_ = G_m_s2 / mean_acc_norm;
            } else {
                acc_scale_factor_ = 1.0;
                LOG(WARNING) << "imu init mean acc norm is abnormal for unit inference: " << mean_acc_norm
                             << ", keep accelerometer scale unchanged";
            }

            LOG(INFO) << "imu init done, bg: " << imu_state.bg_.transpose() << ", grav: " << imu_state.grav_.transpose()
                      << ", acc scale: " << acc_scale_factor_ << ", mean: " << mean_acc_.transpose() << ", "
                      << mean_gyr_.transpose();
        } else {
            LOG(INFO) << "waiting for imu init ... " << init_iter_num_;
        }

        return;
    }

    Timer::Evaluate([&, this]() { UndistortPcl(meas, kf_state, scan); }, "Undistort Pcl");
}
}  // namespace lightning

#endif
