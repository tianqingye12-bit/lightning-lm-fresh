#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <algorithm>
#include <stdexcept>

#include "common/options.h"
#include "core/lightning_math.hpp"
#include "laser_mapping.h"
#include "core/lio/map_detail_cloud.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

namespace lightning {

bool LaserMapping::Init(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, ESKF::state_dim_, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) { ObsModel(s, obs); };
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);

    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();

        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();
        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

        skip_lidar_num_ = yaml["fasterlio"]["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;

        float height_max = yaml["roi"]["height_max"].as<float>();
        float height_min = yaml["roi"]["height_min"].as<float>();

        preprocess_->SetHeightROI(height_max, height_min);

        options_.kf_dis_th_ = yaml["fasterlio"]["kf_dis_th"].as<double>();
        options_.kf_angle_th_ = yaml["fasterlio"]["kf_angle_th"].as<double>() * M_PI / 180.0;
        options_.enable_icp_part_ = yaml["fasterlio"]["enable_icp_part"].as<bool>();
        options_.min_pts = yaml["fasterlio"]["min_pts"].as<int>();
        options_.plane_icp_weight_ = yaml["fasterlio"]["plane_icp_weight"].as<float>();

        const auto prediction = yaml["fasterlio"]["imu_prediction"];
        if (prediction && !prediction.IsMap()) {
            LOG(ERROR) << "fasterlio.imu_prediction must be a map";
            return false;
        }
        if (prediction) {
            imu_prediction_options_.max_step_s = prediction["max_step_s"].as<double>(0.10);
            imu_prediction_options_.max_horizon_s = prediction["max_horizon_s"].as<double>(0.30);
            imu_prediction_options_.stats_interval = prediction["stats_interval"].as<int>(200);
        }
        if (!imu_prediction_options_.Valid()) {
            LOG(ERROR) << "invalid IMU prediction time limits";
            return false;
        }

        // Optional nested settings: old dataset YAML files remain valid.
        const auto feature = yaml["fasterlio"]["feature_quality"];
        if (feature && !feature.IsMap()) {
            LOG(ERROR) << "fasterlio.feature_quality must be a map";
            return false;
        }
        if (feature) {
            options_.plane_quality_.enabled = feature["enabled"].as<bool>(options_.plane_quality_.enabled);
            options_.plane_quality_.min_spread = feature["min_spread"].as<double>(options_.plane_quality_.min_spread);
            options_.plane_quality_.min_tangent_ratio =
                feature["min_tangent_ratio"].as<double>(options_.plane_quality_.min_tangent_ratio);
            options_.plane_quality_.max_normal_ratio =
                feature["max_normal_ratio"].as<double>(options_.plane_quality_.max_normal_ratio);
            options_.robust_plane_weight_ = feature["robust_weight"].as<bool>(options_.robust_plane_weight_);
            options_.plane_huber_delta_ = feature["huber_delta"].as<double>(options_.plane_huber_delta_);
            options_.feature_stats_interval_ = feature["stats_interval"].as<int>(options_.feature_stats_interval_);
            const auto support = feature["surface_support"];
            if (support && !support.IsMap()) throw std::invalid_argument("surface_support must be a map");
            if (support) {
                options_.surface_support_.enabled = support["enabled"].as<bool>(false);
                options_.surface_support_.max_neighbor_distance = support["max_neighbor_distance"].as<double>(.6);
                options_.surface_support_.max_plane_residual = support["max_plane_residual"].as<double>(.15);
                options_.surface_support_.max_tangent_gap = support["max_tangent_gap"].as<double>(.10);
            }
        }
        if (!options_.surface_support_.Valid() || !options_.plane_quality_.Valid() || !std::isfinite(options_.plane_huber_delta_) ||
            options_.plane_huber_delta_ <= 0.0 || options_.feature_stats_interval_ < 0 ||
            !std::isfinite(fasterlio::ESTI_PLANE_THRESHOLD) || fasterlio::ESTI_PLANE_THRESHOLD <= 0.0f) {
            LOG(ERROR) << "invalid feature quality thresholds";
            return false;
        }

        const auto registration = yaml["fasterlio"]["registration_quality"];
        if (registration && !registration.IsMap()) throw std::invalid_argument("registration_quality must be a map");
        if (registration) {
            auto& quality = options_.registration_quality_;
            quality.balance = registration["balance"].as<bool>(false);
            quality.max_constraints = registration["max_constraints"].as<int>(3000);
            quality.diagnostics = registration["diagnostics"].as<bool>(false);
            quality.rotation_scale_m = registration["rotation_scale_m"].as<double>(5.0);
            quality.relative_eigenvalue = registration["relative_eigenvalue"].as<double>(1e-3);
        }
        if (!options_.registration_quality_.Valid()) throw std::invalid_argument("invalid registration_quality options");

        trajectory::Options recording;
        const auto trajectory_config = yaml["fasterlio"]["trajectory_recording"];
        if (trajectory_config && !trajectory_config.IsMap()) throw std::invalid_argument("trajectory_recording must be a map");
        if (trajectory_config) {
            recording.enabled = trajectory_config["enabled"].as<bool>(false);
            recording.max_records = trajectory_config["max_records"].as<int>(200000);
        }
        trajectory_recorder_.Configure(recording);

        const auto detail = yaml["fasterlio"]["map_detail"];
        if (detail && !detail.IsMap()) {
            LOG(ERROR) << "fasterlio.map_detail must be a map";
            return false;
        }
        if (detail) {
            detail_options_.preserve_observed_points = detail["preserve_observed_points"].as<bool>(false);
            detail_options_.input_point_filter_num = detail["input_point_filter_num"].as<int>(0);
            detail_options_.keyframe_resolution = detail["keyframe_resolution"].as<double>(0.0);
            const auto max_points = detail["keyframe_max_points"].as<long long>(0);
            if (max_points < 0) {
                LOG(ERROR) << "keyframe_max_points must be nonnegative";
                return false;
            }
            detail_options_.keyframe_max_points = static_cast<std::size_t>(max_points);
            detail_options_.export_resolution = detail["export_resolution"].as<double>(0.1);
            detail_options_.registration_min_ratio = detail["registration_min_ratio"].as<double>(0.1);
            const auto fill = detail["local_fill"];
            if (fill && !fill.IsMap()) {
                LOG(ERROR) << "fasterlio.map_detail.local_fill must be a map";
                return false;
            }
            if (fill) {
                detail_options_.local_fill_enabled = fill["enabled"].as<bool>(false);
                detail_options_.local_fill_min_interval = fill["min_interval"].as<double>(0.2);
                detail_options_.local_fill_translation = fill["translation"].as<double>(0.2);
                detail_options_.local_fill_rotation_deg = fill["rotation_deg"].as<double>(3.0);
                detail_options_.local_fill_min_effective = fill["min_effective"].as<int>(20);
            }
        }
        if (!detail_options_.Valid() || !std::isfinite(filter_size_scan) || filter_size_scan <= 0 ||
            !std::isfinite(filter_size_map_min_) || filter_size_map_min_ <= 0 ||
            preprocess_->PointFilterNum() < 1) {
            LOG(ERROR) << "invalid map detail / scan / map resolution or point filter settings";
            return false;
        }
        if (detail_options_.input_point_filter_num > 0) {
            preprocess_->PointFilterNum() = detail_options_.input_point_filter_num;
        }

        bool use_imu_filter = yaml["fasterlio"]["imu_filter"].as<bool>();
        p_imu_->SetUseIMUFilter(use_imu_filter);
        options_.proj_kfs_ = yaml["fasterlio"]["proj_kfs"].as<bool>();

    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "[map_detail] input_stride=" << preprocess_->PointFilterNum()
              << " registration_m=" << filter_size_scan << " local_map_m=" << filter_size_map_min_
              << " registration_min_ratio=" << detail_options_.registration_min_ratio
              << " keyframe_m=" << detail_options_.keyframe_resolution
              << " keyframe_max_points=" << detail_options_.keyframe_max_points
              << " export_m=" << detail_options_.export_resolution
              << " local_fill=" << detail_options_.local_fill_enabled
              << " observed_points=" << detail_options_.preserve_observed_points;

    LOG(INFO) << "[surface_support] enabled=" << options_.surface_support_.enabled
              << " neighbor_m=" << options_.surface_support_.max_neighbor_distance
              << " residual_m=" << options_.surface_support_.max_plane_residual
              << " tangent_gap_m=" << options_.surface_support_.max_tangent_gap;
    LOG(INFO) << "[registration_quality] balance=" << options_.registration_quality_.balance
              << " max_constraints_per_term=" << options_.registration_quality_.max_constraints
              << " diagnostics=" << options_.registration_quality_.diagnostics
              << " rotation_scale_m=" << options_.registration_quality_.rotation_scale_m;
    LOG(INFO) << "[scan_trajectory] enabled=" << trajectory_recorder_.Enabled();
    LOG(INFO) << "feature quality: pca=" << options_.plane_quality_.enabled
              << " min_spread=" << options_.plane_quality_.min_spread
              << " tangent_ratio=" << options_.plane_quality_.min_tangent_ratio
              << " normal_ratio=" << options_.plane_quality_.max_normal_ratio
              << " huber=" << options_.robust_plane_weight_
              << " delta_m=" << options_.plane_huber_delta_
              << " stats_interval=" << options_.feature_stats_interval_;

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::ROBOSENSE);
        LOG(INFO) << "Using RoboSense Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    offset_t_lidar_fixed_ = math::VecFromArray<double>(extrinT_);
    offset_R_lidar_fixed_ = math::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

bool LaserMapping::AdvanceImuPrediction(const IMUPtr& imu, const char* source) {
    if (imu_prediction_suspended_) return false;
    const double current = kf_imu_.GetX().timestamp_;
    const auto decision = prediction::CheckTime(current, imu->timestamp, last_accepted_lio_time_,
                                                imu_prediction_options_);
    if (decision == prediction::Decision::already_covered) return false;
    if (decision != prediction::Decision::predict || !imu->angular_velocity.allFinite() ||
        !imu->linear_acceleration.allFinite()) {
        imu_prediction_suspended_ = true;
        if (ui_) ui_->UpdatePredictionStatus(false, prediction::Name(decision), imu->timestamp-last_accepted_lio_time_);
        ++prediction_rejections_;
        if (prediction_rejections_ <= 5 || prediction_rejections_ % 100 == 0) {
            LOG(WARNING) << "[imu_prediction] suspended source=" << source
                         << " reason=" << ((!imu->angular_velocity.allFinite() || !imu->linear_acceleration.allFinite())
                                               ? "invalid_sample" : prediction::Name(decision))
                         << " step_s=" << imu->timestamp - current
                         << " accepted_lio_age_s=" << imu->timestamp - last_accepted_lio_time_
                         << " imu_queue=" << imu_buffer_.size() << " lidar_queue=" << lidar_buffer_.size();
        }
        return false;
    }
    kf_imu_.Predict(imu->timestamp - current, p_imu_->Q_, imu->angular_velocity,
                    imu->linear_acceleration * p_imu_->GetAccScaleFactor());
    kf_imu_.SetTime(imu->timestamp);
    const auto& predicted = kf_imu_.GetX();
    if (!predicted.pos_.allFinite() || !predicted.vel_.allFinite() || !predicted.rot_.matrix().allFinite()) {
        LOG(ERROR) << "[imu_prediction] nonfinite prediction; suspend until next LiDAR rebase";
        kf_imu_ = kf_;
        imu_prediction_suspended_ = true;
        if (ui_) ui_->UpdatePredictionStatus(false, "nonfinite_state", 0);
        return false;
    }
    return true;
}

void LaserMapping::RebaseImuPrediction(bool update_accepted) {
    // Caller has finished the LIO update. Replay and live IMU use the SAME time checks.
    UL lock(mtx_buffer_);
    const auto previous = kf_imu_.GetX();
    if (update_accepted) last_accepted_lio_time_ = measures_.lidar_end_time_;
    kf_imu_ = kf_;
    imu_prediction_suspended_ = last_accepted_lio_time_ <= 0 || p_imu_->HasPropagationFault();
    if (imu_prediction_suspended_ && ui_) ui_->UpdatePredictionStatus(false, "waiting_or_core_fault", 0);
    for (const auto& imu : imu_buffer_) {
        AdvanceImuPrediction(imu, "replay");
        if (imu_prediction_suspended_) break;
    }
    ++prediction_rebases_;
    const auto& current = kf_imu_.GetX();
    const double jump = std::abs(previous.timestamp_ - current.timestamp_) < 1e-6
                            ? (previous.pos_ - current.pos_).norm() : -1.0;
    if (prediction_rebases_ % 20 == 0 || jump > 0.2) {
        LOG(INFO) << "[imu_prediction] rebase accepted=" << update_accepted
                  << " suspended=" << imu_prediction_suspended_
                  << " lead_s=" << current.timestamp_ - measures_.lidar_end_time_
                  << " latest_imu_lead_s=" << last_timestamp_imu_ - measures_.lidar_end_time_
                  << " lio_clock_error_s=" << kf_.GetX().timestamp_ - measures_.lidar_end_time_
                  << " accepted_lio_age_s=" << current.timestamp_ - last_accepted_lio_time_
                  << " same_time_jump_m=" << jump << " speed_mps=" << current.vel_.norm()
                  << " imu_queue=" << imu_buffer_.size() << " lidar_queue=" << lidar_buffer_.size();
    }
}

LaserMapping::InputStats LaserMapping::GetInputStats() {
    UL lock(mtx_buffer_);
    return {imu_buffer_.size(), lidar_buffer_.size(), dropped_input_lidar_, input_buffer_fault_};
}

void LaserMapping::RecordScan(trajectory::Status status) {
    if (!trajectory_recorder_.Enabled()) return;
    auto& scan = current_scan_record_;
    const auto state = kf_.GetX();
    scan.state_timestamp = state.timestamp_;
    const Eigen::Isometry3d pose(state.GetPose().matrix());
    scan.pose_valid = trajectory::Rigid(pose);
    if (scan.pose_valid) scan.raw = trajectory::Pack(pose);
    scan.status = status;
    scan.imu_gap = p_imu_->LastScanHadImuGap();
    const auto previous_faults = trajectory_recorder_.Faults();
    trajectory_recorder_.Add(scan);
    if (trajectory_recorder_.Faults() != previous_faults &&
        (trajectory_recorder_.Faults() == 1 || trajectory_recorder_.Faults() % 100 == 0))
        LOG(ERROR) << "[scan_trajectory] recording incomplete: capacity or non-increasing scan time; faults="
                   << trajectory_recorder_.Faults();
}

void LaserMapping::ExportScanTrajectory(const std::string& directory, const std::vector<Keyframe::Ptr>& keyframes,
                                      const std::vector<SE3>& optimized_poses) const {
    if (!trajectory_recorder_.Enabled()) return;
    if (keyframes.size() != optimized_poses.size()) throw std::runtime_error("trajectory snapshot count mismatch");
    std::vector<trajectory::Anchor> anchors;
    anchors.reserve(keyframes.size());
    for (std::size_t i = 0; i < keyframes.size(); ++i)
        anchors.push_back({keyframes[i]->GetID(), keyframes[i]->GetState().timestamp_,
            Eigen::Isometry3d(keyframes[i]->GetLIOPose().matrix()), Eigen::Isometry3d(optimized_poses[i].matrix())});
    trajectory_recorder_.Export(directory, anchors);
    LOG(INFO) << "[scan_trajectory] saved rows=" << trajectory_recorder_.Scans().size()
              << " recording_faults=" << trajectory_recorder_.Faults();
}

void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu) {
    publish_count_++;
    const double timestamp = imu->timestamp;
    UL lock(mtx_buffer_);
    if (input_buffer_fault_) return;
    if (max_imu_buffer_ && imu_buffer_.size() >= max_imu_buffer_) {
        input_buffer_fault_ = true; imu_prediction_suspended_ = true;
        if (ui_) ui_->UpdatePredictionStatus(false, "core_imu_overflow", 0);
        LOG(ERROR) << "[runtime] core IMU buffer overflow: restart slower or restore LiDAR input";
        return;
    }
    if (!std::isfinite(timestamp) || !imu->angular_velocity.allFinite() ||
        !imu->linear_acceleration.allFinite()) {
        LOG(WARNING) << "[imu_prediction] discard nonfinite IMU input";
        return;
    }
    if (timestamp <= last_timestamp_imu_) {
        LOG(WARNING) << "[imu_prediction] discard non-increasing IMU timestamp; restart SLAM for a new bag epoch";
        return;
    }
    if (p_imu_->IsIMUInited() && AdvanceImuPrediction(imu, "live")) {
        const auto& state = kf_imu_.GetX();
        if (ui_ && prediction::CanPublish(state.timestamp_, last_prediction_published_time_)) {
            ui_->UpdateNavState(state);
            ui_->UpdatePredictionStatus(true, "valid", timestamp-last_accepted_lio_time_);
            last_prediction_published_time_ = state.timestamp_;
        }
        if (imu_prediction_options_.stats_interval > 0 &&
            publish_count_ % imu_prediction_options_.stats_interval == 0) {
            LOG(INFO) << "[imu_prediction] live lead_s=" << timestamp - kf_.GetX().timestamp_
                      << " accepted_lio_age_s=" << timestamp - last_accepted_lio_time_
                      << " offset_m=" << (state.pos_ - kf_.GetX().pos_).norm()
                      << " speed_mps=" << state.vel_.norm();
        }
    }
    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(imu);
}

bool LaserMapping::Run() {
    if (input_buffer_fault_) return false;
    if (!SyncPackages()) {
        LOG(WARNING) << "sync package failed";
        return false;
    }

    if (trajectory_recorder_.Enabled()) {
        current_scan_record_ = trajectory::Scan{};
        auto& record = current_scan_record_;
        record.begin = measures_.lidar_begin_time_;
        record.timestamp = measures_.lidar_end_time_;
        record.points = trajectory::InspectPointTiming(measures_.scan_->points);
        record.propagation_begin = kf_.GetX().timestamp_;
        record.imu_count = measures_.imu_.size();
        if (!measures_.imu_.empty()) {
            record.imu_first = measures_.imu_.front()->timestamp;
            record.imu_last = measures_.imu_.back()->timestamp;
        }
        record.imu_latest = last_timestamp_imu_;
    }

    /// IMU process, kf prediction, undistortion
    Measure(runtime_stats_, RuntimeStats::deskew, [&] { p_imu_->Process(measures_, kf_, scan_undistort_); });

    if (!scan_undistort_ || scan_undistort_->empty()) {
        if (p_imu_->IsIMUInited()) RebaseImuPrediction(false);
        LOG(WARNING) << "No point or unreliable IMU coverage, skip this scan!";
        RecordScan(p_imu_->HasPropagationFault() ? trajectory::Status::PropagationFault :
                   !p_imu_->IsIMUInited() ? trajectory::Status::Initializing :
                   p_imu_->LastScanHadImuGap() ? trajectory::Status::ImuGap : trajectory::Status::NoPoints);
        return false;
    }

    const double imu_clock_error = kf_.GetX().timestamp_ - measures_.lidar_end_time_;
    if (!std::isfinite(imu_clock_error) || std::abs(imu_clock_error) > 1e-5) {
        LOG(ERROR) << "[imu_clock] reject scan: state/frame time mismatch " << imu_clock_error;
        RebaseImuPrediction(false);
        RecordScan(trajectory::Status::ClockMismatch);
        return false;
    }

    /// the first scan
    if (flg_first_scan_) {
        LOG(INFO) << "first scan pts: " << scan_undistort_->size();

        state_point_ = kf_.GetX();
        CloudPtr bootstrap = scan_undistort_;
        if (detail_options_.input_point_filter_num > 0) {
            if (detail_options_.preserve_observed_points) {
                bootstrap = mapping::ObservedVoxelCloud<PointType>(scan_undistort_, voxel_scan_.getLeafSize()[0]);
            } else {
                bootstrap.reset(new PointCloudType());
                voxel_scan_.setInputCloud(scan_undistort_);
                voxel_scan_.filter(*bootstrap);
            }
        }
        scan_down_world_->resize(bootstrap->size());
        for (size_t i = 0; i < bootstrap->size(); i++) {
            PointBodyToWorld(bootstrap->points[i], scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        last_map_update_pose_ = state_point_.GetPose();
        last_map_update_time_ = state_point_.timestamp_;
        has_map_update_ = true;
        flg_first_scan_ = false;
        RebaseImuPrediction(true);
        RecordScan(trajectory::Status::Bootstrap);
        return true;
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;

        if (skip_lidar_cnt_ != 0) {
            /// 更新UI中的内容
            if (ui_) {
                Measure(runtime_stats_, RuntimeStats::ui_prepare, [&] {
                    ui_->UpdateScan(scan_undistort_, kf_.GetX().GetPose(), GetTImuLidar()); });
            }

            RebaseImuPrediction(false);
            RecordScan(trajectory::Status::Skipped);
            return false;
        }
    }

    LOG(INFO) << "=============================";
    LOG(INFO) << "LIO get cloud at beg: " << std::setprecision(14) << measures_.lidar_begin_time_
              << ", end: " << measures_.lidar_end_time_;

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        LOG(ERROR) << "检测到雷达断流，时长：" << (measures_.lidar_begin_time_ - last_lidar_time_);
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    /// downsample
    voxel_scan_.setInputCloud(scan_undistort_);
    Measure(runtime_stats_, RuntimeStats::downsample, [&] {
        if (detail_options_.preserve_observed_points)
            scan_down_body_ = mapping::ObservedVoxelCloud<PointType>(scan_undistort_, voxel_scan_.getLeafSize()[0]);
        else voxel_scan_.filter(*scan_down_body_);
    });

    // if (options_.proj_kfs_) {
    //     ProjectKFs();
    // }

    int cur_pts = scan_down_body_->size();

    if (cur_pts < (scan_undistort_->size() * detail_options_.registration_min_ratio) || cur_pts < options_.min_pts) {
        /// 降采样太狠了,有效点数不够，用0.1分辨率代替
        // LOG(INFO) << "too few points, using 0.1 resol";
        auto v = voxel_scan_;
        const float leaf = std::min(0.1f, voxel_scan_.getLeafSize()[0]);
        v.setLeafSize(leaf, leaf, leaf);
        v.setInputCloud(scan_undistort_);
        Measure(runtime_stats_, RuntimeStats::downsample, [&] {
            if (detail_options_.preserve_observed_points)
                scan_down_body_ = mapping::ObservedVoxelCloud<PointType>(scan_undistort_, leaf);
            else v.filter(*scan_down_body_);
        });

        // LOG(INFO) << "Now pts: " << scan_down_body_->size() << ", before: " << cur_pts;
        cur_pts = scan_down_body_->size();
    }

    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        RecordScan(trajectory::Status::TooFewPoints);
        return false;
    }

    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);

    // 成员变量预分配
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, 1);
    point_selected_icp_.resize(cur_pts, 1);
    plane_coef_.resize(cur_pts, Vec4f::Zero());

    auto pred_state = kf_.GetX();
    // pred_state.pos_ = state_point_.pos_;  // 假定位置不动行不行,防止速度漂移
    // kf_.ChangeX(pred_state);

    feature_stats_ = features::FeatureStats{};
    registration_diagnostic_ = features::RegistrationDiagnostic{};
    surface_candidates_ = icp_candidates_ = 0;
    icp_used_ = 0;
    const bool update_accepted = Measure(runtime_stats_, RuntimeStats::registration,
        [&] { return kf_.Update(ESKF::ObsType::LIDAR, 1.0); });

    ++feature_stats_scan_count_;
    if (options_.feature_stats_interval_ > 0 &&
        feature_stats_scan_count_ % options_.feature_stats_interval_ == 0) {
        LOG(INFO) << "[features] scan=" << feature_stats_scan_count_ << " update_accepted=" << update_accepted
                  << " " << feature_stats_.Summary();
        if (options_.registration_quality_.balance || options_.registration_quality_.diagnostics) {
            LOG(INFO) << "[registration_quality] scan=" << feature_stats_scan_count_
                      << " plane=" << effect_feat_surf_ << "/" << surface_candidates_
                      << " point_icp=" << icp_used_ << "/" << icp_candidates_
                      << " plane_info_valid=" << registration_diagnostic_.valid
                      << " plane_rank=" << registration_diagnostic_.rank
                      << " min_max=" << registration_diagnostic_.min_max_ratio
                      << " weak_t_world_Lrot_body=" << registration_diagnostic_.weakest.transpose();
        }
    }

    state_point_ = kf_.GetX();
    state_point_.timestamp_ = measures_.lidar_end_time_;

    const double delta_translation = (pred_state.pos_ - state_point_.pos_).norm();
    const double delta_rotation_deg = (pred_state.rot_.inverse() * state_point_.rot_).log().norm() * 180.0 / M_PI;
    const double delta_velocity = (pred_state.vel_ - state_point_.vel_).norm();

    const double current_speed = state_point_.vel_.norm();

    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_surf_ << ", "
              << effect_feat_icp_;
    LOG(INFO) << "delta trans: " << (pred_state.pos_ - state_point_.pos_).transpose()
              << ", ang: " << delta_rotation_deg;
    // LOG(INFO) << "P diag: " << kf_.GetP().diagonal().transpose();

    // Vec3d v_from_last = (state_point_.pos_ - last_state.pos_) / (state_point_.timestamp_ - last_state.timestamp_);
    // LOG(INFO) << "v from last: " << v_from_last.transpose();

    // if (delta_velocity > 1.0 || current_speed > 4.0) {
    //     LOG(ERROR) << "detected very large vel change, last: " << last_state.vel_.transpose()
    //                << ", pred: " << pred_state.vel_.transpose() << ", cur:" << state_point_.vel_.transpose();
    //     LOG(ERROR) << "please check";
    // }

    /// Only registered scans may enter the local map or saved keyframes.
    if (!update_accepted) {
        LOG(WARNING) << "LIO registration rejected at " << std::setprecision(14) << measures_.lidar_end_time_
                     << "; retain IMU prediction, skip keyframe and map insertion";
    } else if (last_kf_ == nullptr) {
        MakeKF();
    } else {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = state_point_.GetPose();
        if ((last_pose.translation() - cur_pose.translation()).norm() > options_.kf_dis_th_ ||
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > options_.kf_angle_th_) {
            MakeKF();
        } else if (!options_.is_in_slam_mode_ && (state_point_.timestamp_ - last_kf_->GetState().timestamp_) > 2.0) {
            MakeKF();
        } else if (has_map_update_ && detail_options_.local_fill_enabled) {
            const double translation = (cur_pose.translation() - last_map_update_pose_.translation()).norm();
            const double rotation_deg =
                (last_map_update_pose_.so3().inverse() * cur_pose.so3()).log().norm() * 180.0 / M_PI;
            if (detail_options_.ShouldFill(update_accepted, std::max(effect_feat_surf_, effect_feat_icp_),
                                          state_point_.timestamp_ - last_map_update_time_, translation, rotation_deg)) {
                Timer::Evaluate([this]() { MapIncremental(); }, "    Local Map Fill");
                LOG(INFO) << "[map_detail] local_fill points=" << scan_down_body_->size()
                          << " translation=" << translation << " rotation_deg=" << rotation_deg;
            }
        }
    }

    RebaseImuPrediction(update_accepted);
    RecordScan(update_accepted ? trajectory::Status::Accepted : trajectory::Status::Rejected);

    if (ui_) {
        Measure(runtime_stats_, RuntimeStats::ui_prepare, [&] {
            ui_->UpdateScan(scan_down_body_, state_point_.GetPose(), GetTImuLidar()); });
    }

    LOG(INFO) << "LIO state: " << state_point_.pos_.transpose() << ", yaw "
              << state_point_.rot_.angleZ<double>() * 180 / M_PI << ", vel: " << state_point_.vel_.transpose()
              << ", grav: " << state_point_.grav_.transpose() << ", grav norm: " << state_point_.grav_.norm();

    return update_accepted;
}

void LaserMapping::ProjectKFs(CloudPtr cloud, int size_limit) {
    auto state = kf_.GetX();
    SE3 pose_cur(state.rot_, state.pos_);
    pose_cur = pose_cur.inverse();

    for (auto kf : proj_kfs_) {
        // LOG(INFO) << "projecting kf: " << kf->GetID();
        // if (last_kf_) {
        // auto kf = last_kf_;
        SE3 pose = GetTImuLidar().inverse() * pose_cur * kf->GetLIOPose();

        int cnt = 0;
        for (auto &pt : kf->GetCloud()->points) {
            Vec3d p = pose * ToVec3d(pt);
            PointType pcl_pt;

            pcl_pt.x = p.x();
            pcl_pt.y = p.y();
            pcl_pt.z = p.z();
            pcl_pt.intensity = pt.intensity;

            cloud->push_back(pcl_pt);
            cnt++;

            if (cnt > size_limit) {
                break;
            }
        }
        // }
    }
}

CloudPtr LaserMapping::CloudInImuFrame(const CloudPtr& cloud) const {
    // Always allocate: RetainCloud can alias the deskewed registration input.
    return mapping::CloudInBodyFrame<PointType>(cloud, GetTImuLidar().matrix());
}

CloudPtr LaserMapping::RetainKeyframeCloud() const {
    // Graph poses are T_world_imu, so every graph consumer receives IMU-frame points.
    return CloudInImuFrame(mapping::RetainCloud<PointType>(scan_undistort_, detail_options_));
}

void LaserMapping::MakeKF() {
    CloudPtr retained = RetainKeyframeCloud();
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, retained, state_point_, GetTImuLidar());
    LOG(INFO) << "[map_detail] keyframe_points=" << retained->size()
              << " input_points=" << scan_undistort_->size();

    if (last_kf_) {
        /// opt pose 用之前的递推
        SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose();
        kf->SetOptPose(last_kf_->GetOptPose() * delta);
    } else {
        kf->SetOptPose(kf->GetLIOPose());
    }

    kf->SetState(state_point_);

    LOG(INFO) << "LIO: create kf " << kf->GetID() << ", state: " << state_point_.pos_.transpose()
              << ", kf opt pose: " << kf->GetOptPose().translation().transpose()
              << ", lio pose: " << kf->GetLIOPose().translation().transpose() << ", time: " << std::setprecision(14)
              << state_point_.timestamp_;

    if (options_.is_in_slam_mode_) {
        all_keyframes_.emplace_back(kf);
    }

    last_kf_ = kf;

    // 有keyframes时更新local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    /// 更新project kfs
    if (proj_kfs_.size() >= options_.max_proj_kfs_) {
        auto last = proj_kfs_.back();

        SE3 delta = last->GetLIOPose().inverse() * kf->GetLIOPose();

        if (delta.translation().norm() < 3 || delta.so3().log().norm() < 20 / 180 * M_PI) {
            // proj_kfs_.pop_back();
        } else {
            proj_kfs_.pop_front();
            proj_kfs_.emplace_back(kf);
        }
    } else {
        proj_kfs_.emplace_back(kf);
    }

    // for (auto &kf : proj_kfs_) {
    //     LOG(INFO) << "proj kf: " << kf->GetID();
    // }
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    if (input_buffer_fault_) return;
    if (max_lidar_buffer_ && lidar_buffer_.size() >= max_lidar_buffer_) {
        ++dropped_input_lidar_;
        if (dropped_input_lidar_ == 1 || dropped_input_lidar_ % 20 == 0)
            LOG(WARNING) << "[runtime] drop newest core LiDAR frame count=" << dropped_input_lidar_;
        return;
    }
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, dt: " << timestamp - last_timestamp_lidar_;
                return;
            }

            LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
                      << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            Measure(runtime_stats_, RuntimeStats::preprocess, [&] { preprocess_->Process(msg, cloud); });

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    if (input_buffer_fault_) return;
    if (max_lidar_buffer_ && lidar_buffer_.size() >= max_lidar_buffer_) {
        ++dropped_input_lidar_;
        if (dropped_input_lidar_ == 1 || dropped_input_lidar_ % 20 == 0)
            LOG(WARNING) << "[runtime] drop newest core LiDAR frame count=" << dropped_input_lidar_;
        return;
    }
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            // LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
            //           << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            Measure(runtime_stats_, RuntimeStats::preprocess, [&] { preprocess_->Process(msg, cloud); });

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    UL lock(mtx_buffer_);
    if (input_buffer_fault_) return;
    if (max_lidar_buffer_ && lidar_buffer_.size() >= max_lidar_buffer_) {
        ++dropped_input_lidar_;
        if (dropped_input_lidar_ == 1 || dropped_input_lidar_ % 20 == 0)
            LOG(WARNING) << "[runtime] drop newest core LiDAR frame count=" << dropped_input_lidar_;
        return;
    }
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            double timestamp = math::ToSec(cloud->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        LOG(INFO) << "lidar or imu is empty";
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);

            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;

            if ((lidar_end_time_ - measures_.lidar_begin_time_) > 5 * lo::lidar_time_interval) {
                /// timestamp 有异常
                lidar_end_time_ = measures_.lidar_begin_time_ + lo::lidar_time_interval;
                lidar_mean_scantime_ = lo::lidar_time_interval;
            }
        }

        lo::lidar_time_interval = lidar_mean_scantime_;

        // LOG(INFO) << "recompute lidar end time: " << std::setprecision(14) << lidar_end_time_;
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        LOG(INFO) << "sync failed: " << std::setprecision(14) << last_timestamp_imu_ << ", " << lidar_end_time_;
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }

        measures_.imu_.push_back(imu_buffer_.front());

        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    // LOG(INFO) << "sync: " << std::setprecision(14) << measures_.lidar_begin_time_ << ", " <<
    // measures_.lidar_end_time_;

    return true;
}

void LaserMapping::MapIncremental() {
    RuntimeStats::Scope map_time(runtime_stats_, RuntimeStats::map_update);
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (detail_options_.preserve_observed_points) {
            if (mapping::NeedsObservedVoxelPoint(point_world, nearest_points_[i], filter_size_map_min_))
                points_to_add.emplace_back(point_world);
            return;
        }
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = math::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.emplace_back(point_world);  // FIXME 这并发可能有点问题
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
    last_map_update_pose_ = state_point_.GetPose();
    last_map_update_time_ = state_point_.timestamp_;
    has_map_update_ = true;
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    int cnt_pts = scan_down_body_->size();

    // Each worker writes one enum element; aggregate only after the parallel loop.
    std::vector<features::PlaneStatus> match_status(cnt_pts, features::PlaneStatus::InsufficientNeighbors);
    feature_stats_ = features::FeatureStats{};
    registration_diagnostic_ = features::RegistrationDiagnostic{};
    icp_used_ = 0;

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // LOG(INFO) << "obs from state: " << s.pos_.transpose() << ", " << s.rot_.unit_quaternion().coeffs().transpose();

    Timer::Evaluate(
        [&, this]() {
            RuntimeStats::Scope matching_time(runtime_stats_, RuntimeStats::matching);
            Mat3f R_wl = (s.rot_.matrix() * offset_R_lidar_fixed_).cast<float>();
            Vec3f t_wl = (s.rot_ * offset_t_lidar_fixed_ + s.pos_).cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                Vec3f p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                points_near.clear();
                point_selected_surf_[i] = false;
                point_selected_icp_[i] = false;
                if (!p_body.allFinite() || !point_world.getVector3fMap().allFinite()) {
                    match_status[i] = features::PlaneStatus::InvalidInput;
                    return;
                }

                /** Find the closest surfaces in the map **/
                ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                point_selected_surf_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;

                point_selected_icp_[i] = point_selected_surf_[i];

                // Check local shape before accepting a point-to-plane constraint.
                if (point_selected_surf_[i]) {
                    if (options_.plane_quality_.enabled) {
                        match_status[i] = features::FitPlane(points_near, plane_coef_[i],
                            fasterlio::ESTI_PLANE_THRESHOLD, options_.plane_quality_);
                    } else {
                        const bool fitted = math::esti_plane(plane_coef_[i], points_near,
                                                            fasterlio::ESTI_PLANE_THRESHOLD);
                        match_status[i] = fitted ? features::PlaneStatus::Accepted : features::PlaneStatus::NonPlanar;
                    }
                    point_selected_surf_[i] = match_status[i] == features::PlaneStatus::Accepted;
                    if (point_selected_surf_[i] && (!plane_coef_[i].allFinite() ||
                                                    plane_coef_[i].head<3>().norm() < 1e-6f)) {
                        point_selected_surf_[i] = false;
                        match_status[i] = features::PlaneStatus::InvalidInput;
                    }
                }

                if (point_selected_surf_[i] && options_.surface_support_.enabled &&
                    !features::HasSurfaceSupport(std::array<double, 3>{point_world.x, point_world.y, point_world.z},
                        std::array<double, 3>{plane_coef_[i][0], plane_coef_[i][1], plane_coef_[i][2]},
                        points_near, options_.surface_support_)) {
                    point_selected_surf_[i] = false;
                    point_selected_icp_[i] = false;
                    match_status[i] = features::PlaneStatus::SupportRejected;
                }

                /// 计算平面阈值
                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    if (std::isfinite(pd2) && p_body.norm() > 81 * pd2 * pd2) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    } else {
                        point_selected_surf_[i] = false;
                        match_status[i] = features::PlaneStatus::ResidualRejected;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    // Preserve geometric rejection statistics before applying the optional budget.
    std::vector<std::size_t> surface_ids, icp_ids;
    std::vector<int> surface_bins, icp_bins;
    surface_candidates_ = icp_candidates_ = 0;
    const Eigen::Matrix3d R_lw = (s.rot_.matrix() * offset_R_lidar_fixed_).transpose();
    for (int i = 0; i < cnt_pts; ++i) {
        feature_stats_.Record(match_status[i]);
        if (point_selected_surf_[i]) {
            ++surface_candidates_;
            if (options_.registration_quality_.balance) {
                surface_ids.push_back(i);
                surface_bins.push_back(features::PlaneBin(scan_down_body_->points[i].getVector3fMap().cast<double>(),
                    R_lw * plane_coef_[i].head<3>().cast<double>()));
            }
        }
        if (options_.enable_icp_part_ && point_selected_icp_[i]) {
            ++icp_candidates_;
            if (options_.registration_quality_.balance) {
                icp_ids.push_back(i);
                const Eigen::Vector3d point = scan_down_body_->points[i].getVector3fMap().cast<double>();
                icp_bins.push_back(point_selected_surf_[i]
                    ? features::PlaneBin(point, R_lw * plane_coef_[i].head<3>().cast<double>())
                    : 24 + features::AzimuthBin(point));
            }
        }
    }
    if (options_.registration_quality_.balance) {
        // Point ICP has a separate budget; non-planar edge matches use azimuth-only buckets.
        std::fill(point_selected_surf_.begin(), point_selected_surf_.end(), false);
        std::fill(point_selected_icp_.begin(), point_selected_icp_.end(), false);
        for (const auto j : features::BalancedIndices(surface_bins, options_.registration_quality_.max_constraints))
            point_selected_surf_[surface_ids[j]] = true;
        for (const auto j : features::BalancedIndices(icp_bins, options_.registration_quality_.max_constraints))
            point_selected_icp_[icp_ids[j]] = true;
    }

    effect_feat_surf_ = 0;
    effect_feat_icp_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_surf_] = plane_coef_[i];
            corr_pts_[effect_feat_surf_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_surf_][3] = residuals_[i];

            const double weight = options_.robust_plane_weight_
                ? features::HuberWeight(residuals_[i], options_.plane_huber_delta_) : 1.0;
            feature_stats_.AddAccepted(scan_down_body_->points[i].getVector3fMap().cast<double>(),
                                      plane_coef_[i].head<3>().cast<double>(), weight);

            effect_feat_surf_++;
        }

        if (point_selected_icp_[i]) {
            effect_feat_icp_++;
        }
    }

    corr_pts_.resize(effect_feat_surf_);
    corr_norm_.resize(effect_feat_surf_);

    if (effect_feat_surf_ < 20) {
        obs.valid_ = false;
        LOG(WARNING) << "No enough effective surface points: " << effect_feat_surf_ << ", icp: " << effect_feat_icp_
                     << ", required: " << 20;
        return;
    }

    index.resize(effect_feat_surf_);
    const Mat3f off_R = offset_R_lidar_fixed_.cast<float>();
    const Vec3f off_t = offset_t_lidar_fixed_.cast<float>();
    const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

    /// 点面ICP部分
    obs.HTH_.setZero();
    obs.HTr_.setZero();

    std::vector<Mat6d> JTJ(effect_feat_surf_);
    std::vector<Vec6d> JTr(effect_feat_surf_);

    std::vector<double> res_sq(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
        Vec3f point_this_be = corr_pts_[i].head<3>();
        Vec3f point_this = off_R * point_this_be + off_t;
        Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        Vec3f norm_vec = corr_norm_[i].head<3>();

        /*** calculate the Measurement Jacobian matrix H ***/
        Vec3f C(Rt * norm_vec);
        Vec3f A(point_crossmat * C);

        Eigen::Matrix<double, 1, ESKF::pose_obs_dim_> J;
        J.setZero();
        J << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];

        float res = -corr_pts_[i][3];

        // IRLS information weight: apply the SAME factor to HTH and HTr.
        // Keep res_sq unweighted for the existing ESKF rejection diagnostics.
        const double w = options_.robust_plane_weight_
            ? features::HuberWeight(res, options_.plane_huber_delta_) : 1.0;

        JTJ[i] = (J.transpose() * J).eval() * w;
        JTr[i] = J.transpose() * res * w;

        res_sq[i] = res * res;
    });

    for (int i = 0; i < index.size(); ++i) {
        obs.HTH_ += JTJ[i] * options_.plane_icp_weight_;
        obs.HTr_ += JTr[i] * options_.plane_icp_weight_;
    }

    if (options_.registration_quality_.diagnostics) {
        // Exclude the IMU prior and point-to-point ICP, which can hide plane degeneracy.
        Mat6d plane_information = Mat6d::Zero();
        for (const auto& term : JTJ) plane_information += term;
        registration_diagnostic_ = features::AnalyzePlaneInformation(
            plane_information, feature_stats_.weight_sum, options_.registration_quality_);
    }

    if (!res_sq.empty()) {
        std::sort(res_sq.begin(), res_sq.end());
        obs.lidar_residual_mean_ = res_sq[res_sq.size() / 2];
        obs.lidar_residual_max_ = res_sq[res_sq.size() - 1];
        // LOG(INFO) << "residual mean: " << obs.lidar_residual_mean_ << ", max: " << obs.lidar_residual_max_
        //           << ", 85%: " << res_sq[res_sq.size() * 0.85];
    }

    /// 点到点ICP部分

    if (options_.enable_icp_part_) {
        JTJ.resize(cnt_pts);
        JTr.resize(cnt_pts);

        std::vector<size_t> index(cnt_pts);
        for (size_t i = 0; i < index.size(); ++i) {
            index[i] = i;
        }

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
            if (point_selected_icp_[i] == false) {
                return;
            }

            /// TODO: 外参
            Vec3d q = scan_down_body_->points[i].getVector3fMap().cast<double>();
            Vec3d qs = scan_down_world_->points[i].getVector3fMap().cast<double>();

            Eigen::Matrix<double, 3, ESKF::pose_obs_dim_> J;
            J.setZero();

            /// translation 部分
            J.block<3, 3>(0, 0) = Mat3d::Identity();

            /// rotation 部分
            J.block<3, 3>(0, 3) = -(s.rot_.matrix() * offset_R_lidar_fixed_) * SO3::hat(q);

            Vec3d e = qs - nearest_points_[i][0].getVector3fMap().cast<double>();

            if (e.norm() > 0.5) {
                point_selected_icp_[i] = false;
                return;
            }

            JTJ[i] = J.transpose() * J;
            JTr[i] = -J.transpose() * e;
        });

        for (int i = 0; i < cnt_pts; ++i) {
            if (point_selected_icp_[i] == false) {
                continue;
            }
            obs.HTH_ += JTJ[i] * options_.icp_weight_;
            obs.HTr_ += JTr[i] * options_.icp_weight_;
            ++icp_used_;
        }
    }
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res, std::vector<SE3>* exported_poses) {
    if (res < 0.0f) {
        res = static_cast<float>(detail_options_.export_resolution);
    }
    if (use_voxel && (!std::isfinite(res) || res <= 0.0f)) {
        throw std::invalid_argument("map export resolution must be finite and positive");
    }
    LOG(INFO) << "[map_detail] export voxel=" << use_voxel << " resolution_m=" << res;
    CloudPtr global_map(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    const auto keyframes = all_keyframes_;
    const auto poses = Keyframe::SnapshotPoses(keyframes, use_lio_pose);
    if (exported_poses) *exported_poses = poses;
    if (use_voxel && detail_options_.preserve_observed_points) {
        // Voxelize once in the final world frame. Do not average each keyframe, then average again.
        mapping::CloudObservedSampler<PointType> sampler(res);
        for (size_t i = 0; i < keyframes.size(); ++i) {
            PointCloudType transformed;
            pcl::transformPointCloud(*keyframes[i]->GetCloud(), transformed, poses[i].matrix());
            for (const auto& point : transformed.points) sampler.Add(point);
        }
        global_map->points = sampler.ReleasePoints();
        global_map->height = 1;
        global_map->width = global_map->size();
        global_map->is_dense = false;
        LOG(INFO) << "[map_detail] observed-point global map: " << global_map->size()
                  << ", invalid points skipped: " << sampler.Invalid();
        return global_map;
    }
    for (size_t i = 0; i < keyframes.size(); ++i) {
        const auto& kf = keyframes[i];
        CloudPtr cloud = kf->GetCloud();

        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);

        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        pcl::transformPointCloud(*cloud_filter, *cloud_trans, poses[i].matrix());

        *global_map += *cloud_trans;

        LOG(INFO) << "kf " << kf->GetID() << ", pose: " << poses[i].translation().transpose();
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    LOG(INFO) << "global map: " << global_map_filtered->size();

    return global_map_filtered;
}

void LaserMapping::SaveMap() {
    /// 保存地图
    auto global_map = GetGlobalMap(true);

    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);

    LOG(INFO) << "lio map is saved to ./data/lio.pcd";
}

CloudPtr LaserMapping::GetRecentCloud() {
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

CloudPtr LaserMapping::GetProjCloud() {
    CloudPtr cloud(new PointCloudType(*scan_undistort_));
    ProjectKFs(cloud);
    return cloud;
}

}  // namespace lightning

