#pragma once

// Compact scan-level history. Geometry and CSV export are independent of ROS/PCL.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace lightning::trajectory {

struct Options {
    bool enabled = false;
    int max_records = 200000;
    bool Valid() const { return max_records >= 100 && max_records <= 2000000; }
};

enum class Status { Accepted, Bootstrap, Initializing, ImuGap, PropagationFault,
                    NoPoints, ClockMismatch, Skipped, TooFewPoints, Rejected };
inline const char* Name(Status status) {
    switch (status) {
        case Status::Accepted: return "accepted";
        case Status::Bootstrap: return "bootstrap";
        case Status::Initializing: return "initializing";
        case Status::ImuGap: return "imu_gap";
        case Status::PropagationFault: return "propagation_fault";
        case Status::NoPoints: return "no_points";
        case Status::ClockMismatch: return "clock_mismatch";
        case Status::Skipped: return "skipped_registration";
        case Status::TooFewPoints: return "too_few_points";
        case Status::Rejected: return "registration_rejected";
    }
    return "unknown";
}

inline double Missing() { return std::numeric_limits<double>::quiet_NaN(); }
struct PointTiming {
    std::size_t count = 0, nonfinite = 0, negative = 0;
    double minimum_s = Missing(), maximum_s = Missing(), last_s = Missing();
};

// Inspect PREPROCESSED offsets: the existing internal PointType.time is milliseconds.
// Do not infer raw sensor units, reorder points or change the synchronization policy.
template <class PointRange> PointTiming InspectPointTiming(const PointRange& points) {
    PointTiming out;
    out.count = points.size();
    for (const auto& point : points) {
        const double value = point.time / 1000.0;
        out.last_s = value;
        if (!std::isfinite(value)) { ++out.nonfinite; continue; }
        if (value < 0) ++out.negative;
        if (!std::isfinite(out.minimum_s)) out.minimum_s = out.maximum_s = value;
        else { out.minimum_s = std::min(out.minimum_s, value); out.maximum_s = std::max(out.maximum_s, value); }
    }
    return out;
}

inline bool Rigid(const Eigen::Isometry3d& pose) {
    return pose.matrix().allFinite() &&
        (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-8 &&
        (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-5 &&
        std::abs(pose.linear().determinant() - 1) < 1e-5;
}
using PoseValues = std::array<double, 7>;
inline PoseValues Pack(const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond q(pose.linear());
    return {pose.translation().x(), pose.translation().y(), pose.translation().z(), q.x(), q.y(), q.z(), q.w()};
}
inline Eigen::Isometry3d Unpack(const PoseValues& values) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
    pose.linear() = Eigen::Quaterniond(values[6], values[3], values[4], values[5]).normalized().toRotationMatrix();
    return pose;
}

struct Anchor {
    std::size_t id;
    double timestamp;
    Eigen::Isometry3d raw, optimized;
};
struct Correction {
    Eigen::Isometry3d pose;
    const char* mode;
};

inline Correction Correct(double timestamp, const Eigen::Isometry3d& raw, const std::vector<Anchor>& anchors) {
    const auto upper = std::upper_bound(anchors.begin(), anchors.end(), timestamp,
        [](double t, const Anchor& a) { return t < a.timestamp; });
    if (upper == anchors.begin()) return {anchors.front().optimized * anchors.front().raw.inverse() * raw, "head_hold"};
    const auto& a = *(upper - 1);
    const Eigen::Isometry3d ca = a.optimized * a.raw.inverse();
    if (timestamp == a.timestamp) return {ca * raw, "anchor"};
    if (upper == anchors.end()) return {ca * raw, "tail_hold"};
    const auto& b = *upper;
    const double alpha = (timestamp - a.timestamp) / (b.timestamp - a.timestamp);
    const Eigen::Isometry3d cb = b.optimized * b.raw.inverse();
    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
    correction.translation() = (1-alpha) * ca.translation() + alpha * cb.translation();
    correction.linear() = Eigen::Quaterniond(ca.linear()).slerp(alpha, Eigen::Quaterniond(cb.linear())).normalized().toRotationMatrix();
    return {correction * raw, "interpolated_correction"};
}

struct Scan {
    double begin = 0, timestamp = 0, state_timestamp = 0;
    PoseValues raw{0, 0, 0, 0, 0, 0, 1};
    bool pose_valid = false;
    Status status = Status::NoPoints;
    PointTiming points;
    std::size_t imu_count = 0;
    double imu_first = Missing(), imu_last = Missing(), imu_latest = Missing(), propagation_begin = Missing();
    bool imu_gap = false;
};

class Recorder {
 public:
    void Configure(const Options& options) {
        if (!options.Valid()) throw std::invalid_argument("invalid scan trajectory options");
        options_ = options;
    }
    bool Enabled() const { return options_.enabled; }
    const std::vector<Scan>& Scans() const { return scans_; }
    std::size_t Faults() const { return faults_; }
    void Add(const Scan& scan) {
        if (!Enabled()) return;
        if (scans_.size() >= static_cast<std::size_t>(options_.max_records) ||
            !std::isfinite(scan.begin) || !std::isfinite(scan.timestamp) || scan.timestamp < scan.begin ||
            (!scans_.empty() && scan.timestamp <= scans_.back().timestamp)) {
            ++faults_;  // Never silently publish an apparently complete partial trajectory.
            return;
        }
        scans_.push_back(scan);
    }
    void Export(const std::filesystem::path& directory, const std::vector<Anchor>& anchors) const {
        if (!Enabled()) return;
        if (anchors.empty()) throw std::runtime_error("dense trajectory needs keyframe anchors");
        for (std::size_t i = 0; i < anchors.size(); ++i) {
            if (!std::isfinite(anchors[i].timestamp) || !Rigid(anchors[i].raw) || !Rigid(anchors[i].optimized) ||
                (i && anchors[i].timestamp <= anchors[i-1].timestamp))
                throw std::runtime_error("invalid trajectory anchor snapshot");
        }
        const auto open = [&](const char* name) {
            std::ofstream stream(directory / name);
            stream.exceptions(std::ios::failbit | std::ios::badbit);
            stream << std::setprecision(18);
            return stream;
        };
        const auto write_pose = [](std::ostream& stream, const PoseValues& values) {
            for (const auto value : values) stream << ',' << value;
        };
        auto anchor_file = open("trajectory_anchors.csv");
        anchor_file << "id,timestamp,tx,ty,tz,qx,qy,qz,qw,raw_tx,raw_ty,raw_tz,raw_qx,raw_qy,raw_qz,raw_qw\n";
        for (const auto& anchor : anchors) {
            anchor_file << anchor.id << ',' << anchor.timestamp;
            write_pose(anchor_file, Pack(anchor.optimized)); write_pose(anchor_file, Pack(anchor.raw));
            anchor_file << '\n';
        }
        anchor_file.close();
        auto file = open("scan_trajectory.csv");
        file << "scan_id,begin_time,timestamp,state_timestamp,status,usable,tx,ty,tz,qx,qy,qz,qw,"
                "raw_tx,raw_ty,raw_tz,raw_qx,raw_qy,raw_qz,raw_qw,correction_mode,point_count,"
                "point_min_s,point_max_s,point_last_s,point_nonfinite,point_negative,scan_duration_s,"
                "point_end_difference_s,imu_count,imu_first,imu_last,imu_latest,propagation_begin,imu_gap,clock_error_s\n";
        std::size_t usable_count = 0;
        for (std::size_t i = 0; i < scans_.size(); ++i) {
            const auto& s = scans_[i];
            const auto raw = Unpack(s.raw);
            const auto corrected = Correct(s.timestamp, raw, anchors);
            const bool usable = s.pose_valid && Rigid(raw) &&
                (s.status == Status::Accepted || s.status == Status::Bootstrap) && !s.imu_gap &&
                s.points.nonfinite == 0 && s.points.negative == 0 &&
                std::isfinite(s.state_timestamp) && std::abs(s.state_timestamp-s.timestamp) <= 1e-5;
            usable_count += usable;
            file << i << ',' << s.begin << ',' << s.timestamp << ',' << s.state_timestamp << ',' << Name(s.status) << ',' << usable;
            write_pose(file, Pack(corrected.pose)); write_pose(file, s.raw);
            file << ',' << corrected.mode << ',' << s.points.count << ',' << s.points.minimum_s << ',' << s.points.maximum_s
                 << ',' << s.points.last_s << ',' << s.points.nonfinite << ',' << s.points.negative
                 << ',' << s.timestamp-s.begin << ',' << s.points.maximum_s-(s.timestamp-s.begin)
                 << ',' << s.imu_count << ',' << s.imu_first << ',' << s.imu_last << ',' << s.imu_latest
                 << ',' << s.propagation_begin << ',' << s.imu_gap << ',' << s.state_timestamp-s.timestamp << '\n';
        }
        file.close();
        auto metadata = open("trajectory.yaml");
        metadata << "format_version: 1\npose_frame: imu\ncoordinate_frame: exported_map\n"
                 << "timestamp_reference: lidar_end\ncorrection_method: interpolated_keyframe_correction\n"
                 << "complete: " << (faults_ == 0 ? "true" : "false") << "\nrows: " << scans_.size()
                 << "\nusable_rows: " << usable_count << "\nrecording_faults: " << faults_
                 << "\nkeyframes: " << anchors.size() << '\n';
        metadata.close();
    }
 private:
    Options options_;
    std::vector<Scan> scans_;
    std::size_t faults_ = 0;
};

}  // namespace lightning::trajectory
