#pragma once

// Geometry-only helpers: intentionally independent of ROS and PCL for regression tests.
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>

namespace lightning::features {

enum class PlaneStatus : std::size_t {
    Accepted, InsufficientNeighbors, InvalidInput, Degenerate, NonPlanar, ResidualRejected, SupportRejected, Count
};

struct PlaneQualityOptions {
    bool enabled = true;
    double min_spread = 0.03;          // metres; sqrt(second covariance eigenvalue)
    double min_tangent_ratio = 0.01;   // lambda_middle / lambda_largest
    double max_normal_ratio = 0.10;    // lambda_smallest / lambda_middle

    bool Valid() const {
        return std::isfinite(min_spread) && min_spread > 0.0 &&
               std::isfinite(min_tangent_ratio) && min_tangent_ratio > 0.0 && min_tangent_ratio <= 1.0 &&
               std::isfinite(max_normal_ratio) && max_normal_ratio > 0.0 && max_normal_ratio < 1.0;
    }
};

// Fit n.dot(x) + d = 0. A centred covariance can represent planes through the origin.
// PointRange elements expose x/y/z fields (PCL points or lightweight test points).
template <typename PointRange>
PlaneStatus FitPlane(const PointRange& points, Eigen::Vector4f& plane,
                     double max_distance, const PlaneQualityOptions& options) {
    plane.setZero();
    if (!options.Valid() || !std::isfinite(max_distance) || max_distance <= 0.0) {
        return PlaneStatus::InvalidInput;
    }
    if (points.size() < 3) {
        return PlaneStatus::InsufficientNeighbors;
    }
    const auto xyz = [](const auto& p) { return Eigen::Vector3d(p.x, p.y, p.z); };
    const Eigen::Vector3d anchor = xyz(points.front());
    if (!anchor.allFinite()) {
        return PlaneStatus::InvalidInput;
    }
    // Shift before summing to reduce cancellation at large map coordinates.
    Eigen::Vector3d mean_offset = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        const Eigen::Vector3d v = xyz(p);
        if (!v.allFinite()) {
            return PlaneStatus::InvalidInput;
        }
        mean_offset += v - anchor;
    }
    mean_offset /= static_cast<double>(points.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        const Eigen::Vector3d v = (xyz(p) - anchor) - mean_offset;
        covariance.noalias() += v * v.transpose();
    }
    covariance /= static_cast<double>(points.size());
    if (!covariance.allFinite()) {
        return PlaneStatus::InvalidInput;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
        return PlaneStatus::InvalidInput;
    }
    const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    if (eigenvalues[1] < options.min_spread * options.min_spread || eigenvalues[2] <= 0.0 ||
        eigenvalues[1] / eigenvalues[2] < options.min_tangent_ratio) {
        return PlaneStatus::Degenerate;
    }
    if (eigenvalues[0] / eigenvalues[1] > options.max_normal_ratio) {
        return PlaneStatus::NonPlanar;
    }
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (!normal.allFinite() || normal.norm() < 1e-12) {
        return PlaneStatus::InvalidInput;
    }
    normal.normalize();
    // Deterministic sign; residual/Jacobian sign changes together and does not change the update.
    Eigen::Index axis;
    normal.cwiseAbs().maxCoeff(&axis);
    if (normal[axis] < 0.0) {
        normal = -normal;
    }
    const Eigen::Vector3d mean = anchor + mean_offset;
    for (const auto& p : points) {
        if (std::abs(normal.dot((xyz(p) - anchor) - mean_offset)) > max_distance) {
            return PlaneStatus::ResidualRejected;
        }
    }
    plane.head<3>() = normal.cast<float>();
    plane[3] = static_cast<float>(-normal.dot(mean));
    if (!plane.allFinite()) {
        plane.setZero();
        return PlaneStatus::InvalidInput;
    }
    return PlaneStatus::Accepted;
}

inline double HuberWeight(double residual, double delta) {
    if (!std::isfinite(residual) || !std::isfinite(delta) || delta <= 0.0) {
        return 0.0;
    }
    const double magnitude = std::abs(residual);
    return magnitude <= delta ? 1.0 : delta / magnitude;
}

struct FeatureStats {
    std::array<std::size_t, static_cast<std::size_t>(PlaneStatus::Count)> status{};
    std::array<std::size_t, 8> azimuth{};       // lidar frame; [-pi, pi), 45-degree bins
    std::array<std::size_t, 3> normal_axis{};  // world frame; dominant absolute X/Y/Z
    std::array<std::size_t, 3> range{};        // lidar frame; [0,5), [5,15), [15,inf) metres
    std::size_t downweighted = 0;
    double weight_sum = 0.0;

    void Record(PlaneStatus result) { ++status[static_cast<std::size_t>(result)]; }

    void AddAccepted(const Eigen::Vector3d& point, const Eigen::Vector3d& normal, double weight) {
        constexpr double pi = 3.14159265358979323846;
        const double angle = std::atan2(point.y(), point.x());
        const int bin = std::clamp(static_cast<int>((angle + pi) * 4.0 / pi), 0, 7);
        ++azimuth[bin];
        Eigen::Index axis;
        normal.cwiseAbs().maxCoeff(&axis);
        ++normal_axis[axis];
        const double distance = point.norm();
        ++range[distance < 5.0 ? 0 : (distance < 15.0 ? 1 : 2)];
        weight_sum += weight;
        downweighted += weight < 1.0;
    }

    std::string Summary() const {
        std::ostringstream out;
        std::size_t candidates = 0;
        for (auto n : status) candidates += n;
        const auto accepted = status[static_cast<std::size_t>(PlaneStatus::Accepted)];
        out << "candidates=" << candidates << " accepted=" << accepted
            << " no_neighbors=" << status[1] << " invalid=" << status[2]
            << " degenerate=" << status[3] << " nonplanar=" << status[4]
            << " residual_rejected=" << status[5] << " support_rejected=" << status[6] << " huber_downweighted=" << downweighted
            << " mean_weight=" << (accepted ? weight_sum / accepted : 0.0);
        const auto bins = [&](const char* name, const auto& values) {
            out << ' ' << name << "=[";
            for (std::size_t i = 0; i < values.size(); ++i) out << (i ? "," : "") << values[i];
            out << ']';
        };
        bins("azimuth", azimuth);
        bins("normal_xyz", normal_axis);
        bins("range", range);
        return out.str();
    }
};

}  // namespace lightning::features
