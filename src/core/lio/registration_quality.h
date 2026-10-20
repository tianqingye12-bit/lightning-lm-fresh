#pragma once

// Bounded correspondence selection and plane-only observability diagnostics.
// No ROS/PCL dependency. Diagnostics never modify the filter state or covariance.
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

namespace lightning::features {

struct RegistrationQualityOptions {
    bool balance = false;
    int max_constraints = 3000;  // Separate upper bounds for plane and optional point ICP.
    bool diagnostics = false;
    double rotation_scale_m = 5.0;  // Normalized pose coordinates: [translation, L * rotation].
    double relative_eigenvalue = 1e-3;

    bool Valid() const {
        return max_constraints >= 20 && max_constraints <= 1000000 &&
               std::isfinite(rotation_scale_m) && rotation_scale_m > 0 &&
               std::isfinite(relative_eigenvalue) && relative_eigenvalue > 0 && relative_eigenvalue < 1;
    }
};

inline int AzimuthBin(const Eigen::Vector3d& point) {
    constexpr double pi = 3.14159265358979323846;
    const double angle = std::atan2(point.y(), point.x());
    return std::clamp(static_cast<int>((angle + pi) * 8 / (2 * pi)), 0, 7);
}

// Both arguments are in the LiDAR frame. Normal sign cannot change the bucket.
inline int PlaneBin(const Eigen::Vector3d& point, const Eigen::Vector3d& normal) {
    Eigen::Index axis;
    normal.cwiseAbs().maxCoeff(&axis);
    return static_cast<int>(axis) * 8 + AzimuthBin(point);
}

// bins is compact: one entry per eligible correspondence, never per rejected point.
// Water-fill available buckets without replacement, then sample throughout each bucket.
inline std::vector<std::size_t> BalancedIndices(const std::vector<int>& bins, std::size_t budget) {
    const auto count = bins.size();
    if (count <= budget) {
        std::vector<std::size_t> all(count);
        std::iota(all.begin(), all.end(), 0);
        return all;
    }
    // 24 plane buckets, plus 8 optional azimuth buckets for non-planar point ICP.
    std::array<std::vector<std::size_t>, 32> buckets;
    for (std::size_t i = 0; i < count; ++i) buckets.at(bins[i]).push_back(i);
    std::array<std::size_t, 32> quota{};
    std::size_t remaining = budget;
    while (remaining) {
        for (std::size_t b = 0; b < buckets.size() && remaining; ++b) {
            if (quota[b] < buckets[b].size()) { ++quota[b]; --remaining; }
        }
    }
    std::vector<std::size_t> selected;
    selected.reserve(budget);
    for (std::size_t b = 0; b < buckets.size(); ++b) {
        for (std::size_t j = 0; j < quota[b]; ++j) {
            const auto k = static_cast<std::size_t>((j + .5) * buckets[b].size() / quota[b]);
            selected.push_back(buckets[b][k]);
        }
    }
    std::sort(selected.begin(), selected.end());
    return selected;
}

struct RegistrationDiagnostic {
    bool valid = false;
    int rank = 0;
    double min_max_ratio = 0;
    Eigen::Matrix<double, 6, 1> eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
    // Weakest normalized direction; may mix translation and rotation. Sign is arbitrary.
    Eigen::Matrix<double, 6, 1> weakest = Eigen::Matrix<double, 6, 1>::Zero();
};

inline RegistrationDiagnostic AnalyzePlaneInformation(const Eigen::Matrix<double, 6, 6>& information,
                                                       double weight_sum,
                                                       const RegistrationQualityOptions& options) {
    RegistrationDiagnostic out;
    if (!options.Valid() || !information.allFinite() || !std::isfinite(weight_sum) || weight_sum <= 0) return out;
    Eigen::Matrix<double, 6, 1> scale = Eigen::Matrix<double, 6, 1>::Ones();
    scale.tail<3>().setConstant(1.0 / options.rotation_scale_m);
    const Eigen::Matrix<double, 6, 6> normalized =
        scale.asDiagonal() * ((information + information.transpose()).eval() * (0.5 / weight_sum)) * scale.asDiagonal();
    if (!normalized.allFinite()) return out;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(normalized);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) return out;
    const double largest = solver.eigenvalues()[5];
    if (largest <= 1e-12 || solver.eigenvalues()[0] < -1e-8 * largest) return out;
    out.eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    out.rank = (out.eigenvalues.array() > largest * options.relative_eigenvalue).count();
    out.min_max_ratio = out.eigenvalues[0] / largest;
    out.weakest = solver.eigenvectors().col(0);
    out.valid = true;
    return out;
}

}  // namespace lightning::features
