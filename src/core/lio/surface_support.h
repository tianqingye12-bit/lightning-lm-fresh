#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace lightning::features {
struct SurfaceSupportOptions {
    bool enabled = false;
    double max_neighbor_distance = .6;  // metres
    double max_plane_residual = .15;    // metres, independent of lidar range
    double max_tangent_gap = .10;       // permitted extension beyond neighbor patch radius
    bool Valid() const {
        return std::isfinite(max_neighbor_distance) && max_neighbor_distance > 0 &&
               std::isfinite(max_plane_residual) && max_plane_residual > 0 &&
               std::isfinite(max_tangent_gap) && max_tangent_gap >= 0;
    }
};

// Centred calculation avoids subtracting large world-frame plane offsets.
// This is a conservative radius bound, not a convex-hull or visibility test.
template<class PointRange>
bool HasSurfaceSupport(const std::array<double, 3>& query, const std::array<double, 3>& normal,
                       const PointRange& neighbors, const SurfaceSupportOptions& options) {
    if (!options.enabled) return true;
    if (!options.Valid() || neighbors.size() < 3) return false;
    const auto norm2 = [](const auto& v) { return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]; };
    const auto dot = [](const auto& a, const auto& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; };
    for (int j = 0; j < 3; ++j) if (!std::isfinite(query[j]) || !std::isfinite(normal[j])) return false;
    const double n2 = norm2(normal);
    if (!std::isfinite(n2) || n2 < 1e-12) return false;
    const double inv_n = 1.0 / std::sqrt(n2);
    const std::array<double, 3> n{normal[0]*inv_n, normal[1]*inv_n, normal[2]*inv_n};
    const std::array<double, 3> anchor{neighbors.front().x, neighbors.front().y, neighbors.front().z};
    std::array<double, 3> center{0,0,0};
    double nearest2 = std::numeric_limits<double>::infinity();
    for (const auto& p : neighbors) {
        const std::array<double, 3> v{p.x-anchor[0], p.y-anchor[1], p.z-anchor[2]};
        const std::array<double, 3> dq{p.x-query[0], p.y-query[1], p.z-query[2]};
        for (int j = 0; j < 3; ++j) {
            if (!std::isfinite(v[j])) return false;
            center[j] += v[j] / static_cast<double>(neighbors.size());
        }
        nearest2 = std::min(nearest2, norm2(dq));
    }
    if (!std::isfinite(nearest2) || std::sqrt(nearest2) > options.max_neighbor_distance) return false;
    const std::array<double, 3> q{(query[0]-anchor[0])-center[0], (query[1]-anchor[1])-center[1],
                                (query[2]-anchor[2])-center[2]};
    const double residual = dot(n, q);
    if (!std::isfinite(residual) || std::abs(residual) > options.max_plane_residual) return false;
    double radius2 = 0;
    for (const auto& p : neighbors) {
        const std::array<double, 3> v{(p.x-anchor[0])-center[0], (p.y-anchor[1])-center[1],
                                    (p.z-anchor[2])-center[2]};
        const double height = dot(n, v);
        radius2 = std::max(radius2, std::max(0.0, norm2(v)-height*height));
    }
    const double tangent2 = std::max(0.0, norm2(q)-residual*residual);
    return std::isfinite(tangent2) && std::isfinite(radius2) &&
           std::sqrt(tangent2) <= std::sqrt(radius2) + options.max_tangent_gap;
}
}  // namespace lightning::features
