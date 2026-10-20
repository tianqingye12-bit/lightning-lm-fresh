#pragma once

// Select an observed point per voxel; never synthesize a centroid between surfaces.
// Independent of Eigen/PCL so geometry and streaming behavior can be tested directly.
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lightning::mapping {
struct ObservedVoxelKey {
    std::int64_t x, y, z;
    bool operator==(const ObservedVoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};
struct ObservedVoxelHash {
    std::size_t operator()(const ObservedVoxelKey& key) const {
        std::size_t seed = 0;
        for (auto value : {key.x, key.y, key.z})
            seed ^= std::hash<std::int64_t>{}(value) + std::size_t(0x9e3779b9) + (seed << 6) + (seed >> 2);
        return seed;
    }
};

inline bool ObservedVoxelCoordinate(double value, double leaf, std::int64_t& index, double& distance) {
    if (!std::isfinite(value) || !std::isfinite(leaf) || leaf <= 0) return false;
    const long double scaled = static_cast<long double>(value) / leaf;
    const long double cell = std::floor(scaled);
    if (!std::isfinite(cell) || cell < std::numeric_limits<std::int64_t>::min() ||
        cell > std::numeric_limits<std::int64_t>::max()) return false;
    index = static_cast<std::int64_t>(cell);
    const double offset = static_cast<double>(scaled - cell - .5L);
    distance += offset * offset;
    return true;
}
template<class Point>
bool ObservedVoxelLocation(const Point& p, double leaf, ObservedVoxelKey& key, double& distance) {
    distance = 0;
    return ObservedVoxelCoordinate(p.x, leaf, key.x, distance) &&
           ObservedVoxelCoordinate(p.y, leaf, key.y, distance) &&
           ObservedVoxelCoordinate(p.z, leaf, key.z, distance);
}

// A point in another voxel must not suppress a new contour point just because it is nearby.
template<class Point, class Range>
bool NeedsObservedVoxelPoint(const Point& point, const Range& neighbors, double leaf) {
    ObservedVoxelKey key{}; double distance = 0;
    if (!ObservedVoxelLocation(point, leaf, key, distance)) return false;
    for (const auto& neighbor : neighbors) {
        ObservedVoxelKey other{}; double other_distance = 0;
        if (ObservedVoxelLocation(neighbor, leaf, other, other_distance) && key == other &&
            other_distance <= distance + 1e-12) return false;
    }
    return true;
}

template<class Point, class Storage = std::vector<Point>>
class ObservedVoxelSampler {
 public:
    explicit ObservedVoxelSampler(double leaf) : leaf_(leaf) {
        if (!std::isfinite(leaf) || leaf <= 0) throw std::invalid_argument("invalid observed voxel resolution");
    }
    bool Add(const Point& point) {
        ObservedVoxelKey key{};
        double distance = 0;
        if (!ObservedVoxelLocation(point, leaf_, key, distance)) { ++invalid_; return false; }
        const auto it = cells_.find(key);
        if (it == cells_.end()) {
            const auto index = points_.size();
            points_.push_back(point);
            try { cells_.emplace(key, Entry{index, distance}); }
            catch (...) { points_.pop_back(); throw; }
        } else {
            auto& entry = it->second;
            const auto& old = points_[entry.index];
            // Stable geometry on ties; preserve the entire selected input record, including intensity/time.
            const bool before = point.x < old.x || (point.x == old.x &&
                (point.y < old.y || (point.y == old.y && point.z < old.z)));
            if (distance < entry.distance || (distance == entry.distance && before)) {
                points_[entry.index] = point; entry.distance = distance;
            }
        }
        return true;
    }
    const Storage& Points() const { return points_; }
    Storage ReleasePoints() {
        cells_.clear();
        return std::move(points_);
    }
    std::size_t Invalid() const { return invalid_; }
 private:
    struct Entry { std::size_t index; double distance; };
    double leaf_;
    std::size_t invalid_ = 0;
    Storage points_;
    std::unordered_map<ObservedVoxelKey, Entry, ObservedVoxelHash> cells_;
};
}  // namespace lightning::mapping
