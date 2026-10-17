#include "core/lio/feature_quality.h"
#include <Eigen/Geometry>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
struct Point { double x, y, z; };
using Points = std::vector<Point>;
using lightning::features::FitPlane;
using lightning::features::PlaneStatus;
using lightning::features::PlaneQualityOptions;
using lightning::features::HuberWeight;

void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void TestPlanes() {
    PlaneQualityOptions options;
    Eigen::Vector4f plane;
    const Points ground{{-1,-1,0}, {1,-1,0}, {1,1,0}, {-1,1,0}, {0,0,0}};
    Check(FitPlane(ground, plane, 0.1, options) == PlaneStatus::Accepted, "origin plane rejected");
    Check(std::abs(plane[2] - 1.0f) < 1e-6f && std::abs(plane[3]) < 1e-6f, "wrong ground plane");

    // Rigid transform should preserve plane validity and fitted geometry.
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    const Eigen::Vector3d translation(1000, -2000, 300);
    Points moved;
    for (const auto& p : ground) {
        const Eigen::Vector3d q = rotation * Eigen::Vector3d(p.x,p.y,p.z) + translation;
        moved.push_back({q.x(),q.y(),q.z()});
    }
    Check(FitPlane(moved, plane, 0.1, options) == PlaneStatus::Accepted, "transformed plane rejected");
    const Eigen::Vector3d normal = plane.head<3>().cast<double>();
    Check(std::abs(std::abs(normal.dot(rotation.col(2))) - 1.0) < 1e-6, "normal not rotation invariant");
    for (const auto& p : moved) {
        Check(std::abs(normal.dot(Eigen::Vector3d(p.x,p.y,p.z)) + plane[3]) < 0.001,
              "incorrect transformed offset");
    }

    const Points vertical{{0,-1,-1},{0,1,-1},{0,1,1},{0,-1,1},{0,0,0}};
    Check(FitPlane(vertical, plane, 0.1, options) == PlaneStatus::Accepted, "vertical origin plane rejected");
    Check(std::abs(plane[0] - 1.0f) < 1e-6f, "wrong vertical normal");
    const Points line{{0,0,0},{1,0,0},{2,0,0},{3,0,0},{4,0,0}};
    Check(FitPlane(line, plane, 0.1, options) == PlaneStatus::Degenerate, "collinear points accepted");
    const Points repeated(5, Point{1,2,3});
    Check(FitPlane(repeated, plane, 0.1, options) == PlaneStatus::Degenerate, "duplicates accepted");
    const Points narrow{{-2,0,0},{-1,.001,0},{0,-.001,0},{1,.001,0},{2,0,0}};
    Check(FitPlane(narrow, plane, 0.1, options) == PlaneStatus::Degenerate, "near-line accepted");
    const Points elongated{{-10,-.1,0},{10,-.1,0},{10,.1,0},{-10,.1,0},{0,0,0}};
    Check(FitPlane(elongated, plane, .1, options) == PlaneStatus::Degenerate, "tangent ratio ignored");
    auto permissive = options;
    permissive.min_tangent_ratio = .000001;
    Check(FitPlane(elongated, plane, .1, permissive) == PlaneStatus::Accepted, "tangent ratio not configurable");
    Points tiny = ground;
    for (auto& p : tiny) { p.x *= .01; p.y *= .01; }
    Check(FitPlane(tiny, plane, 0.1, options) == PlaneStatus::Degenerate, "tiny patch accepted");
    const Points volume{{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
    Check(FitPlane(volume, plane, 2.0, options) == PlaneStatus::NonPlanar, "volumetric neighbors accepted");
    Points warped = ground;
    warped.back().z = .1;
    Check(FitPlane(warped, plane, .01, options) == PlaneStatus::ResidualRejected, "distance threshold bypassed");
    Points nonfinite = ground;
    nonfinite[0].x = std::numeric_limits<double>::quiet_NaN();
    Check(FitPlane(nonfinite, plane, .1, options) == PlaneStatus::InvalidInput, "NaN accepted");
    nonfinite[0].x = std::numeric_limits<double>::infinity();
    Check(FitPlane(nonfinite, plane, .1, options) == PlaneStatus::InvalidInput, "Inf accepted");
    Check(FitPlane(Points{}, plane, .1, options) == PlaneStatus::InsufficientNeighbors, "empty input accepted");
    Check(FitPlane(Points{{0,0,0},{1,0,0}}, plane, .1, options) == PlaneStatus::InsufficientNeighbors,
          "two neighbors accepted");
    options.min_tangent_ratio = -1;
    Check(FitPlane(ground, plane, .1, options) == PlaneStatus::InvalidInput, "bad options accepted");
    Check(!options.Valid(), "invalid options validated");
    options = PlaneQualityOptions{};
    Check(FitPlane(ground, plane, 0, options) == PlaneStatus::InvalidInput, "zero residual tolerance accepted");
}

void TestWeights() {
    Check(HuberWeight(0.0, .1) == 1.0, "zero residual downweighted");
    Check(HuberWeight(.1, .1) == 1.0, "Huber boundary incorrect");
    Check(std::abs(HuberWeight(.5, .1) - .2) < 1e-12, "outlier weight incorrect");
    Check(HuberWeight(-.5, .1) == HuberWeight(.5, .1), "weight not symmetric");
    Check(HuberWeight(1, 0) == 0 && HuberWeight(std::numeric_limits<double>::quiet_NaN(), .1) == 0,
          "invalid weight inputs not rejected");
    // One IRLS translation update: nine inliers and one bad correspondence.
    double information = 0, rhs = 0, ordinary_rhs = 0;
    for (double residual : {.01,.01,.01,.01,.01,.01,.01,.01,.01,.5}) {
        const double weight = HuberWeight(residual, .1);
        information += weight;
        rhs += weight * residual;
        ordinary_rhs += residual;
    }
    Check(rhs / information < ordinary_rhs / 10.0, "robust update failed to reduce outlier influence");
    Check(rhs / information > .01, "outlier was hard-deleted rather than downweighted");
}

void TestDistribution() {
    lightning::features::FeatureStats stats;
    Check(stats.Summary().find("mean_weight=0") != std::string::npos, "empty statistics divide by zero");
    constexpr double pi = 3.14159265358979323846;
    for (int i = 0; i < 8; ++i) {
        const double angle = -pi + (i + .5) * pi / 4.0;
        const double r = i < 3 ? 1 : (i < 6 ? 10 : 20);
        Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        normal[i % 3] = i % 2 ? -1 : 1;
        stats.Record(PlaneStatus::Accepted);
        stats.AddAccepted(Eigen::Vector3d(r*std::cos(angle),r*std::sin(angle),0), normal, i ? 1.0 : .5);
    }
    for (auto bin : stats.azimuth) Check(bin == 1, "azimuth coverage bins incorrect");
    Check(stats.normal_axis == std::array<std::size_t,3>{3,3,2}, "normal bins depend on sign");
    Check(stats.range == std::array<std::size_t,3>{3,3,2}, "range bins incorrect");
    Check(stats.downweighted == 1 && stats.weight_sum == 7.5, "weight statistics incorrect");
    stats.Record(PlaneStatus::Degenerate);
    Check(stats.Summary().find("candidates=9 accepted=8") != std::string::npos, "rejection statistics incorrect");
}
}

int main() {
    try {
        TestPlanes();
        TestWeights();
        TestDistribution();
        std::cout << "feature_quality: geometry, robust weights and distribution regressions passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
