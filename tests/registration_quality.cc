#include "core/lio/registration_quality.h"
#include <Eigen/Geometry>
#include <cassert>
#include <iostream>
#include <limits>
#include <set>

using namespace lightning::features;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec6 = Eigen::Matrix<double, 6, 1>;

Mat6 Planes(int axes) {
    Mat6 H = Mat6::Zero();
    for (int axis = 0; axis < axes; ++axis) {
        const Eigen::Vector3d n = Eigen::Vector3d::Unit(axis);
        for (int a = -4; a <= 4; ++a) for (int b = -4; b <= 4; ++b) {
            Eigen::Vector3d p = 3 * n;
            p[(axis + 1) % 3] = a; p[(axis + 2) % 3] = b;
            Vec6 J; J << n, p.cross(n);
            H += J * J.transpose();
        }
    }
    return H;
}

int main() {
    RegistrationQualityOptions opts;
    assert(opts.Valid()); assert(!opts.balance && !opts.diagnostics);
    auto invalid = opts; invalid.max_constraints = 19; assert(!invalid.Valid());
    invalid = opts; invalid.rotation_scale_m = 0; assert(!invalid.Valid());
    invalid = opts; invalid.relative_eigenvalue = 1; assert(!invalid.Valid());

    std::vector<int> bins(8000, 0);
    bins.insert(bins.end(), 500, 1); bins.insert(bins.end(), 100, 2);
    const auto selected = BalancedIndices(bins, 900);
    assert(selected.size() == 900 && std::set<std::size_t>(selected.begin(), selected.end()).size() == 900);
    std::array<int, 3> counts{};
    for (auto i : selected) ++counts[bins[i]];
    assert(counts[0] == 400 && counts[1] == 400 && counts[2] == 100);
    assert(selected.front() > 0 && selected[399] > 7900);
    assert(selected == BalancedIndices(bins, 900));
    const auto all = BalancedIndices(bins, 10000);
    assert(all.size() == bins.size() && all.front() == 0 && all.back() == bins.size()-1);
    assert(BalancedIndices({}, 900).empty() && BalancedIndices(bins, 0).empty());
    const auto with_edge = BalancedIndices({0, 0, 0, 31}, 2);
    assert(with_edge.size() == 2 && with_edge.back() == 3);

    Eigen::Vector3d p(2, -3, 1), normal(0, 1, 0);
    const Eigen::Matrix3d R = Eigen::AngleAxisd(.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    assert(PlaneBin(p, normal) == PlaneBin(p, -normal));
    assert(PlaneBin(p, normal) == PlaneBin(p, R.transpose() * (R * normal)));
    assert(AzimuthBin(Eigen::Vector3d(-1, 0, 0)) >= 0 && AzimuthBin(Eigen::Vector3d(-1, 0, 0)) < 8);

    for (int axes = 1; axes <= 3; ++axes) {
        const Mat6 H = Planes(axes);
        const auto result = AnalyzePlaneInformation(H, 81*axes, opts);
        assert(result.valid && result.rank == (axes == 1 ? 3 : axes == 2 ? 5 : 6));
        const auto duplicated = AnalyzePlaneInformation(H * 7, 81*axes*7, opts);
        assert((result.eigenvalues - duplicated.eigenvalues).norm() < 1e-10);
        Mat6 Q = Mat6::Identity(); Q.topLeftCorner<3, 3>() = R;
        const auto rotated = AnalyzePlaneInformation(Q * H * Q.transpose(), 81*axes, opts);
        assert(rotated.rank == result.rank && (rotated.eigenvalues - result.eigenvalues).norm() < 1e-10);
    }
    assert(!AnalyzePlaneInformation(Mat6::Zero(), 1, opts).valid);
    Mat6 bad = Mat6::Identity(); bad(0, 0) = std::numeric_limits<double>::quiet_NaN();
    assert(!AnalyzePlaneInformation(bad, 1, opts).valid);
    assert(!AnalyzePlaneInformation(-Mat6::Identity(), 1, opts).valid);
    assert(!AnalyzePlaneInformation(Mat6::Identity(), 0, opts).valid);
    std::cout << "registration_quality: budgets, sign/frame invariance, rank and invalid input passed\n";
}
