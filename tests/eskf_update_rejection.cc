#include "core/lio/eskf.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using namespace lightning;

namespace {

bool Check(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

ESKF::Options Options() {
    ESKF::Options options;
    options.max_iterations_ = 4;
    options.epsi_.setConstant(1e-8);
    options.use_aa_ = false;
    return options;
}

bool CheckRollback(const std::string& failure) {
    NavState initial;
    initial.timestamp_ = 123.0;
    initial.pos_ = Vec3d(1.0, -2.0, 0.3);
    initial.rot_ = SO3::exp(Vec3d(0.01, 0.02, 0.03));
    initial.vel_ = Vec3d(0.2, 0.3, 0.4);
    initial.bg_ = Vec3d(0.001, -0.003, 0.002);
    ESKF::CovType prior = ESKF::CovType::Identity();
    prior(NavState::kPosIdx, NavState::kVelIdx) = 0.2;
    prior(NavState::kVelIdx, NavState::kPosIdx) = 0.2;
    ESKF filter(initial, prior);
    auto options = Options();
    options.use_aa_ = failure == "AA residual increase";
    int calls = 0;
    options.lidar_obs_func_ = [&](NavState&, ESKF::CustomObservationModel& obs) {
        ++calls;
        obs.HTH_.setIdentity();
        obs.HTr_.setZero();
        obs.HTr_[0] = 0.2;
        obs.HTr_[5] = 0.02;
        obs.lidar_residual_mean_ = 1.0;
        // Fail on the second evaluation to exercise rollback after a partial
        // state update, including rotation and cross-correlated velocity.
        if (calls < 2) {
            return;
        }
        if (failure == "oversized increment") {
            obs.HTr_[0] = 10.0;
        } else if (failure == "invalid observation") {
            obs.valid_ = false;
        } else if (failure == "NaN observation") {
            obs.HTr_[0] = std::numeric_limits<double>::quiet_NaN();
        } else if (failure == "infinite observation") {
            obs.HTr_[0] = std::numeric_limits<double>::infinity();
        } else if (failure == "AA residual increase") {
            obs.lidar_residual_mean_ = 2.0;
        } else if (failure == "zero information") {
            obs.HTH_.setZero();
            obs.HTr_.setZero();
        }
    };
    filter.Init(options);
    const bool accepted = filter.Update(ESKF::ObsType::LIDAR, 1.0);
    auto final = filter.GetX();
    return Check(!accepted, failure + " returns false") &&
           Check(calls == 2, failure + " occurs after a partial update") &&
           Check(final.boxminus(initial).norm() < 1e-12, failure + " restores full state") &&
           Check(final.timestamp_ == initial.timestamp_ && final.grav_.isApprox(initial.grav_),
                 failure + " preserves timestamp and gravity") &&
           Check((filter.GetP() - prior).norm() < 1e-12, failure + " restores prior covariance");
}

bool CheckScaledObservation(bool degenerate) {
    ESKF filter{NavState(), ESKF::CovType::Identity()};
    auto options = Options();
    const int translation_axis = degenerate ? 1 : 0;
    options.lidar_obs_func_ = [&](NavState& state, ESKF::CustomObservationModel& obs) {
        obs.HTH_.setZero();
        obs.HTH_.diagonal() << (degenerate ? 0.0 : 1e3), 1e3, 1e3, 1e7, 1e7, 1e7;
        obs.HTr_.setZero();
        obs.HTr_[translation_axis] = 1e3 * (0.1 - state.pos_[translation_axis]);
        obs.lidar_residual_mean_ = std::pow(0.1 - state.pos_[translation_axis], 2);
    };
    filter.Init(options);
    const bool accepted = filter.Update(ESKF::ObsType::LIDAR, 1.0);
    // Unit covariance prior and information 1000 yield this scalar posterior.
    const double expected_translation = 100.0 / 1001.0;
    if (!Check(accepted, "scaled observation is accepted") ||
        !Check(std::abs(filter.GetX().pos_[translation_axis] - expected_translation) < 1e-9,
               "translation remains observable despite large rotation information") ||
        !Check(filter.GetP().allFinite() && filter.GetP()(translation_axis, translation_axis) < 0.002,
               "accepted update commits posterior covariance")) {
        return false;
    }
    return !degenerate ||
           (Check(std::abs(filter.GetX().pos_.x()) < 1e-12, "unobserved axis retains its prior mean") &&
            Check(filter.GetP()(0, 0) >= 1.0, "unobserved axis does not gain false confidence"));
}

}  // namespace

int main() {
    for (const std::string failure : {"oversized increment", "invalid observation", "NaN observation",
                                      "infinite observation", "AA residual increase", "zero information"}) {
        if (!CheckRollback(failure)) {
            return 1;
        }
    }
    if (!CheckScaledObservation(false) || !CheckScaledObservation(true)) {
        return 2;
    }
    std::cout << "PASS: ESKF rejection rollback, committed posterior, normalized observability\n";
}
