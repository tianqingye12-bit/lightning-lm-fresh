#include "common/nav_state.h"
#include "core/lio/eskf.hpp"
#include <cmath>
#include <iostream>
using namespace lightning;
int main() {
    NavState s;
    s.grav_ = Vec3d(0, 0, -9.81);
    // Stationary IMU must not accelerate in world coordinates.
    for (int i = 0; i < 100; ++i) s.oplus(s.get_f(Vec3d::Zero(), Vec3d(0, 0, 9.81)), .01);
    if (s.vel_.norm() > 1e-10 || s.pos_.norm() > 1e-10) return 1;
    // One second of 2 m/s^2 must produce 2 m/s (forward Euler position: .99 m).
    for (int i = 0; i < 100; ++i) s.oplus(s.get_f(Vec3d::Zero(), Vec3d(2, 0, 9.81)), .01);
    if (std::abs(s.vel_.x() - 2.) > 1e-10 || std::abs(s.pos_.x() - .99) > 1e-10) return 2;
    // Removing acceleration retains the acquired velocity.
    for (int i = 0; i < 100; ++i) s.oplus(s.get_f(Vec3d::Zero(), Vec3d(0, 0, 9.81)), .01);
    if (std::abs(s.pos_.x() - 2.99) > 1e-10) return 3;
    NavState initial;
    initial.grav_ = Vec3d(0,0,-9.81);
    ESKF filter(initial);
    ESKF::ProcessNoiseType noise = ESKF::ProcessNoiseType::Identity() * 1e-4;
    for (int i=0; i<100; ++i) filter.Predict(.01, noise, Vec3d::Zero(), Vec3d(2,0,9.81));
    if (std::abs(filter.GetX().vel_.x()-2.) > 1e-10) return 4;
    if (!filter.GetP().allFinite()) return 5;
    std::cout << "PASS: stationary, acceleration, constant velocity, ESKF prediction\n";
}
