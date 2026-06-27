#pragma once

#include <Eigen/Core>
#include <array>

namespace social_mpc {

struct RobotParams {
    double v_max{1.5};        // max forward speed [m/s]
    double v_min{-0.5};       // max backward speed [m/s]
    double omega_max{1.0};    // max angular velocity [rad/s]
};

using RobotState   = Eigen::Vector3d; // [X, Y, theta]
using RobotControl = Eigen::Vector2d; // [v, omega]

RobotState kinematics(const RobotState& x, const RobotControl& u);
RobotState integrate_rk4(const RobotState& x, const RobotControl& u, double dt);
RobotControl clamp_control(const RobotControl& u, const RobotParams& p);
double wrap_angle(double a);

} // namespace social_mpc
