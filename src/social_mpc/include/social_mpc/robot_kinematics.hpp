#pragma once

#include <Eigen/Core>
#include <array>

namespace social_mpc {

struct RobotParams {
    double v_max{1.5};        // max forward speed [m/s]
    double v_min{-0.5};       // max backward speed [m/s]
    double omega_max{1.0};    // max angular velocity [rad/s]
};

// State: [X, Y, theta]
using RobotState   = Eigen::Vector3d;
// Control: [v, omega]
using RobotControl = Eigen::Vector2d;

// Continuous kinematics: x_dot = f(x, u)
RobotState kinematics(const RobotState& x, const RobotControl& u);

// RK4 integration
RobotState integrate_rk4(const RobotState& x, const RobotControl& u, double dt);

// Clamp to box constraints
RobotControl clamp_control(const RobotControl& u, const RobotParams& p);

// Wrap angle to [-pi, pi]
double wrap_angle(double a);

} // namespace social_mpc
