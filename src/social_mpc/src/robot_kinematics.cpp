#include "social_mpc/robot_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace social_mpc {

RobotState kinematics(const RobotState& x, const RobotControl& u) {
    return RobotState{
        u[0] * std::cos(x[2]),
        u[0] * std::sin(x[2]),
        u[1]
    };
}

RobotState integrate_rk4(const RobotState& x, const RobotControl& u, double dt) {
    const RobotState k1 = kinematics(x,                  u);
    const RobotState k2 = kinematics(x + 0.5 * dt * k1, u);
    const RobotState k3 = kinematics(x + 0.5 * dt * k2, u);
    const RobotState k4 = kinematics(x + dt * k3,        u);
    RobotState x_next   = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    x_next[2]           = wrap_angle(x_next[2]);
    return x_next;
}

RobotControl clamp_control(const RobotControl& u, const RobotParams& p) {
    return RobotControl{
        std::clamp(u[0], p.v_min,     p.v_max),
        std::clamp(u[1], -p.omega_max, p.omega_max)
    };
}

double wrap_angle(double a) {
    constexpr double pi = std::numbers::pi;
    while (a >  pi) a -= 2.0 * pi;
    while (a < -pi) a += 2.0 * pi;
    return a;
}

} // namespace social_mpc
