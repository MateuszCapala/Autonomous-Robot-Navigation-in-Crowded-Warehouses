#include "social_mpc/robot_kinematics.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

using namespace social_mpc;

TEST(RobotKinematics, StraightForward) {
    RobotState x{0, 0, 0};
    RobotControl u{1.0, 0.0};
    const RobotState xdot = kinematics(x, u);
    EXPECT_DOUBLE_EQ(xdot[0], 1.0);
    EXPECT_DOUBLE_EQ(xdot[1], 0.0);
    EXPECT_DOUBLE_EQ(xdot[2], 0.0);
}

TEST(RobotKinematics, PureRotation) {
    RobotState x{0, 0, 0};
    RobotControl u{0.0, 1.0};
    const RobotState xdot = kinematics(x, u);
    EXPECT_DOUBLE_EQ(xdot[0], 0.0);
    EXPECT_DOUBLE_EQ(xdot[1], 0.0);
    EXPECT_DOUBLE_EQ(xdot[2], 1.0);
}

TEST(RobotKinematics, Rk4StraightMotion) {
    RobotState x{0, 0, 0};
    RobotControl u{1.0, 0.0};
    const RobotState x_next = integrate_rk4(x, u, 1.0);
    EXPECT_NEAR(x_next[0], 1.0, 1e-9);
    EXPECT_NEAR(x_next[1], 0.0, 1e-9);
    EXPECT_NEAR(x_next[2], 0.0, 1e-9);
}

TEST(RobotKinematics, WrapAngle) {
    EXPECT_NEAR(wrap_angle(4.0), 4.0 - 2.0 * std::numbers::pi, 1e-9);
    EXPECT_NEAR(wrap_angle(-4.0), -4.0 + 2.0 * std::numbers::pi, 1e-9);
    EXPECT_NEAR(wrap_angle(0.5), 0.5, 1e-9);
}

TEST(RobotKinematics, ClampControl) {
    RobotParams p;
    RobotControl u{5.0, 3.0};
    const RobotControl clamped = clamp_control(u, p);
    EXPECT_DOUBLE_EQ(clamped[0], p.v_max);
    EXPECT_DOUBLE_EQ(clamped[1], p.omega_max);
}

TEST(RobotKinematics, ClampControlReverse) {
    RobotParams p;
    RobotControl u{-5.0, -3.0};
    const RobotControl clamped = clamp_control(u, p);
    EXPECT_DOUBLE_EQ(clamped[0], p.v_min);
    EXPECT_DOUBLE_EQ(clamped[1], -p.omega_max);
}
