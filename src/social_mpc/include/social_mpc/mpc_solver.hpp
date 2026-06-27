#pragma once

#include <array>
#include <Eigen/Core>

namespace social_mpc {

constexpr int MPC_NX = 3;   // [X, Y, theta]
constexpr int MPC_NU = 2;   // [v, omega]
constexpr int MPC_NP = 10;  // per stage: M=5 humans * [xh, yh]
constexpr int MPC_N  = 30;
constexpr int MPC_M  = 5;

struct MpcInput {
    Eigen::Vector3d x0;                             // current robot state [X, Y, theta]
    Eigen::Vector3d goal;                           // [X_ref, Y_ref, theta_ref]
    std::array<double, MPC_N * MPC_NP> human_params; // per stage: [xh0,yh0,...,xh4,yh4]
};

struct MpcOutput {
    Eigen::Vector2d u0;     // [v, omega]
    int             status; // 0 = success
    std::array<Eigen::Vector3d, MPC_N + 1> trajectory; // predicted states [X, Y, theta]
};

class MpcSolver {
public:
    MpcSolver();
    ~MpcSolver();

    MpcSolver(const MpcSolver&)            = delete;
    MpcSolver& operator=(const MpcSolver&) = delete;

    MpcOutput solve(const MpcInput& input);

private:
    // Opaque pointer — keeps ACADOS C headers out of this header
    struct Impl;
    Impl* impl_{nullptr};
};

} // namespace social_mpc
