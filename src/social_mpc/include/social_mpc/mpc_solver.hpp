#pragma once

#include <array>
#include <Eigen/Core>

namespace social_mpc {

constexpr int MPC_NX   = 3;   // [X, Y, theta]
constexpr int MPC_NU   = 2;   // [v, omega]
constexpr int MPC_NP   = 20;  // M*[xh,yh] + M*[vhx,vhy] per stage
constexpr int MPC_N    = 30;
constexpr int MPC_M    = 5;
constexpr int MPC_NY   = MPC_NX + MPC_NU + MPC_M;
constexpr int MPC_NY_E = MPC_NX + MPC_M;

struct MpcInput {
    Eigen::Vector3d x0;
    Eigen::Vector3d goal;
    std::array<double, MPC_N * MPC_NP> human_params;
};

struct MpcOutput {
    Eigen::Vector2d u0;
    int             status;
    std::array<Eigen::Vector3d, MPC_N + 1> trajectory;
};

class MpcSolver {
public:
    MpcSolver();
    ~MpcSolver();

    MpcSolver(const MpcSolver&)            = delete;
    MpcSolver& operator=(const MpcSolver&) = delete;

    MpcOutput solve(const MpcInput& input);

private:
    // pimpl: keeps ACADOS C headers out of this header
    struct Impl;
    Impl* impl_{nullptr};
};

} // namespace social_mpc
