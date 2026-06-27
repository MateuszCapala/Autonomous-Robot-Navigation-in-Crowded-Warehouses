#include "social_mpc/mpc_solver.hpp"

#include "acados_solver_social_mpc.h"
#include "acados_c/ocp_nlp_interface.h"

#include <cstring>
#include <stdexcept>

namespace social_mpc {

struct MpcSolver::Impl {
    social_mpc_solver_capsule* capsule{nullptr};
    ocp_nlp_config*            config{nullptr};
    ocp_nlp_dims*              dims{nullptr};
    ocp_nlp_in*                nlp_in{nullptr};
    ocp_nlp_out*               nlp_out{nullptr};
    ocp_nlp_solver*            solver{nullptr};
};

MpcSolver::MpcSolver() : impl_(new Impl()) {
    impl_->capsule = social_mpc_acados_create_capsule();
    if (!impl_->capsule)
        throw std::runtime_error("social_mpc_acados_create_capsule failed");

    if (social_mpc_acados_create(impl_->capsule))
        throw std::runtime_error("social_mpc_acados_create failed");

    impl_->config  = social_mpc_acados_get_nlp_config(impl_->capsule);
    impl_->dims    = social_mpc_acados_get_nlp_dims(impl_->capsule);
    impl_->nlp_in  = social_mpc_acados_get_nlp_in(impl_->capsule);
    impl_->nlp_out = social_mpc_acados_get_nlp_out(impl_->capsule);
    impl_->solver  = social_mpc_acados_get_nlp_solver(impl_->capsule);

    // Warm-up solve to trigger lazy HPIPM initialization.
    // Avoids ~200ms spike on the first real solve call.
    MpcInput warmup{};
    warmup.human_params.fill(999.0);
    solve(warmup);
    social_mpc_acados_reset(impl_->capsule, 1);
}

MpcSolver::~MpcSolver() {
    if (impl_) {
        social_mpc_acados_free(impl_->capsule);
        social_mpc_acados_free_capsule(impl_->capsule);
        delete impl_;
    }
}

MpcOutput MpcSolver::solve(const MpcInput& input) {
    auto* cfg    = impl_->config;
    auto* dims   = impl_->dims;
    auto* nlp_in = impl_->nlp_in;
    auto* nlp_out= impl_->nlp_out;

    // Initial state constraint (equality: lbx0 = ubx0 = x0)
    double x0[MPC_NX] = {input.x0[0], input.x0[1], input.x0[2]};
    ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, 0, "ubx", x0);

    // Stage cost reference [X_ref, Y_ref, theta_ref, 0, 0]
    double yref[MPC_NX + MPC_NU] = {
        input.goal[0], input.goal[1], input.goal[2], 0.0, 0.0
    };
    for (int k = 0; k < MPC_N; ++k)
        ocp_nlp_cost_model_set(cfg, dims, nlp_in, k, "yref", yref);

    // Terminal cost reference [X_ref, Y_ref, theta_ref]
    double yref_e[MPC_NX] = {input.goal[0], input.goal[1], input.goal[2]};
    ocp_nlp_cost_model_set(cfg, dims, nlp_in, MPC_N, "yref", yref_e);

    // Human parameters per stage
    for (int k = 0; k < MPC_N; ++k) {
        const double* p = input.human_params.data() + k * MPC_NP;
        social_mpc_acados_update_params(impl_->capsule, k, const_cast<double*>(p), MPC_NP);
    }
    // Terminal stage parameters (use last prediction step)
    const double* p_last = input.human_params.data() + (MPC_N - 1) * MPC_NP;
    social_mpc_acados_update_params(
        impl_->capsule, MPC_N, const_cast<double*>(p_last), MPC_NP);

    const int status = social_mpc_acados_solve(impl_->capsule);

    MpcOutput out;
    out.status = status;
    ocp_nlp_out_get(cfg, dims, nlp_out, 0, "u", out.u0.data());

    for (int k = 0; k <= MPC_N; ++k) {
        double x_k[MPC_NX];
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "x", x_k);
        out.trajectory[k] = {x_k[0], x_k[1], x_k[2]};
    }
    return out;
}

} // namespace social_mpc
