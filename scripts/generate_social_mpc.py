#!/usr/bin/env python3
"""
Generates the ACADOS C solver for social MPC.
Run once (or whenever MPC parameters change) before building the ROS2 package:

    python3 scripts/generate_social_mpc.py

Output: src/social_mpc/acados_generated/
"""

import os
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# ── Problem dimensions ────────────────────────────────────────────────────────
N   = 30    # horizon steps
DT  = 0.1   # [s] — must match Kalman tracker dt
M   = 5     # max tracked humans (matches HumanArray padding)

# ── Ellipse safety zone ───────────────────────────────────────────────────────
A_ELL    = 2.0  # semi-axis along X [m]
B_ELL    = 1.2  # semi-axis along Y [m]
K_FWD    = 1.5  # [s] forward shift of ellipse center along human velocity:
                # constraint = "stay away from where human will be in K_FWD seconds"
                # prevents robot from cutting in front of moving humans

# ── Cost weights ──────────────────────────────────────────────────────────────
W_X     = 5.0   # position error X
W_Y     = 5.0   # position error Y
W_THETA = 0.1   # heading error — low: don't constrain turning during avoidance
W_V     = 0.5   # linear velocity regularisation
W_OMEGA = 2.5   # angular velocity regularisation — high to suppress steering oscillation

W_X_e     = 10.0  # terminal position X
W_Y_e     = 10.0  # terminal position Y
W_THETA_e = 2.0   # terminal heading

W_SLACK = 1500.0  # soft ellipse hard-safety penalty
W_REP   = 8.0    # social repulsion: always-active gradient away from humans
                  # (Voronoi-like: pushes robot toward open space between obstacles)

# ── Control bounds ────────────────────────────────────────────────────────────
V_MAX   =  1.5   # [m/s]
V_MIN   = -0.5   # [m/s]
W_MAX   =  1.0   # [rad/s]


def build_model() -> AcadosModel:
    model = AcadosModel()
    model.name = "social_mpc"

    # States: [X, Y, theta]
    X_s     = ca.SX.sym('X')
    Y_s     = ca.SX.sym('Y')
    theta_s = ca.SX.sym('theta')
    x = ca.vertcat(X_s, Y_s, theta_s)

    # Controls: [v, omega]
    v_s     = ca.SX.sym('v')
    omega_s = ca.SX.sym('omega')
    u = ca.vertcat(v_s, omega_s)

    # Parameters per stage:
    #   [xh0, yh0, xh1, yh1, ..., xh4, yh4,   <- predicted positions (M*2)
    #    vhx0, vhy0, ..., vhx4, vhy4]           <- velocities          (M*2)
    p_syms = []
    for i in range(M):
        p_syms += [ca.SX.sym(f'xh{i}'), ca.SX.sym(f'yh{i}')]
    v_syms = []
    for i in range(M):
        v_syms += [ca.SX.sym(f'vhx{i}'), ca.SX.sym(f'vhy{i}')]
    p = ca.vertcat(*p_syms, *v_syms)

    # Diff-drive kinematics
    f_expl = ca.vertcat(
        v_s * ca.cos(theta_s),
        v_s * ca.sin(theta_s),
        omega_s,
    )

    # Hard-safety ellipse constraints + social repulsion.
    # Ellipse center is shifted K_FWD seconds forward along human velocity:
    #   xc_i = xh_i + K_FWD * vhx_i
    #   yc_i = yh_i + K_FWD * vhy_i
    # Stationary humans (vhx=vhy=0) get no shift (correct).
    # Moving humans: robot must stay outside the ellipse around where they WILL BE,
    # not just where they are — prevents robot from cutting in front.
    h_terms = []
    rep_terms = []
    for i in range(M):
        xhi  = p_syms[2 * i]
        yhi  = p_syms[2 * i + 1]
        vhxi = v_syms[2 * i]
        vhyi = v_syms[2 * i + 1]
        xc = xhi + K_FWD * vhxi
        yc = yhi + K_FWD * vhyi
        d_ell = (X_s - xc)**2 / A_ELL**2 + (Y_s - yc)**2 / B_ELL**2
        h_terms.append(d_ell)
        rep_terms.append(1.0 / (d_ell + 1.0))

    h_expr = ca.vertcat(*h_terms)

    # NONLINEAR_LS cost outputs:
    # Stage:    [X, Y, theta, v, omega, rep_h0, ..., rep_h4]
    # Terminal: [X, Y, theta,           rep_h0, ..., rep_h4]
    model.cost_y_expr   = ca.vertcat(X_s, Y_s, theta_s, v_s, omega_s, *rep_terms)
    model.cost_y_expr_e = ca.vertcat(X_s, Y_s, theta_s,               *rep_terms)

    model.x             = x
    model.u             = u
    model.p             = p
    model.f_expl_expr   = f_expl
    model.con_h_expr    = h_expr
    model.con_h_expr_e  = h_expr
    return model


def generate():
    model = build_model()
    nx  = model.x.shape[0]   # 3
    nu  = model.u.shape[0]   # 2
    np_ = model.p.shape[0]   # M * 2 = 10

    ny   = nx + nu + M  # 10: goal(3) + control(2) + repulsion(5)
    ny_e = nx      + M  #  8: goal(3)              + repulsion(5)

    ocp = AcadosOcp()
    ocp.model   = model
    ocp.dims.N  = N
    ocp.dims.ny   = ny
    ocp.dims.ny_e = ny_e

    # ── Cost: NONLINEAR_LS ────────────────────────────────────────────────────
    # Gauss-Newton Hessian: J^T W J — always PSD, works well for sum-of-squares.
    ocp.cost.cost_type   = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    ocp.cost.W   = np.diag([W_X, W_Y, W_THETA, W_V, W_OMEGA] + [W_REP] * M)
    ocp.cost.W_e = np.diag([W_X_e, W_Y_e, W_THETA_e]         + [W_REP] * M)

    # yref[0:5] overridden at runtime with goal; yref[5:] stays 0 (repulsion target)
    ocp.cost.yref   = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # ── Control bounds ────────────────────────────────────────────────────────
    ocp.constraints.lbu   = np.array([V_MIN, -W_MAX])
    ocp.constraints.ubu   = np.array([V_MAX,  W_MAX])
    ocp.constraints.idxbu = np.arange(nu)

    # ── Soft ellipse constraints (stage + terminal) ───────────────────────────
    ocp.constraints.lh   = np.ones(M)
    ocp.constraints.uh   = np.ones(M) * 1e6
    ocp.constraints.lh_e = np.ones(M)
    ocp.constraints.uh_e = np.ones(M) * 1e6

    ocp.constraints.idxsh   = np.arange(M)
    ocp.constraints.idxsh_e = np.arange(M)

    ocp.cost.Zl   = W_SLACK * np.ones(M)
    ocp.cost.Zu   = np.zeros(M)
    ocp.cost.zl   = np.zeros(M)
    ocp.cost.zu   = np.zeros(M)
    ocp.cost.Zl_e = W_SLACK * np.ones(M)
    ocp.cost.Zu_e = np.zeros(M)
    ocp.cost.zl_e = np.zeros(M)
    ocp.cost.zu_e = np.zeros(M)

    # ── Solver options ────────────────────────────────────────────────────────
    ocp.solver_options.tf                  = N * DT
    ocp.solver_options.integrator_type     = 'ERK'
    ocp.solver_options.nlp_solver_type     = 'SQP'
    ocp.solver_options.nlp_solver_max_iter = 5
    ocp.solver_options.qp_solver           = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_cond_N    = 5
    ocp.solver_options.hessian_approx      = 'GAUSS_NEWTON'
    ocp.solver_options.print_level         = 0

    # ── Initial values (overridden at runtime) ────────────────────────────────
    ocp.constraints.x0   = np.zeros(nx)
    ocp.parameter_values = np.zeros(np_)

    # ── Output directory ──────────────────────────────────────────────────────
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir    = os.path.join(script_dir, '..', 'src', 'social_mpc', 'acados_generated')
    out_dir    = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    ocp.code_export_directory = out_dir

    AcadosOcpSolver(ocp, json_file=os.path.join(out_dir, 'acados_ocp.json'))
    print(f"\nGenerated solver to: {out_dir}")
    print("Next: colcon build --packages-select social_mpc")


if __name__ == "__main__":
    generate()
