# MPC solver debug data published each control cycle
std_msgs/Header header

# Robot predicted trajectory (N=30 steps)
float64[] pred_x
float64[] pred_y
float64[] pred_theta

# Reference trajectory
float64[] ref_x
float64[] ref_y

# Solver stats
float64 solve_time_us
float64 cost_value
bool solver_success
bool infeasible

# Per-human slack values (M * N values, row-major: human_0_step_0, ..., human_0_step_N, ...)
float64[] slack_values
float64 max_slack
