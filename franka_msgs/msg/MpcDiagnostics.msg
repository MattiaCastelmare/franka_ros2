# One MPC solve, published by mpc_commander on /mpc/diagnostics and logged by
# human_logging. The fields are those of franka_experiments' MPCDiagnostics
# (utils/mpc_problem.py), plus fallback, which only the node knows.
std_msgs/Header header
# Solver status ('solved', 'maximum iterations reached', 'primal infeasible', ...)
string status
bool solved
# True when the q̈ published after this solve is not its plan (previous plan or brake)
bool fallback
uint32 iterations
# [ms] solver time only, without building the QP and the kinematics
float64 solve_time_ms
# Cost at the solution without its constant term: compare cycles, not runs
float64 cost
# Human rows in use this cycle, and how many of them are at their bound or violated
uint32 human_rows
uint32 human_binding
# [m] smallest margin of the linearised distance over d_safe + delta + m(tau) along
# the horizon; negative = the plan leans on slack; NaN without human rows
float64 human_clearance
# [m] largest slack
float64 max_slack
# Share of the planned q̈ at the q̈ box
float64 accel_saturation
