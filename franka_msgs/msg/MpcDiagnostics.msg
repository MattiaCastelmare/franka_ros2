# One MPC solve, published by mpc_commander on /mpc/diagnostics and logged by
# human_logging. The fields are those of franka_experiments' MPCDiagnostics
# (utils/mpc_problem.py), plus fallback and input_delay, which only the node knows.
std_msgs/Header header
# Solver status ('solved', 'maximum iterations reached', 'primal infeasible', ...)
string status
bool solved
# True when the q̈ published after this solve is not its plan (previous plan or brake)
bool fallback
# [s] the plan starts this long after the stamp: when its u_0 reaches the controller
# (mpc.yaml input_delay_s). Its first state is the measured one predicted to then
float64 input_delay
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
# The optimal cost V (cost plus its constant term, >= 0: comparable across cycles) and two
# of its parts: the stage cost l(x_0, u_0) of step 1 (q_1, q̇_1, u_0, u_0 - u_prev, its
# slacks) and the soft terminal term w_terminal·|q̇_N|². For analysis/mpc_analysis.py
float64 cost_full
float64 stage_cost
float64 terminal_cost
# Largest |Lagrange multiplier| of each row family (0: none active, NaN: no solution or no
# duals): how much V would drop per unit the bound gave way. Human rows in cost per metre,
# unscaled: the slack penalty is exact while lambda_human < slack_linear
float64 lambda_human
float64 lambda_accel
float64 lambda_position
float64 lambda_velocity