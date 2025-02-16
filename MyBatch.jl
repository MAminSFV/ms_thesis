# SECTION - Setup
using ForwardDiff, LinearAlgebra, Plots, StaticArrays
using Combinatorics
using Rotations
using Distributions, Random
using TrajectoryOptimization
using Revise

const TO = TrajectoryOptimization
include("visualization.jl")
include("problem.jl")
include("methods.jl")
# !SECTION

# SECTION - Solver options

verbose=false

# First create UnconstrainedSolverOptions instead of iLQRSolverOptions
opts_ilqr = TO.UnconstrainedSolverOptions(verbose=verbose,
      iterations=75)

opts_al = AugmentedLagrangianSolverOptions{Float64}(verbose=verbose,
    opts_uncon=opts_ilqr,
    cost_tolerance=1.0e-5,
    constraint_tolerance=1.0e-3,
    cost_tolerance_intermediate=1.0e-4,
    iterations=20,
    penalty_scaling=10.0,
    penalty_initial=1.0e-3)
# !SECTION

# SECTION - Set Params and Create the Problem

# Scenario, Config, & agent selection

num_lift = 4 # Number of Quadrotors

scenario = :hover
config = :platform
agent  = :platform_batch
cable = :elastic

tf = 5
dt = 0.01
N = convert(Int,floor(tf/dt))+1 # Determining the Number of knot points
# Using quaternions
quat = true

# Platform properties
m_plat = 0.5
r_plat = 1.5
h_plat = 0.05
r_ext = [0.5 1. 0.]
m_ext = 0.
t_imp = tf/2

platform_params = set_platform_params(m_plat, r_plat, h_plat; num_lift=num_lift, r_ext=r_ext, m_ext=m_ext, t_imp=t_imp)

# platform/Load initial position
r0_platform = [0. 0. 1.]
q0_platform = [1. 0. 0. 0.]
x0_platform = [r0_platform q0_platform]

# Generate the problem
# REVIEW - Maybe it is better for me to define a gen_prob_v2 with extra options
prob, xf = gen_prob(agent, quad_params, platform_params, r0_platform;
                scenario=scenario, num_lift=num_lift, N=N, quat=quat, config=:platform, dt=dt)
TO.has_quat(prob.model)

# !SECTION

# SECTION - Solve the optimization problem + Simulation(Viz) @time
solver = solve!(prob, opts_al)

#vis = Visualizer()
#open(vis)

#visualize_platform_batch(vis, prob.N, prob.dt, prob.x0, prob.X, prob.U, xf, platform_params, false, num_lift)
solver.stats[:iterations]

max_violation(prob)
name = "Load Transportation with $num_lift Quadrotors"
plot_agents(prob.N, prob.dt, prob.x0, prob.X, prob.U, xf, name, num_lift)
# !SECTION
