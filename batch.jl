# SECTION - Setup
using ForwardDiff, LinearAlgebra, Plots, StaticArrays
using Combinatorics
using TrajectoryOptimization
const TO = TrajectoryOptimization
include("animation.jl")
include("problem.jl")
include("methods.jl")
# !SECTION

# SECTION - Solver options

verbose=true

opts_ilqr = iLQRSolverOptions(verbose=verbose,
      iterations=50)

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
# Number of Quadrotors
num_lift = 3
# Using quaternions
quat = true
# platform/Load initial position
r0_load = [0,0,0.25]
# TODO - Platform initial orientation

# Scenario selection REVIEW - Where should I define that? probably problem.jl
scenario = :doorway

# Generate the problem
prob = gen_prob(:batch, quad_params, load_params, r0_load, scenario=scenario,num_lift=num_lift,quat=quat)
TO.has_quat(prob.model)

# !SECTION

# SECTION - Solve the optimization problem + Simulation
# REVIEW - what Are trim conditions? What do they do? Do they do things?
# REVIEW - What's the commented line for? what is going on with the two solves and timing? It seems one is before trim and one after trim

# NOTE - If you are wondering what $ means in the use case below, https://stackoverflow.com/questions/68261871/what-is-the-dollar-sign-prefix-in-function-arguments-used-for-in-julia
# @btime solve($prob,$opts_al)
prob = trim_conditions_batch(num_lift, r0_load, quad_params, load_params, quat, opts_al)
@btime solve($prob, $opts_al)
@time solver = solve!(prob, opts_al)

# !SECTION

# SECTION - The Holly task of visualization and post processing
vis = Visualizer()
open(vis)

visualize_batch(vis,prob,true,num_lift)
solver.stats[:iterations]

max_violation(prob)
#=
Notes:
N lift is faster with trim conditions
Doorway is also faster with trim conditions
=#
# !SECTION
