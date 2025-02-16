
using ForwardDiff, LinearAlgebra, Plots, StaticArrays
using Combinatorics
using Distributions, Random
using TrajectoryOptimization

const TO = TrajectoryOptimization
include("visualization.jl")
include("problem.jl")
include("methods.jl")


verbose=false

opts_ilqr = iLQRSolverOptions(verbose=verbose,
      iterations=50) #50

opts_al = AugmentedLagrangianSolverOptions{Float64}(verbose=verbose,
    opts_uncon=opts_ilqr,
    cost_tolerance=1.0e-5,
    constraint_tolerance=1.0e-3,
    cost_tolerance_intermediate=1.0e-4,
    iterations=20,
    penalty_scaling=10.0,
    penalty_initial=1.0e-3)


# Scenario, Config, & agent selection 

num_lift = 6 # Number of Quadrotors

scenario = :hover
config = :platform
agent  = :platform_batch
cable = :elastic

tf = 5
dt = 0.05
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

expList = []
for i=1:5 
    # Generate the problem
    prob, xf = gen_prob(agent, quad_params, platform_params, r0_platform;
                scenario=scenario, num_lift=num_lift, N=N, quat=quat, config=:platform, dt=dt)
    TO.has_quat(prob.model)

    solver = solve!(prob, opts_al)
    exp = [prob.N, prob.dt, prob.x0, prob.X, prob.U, xf, "Hover Scenario $i"]
    push!(expList, exp)
end

for exp in expList
    #
    N = exp[1]
    dt = exp[2]
    x0 = exp[3]
    X = exp[4]
    U = exp[5]
    xf = exp[6]
    name = exp[7]
    #
    vis = Visualizer()
    open(vis)
    visualize_platform_batch(vis, N, dt, x0, X, U, xf, platform_params, false, num_lift)
    #
    plot_agents( N, dt, x0, X, U, xf, name, num_lift) 
end



