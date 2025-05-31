# SECTION - Set up
using Random
# using Distributions
using Logging
using Distributed
using DistributedArrays
using TimerOutputs
using BenchmarkTools

if nworkers() != 3
	addprocs(3,exeflags="--project=$(@__DIR__)")
end
addworkers(n) = addprocs(n,exeflags="--project=$(@__DIR__)")
import TrajectoryOptimization: Discrete

using TrajectoryOptimization
const TO = TrajectoryOptimization
@everywhere using TrajectoryOptimization
@everywhere const TO = TrajectoryOptimization
@everywhere using StaticArrays
@everywhere using LinearAlgebra
@everywhere using DistributedArrays
@everywhere using ForwardDiff
using ms_thesis
# !SECTION

# SECTION - Setting the problem, methods, scenario

function init_dist(;num_lift=3, quat=false, scenario=:doorway)
	probs = ddata(T=Problem{Float64,Discrete});
	@show length(probs)
	@sync for (j,w) in enumerate(worker_quads(num_lift))
		@spawnat w probs[:L] = gen_prob(j, quad_params, load_params,
			num_lift=num_lift, quat=quat, scenario=scenario)
	end
	prob_load = gen_prob(:load, quad_params, load_params,
		num_lift=num_lift, quat=quat, scenario=scenario)

	return probs, prob_load
end

verbose = false

opts_ilqr = iLQRSolverOptions(verbose=verbose,
      iterations=200)

opts_al = AugmentedLagrangianSolverOptions{Float64}(verbose=verbose,
    opts_uncon=opts_ilqr,
    cost_tolerance=1.0e-6,
    constraint_tolerance=1.0e-3,
    cost_tolerance_intermediate=1.0e-5,
    iterations=10,
    penalty_scaling=2.0,
    penalty_initial=10.)

quat = true
num_lift = 3
scenario = :doorway
scenario == :doorway ? obs = true : obs = false
# !SECTION

# SECTION - Distributed Trajectory Optimization

probs, prob_load = init_dist(num_lift=num_lift, quat=quat, scenario=scenario);
wait.([@spawnat w reset_control_reference!(probs[:L]) for w in worker_quads(num_lift)])
@time sol, sol_solvers, xx = solve_admm(probs, prob_load, quad_params, load_params, true, opts_al, max_iters=1);
TO.solve_aula!(sol[4], sol_solvers[4]) # what is teh 5th element? It doesn't exist, each is an object
prob_lift = fetch(@spawn w probs[])
length(sol)
size(sol[2])

@btime begin
	wait.([@spawnat w reset_control_reference!(probs[:L]) for w in workers()])
	@time solve_admm($probs, $prob_load, $quad_params, $load_params, true, $opts_al);
end

@btime solve_admm($probs, $prob_load, $quad_params, $load_params, $true, $opts_al);
if false


X_cache, U_cache, X_lift, U_lift = init_cache(prob_load, probs);
fetch(@spawnat 1 length(X_cache[:L]))
cache = (X_cache=X_cache, U_cache=U_cache, X_lift=X_lift, U_lift=U_lift);
# !SECTION

# SECTION - Visualization


kk = [1,13,26,39,51]
k = 51
n_slack = 3
x_load = sol[1].X[k][1:n_slack]

for i = 1:num_lift
	x_lift = sol[i+1].X[k][1:n_slack]
	settransform!(vis["cable"]["$i"], cable_transform(x_lift,x_load))
	settransform!(vis["lift$i"], compose(Translation(x_lift...),LinearMap(Quat(sol[i+1].X[k][4:7]...))))
end
settransform!(vis["load"], Translation(x_load...))
##

idx = [(1:3)...,(7 .+ (1:3))...]
output_traj(sol[2],idx,joinpath(pwd(),"examples/ADMM_v2/trajectories/traj0.txt"))
output_traj(sol[3],idx,joinpath(pwd(),"examples/ADMM_v2/trajectories/traj1.txt"))
output_traj(sol[4],idx,joinpath(pwd(),"examples/ADMM_v2/trajectories/traj2.txt"))
println(sol[2].x0[1:3])
println(sol[3].x0[1:3])
println(sol[4].x0[1:3])
end #?

pwd()
output_traj(sol[2],idx,joinpath(pwd(),"examples/ADMM_v2/trajectories/traj0.txt"))
output_traj(sol[3],idx,joinpath(pwd(),"examples/ADMM_v2/trajectories/traj1.txt"))
output_traj(sol[4],idx,joinpath(pwd(),"examples/ADMM_v2/trajectories/traj2.txt"))
println(sol[2].x0[1:3])
println(sol[3].x0[1:3])
println(sol[4].x0[1:3])

# REVIEW - What again?
include("animation.jl")
vis = Visualizer()
open(vis)
visualize_quadrotor_lift_system(vis, sol)
# !SECTION
