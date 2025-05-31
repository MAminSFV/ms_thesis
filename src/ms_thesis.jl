module ms_thesis

# Re-export external dependencies commonly used in scripts
using LinearAlgebra
using StaticArrays
using Rotations
using ForwardDiff
using Plots
using MeshCat
using GeometryTypes
using CoordinateTransformations
using Combinatorics
using TrajectoryOptimization
const TO = TrajectoryOptimization

# Include all source files
include("models.jl")
include("config_sets.jl")
include("methods.jl")
include("methods_distributed.jl")
include("animation.jl")
include("plotting.jl")
include("problem.jl")

# Export main functions/types for use in scripts
export get_anchors_local, get_anchors_global, set_platform_params, load_params, quad_params, cable_params
export gen_prob, trim_conditions, trim_conditions_batch, solve_admm, solve_admm_1slack, solve_admm_1slack_dist
export visualize_quadrotor_lift_system, visualize_batch, visualize_platform_batch
export plot_agents
export gen_set, get_states, get_quad_locations, calc_static_forces

end
