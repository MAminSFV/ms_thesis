# NOTE - This is where the fun stuff happens!
using Plots
using MeshCat
using GeometryTypes
using CoordinateTransformations
using LinearAlgebra
import TrajectoryOptimization: AbstractSolver, solve_aula!

# NOTE - Cable Util function
function cable_transform(y,z)
    v1 = [0,0,1]
    v2 = y[1:3,1] - z[1:3,1]
    normalize!(v2)
    ax = cross(v1,v2)
    ang = acos(v1'v2)
    R = AngleAxis(ang,ax...)
    compose(Translation(z),LinearMap(R))
end

# NOTE - Obstacle course!
function plot_cylinder(vis,c1,c2,radius,mat,name="")
    geom = Cylinder(Point3f0(c1),Point3f0(c2),convert(Float32,radius))
    setobject!(vis["cyl"][name],geom,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
end

function addcylinders!(vis,cylinders,height=1.5)
    for (i,cyl) in enumerate(cylinders)
        plot_cylinder(vis,[cyl[1],cyl[2],0],[cyl[1],cyl[2],height],cyl[3],MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)),"cyl_$i")
    end
end

# SECTION - Main Vis function
function visualize_quadrotor_lift_system(vis, probs, obs=false, n_slack=3)
    prob_load = probs[1]
    prob_lift = probs[2:end]
    r_lift = .275
    r_load = .2

    if obs
        _cyl = door_obstacles()
        addcylinders!(vis,_cyl,3.)
    end


    num_lift = length(prob_lift)
    # Storing  cable distances ?
    d = [norm(prob_lift[i].x0[1:n_slack] - prob_load.x0[1:n_slack]) for i = 1:num_lift]

    # Camera angle
    settransform!(vis["/Cameras/default"], compose(Translation(5., -3, 3.),LinearMap(RotX(pi/25)*RotZ(-pi/2))))

    # Load in quad mesh
    traj_folder = joinpath(dirname(pathof(TrajectoryOptimization)),"..")
    urdf_folder = joinpath(traj_folder, "dynamics","urdf")
    obj = joinpath(urdf_folder, "quadrotor_base.obj")

    quad_scaling = 0.085
    robot_obj = FileIO.load(obj)
    robot_obj.vertices .= robot_obj.vertices .* quad_scaling

    # Initialize system
    # NOTE - Setting only agents, their quad mesh and cables, later positioning them
    for i = 1:num_lift
        # Quadrotor
        # setobject!(vis["lift$i"]["sphere"],HyperSphere(Point3f0(0), convert(Float32,r_lift)) ,MeshPhongMaterial(color=RGBA(0, 0, 0, 0.25)))
        setobject!(vis["lift$i"]["robot"],robot_obj,MeshPhongMaterial(color=RGBA(0, 0, 0, 1.0)))
        # Cable
        cable = Cylinder(Point3f0(0,0,0),Point3f0(0,0,d[i]),convert(Float32,0.01))
        setobject!(vis["cable"]["$i"],cable,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
    end

    # NOTE - Setting the load
    setobject!(vis["load"],HyperSphere(Point3f0(0), convert(Float32,r_load)) ,MeshPhongMaterial(color=RGBA(0, 1, 0, 1.0)))

    anim = MeshCat.Animation(convert(Int,floor(1/prob_lift[1].dt)))
    for k = 1:prob_lift[1].N
        MeshCat.atframe(anim,vis,k) do frame
            # cables
            x_load = prob_load.X[k][1:n_slack]
            for i = 1:num_lift
                x_lift = prob_lift[i].X[k][1:n_slack]
                settransform!(frame["cable"]["$i"], cable_transform(x_lift,x_load))
                settransform!(frame["lift$i"], compose(Translation(x_lift...),LinearMap(Quat(prob_lift[i].X[k][4:7]...))))

            end
            settransform!(frame["load"], Translation(x_load...))
        end
    end
    MeshCat.setanimation!(vis,anim)
end
# !SECTION

# SECTION - A Vis function for Batch solve
function visualize_batch(vis,prob,obs=true,num_lift=3)

    # camera angle
    # settransform!(vis["/Cameras/default"], compose(Translation(5., -3, 3.),LinearMap(RotX(pi/25)*RotZ(-pi/2))))

    if obs
        _cyl = door_obstacles()
        addcylinders!(vis, _cyl, 2.1)
    end
    x0 = prob.x0
    d = norm(x0[1:3] - x0[num_lift*13 .+ (1:3)])

    # intialize system
    traj_folder = joinpath(dirname(pathof(TrajectoryOptimization)),"..")
    urdf_folder = joinpath(traj_folder, "dynamics","urdf")
    obj = joinpath(urdf_folder, "quadrotor_base.obj")

    quad_scaling = 0.085
    robot_obj = FileIO.load(obj)
    robot_obj.vertices .= robot_obj.vertices .* quad_scaling
    for i = 1:num_lift
        setobject!(vis["lift$i"],robot_obj,MeshPhongMaterial(color=RGBA(0, 0, 0, 1.0)))
        cable = Cylinder(Point3f0(0,0,0),Point3f0(0,0,d),convert(Float32,0.01))
        setobject!(vis["cable"]["$i"],cable,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
    end
    setobject!(vis["load"],HyperSphere(Point3f0(0), convert(Float32,0.2)) ,MeshPhongMaterial(color=RGBA(0, 1, 0, 1.0)))

    anim = MeshCat.Animation(convert(Int,floor(1.0/prob.dt)))
    for k = 1:prob.N
        MeshCat.atframe(anim,vis,k) do frame
            # cables
            x_load = prob.X[k][num_lift*13 .+ (1:3)]
            for i = 1:num_lift
                x_lift = prob.X[k][(i-1)*13 .+ (1:3)]
                q_lift = prob.X[k][((i-1)*13 + 3) .+ (1:4)]
                settransform!(frame["cable"]["$i"], cable_transform(x_lift,x_load))
                settransform!(frame["lift$i"], compose(Translation(x_lift...),LinearMap(Quat(q_lift...))))
            end
            settransform!(frame["load"], Translation(x_load...))
        end
    end
    MeshCat.setanimation!(vis,anim)
end
# !SECTION

# SECTION - A Vis function for Platform Batch solve
function visualize_platform_batch(vis, N, dt, x0, X, U, xf, params, obs=false, num_lift=3)
    # NOTE - Unpacking params
    r_anc = params.r_anc
    r_plat = params.r
    h = params.h

    # TODO - Add and option for trajectory plot
    # TODO - Adding Spheres for anchor points

    # camera angle
    # settransform!(vis["/Cameras/default"], compose(Translation(5., -3, 3.),LinearMap(RotX(pi/25)*RotZ(-pi/2))))

    if obs
        _cyl = door_obstacles()
        addcylinders!(vis, _cyl, 2.1)
    end
    #x0 = prob.x0
    # cable length calculation from initial configuration #FIXME
    platform_inds = (1:13) .+ 13*num_lift # Extracting the platform's state indices, located in the end of the array
    r_inds = [(1:3) .+ i for i in (0:13:13*num_lift)] # XYZ indices of Lift agents
    x_anc = get_anchors_global(x0[platform_inds], r_anc)
    d = [norm(x0[r_inds[i]] - x_anc[i]) for i = 1:num_lift]

    # Initialize System
    obj = joinpath("assets/quadrotor.obj")

    quad_scaling = 0.085
    robot_obj = MeshFileGeometry(obj)
    # Spawn the platform
    setobject!(vis["platform"], Cylinder(Point3f0(0,0,-h/2), Point3f0(0,0,h/2), convert(Float32,r_plat)), MeshPhongMaterial(color=RGBA(0, 1, 0, 1.0)))
    setobject!(vis["platform"]["center"], HyperSphere(Point3f0(zeros(3)), convert(Float32,0.05)) ,MeshPhongMaterial(color=RGBA(1, 0, 1, 0.99)))
    rfLoad = xf[num_lift+1][1:3]
    setobject!(vis["GoalLoad"],HyperSphere(Point3f0(rfLoad), convert(Float32,0.1)) ,MeshPhongMaterial(color=RGBA(1, 0, 0, 0.7)))
    # Spawn Lift agents
    for i = 1:num_lift
        rfLift = xf[i][1:3]
        setobject!(vis["lift$i"], robot_obj, MeshPhongMaterial(color=RGBA(0, 0, 0, 1.0)))
        cable = Cylinder(Point3f0(0,0,0), Point3f0(0,0,d[i]),convert(Float32,0.01))
        setobject!(vis["cable"]["$i"],cable,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
        setobject!(vis["platform"]["anchor$i"],HyperSphere(Point3f0(r_anc[i]), convert(Float32,0.05)) ,MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)))
        setobject!(vis["Goal$i"],HyperSphere(Point3f0(rfLift), convert(Float32,0.1)) ,MeshPhongMaterial(color=RGBA(1, 0, 0, 0.7)))
    end

    # NOTE - Making the Animation
    anim = MeshCat.Animation() #prob.dt convert(Int,floor(1.0/dt))
    # For for frames
    for k = 1:N #prob.N
        MeshCat.atframe(anim, k) do
            # Cables
            x_platform = X[k][num_lift*13 .+ (1:13)] #prob.X
            r_platform = x_platform[1:3]
            q_platform = x_platform[4:7]
            x_anc = get_anchors_global(x_platform, r_anc)
            # For for lift agents
            for i = 1:num_lift
                r_lift = X[k][(i-1)*13 .+ (1:3)] #prob.X
                q_lift = X[k][((i-1)*13 + 3) .+ (1:4)] #prob.X
                settransform!(vis["cable"]["$i"], cable_transform(r_lift, x_anc[i]))
                settransform!(vis["lift$i"], compose(Translation(r_lift...),LinearMap(Quat(q_lift...))))
            end
            settransform!(vis["platform"], compose(Translation(r_platform...),LinearMap(Quat(q_platform...))))
        end
    end
    MeshCat.setanimation!(vis, anim)
end
