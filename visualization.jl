# NOTE - This is where the fun stuff happens!
using Plots
using MeshCat
using GeometryTypes
using CoordinateTransformations
using FileIO
using MeshIO
using LinearAlgebra
import TrajectoryOptimization: AbstractSolver, solve_aula!

include("models.jl")

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
    robot_obj = FileIO.load(obj)
    if isnothing(robot_obj)
        error("Failed to load OBJ file: $obj")
    end
    #robot_obj.vertices .= robot_obj.vertices .* quad_scaling
    # Spawn the platform
    setobject!(vis["platform"], Cylinder(Point3f0(0,0,-h/2), Point3f0(0,0,h/2),convert(Float32,r_plat)),MeshPhongMaterial(color=RGBA(0, 1, 0, 1.0)))
    setobject!(vis["platform"]["center"], HyperSphere(Point3f0(zeros(3)), convert(Float32,0.05)) ,MeshPhongMaterial(color=RGBA(1, 0, 1, 0.99)))
    rfLoad = xf[num_lift+1][1:3]
    setobject!(vis["GoalLoad"],HyperSphere(Point3f0(rfLoad), convert(Float32,0.1)) ,MeshPhongMaterial(color=RGBA(1, 0, 0, 0.7)))
    # Spawn Lift agents
    for i = 1:num_lift
        rfLift = xf[i][1:3]
        setobject!(vis["lift$i"],robot_obj,MeshPhongMaterial(color=RGBA(0, 0, 0, 1.0)))
        cable = Cylinder(Point3f0(0,0,0), Point3f0(0,0,d[i]),convert(Float32,0.01))
        setobject!(vis["cable"]["$i"],cable,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
        setobject!(vis["platform"]["anchor$i"],HyperSphere(Point3f0(r_anc[i]), convert(Float32,0.05)) ,MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)))
        setobject!(vis["Goal$i"],HyperSphere(Point3f0(rfLift), convert(Float32,0.1)) ,MeshPhongMaterial(color=RGBA(1, 0, 0, 0.7)))
    end



    # NOTE - Making the Animation
    anim = MeshCat.Animation(convert(Int,floor(1.0/dt))) #prob.dt
    # For for frames
    for k = 1:N #prob.N
        MeshCat.atframe(anim,vis,k) do frame
            # Cables
            x_platform = X[k][num_lift*13 .+ (1:13)] #prob.X
            r_platform = x_platform[1:3]
            q_platform = x_platform[4:7]
            x_anc = get_anchors_global(x_platform, r_anc)
            # For for lift agents
            for i = 1:num_lift
                r_lift = X[k][(i-1)*13 .+ (1:3)] #prob.X
                q_lift = X[k][((i-1)*13 + 3) .+ (1:4)] #prob.X
                settransform!(frame["cable"]["$i"], cable_transform(r_lift, x_anc[i]))
                settransform!(frame["lift$i"], compose(Translation(r_lift...),LinearMap(Quat(q_lift...))))
            end
            settransform!(frame["platform"], compose(Translation(r_platform...),LinearMap(Quat(q_platform...))))
        end
    end
    MeshCat.setanimation!(vis,anim)
end
# !SECTION


function plot_agents(N, dt, x0, X, U, xf, name ,num_lift)
    # Unpacking params
    r_inds = [(1:3) .+ i for i in (0:13:13*num_lift)]
    r_plat = (1:3) .+ 13*num_lift
    s_inds = 5:5:5*num_lift
    s_load = (1:num_lift) .+ 5*num_lift
    u_inds = [(1:4) .+ i for i in (0:5:5*num_lift-1)] # NOTE - this takes quads RPMs
    #
    xInds = [1 + i for i in (0:13:13*num_lift)]
    yInds = [2 + i for i in (0:13:13*num_lift)]
    zInds = [3 + i for i in (0:13:13*num_lift)]
    q1Inds = [4 + i for i in (0:13:13*num_lift)]
    q2Inds = [5 + i for i in (0:13:13*num_lift)]
    q3Inds = [6 + i for i in (0:13:13*num_lift)]
    q4Inds = [7 + i for i in (0:13:13*num_lift)]
    #
    xdInds = [8 + i for i in (0:13:13*num_lift)]
    ydInds = [9 + i for i in (0:13:13*num_lift)]
    zdInds = [10 + i for i in (0:13:13*num_lift)]
    qd1Inds = [11 + i for i in (0:13:13*num_lift)]
    qd2Inds = [12 + i for i in (0:13:13*num_lift)]
    qd3Inds = [13 + i for i in (0:13:13*num_lift)]
    #
    #N = prob.N
    #dt = prob.dt
    t = 0:dt:dt*(N-1)
    #
    #X = prob.X
    #U = prob.U

    l = @layout [x q1; y q2; z q3]
    lbll = ["Quad $i" for i=1:num_lift]
    push!(lbll, "Load")
    lbl = reshape(lbll, 1, length(lbll))

    xAgents = zeros(N,num_lift+1)
    yAgents = zeros(N,num_lift+1)
    zAgents = zeros(N,num_lift+1)
    q1Agents = zeros(N,num_lift+1)
    q2Agents = zeros(N,num_lift+1)
    q3Agents = zeros(N,num_lift+1)
    #
    xdAgents = zeros(N,num_lift+1)
    ydAgents = zeros(N,num_lift+1)
    zdAgents = zeros(N,num_lift+1)
    qd1Agents = zeros(N,num_lift+1)
    qd2Agents = zeros(N,num_lift+1)
    qd3Agents = zeros(N,num_lift+1)
    #uCable = zeros(N-1,num_lift)
    #rotors = zeros(N-1,num_lift)
    for i=1:N
        x = X[i]
        #u = U[i]
        #
        xAgents[i,:] = [x[j] for j in xInds] - [xf[k][1] for k=1:num_lift+1]
        yAgents[i,:] = [x[j] for j in yInds] - [xf[k][2] for k=1:num_lift+1]
        zAgents[i,:] = [x[j] for j in zInds] - [xf[k][3] for k=1:num_lift+1]
        q1Agents[i,:] = [x[j] for j in q1Inds] - [xf[k][4] for k=1:num_lift+1]
        q2Agents[i,:] = [x[j] for j in q2Inds] - [xf[k][5] for k=1:num_lift+1]
        q3Agents[i,:] = [x[j] for j in q3Inds] - [xf[k][6] for k=1:num_lift+1]
        #
        xdAgents[i,:] = [x[j] for j in xdInds]
        ydAgents[i,:] = [x[j] for j in ydInds]
        zdAgents[i,:] = [x[j] for j in zdInds]
        qd1Agents[i,:] = [x[j] for j in qd1Inds]
        qd2Agents[i,:] = [x[j] for j in qd2Inds]
        qd3Agents[i,:] = [x[j] for j in qd3Inds]
        #
        #uCable[i,:] = [u[j] for j in s_inds]
        #rotors[i,:] = [u[j] for j in u_inds] #not the same type of stuff
    end

    lwd = 4.
    wp = 800
    hp = 600

    xp = plot(t, xAgents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent State X Error", ylabel ="Error X (m)", xlabel="Time (s)")
    yp = plot(t, yAgents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent States Y Error", ylabel ="Error Y (m)", xlabel="Time (s)")
    zp = plot(t, zAgents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent States Z Error", ylabel ="Error Z (m)", xlabel="Time (s)")
    q1p = plot(t, q1Agents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent States Q₁ Error", ylabel ="Error Q₁ (m)", xlabel="Time (s)")
    q2p = plot(t, q2Agents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent States Q₂ Error", ylabel ="Error Q₂ (rad)", xlabel="Time (s)")
    q3p = plot(t, q3Agents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent States Q₃ Error", ylabel ="Error Q₃ (rad)", xlabel="Time (s)")

    xdp = plot(t, xdAgents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent Velocity", ylabel ="Ẋ (m/s)", xlabel="Time (s)")
    ydp = plot(t, ydAgents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent Velocity", ylabel ="Ẏ (m/s)", xlabel="Time (s)")
    zdp = plot(t, zdAgents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent Velocity", ylabel ="Ż (m/s)", xlabel="Time (s)")
    qd1p = plot(t, qd1Agents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent Velocity", ylabel ="Q̇₁ (rad/s)", xlabel="Time (s)")
    qd2p = plot(t, qd2Agents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent Velocity", ylabel ="Q̇₂ (rad/s)", xlabel="Time (s)")
    qd3p = plot(t, qd3Agents, lw = lwd, label = lbl, legend =:outertopright, size=(wp, hp), title = "$name - Agent Velocity", ylabel ="Q̇₃ (rad/s)", xlabel="Time (s)")

    #state = plot(xp, q1p, yp, q2p, zp, q3p, layout=l) #FIXME label = lbl, title = "Agent States"
    #statedot = plot(xdp, qd1p, ydp, qd2p, zdp, qd3p, layout=l, title = "Agent State Velocities") #FIXME
    #uCable = plot(t, uCable, title = "Cable Forces", label = lbl2, lw = lwd)
    #rotor = plot(t, uRotor, title = "Cable Forces", label = lbl2, lw = lwd) #FIXME
    dir = mkpath("./plots/$name")
    savefig(xp, dir*"/$name-Ex.svg")
    savefig(yp, dir*"/$name-Ey.svg")
    savefig(zp, dir*"/$name-Ez.svg")
    savefig(q1p, dir*"/$name-Eq1.svg")
    savefig(q2p, dir*"/$name-Eq2.svg")
    savefig(q3p, dir*"/$name-Eq3.svg")
    #
    savefig(xdp, dir*"/$name-xd.svg")
    savefig(ydp, dir*"/$name-yd.svg")
    savefig(zdp, dir*"/$name-zd.svg")
    savefig(qd1p, dir*"/$name-qd1.svg")
    savefig(qd2p, dir*"/$name-qd2.svg")
    savefig(qd3p, dir*"/$name-qd3.svg")
    #savefig(statedot, "statedot.svg")
    #savefig(uCable, "uCable.svg")
    #savefig(rotor, "rotors.svg")
end
