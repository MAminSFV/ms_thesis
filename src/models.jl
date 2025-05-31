# SECTION - Model Parameters

"""
A function that creates anchor positions on the platform. It basically distributes points on a circle equally in a deterministic fashion.
Since the selection process is deterministic, this function can be called anywhere and produce the same location for each agent.
In another words, there is no need to store the anchor points in memory. (it would be more compute friendly if we did.
r is the radius of the circle/polygon like platform.
"""
function get_anchors_local(r::Real, num_lift=3)
    circle(θ) = [r*cos(θ), r*sin(θ)]
    θ = range(0,2π,length=num_lift+1)
    r_anc = [zeros(3) for i = 1:num_lift]
    for i = 1:num_lift
        if num_lift == 2
            r_anc[i][1:2] = circle(θ[i] + pi/2)
        else
            r_anc[i][1:2] = circle(θ[i])
        end
        r_anc[i][3] = 0.
    end
return r_anc
end


function get_anchors_global(x_platform, r_anc)
    p = x_platform[1:3]
    q = normalize(Quaternion(x_platform[4:7]))
    x_anc = [p + q*r_anc[i] for i = 1:length(r_anc)]
    return x_anc
end


# NOTE - Platform is assumed to be a Cylinder, can be polygon in the future for more aesthetic reasons.
function set_platform_params(m::Real,
                            r::Real,
                            h::Real;
                            num_lift =3,
                            r_ext=[0. 0. 0.],
                            m_ext=0.,
                            t_imp=0.)
    # Calculating the inertia tensor
    I_x = m*(3*r^2 + h^2)/12
    I_y = I_x
    I_z = (m*r^2)/2
    m_p = m
    r_p = r
    h_p = h

    platform_params = (m = m_p,
            r = r_p,
            h = h_p,
            r_ext = r_ext,
            m_ext = m_ext,
            t_imp = t_imp,
            J=SMatrix{3,3}(Diagonal([I_x, I_y, I_z])),
            Jinv=SMatrix{3,3}(Diagonal(1.0./[I_x, I_y, I_z])),
            gravity=SVector(0,0,-9.81),
            r_anc = get_anchors_local(r_p, num_lift),
            )
    return platform_params
end


load_params = (m=1.,
            gravity=SVector(0,0,-9.81),
            radius=0.2)

quad_params = (m=1.,
            J=SMatrix{3,3}(Diagonal([0.0023, 0.0023, 0.004])),
            Jinv=SMatrix{3,3}(Diagonal(1.0./[0.0023, 0.0023, 0.004])),
            gravity=SVector(0,0,-9.81),
            motor_dist=0.175,
            kf=1.0,
            km=0.0245,
            radius=0.275)

# TODO - can add params for visualization like R
cable_params = (Kp = 1.,
            Kd = 0.25)
# !SECTION

# SECTION - Batch Problem (Load/Platform)
# SECTION - Subsystem models

"""
 u is all the agents forces+directions appended in matrix form, agents to external load
 storing: platform_dynamics!(ẋ, x, u, r_ext, u_ext, params)
"""
function platform_dynamics!(ẋ, x, u, params)

    # States
    q = normalize(Quaternion(view(x,4:7)))
    v = view(x,8:10)
    omega = view(x,11:13)

    # Parameters
    m = params[:m] # mass
    J = params[:J] # inertia matrix
    Jinv = params[:Jinv] # inverted inertia matrix
    g = params[:gravity] # gravity
    r_ext = params[:r_ext]
    m_ext = params[:m_ext]
    r = params[:r_anc]


    # Superposing forces
    F = sum(u)
    # Calculating the moments , anchors must correspond to the force. #NOTE - use local anchor
    τ = -1.0*sum([cross(r[i],u[i]) for i=1:length(r)])  # REVIEW - -1 or 1 ? Test it out.

    # State equations -Rigid Body Dynamics - In World Frame
    ẋ[1:3] = v # velocity in world frame
    ẋ[4:7] = SVector(0.5*q*Quaternion(zero(x[1]), omega...))

    ẋ[8:10] = g + (1/m)*F #acceleration in world frame
    ẋ[11:13] = Jinv*(τ - cross(omega,J*omega)) # Euler's equation: I*ω + ω x I*ω = constraint_decrease_ratio
    #NOTE - The external force does not affect the rotational dynamics. It is assumed that the external force is applied at COM
    return τ, omega, J, Jinv
end

# NOTE - load weight is passed by pushing u/m as u
function load_dynamics!(ẋ,x,u)
    ẋ[1:3] = x[4:6]
    ẋ[4:6] = u[1:3]
    ẋ[6] -= 9.81 # gravity for ż
end

# TODO - Test the function in a dummy setting
# FIXME - Decide for input arguments, make it easy to use and compact
# TODO - Different Cable dynamics !
function cable_dynamics!(ẋl,ẋa,xl,xa, params)
    # get params
    Kp = params[:Kp]
    Kd = params[:Kd]
    # Linear States
    Pl = view(xl,1:3)
    Pa = view(xa,1:3)
    Ṗl = view(ẋl,1:3)
    Ṗa = view(ẋa,1:3)

    # Cable Direction
    q = normalize(Pl - Pa)

    # Cable force
    ΔP = norm(Pl - Pa)
    ΔṖ = Ṗl - Ṗl
    # Force Magnitudes
    Fp = Kp*ΔP
    Fd = Kd*(ΔṖ ⋅ q)
    # force vector
    u = (Fd + Fp).*q

    #NOTE - The cable does not have rotational dynamics and the end points rotational states does not play a role.
    return u
end

function lift_dynamics!(ẋ,x,u,params)

      # States
      q = normalize(Quaternion(view(x,4:7)))
      v = view(x,8:10)
      omega = view(x,11:13)

      # Parameters
      m = params[:m] # mass
      J = params[:J] # inertia matrix
      Jinv = params[:Jinv] # inverted inertia matrix
      g = params[:gravity] # gravity
      L = params[:motor_dist] # distance between motors

      # Thrust and moment Calculations
      w1 = u[1]
      w2 = u[2]
      w3 = u[3]
      w4 = u[4]

      kf = params[:kf]; # 6.11*10^-8;
      F1 = kf*w1;
      F2 = kf*w2;
      F3 = kf*w3;
      F4 = kf*w4;
      F = @SVector [0., 0., F1+F2+F3+F4] #total rotor force in body frame

      km = params[:km]
      M1 = km*w1;
      M2 = km*w2;
      M3 = km*w3;
      M4 = km*w4;
      tau = @SVector [L*(F2-F4), L*(F3-F1), (M1-M2+M3-M4)] #total rotor torque in body frame

      # State equations -Rigid Body Dynamics - In World Frame
      ẋ[1:3] = v # velocity in world frame
      ẋ[4:7] = SVector(0.5*q*Quaternion(zero(x[1]), omega...))
      ẋ[8:10] = g + (1/m)*(q*F + u[5:7]) #acceleration in world frame
      #NOTE - u[5,6,7]is the external force, expressed in world frame
      ẋ[11:13] = Jinv*(tau - cross(omega,J*omega)) #Euler's equation: I*ω + ω x I*ω = constraint_decrease_ratio
      #NOTE - The external force does not affect the rotational dynamics. It is assumed that the external force is applied at COM
      return tau, omega, J, Jinv
end
# !SECTION

# SECTION - Point Load Batch Dynamics
# REVIEW - What is the dimension of u?
function batch_dynamics!(ẋ,x,u, params)
    # Setting the params
    lift_params = params.lift
    load_params = params.load
    load_mass = load_params.m
    #
    n_batch = length(x)
    num_lift = (n_batch - 6) ÷ 13
    # REVIEW - setting indices, understand and visualize the indices for a test case
    # REVIEW -  Are ẋ included in the indices?
    # It is a three dimensional indexing system. 1:2:3 but what does it mean?
    lift_inds = [(1:13) .+ i for i in (0:13:13*num_lift-1)]
    load_inds = (1:6) .+ 13*num_lift # takes the load indices, located in the end of the array
    r_inds = [(1:3) .+ i for i in (0:13:13*num_lift)]
    s_inds = 5:5:5*num_lift # REVIEW - Takes the indices of cable forces of each quad (force magnitude)
    # NOTE - 5:5:15 means start by taking 5 and add 5 till you reach 15. (matlab style)
    u_inds = [(1:4) .+ i for i in (0:5:5*num_lift-1)] # NOTE - this takes quads RPMs

    # Get 3D positions
    r = [x[inds] for inds in r_inds]
    rlift = r[1:num_lift]
    rload = r[end]

    # Get control values
    u_quad = [u[ind] for ind in u_inds]
    u_load = u[(1:num_lift) .+ 5*num_lift]
    s_quad = u[s_inds] # REVIEW - What is this exactly?
    # Cable directions
    dir = [(rload - r)/norm(rload - r) for r in rlift]
    # TODO - Cable Dynamics should be here
    lift_control = [[u_quad[i]; s_quad[i]*dir[i]] for i = 1:num_lift] # REVIEW - The first part is each quad control and the last one is cable force I guess
    u_slack_load = -1.0*sum(dir .* u_load) # REVIEW - What does this represent?

    for i = 1:num_lift
        inds = lift_inds[i]
        lift_dynamics!(view(ẋ,inds), x[inds], lift_control[i], lift_params)
    end
    # lift_dynamics!(view(ẋ,13 .+ (1:13)),x[13 .+ (1:13)],lift_control_2,lift_params)
    # lift_dynamics!(view(ẋ,2*13 .+ (1:13)),x[2*13 .+ (1:13)],lift_control_3,lift_params)

    # load_dynamics!(view(ẋ,3*13 .+ (1:6)),x[3*13 .+ (1:6)],u_slack_load/load_mass)
    load_dynamics!(view(ẋ, load_inds), x[load_inds], u_slack_load/load_mass)

    return nothing
end
# !SECTION

# SECTION - Platform Batch Dynamics
# REVIEW - What is the dimension of u?
function platform_batch_dynamics!(ẋ,x,u, params)

    # Setting the params
    lift_params = params.lift
    platform_params = params.platform

    #
    n_batch = length(x)
    num_lift = (n_batch - 13) ÷ 13 # 3 position + 4 orientation + 3 linear velocity + 3 body angular velocity = 13 in that order.
    #
    lift_inds = [(1:13) .+ i for i in (0:13:13*num_lift-1)] # Extracting the lift agents' state indices(1x13)
    platform_inds = (1:13) .+ 13*num_lift # Extracting the platform's state indices, located in the end of the array
    #
    r_inds = [(1:3) .+ i for i in (0:13:13*num_lift)] # XYZ indices of Lift agents
    s_inds = 5:5:5*num_lift # NOTE - Takes the indices of cable forces of each quad (force magnitude), cable force magnitude of lift agents are only stored in their u vector
    # NOTE - 5:5:15 means start by taking 5 and add 5 till you reach 15. (matlab style)
    u_inds = [(1:4) .+ i for i in (0:5:5*num_lift-1)] # NOTE - this takes quads RPMs

    # Get 3D positions
    # REVIEW - we might encounter a bug here, with  the r[end] part, but that might not happen. ?????????????
    r = [x[inds] for inds in r_inds]
    r_lift = r[1:num_lift]
    r_anc = platform_params.r_anc

    # Get control values
    u_quad = [u[ind] for ind in u_inds]
    u_platform = u[(1:num_lift) .+ 5*num_lift] # Extracting Cable force magnitude on the platform
    s_quad = u[s_inds] # NOTE - Taking the cable force magnitude
    # calculating the anchor positions
    x_anc = get_anchors_global(x[platform_inds], r_anc)

    # Cable directions
    dir = [(x_anc[i] - r_lift[i])/norm(x_anc[i] - r_lift[i]) for i=1:length(x_anc)]

    # TODO - Cable Dynamics should be here? or

    lift_control = [[u_quad[i]; s_quad[i]*dir[i]] for i = 1:num_lift] # NOTE - The control input vector for the lifts
    u_slack_platform = (dir .* u_platform) # NOTE - This represents the cable force on the platform (Control input)
     # REVIEW - I hope the dims workout

    # Passing Lift agent Dynamics
    for i = 1:num_lift
        inds = lift_inds[i]
        lift_dynamics!(view(ẋ,inds), x[inds], lift_control[i], lift_params)
    end

    # Passing Platform Agent Dynamics

    platform_dynamics!(view(ẋ, platform_inds), x[platform_inds], u_slack_platform , platform_params)

    return nothing
end
# !SECTION
# !SECTION


# SECTION - Sequential Problems
# TODO - Replace the double double_integrator_3D_dynamics with rigid body dynamics
# REVIEW - What is this? is it the static model?
function gen_load_model_initial(xload0,xlift0,load_params)
    num_lift = length(xlift0)
    mass_load = load_params.m
    function double_integrator_3D_dynamics_load!(ẋ,x,u) where T
        Δx = [xlift[1:3] - xload0[1:3] for xlift in xlift0]
        u_slack = @SVector zeros(3)
        for i = 1:num_lift
            u_slack += u[i]*normalize(Δx[i])
        end
        Dynamics.double_integrator_3D_dynamics!(ẋ, x, u_slack/mass_load)
    end
    Model(double_integrator_3D_dynamics_load!,6,num_lift)
end

function gen_lift_model_initial(xload0,xlift0,quad_params,quat=false)

        function quadrotor_lift_dynamics!(ẋ::AbstractVector,x::AbstractVector,u::AbstractVector,params)
            q = normalize(Quaternion(view(x,4:7)))
            v = view(x,8:10)
            omega = view(x,11:13)

            # Parameters
            m = params[:m] # mass
            J = params[:J] # inertia matrix
            Jinv = params[:Jinv] # inverted inertia matrix
            g = params[:gravity] # gravity
            L = params[:motor_dist] # distance between motors

            w1 = u[1]
            w2 = u[2]
            w3 = u[3]
            w4 = u[4]

            kf = params[:kf]; # 6.11*10^-8;
            F1 = kf*w1;
            F2 = kf*w2;
            F3 = kf*w3;
            F4 = kf*w4;
            F = @SVector [0., 0., F1+F2+F3+F4] #total rotor force in body frame

            km = params[:km]
            M1 = km*w1;
            M2 = km*w2;
            M3 = km*w3;
            M4 = km*w4;
            tau = @SVector [L*(F2-F4), L*(F3-F1), (M1-M2+M3-M4)] #total rotor torque in body frame

            ẋ[1:3] = v # velocity in world frame
            ẋ[4:7] = SVector(0.5*q*Quaternion(zero(x[1]), omega...))
            Δx = xload0[1:3] - xlift0[1:3]
            dir = Δx/norm(Δx)
            ẋ[8:10] = g + (1/m)*(q*F + u[5]*dir) # acceleration in world frame
            ẋ[11:13] = Jinv*(tau - cross(omega,J*omega)) #Euler's equation: I*ω + ω x I*ω = constraint_decrease_ratio
            return tau, omega, J, Jinv
        end
        info = Dict{Symbol,Any}()
        if quat
            info[:quat] = [4:7]
        end
        Model(quadrotor_lift_dynamics!,13,5,quad_params,info)
end

function gen_lift_model(X_load,N,dt,quad_params,quat=false)
      model = Model[]

      for k = 1:N-1
        function quadrotor_lift_dynamics!(ẋ::AbstractVector,x::AbstractVector,u::AbstractVector,params)
            q = normalize(Quaternion(view(x,4:7)))
            v = view(x,8:10)
            omega = view(x,11:13)

            # Parameters
            m = params[:m] # mass
            J = params[:J] # inertia matrix
            Jinv = params[:Jinv] # inverted inertia matrix
            g = params[:gravity] # gravity
            L = params[:motor_dist] # distance between motors

            w1 = u[1]
            w2 = u[2]
            w3 = u[3]
            w4 = u[4]

            kf = params[:kf]; # 6.11*10^-8;
            F1 = kf*w1;
            F2 = kf*w2;
            F3 = kf*w3;
            F4 = kf*w4;
            F = @SVector [0., 0., F1+F2+F3+F4] #total rotor force in body frame

            km = params[:km]
            M1 = km*w1;
            M2 = km*w2;
            M3 = km*w3;
            M4 = km*w4;
            tau = @SVector [L*(F2-F4), L*(F3-F1), (M1-M2+M3-M4)] #total rotor torque in body frame

            ẋ[1:3] = v # velocity in world frame
            ẋ[4:7] = SVector(0.5*q*Quaternion(zero(x[1]), omega...))
            Δx = X_load[k][1:3] - x[1:3]
            dir = Δx/norm(Δx)
            ẋ[8:10] = g + (1/m)*(q*F + u[5]*dir) # acceleration in world frame
            ẋ[11:13] = Jinv*(tau - cross(omega,J*omega)) #Euler's equation: I*ω + ω x I*ω = constraint_decrease_ratio
            return tau, omega, J, Jinv
        end
        info = Dict{Symbol,Any}()
        if quat
            info[:quat] = [4:7]
        end
        model_k = Model(quadrotor_lift_dynamics!,13,5,quad_params,info)
        push!(model,midpoint(model_k,dt))
    end
    model
end

function gen_load_model(X_lift,N,dt,load_params)
    model = Model[]
    mass_load = load_params.m
    num_lift = length(X_lift)
    for k = 1:N-1
        function double_integrator_3D_dynamics_load!(ẋ,x,u) where T
            Δx = [xlift[k][1:3] - x[1:3] for xlift in X_lift]
            u_slack = @SVector zeros(3)
            for i = 1:num_lift
                u_slack += u[i]*Δx[i]/norm(Δx[i])
            end
            Dynamics.double_integrator_3D_dynamics!(ẋ, x, u_slack/mass_load)
        end
        push!(model,midpoint(Model(double_integrator_3D_dynamics_load!,6, num_lift),dt))
    end
    model
end
# !SECTION
