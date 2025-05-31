using Combinatorics

# NOTE - Problem.jl creates the optimization problem for each scenario by doing the following things:
# 1. Setting the initial conditions
# 2. Setting the initial controls
# 3. setting the objective function
# 4. Setting the cost function
#...

"""
Return the 3D positions of the quads given the position of the load
Default Config:
    Distribute quads evenly around a circle centered around the load, each at a distance `d` from the load.
    The angle `α` specifies the angle between the rope and vertical (i.e. α=pi/2 puts the quads in plane with the load)
    The angle `ϕ` specifies how much the formation is rotated about Z
Doorway Config:
    Distribute quads evenly over an arc of `2α` degrees, centered at vertical, in the x-z plane
Platform Config:
    # TODO - add description
"""

# REVIEW - How many scenarios are there? What are the scenarios that I need?
# 1. Hover
# 2. Doorway
# 3. p2p- What is that?

# REVIEW - How many agent types are there?
# 1. Load
# 2. Lift
# 3. Batch

# TODO - The Get_states function and get_quad_locations are very similar can be merged (later iterations)

# NOTE - Orientation of the platform does not matter as long as we assume that the platform is completely flat and horizontal for the initial/end and mid point configuration
# TODO - Can add a config for the two cable per agent, must use phi as Shift (later iterations)
# TODO - add a config for platform because platform has position and orientation (later iterations)
# REVIEW - How springs can affect this? probably from calc_static_forces and another function we can determine a good "h". Add a config for it

# SECTION - Generating the Problem
# NOTE - This is a critical part of the project
"""
 The Settings are:
 agent: ?
 system params: system's physical properties
 r0_load: ?
 num_lift: Number of quadrotors
 N: # knot points
 quat: using quat, what are the other options that are coded?!
 scenario:

 """
function gen_prob(agent, quad_params, load_params, r0_load=[0,0,0.25];
        num_lift=3, N=51, quat=false, scenario=:doorway, config=config, dt =0.2)

    scenario == :doorway ? obs = false : obs = false # NOTE - I can turn obs avoidance on/off here


    # Params
    ##tf = 10.0  # sec
    Nmid = convert(Int,floor(N/2))+1 # Mid-knot

    goal_dist = 0.0  # TODO - Determined by scenario
    if scenario == :hover
        goal_dist = 5.0
    end

    #α = asin(r_config/d)
    #β = deg2rad(50)  # fan angle (radians)

    # Obst Params
    r_cylinder = 0.5 # cylinder radius
    ceiling = 20 #


    # Constants -  State(x) size: n and action(u) size: m
    n_lift = 13
    m_lift = 5

    # FIXME - Incomplete for all scenarios
    if agent == :platform_batch
        n_load = 13
        d = load_params.r
        α = 0.1
        r_anc = load_params.r_anc
    else
        n_load = 6
    end

    m_load = num_lift

    # Calculated Params
    n_batch = num_lift*n_lift + n_load
    m_batch = num_lift*m_lift + m_load


    # Robot sizes
    lift_radius = 0.0275
    load_radius = 0.2
    platform_radius = 0.2

    mass_load = load_params.m::Float64
    mass_ext = load_params.m_ext::Float64
    mass_lift = quad_params.m::Float64
    mass_platform = agent == :platform_batch ? load_params.m::Float64 : load_params.m::Float64

    # SECTION - State/Controls Conditions
    # Load Pose Conditions
    rm_load = [goal_dist, goal_dist/2, r0_load[3]] # Mid-point position of the load
    rf_load = [goal_dist, goal_dist/2, r0_load[3]] # Final position of the load
    # Random Goal
    #vDst = Distributions.Normal(0,4)
    #hDst = Distributions.Normal(r0_load[3]+2,1)
    #xGoal = Random.rand(vDst)
    #yGoal = Random.rand(vDst)
    #zGoal = Random.rand(hDst)
    #rf_load = [xGoal, yGoal, zGoal] # Final position of the load



    # Lift agents conditions based on static eq.
    ini_cond = gen_set(r0_load, num_lift; α=deg2rad(5), rnd=false, config=config, quad_params=quad_params, load_params=load_params)
    mid_cond = gen_set(rm_load, num_lift; config=config, quad_params=quad_params, load_params=load_params)
    fin_cond = gen_set(rf_load, num_lift; config=config, quad_params=quad_params, load_params=load_params)

    # Setting Initial Conditions
    xlift0 = ini_cond.xlift
    xload0 = ini_cond.xload
    ulift0 = ini_cond.ulift
    uload0 = ini_cond.uload

    # Setting Mid-point Conditions
    xliftm = mid_cond.xlift
    xloadm = mid_cond.xload
    uliftm = mid_cond.ulift
    uloadm = mid_cond.uload

    # Setting Final (Goal) Conditions
    xliftf = fin_cond.xlift
    xloadf = fin_cond.xload
    uliftf = fin_cond.ulift
    uloadf = fin_cond.uload

    # !SECTION


    # SECTION - Constraints and Objectives for each scenario
    # Gen costs (objectives)
    q_lift, r_lift, qf_lift = quad_costs(n_lift, m_lift, scenario)
    q_load, r_load, qf_load = load_costs(n_load, m_load, scenario)

    # Midpoint objective
    q_lift_mid = copy(qf_lift)
    q_load_mid = copy(qf_load)
    #q_lift_mid[1:3] .= 10
    #q_load_mid[1:3] .= 10

    # TODO - Putting limits on cable extension

    # NOTE - Control limits
    u_min_lift = [0,0,0,0,-Inf]
    u_max_lift = 1.5*ones(m_lift)*(mass_ext + mass_load + mass_lift)*9.81/4 # max rpm? REVIEW Does this mean that a single quad can lift the entire system?
    u_max_lift[end] = Inf # NOTE - that is the cable force.

    # NOTE - State/Workspace limits
    x_min_lift = -Inf*ones(n_lift)
    x_min_lift[3] = 0 #can not go underground :)
    x_max_lift = Inf*ones(n_lift)

    u_min_load = zeros(num_lift)
    u_max_load = ones(m_load)*Inf

    x_min_load = -Inf*ones(n_load)
    x_min_load[3] = 0
    x_max_load = Inf*ones(n_load)

    if scenario == :doorway
        x_max_lift[3] = ceiling
        x_max_load[3] = ceiling
    end

    # NOTE - Obstacles avoidance - for lift agents
    _cyl = door_obstacles(r_cylinder, goal_dist/2)
    function cI_cylinder_lift(c,x,u)
        for i = 1:length(_cyl)
            c[i] = circle_constraint(x[1:3],_cyl[i][1],_cyl[i][2],_cyl[i][3] + 1.25*lift_radius) # NOTE: mind lift_radius
        end
    end
    # NOTE - Obstacles avoidance - for load agents
    function cI_cylinder_load(c,x,u)
        for i = 1:length(_cyl)
            c[i] = circle_constraint(x[1:3],_cyl[i][1],_cyl[i][2],_cyl[i][3] + 1.25*load_radius) # NOTE: mind load_radius
        end
    end

    # NOTE - Batch constraints

    r_inds = [(1:3) .+ i for i in (0:13:13*num_lift)]
    s_inds = 5:5:5*num_lift
    s_load = (1:num_lift) .+ 5*num_lift
    platform_inds = (1:13) .+ 13*num_lift

    # NOTE - Cart distance between load and lift
    # TODO - Elastic cables are different, d is varies... I can just define a critical point. needs testing
    # REVIEW - C is the cost that needs to be zero, ie cable length must remain as d, I guess, read the paper again.
    function distance_constraint(c,x,u=zeros(m_batch), config=config) # what is u ???
        r_load = x[r_inds[end]]
        if config ==:platform
            x_load = x[platform_inds]
            x_anc = get_anchors_global(x_load, r_anc)
        end

        for i = 1:num_lift
            if config ==:platform
                r_lift = x[r_inds[i]]
                c[i] = norm(r_lift - x_anc[i])^2 - d^2
            else
                # Cable distance
                r_lift = x[r_inds[i]]
                c[i] = norm(r_lift - r_load)^2 - d^2
            end
        end
        return nothing
    end

    # NOTE - Cable Force Constraint
    # TODO - might be able to add the cable model here.
    # REVIEW - Ensuring the reaction forces stay the same
    function force_constraint(c,x,u)
        u_load = u[s_load]
        for i = 1:num_lift
            c[i] = u[s_inds[i]] - u_load[i]
        end
        return nothing
    end

    # SECTION - Collision Constraints
    # NOTE - Creating the combination of all the quad pairs
    quad_pairs = combinations(1:num_lift, 2)
    function collision_constraint(c,x,u=zeros(m_batch))
        r_lift = [x[inds] for inds in r_inds]
        for (p,pair) in enumerate(quad_pairs)
            i,j = pair
            c[p] = circle_constraint(r_lift[i], r_lift[j][1], r_lift[j][2], 2*lift_radius)
        end
        return nothing
    end

    # REVIEW - Door related constraints?
    _cyl = door_obstacles(r_cylinder)
    function cI_cylinder(c,x,u)
        c_shift = 1
        n_slack = 3
        for p = 1:length(_cyl)
            n_shift = 0
            for i = 1:num_lift
                idx_pos = (n_shift .+ (1:13))[1:3]
                c[c_shift] = circle_constraint(x[idx_pos],_cyl[p][1],_cyl[p][2],_cyl[p][3] + 1.25*lift_radius)
                c_shift += 1
                n_shift += 13
            end
            c[c_shift] = circle_constraint(x[num_lift*13 .+ (1:3)],_cyl[p][1],_cyl[p][2],_cyl[p][3] + 1.25*lift_radius)
            c_shift += 1
        end
    end
    # !SECTION


    # SECTION - Load agent Objective and cost functions + constraints

    if agent == :load

        # Objective
        Q_load = Diagonal(q_load)
        R_load = Diagonal(r_load)
        Qf_load = Diagonal(qf_load)

        obj_load = LQRObjective(Q_load,R_load,Qf_load,xloadf,N,uload)
        Q_mid_load = Diagonal(q_load_mid)
        cost_mid_load = LQRCost(Q_mid_load,R_load,xloadm,uloadm)
        if obs
            obj_load.cost[Nmid] = cost_mid_load
        end
        # Constraints
        obs_load = Constraint{Inequality}(cI_cylinder_load,n_load,m_load,length(_cyl),:obs_load)
        bnd_load = BoundConstraint(n_load,m_load, x_min=x_min_load, u_min=u_min_load)
        constraints_load = Constraints(N)
        for k = 2:N-1
            constraints_load[k] += bnd_load
            if obs
                constraints_load[k] += obs_load
            end
        end
        constraints_load[N] += goal_constraint(xloadf) + bnd_load
        if obs
            constraints_load[N] += obs_load
        end

        # Initial controls
        U0_load = [uload for k = 1:N-1]

        # Create problem
        prob_load = Problem(gen_load_model_initial(xload0,xlift0,load_params),
            obj_load,
            U0_load,
            integration=:midpoint,
            constraints=constraints_load,
            x0=xload0,
            xf=xloadf,
            N=N,
            dt=dt)
    # !SECTION

    # SECTION - Lift agent Objective and cost functions + constraints
    elseif agent ∈ 1:num_lift

        u0 = ones(m_lift)*9.81*(load_params.m + quad_params.m/num_lift)/4  # REVIEW - something changed compared to the top, it not using super lifters
        u0[end] = 0 #ulift[1][end]

        # Model
        model = gen_lift_model_initial(xload0,xlift0[agent],quad_params,quat)

        # Objective
        Q_lift = Diagonal(q_lift)
        R_lift = Diagonal(r_lift)
        Qf_lift = Diagonal(qf_lift)
        if quat
            Gf = TO.state_diff_jacobian(model, xliftf[agent])
            q_diag = diag(Q_lift)[1:n_lift .!= 4]
            Q_lift = Gf'*Diagonal(q_diag)*Gf
            qf_diag = diag(Qf_lift)[1:n_lift .!= 4]
            Qf_lift = Gf'*Diagonal(qf_diag)*Gf
        end
        obj_lift = LQRObjective(Q_lift,R_lift,Qf_lift,xliftf[agent],N,u0)
        obj_lift[1].c = -ulift[1][end]*r_lift[end]

        if obs
            Q_mid_lift = Diagonal(q_lift_mid)
            cost_mid_lift = LQRCost(Q_mid_lift,R_lift,xliftm[agent],u0)
            for i = 1:num_lift
                obj_lift.cost[Nmid] = cost_mid_lift
            end
        end


        # Constraints
        bnd_lift = BoundConstraint(n_lift,m_lift,u_min=u_min_lift,u_max=u_max_lift,x_min=x_min_lift,x_max=x_max_lift)
        obs_lift = Constraint{Inequality}(cI_cylinder_lift,n_lift,m_lift,length(_cyl),:obs_lift)

        con = Constraints(N)
        for k = 1:N
            con[k] += bnd_lift
            if obs
                con[k] += obs_lift
            end
        end

        # Initial controls
        U0_lift = [[ulift[i] for k = 1:N-1] for i = 1:num_lift]
        U0 = [u0 for k = 1:N-1]
        # U0 = U0_lift[agent]

        # Create problem
        i = agent
        prob_lift = Problem(model,
            obj_lift,
            U0,
            integration=:midpoint,
            constraints=con,
            x0=xlift0[i],
            xf=xliftf[i],
            N=N,
            dt=dt)
    # !SECTION

    # SECTION - Batch Objective and cost functions + constraints
    elseif agent == :batch
        u0 = ones(m_lift)*9.81*(load_params.m + quad_params.m/num_lift)/4
        @show ulift[1][end]
        u0[end] = ulift[1][end]
        ulift = [u0 for i = 1:num_lift]

        # Dynamics
        info = Dict{Symbol,Any}()
        if quat
            info[:quat] = [(4:7) .+ i for i in 0:n_lift:n_lift*num_lift]
        end
        batch_params = (lift=quad_params, load=load_params)
        model_batch = Model(batch_dynamics!, n_batch, m_batch, batch_params, info) # REVIEW - Arguments are important

        # Initial and final conditions
        x0 = vcat(xlift0...,xload0)
        xf = vcat(xliftf...,xloadf)

        # objective costs
        R = Diagonal([repeat(r_lift, num_lift); r_load])
        if quat
            Gf = TO.state_diff_jacobian(model_batch, xf)
            q_lift = q_lift[1:n_lift .!= 4]
            qf_lift = qf_lift[1:n_lift .!= 4]

            Q = Diagonal([repeat(q_lift, num_lift); q_load])
            Qf = Diagonal([repeat(qf_lift, num_lift); qf_load])

            Q= Gf'*Q*Gf
            Qf= Gf'*Qf*Gf
        else
            Q = Diagonal([repeat(q_lift, num_lift); q_load])
            Qf = Diagonal([repeat(qf_lift, num_lift); qf_load])
        end

        # Create objective
        u0 = vcat(ulift...,uload)
        obj = LQRObjective(Q,R,Qf,xf,N,u0)

        # Midpoint
        if obs
            xm = vcat(xliftmid...,xloadm)
            um = vcat(uliftm...,uloadm)
            Q_mid = Diagonal([repeat(q_lift_mid, num_lift); q_load_mid])
            cost_mid = LQRCost(Q_mid,R,xm,um)
            obj.cost[Nmid] = cost_mid
        end
        # Bound Constraints
        u_l = [repeat(u_min_lift, num_lift); u_min_load]
        u_u = [repeat(u_max_lift, num_lift); u_max_load]
        x_l = [repeat(x_min_lift, num_lift); x_min_load]
        x_u = [repeat(x_max_lift, num_lift); x_max_load]
        x_l_N = copy(x_l)
        x_u_N = copy(x_u)
        x_l_N[end-(n_load-1):end] = [rf_load;zeros(3)]
        x_u_N[end-(n_load-1):end] = [rf_load;zeros(3)]

        bnd = BoundConstraint(n_batch,m_batch,u_min=u_l,u_max=u_u, x_min=x_l, x_max=x_u)
        bndN = BoundConstraint(n_batch,m_batch,x_min=x_l_N,x_max=x_u_N)

        # Constraints
        cyl = Constraint{Inequality}(cI_cylinder,n_batch,m_batch,(num_lift+1)*length(_cyl),:cyl)
        dist_con = Constraint{Equality}(distance_constraint,n_batch,m_batch, num_lift, :distance)
        for_con = Constraint{Equality}(force_constraint,n_batch,m_batch, num_lift, :force)
        col_con = Constraint{Inequality}(collision_constraint,n_batch,m_batch, binomial(num_lift, 2), :collision)
        goal = goal_constraint(xf)

        con = Constraints(N)
        for k = 1:N-1
            con[k] += dist_con + for_con + bnd + col_con
            if obs
                con[k] += cyl
            end
        end
        con[N] +=  col_con  + dist_con + bndN
        if obs
            con[N] += cyl
        end

        # Create problem
        prob = Problem(model_batch, obj, constraints=con,
                dt=dt, N=N, xf=xf, x0=x0,
                integration=:midpoint)

        # Initial controls
        U0 = [u0 for k = 1:N-1]
        initial_controls!(prob, U0)

        prob

    # !SECTION

    # SECTION - Platform Batch agent
    elseif agent == :platform_batch
        # setting u0
        @show ulift0[1][end] # Cable force magnitude of each lift agent

        # Dynamics
        info = Dict{Symbol,Any}()
        if quat
            info[:quat] = [(4:7) .+ i for i in 0:n_lift:n_lift*num_lift]
        end
        batch_params = (lift=quad_params, platform=load_params) #NOTE - Loading the params
        model_batch = Model(platform_batch_dynamics!, n_batch, m_batch, batch_params, info)

        # Initial and final conditions
        x0 = vcat(xlift0...,xload0)
        xf = vcat(xliftf...,xloadf)

        # Objective costs
        # REVIEW - I didnt get it and I didnt change it but it might be important dimension wise, platform(load) has qauts
        R = Diagonal([repeat(r_lift, num_lift); r_load]) # REVIEW - Why r? id it position only?
        if quat
            Gf = TO.state_diff_jacobian(model_batch, xf)
            q_lift = q_lift[1:n_lift .!= 4] # REVIEW - What does this mean?
            qf_lift = qf_lift[1:n_lift .!= 4]

            if n_load == 13
                q_load = q_load[1:n_load .!= 4]
                qf_load = qf_load[1:n_load .!= 4]
            end

            Q = Diagonal([repeat(q_lift, num_lift); q_load])
            Qf = Diagonal([repeat(qf_lift, num_lift); qf_load])

            Q = Gf'*Q*Gf
            Qf = Gf'*Qf*Gf
        else
            Q = Diagonal([repeat(q_lift, num_lift); q_load])
            Qf = Diagonal([repeat(qf_lift, num_lift); qf_load])
        end

        # Create objective
        u0 = vcat(ulift0...,uload0)
        obj = LQRObjective(Q,R,Qf,xf,N,u0)

        # Midpoint
        xm = vcat(xliftm...,xloadm)
        um = vcat(uliftm...,uloadm)
        Q_mid = Diagonal([repeat(q_lift_mid, num_lift); q_load_mid])
        cost_mid = LQRCost(Q_mid,R,xm,um)
        obj.cost[Nmid] = cost_mid

        # Bound Constraints
        u_l = [repeat(u_min_lift, num_lift); u_min_load]
        u_u = [repeat(u_max_lift, num_lift); u_max_load]
        x_l = [repeat(x_min_lift, num_lift); x_min_load]
        x_u = [repeat(x_max_lift, num_lift); x_max_load]
        x_l_N = copy(x_l)
        x_u_N = copy(x_u)

        if n_load == 13
            x_l_N[end-(n_load-1):end] = xloadf
            x_u_N[end-(n_load-1):end] = xloadf
        else
            x_l_N[end-(n_load-1):end] = [rf_load;zeros(3)]
            x_u_N[end-(n_load-1):end] = [rf_load;zeros(3)]
        end

        bnd = BoundConstraint(n_batch,m_batch,u_min=u_l,u_max=u_u, x_min=x_l, x_max=x_u)
        bndN = BoundConstraint(n_batch,m_batch,x_min=x_l_N,x_max=x_u_N)

        # Constraints
        cyl = Constraint{Inequality}(cI_cylinder,n_batch,m_batch,(num_lift+1)*length(_cyl),:cyl)
        dist_con = Constraint{Equality}(distance_constraint,n_batch,m_batch, num_lift, :distance)
        for_con = Constraint{Equality}(force_constraint,n_batch,m_batch, num_lift, :force)
        col_con = Constraint{Inequality}(collision_constraint,n_batch,m_batch, binomial(num_lift, 2), :collision)
        goal = goal_constraint(xf)

        con = Constraints(N)
        for k = 1:N-1
            con[k] += dist_con + for_con #+ col_con #+ bndN
            if obs
                con[k] += cyl
            end
        end
        con[N] +=   dist_con #+col_con #+ bndN
        if obs
            con[N] += cyl
        end

        # Create problem
        prob = Problem(model_batch, obj, constraints=con,
                dt=dt, N=N, xf=xf, x0=x0,
                integration=:midpoint)

        # Initial controls
        U0 = [u0 for k = 1:N-1]
        initial_controls!(prob, U0)

        return prob, push!(xliftf,xloadf)
    end
    # !SECTION
    # !SECTION

end
# !SECTION

# SECTION - Utils and cost functions

# TODO - Add the new Scenarios accordingly
function quad_costs(n_lift, m_lift, scenario=:doorway)
    if scenario == :hover
        q_diag = 10.0*ones(n_lift)
        q_diag[4:7] .= 1e-6

        r_diag = 1.0e-3*ones(m_lift)
        r_diag[end] = 1.

        qf_diag = copy(q_diag)*10.0

    elseif scenario == :p2pa
        q_diag = 1.0*ones(n_lift)
        q_diag[1] = 1e-5
        # q_diag[4:7] .*= 25.0
        # q_diag

        r_diag = 1.0e-3*ones(m_lift)
        # r_diag = 1.0e-3*ones(m_lift)
        r_diag[end] = 1.

        qf_diag = 100*ones(n_lift)

    else
        q_diag = 1e-1*ones(n_lift)
        q_diag[1] = 1e-3
        q_diag[4:7] .*= 25.0

        r_diag = 2.0e-3*ones(m_lift)

        # r_diag = 1.0e-3*ones(m_lift)
        r_diag[end] = 1.

        qf_diag = 100*ones(n_lift)
    end
    return q_diag, r_diag, qf_diag
end

# TODO - Add scenarios w/ platform
# REVIEW - Tune'em all
function load_costs(n_load, m_load, scenario=:doorway)
    if scenario == :hover
        q_diag = 10.0*ones(n_load) #

        r_diag = 1*ones(m_load)
        qf_diag = 10.0*ones(n_load)
    elseif scenario == :p2p
        q_diag = 1.0*ones(n_load) #

        # q_diag = 0*ones(n_load)
        q_diag[1] = 1.0e-5
        r_diag = 1*ones(m_load)
        qf_diag = 0.0*ones(n_load)

        # q_diag = 1000.0*ones(n_load)
        # r_diag = 1*ones(m_load)
        # qf_diag = 1000.0*ones(n_load)
    else
        q_diag = 0.5*ones(n_load) #

        # q_diag = 0*ones(n_load)
        # q_diag[1] = 1e-3
        r_diag = 1*ones(m_load)
        qf_diag = 0.1e5*ones(n_load)

        # q_diag = 1000.0*ones(n_load)
        # r_diag = 1*ones(m_load)
        # qf_diag = 1000.0*ones(n_load)

    end
    return q_diag, r_diag, qf_diag
end

"""
# TODO - Elastic cable affects this, add is_elastic and if statements to add functionality
function calc_static_forces(α::Float64, lift_mass, load_mass, num_lift)
    # Thrust per rotor of each lift agent
    thrust = 9.81*(lift_mass + load_mass/num_lift)/4
    # Cable Force magnitude (The vertical component) The horizontal components are cancelled out due to symmetry.
    f_mag = load_mass*9.81/(num_lift*cos(α))
    # Assembling in the lift control input formation
    ulift = [[thrust; thrust; thrust; thrust; f_mag] for i = 1:num_lift]
    # Assembling in the load/platform control input formation
    uload = ones(num_lift)*f_mag
    return ulift, uload
end
"""

function door_obstacles(r_cylinder=0.5, x_door=3.0)
    _cyl = NTuple{3,Float64}[]

    push!(_cyl,(x_door, 1.,r_cylinder))
    push!(_cyl,(x_door,-1.,r_cylinder))
    push!(_cyl,(x_door-0.5, 1.,r_cylinder))
    push!(_cyl,(x_door-0.5,-1.,r_cylinder))
    # push!(_cyl,(x_door+0.5, 1.,r_cylinder))
    # push!(_cyl,(x_door+0.5,-1.,r_cylinder))
    return _cyl
end

function reset_control_reference!(prob::Problem)
    for k = 1:prob.N-1
        prob.obj[k].r[5] = 0
    end
end
# !SECTION
