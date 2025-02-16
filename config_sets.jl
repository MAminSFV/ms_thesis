"""
Creating system configuration sets ([xlift, ulift],[xload,uload]) for each scenario and config's initial,mid and final condition.

Configs:
1. Point Load
2. Rigid body platform
3. Doorway?

Scenarios:
1. Hover (Point/Platform)
2. Transport (Point/Platform))
3. Doorway (Point/Platform))
4. Falling load while hovering (Platform)
5. Falling load while Transporting (Platform)
5. Impulse Impact While hovering (Platform)
6. Impulse Impact While Transporting (Platform)

"""
# SECTION - Generating the sets
using Rotations

# !SECTION

# SECTION - Util functions

"""
This function takes the load position and orientation and gives out the global coords of the anchors on the platform.
Arguments:
x_platform: full state of the platform (dim=13) #REVIEW - Can be reduced to 7 by eliminating velocities
r_anc: list of anchor points on the platform expressed in the body coordinate system.
"""
function get_anchors_global(x_platform, r_anc)
    p = x_platform[1:3]
    q = normalize(Quaternion(x_platform[4:7]))
    x_anc = [p + q*r_anc[i] for i = 1:length(r_anc)]
    return x_anc
end

"""
Takes the location of the load and generates a formation of lift agents positions.
"""
function get_quad_locations(x_load::VecOrMat, d::Real, α=π/4, num_lift=3;
    config=:platform, r_cables=[zeros(3) for i = 1:num_lift], ϕ=0.0, rnd= false)
    if config == :pointLoad
        h = d*cos(α)
        r = d*sin(α)
        z = x_load[3] + h
        circle(θ) = [x_load[1] + r*cos(θ), x_load[2] + r*sin(θ)]
        θ = range(0,2π,length=num_lift+1) .+ ϕ
        x_lift = [zeros(3) for i = 1:num_lift]
        for i = 1:num_lift
            if num_lift == 2
                x_lift[i][1:2] = circle(θ[i] + pi/2)
            else
                x_lift[i][1:2] = circle(θ[i])
            end
            x_lift[i][3] = z
            x_lift[i] += r_cables[i]  # Shift by attachment location
            end
    elseif config == :doorway
        y = x_load[2]
        fan(θ) = [x_load[1] - d*sin(θ), y, x_load[3] + d*cos(θ)]
        θ = range(-α,α, length=num_lift)
        x_lift = [zeros(3) for i = 1:num_lift]
        for i = 1:num_lift
            x_lift[i][1:3] = fan(θ[i])
        end

    elseif config == :platform # carrying a thin cylindrical platform

        circle2(r,θ) = [r*cos(θ), r*sin(θ)]
        θ = range(0,2π,length=num_lift+1) .+ ϕ

        x_lift = [zeros(3) for i = 1:num_lift]
        for i = 1:num_lift
            # Random formation
            if rnd
                dst = Distributions.Normal(0,40+10)
                α₀ = deg2rad(Random.rand(dst))
                β₀ = deg2rad(Random.rand(dst))
            else
                α₀ = 0.
                β₀ = 0.
            end

            h = d*cos(α + α₀)
            r = d*sin(α + α₀)

            if num_lift == 2
                x_lift[i][1:2] = circle2(r, θ[i] + pi/2)
            else
                x_lift[i][1:2] = circle2(r, θ[i] + β₀)
            end
            x_lift[i][3] = h

            r_anc = r_cables
            x_anc = get_anchors_global(x_load, r_anc)
            x_lift[i] += x_anc[i]  # Shift by anchor locations
        end
    end
    return x_lift
end

"""
Takes the location of the load and generates a formation of lift agents poses (position + orientation).
"""
function get_states(r_load, n_lift, n_load, num_lift, d=1.55, α=deg2rad(50); config=config, r_anc=[zeros(3) for i = 1:num_lift], ϕ=0.0, rnd=false)

    xload = zeros(n_load)
    xload[1:3] = r_load

    if config == :platform
        xload[4] = 1. # for the quaternions
    end

    r_lift = get_quad_locations(xload, d, α, num_lift; config=config, r_cables=r_anc, ϕ=0.0, rnd=rnd)
    xlift = [zeros(n_lift) for i = 1:num_lift]
    for i = 1:num_lift
        xlift[i][1:3] = r_lift[i]
        xlift[i][4] = 1.0
    end

    return xlift, xload
end

# TODO - Elastic cable affects this, add is_elastic and if statements to add functionality
"""
Setting initial controls by calculating static forces.
"""
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
Takes the location of the load and generates a formation of lift agents poses (position + orientation).
"""
function gen_set(r_load, num_lift=4; α=deg2rad(2), rnd=false, config=:platform, quad_params=quad_params, load_params=platform_params)

    # Cal d and α based on config
    # Can add scaling functions based on num_lift and r_plat
    if config == :platform
        d = load_params.r
        r_anc = load_params.r_anc
        mass_load = load_params.m
        n_load = 13

    elseif config == :pointLoad
        d = 2.
        r_config = 1.2  # radius of initial configuration
        mass_load = load_params.m
        if num_lift > 6
            d *= 2.5
            r_config *= 2.5
        end
        n_load = 6
        α = asin(r_config/d)
    end
    n_lift = 13

    xlift, xload = get_states(r_load, n_lift, n_load, num_lift, d, α; config = config, r_anc=r_anc, rnd=rnd)
    ulift, uload = calc_static_forces(α, quad_params.m, mass_load, num_lift)

    set = (xlift = xlift,
           xload = xload,
           ulift = ulift,
           uload = uload)

    return set
end
# !SECTION
