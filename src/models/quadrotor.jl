struct Quadrotor{R} <: RigidBody{R}
    n::Int
    m::Int
    mass::Float64
    J::Diagonal{Float64,SVector{3,Float64}}
    Jinv::Diagonal{Float64,SVector{3,Float64}}
    gravity::SVector{3,Float64}
    motor_dist::Float64
    kf::Float64
    km::Float64
    bodyframe::Bool  # velocity in body frame?
    info::Dict{Symbol,Any}
end

function Quadrotor{R}(;
        mass=0.5,
        J=Diagonal(@SVector [0.0023, 0.0023, 0.004]),
        gravity=SVector(0,0,-9.81),
        motor_dist=0.1750,
        kf=1.0,
        km=0.0245,
        bodyframe=false,
        info=Dict{Symbol,Any}()) where R
    Quadrotor{R}(13,4,mass,J,inv(J),gravity,motor_dist,kf,km,bodyframe,info)
end

(::Type{Quadrotor})(;kwargs...) = Quadrotor{QuatRotation{Float64}}(;kwargs...)
RobotDynamics.control_dim(::Quadrotor) = 4

function RobotDynamics.forces(model::Quadrotor, x, u)
    q = orientation(model, x)
    kf = model.kf
    g = model.gravity
    m = model.mass

    # Extract motor speeds
    w1 = u[1]
    w2 = u[2]
    w3 = u[3]
    w4 = u[4]

    # Calculate motor forces
    F1 = max(0,kf*w1);
    F2 = max(0,kf*w2);
    F3 = max(0,kf*w3);
    F4 = max(0,kf*w4);
    F = @SVector [0., 0., F1+F2+F3+F4] #total rotor force in body frame

    m*g + q*F # forces in world frame
end

function RobotDynamics.moments(model::Quadrotor, x, u)

    kf, km = model.kf, model.km
    L = model.motor_dist

    # Extract motor speeds
    w1 = u[1]
    w2 = u[2]
    w3 = u[3]
    w4 = u[4]

    # Calculate motor forces
    F1 = max(0,kf*w1);
    F2 = max(0,kf*w2);
    F3 = max(0,kf*w3);
    F4 = max(0,kf*w4);

    # Calculate motor torques
    M1 = km*w1;
    M2 = km*w2;
    M3 = km*w3;
    M4 = km*w4;
    tau = @SVector [L*(F2-F4), L*(F3-F1), (M1-M2+M3-M4)] #total rotor torque in body frame
end

RobotDynamics.inertia(model::Quadrotor) = model.J
RobotDynamics.inertia_inv(model::Quadrotor) = model.Jinv
RobotDynamics.mass(model::Quadrotor) = model.mass
