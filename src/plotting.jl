# NOTE - This is where the fun stuff happens!
using Plots
using FileIO
using LinearAlgebra


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
    plot_dir = joinpath("plots", name)
    mkpath(plot_dir)
    format = "svg"
    savefig(xp, joinpath(plot_dir, "Ex.$format"))
    savefig(yp, joinpath(plot_dir, "Ey.$format"))
    savefig(zp, joinpath(plot_dir, "Ez.$format"))
    savefig(q1p, joinpath(plot_dir, "Eq1.$format"))
    savefig(q2p, joinpath(plot_dir, "Eq2.$format"))
    savefig(q3p, joinpath(plot_dir, "Eq3.$format"))
    savefig(xdp, joinpath(plot_dir, "xd.$format"))
    savefig(ydp, joinpath(plot_dir, "yd.$format"))
    savefig(zdp, joinpath(plot_dir, "zd.$format"))
    savefig(qd1p, joinpath(plot_dir, "qd1.$format"))
    savefig(qd2p, joinpath(plot_dir, "qd2.$format"))
    savefig(qd3p, joinpath(plot_dir, "qd3.$format"))
    #savefig(statedot, "statedot.$format")
    #savefig(uCable, "uCable.$format")
    #savefig(rotor, "rotors.$format")
end
