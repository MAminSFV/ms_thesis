FROM ubuntu:22.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Install juliaup and Julia 1.6
RUN curl -fsSL https://install.julialang.org | sh -s -- --yes
ENV PATH="/root/.juliaup/bin:${PATH}"
RUN juliaup add 1.6 && juliaup default 1.6

# Set GR to headless mode
ENV GKSwstype=100

# Prepare the environment
RUN julia --project=/workspaces/ms_thesis -e 'using Pkg; Pkg.activate(); Pkg.instantiate(); Pkg.precompile();'
#RUN julia --project=/workspaces/ms_thesis -e 'using Pkg; Pkg.activate(); Pkg.build("MeshCat")'
