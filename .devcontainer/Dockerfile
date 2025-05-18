FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
# Set GR to headless mode
ENV GKSwstype=100
ENV PATH="/root/.juliaup/bin:${PATH}"

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    sudo \
    unzip \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install juliaup & add Julia 1.6 in one layer
RUN curl -fsSL https://install.julialang.org | sh -s -- --yes && \
    juliaup add 1.6 && juliaup default 1.6

# Copy the project files
COPY . /workspaces/build

# Prepare the environment (TODO: Find a reliable way to bring your deps and build them and use them in dev container)
RUN julia --project=/workspaces/build/ -e 'using Pkg; Pkg.instantiate(); Pkg.precompile();'

# Build the sysimage
#RUN mkdir /workspaces/build/sysimage && julia --project=/workspaces/build/sysimage -e 'using Pkg; Pkg.develop(path="/workspaces/build/"); Pkg.add("PackageCompiler");  using PackageCompiler; create_sysimage(["ms_thesis"]; sysimage_path="/workspace/build/ms_thesis.so")'
