FROM ubuntu:22.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install juliaup and Julia 1.6
RUN curl -fsSL https://install.julialang.org | sh -s -- --yes
ENV PATH="/root/.juliaup/bin:${PATH}"
RUN juliaup add 1.6 && juliaup default 1.6

# Set working directory
WORKDIR /project

# Copy the current directory contents into the container at /proje
COPY . /project/

# Prepare the environment
RUN julia --project=/app -e 'using Pkg; Pkg.instantiate(); Pkg.precompile()'
