# Use Ubuntu as the base image
FROM ubuntu:22.04

# Set non-interactive mode for apt-get
ENV DEBIAN_FRONTEND=noninteractive

# Update and install essential development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    gcc \
    g++ \
    gdb \
    make \
    git \
    clang-format \
    cmake \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Install arm-none-eabi by xpack
ADD https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.3.1-1.1/xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz /tmp/archive.tar.gz
RUN tar -xzf /tmp/archive.tar.gz
ENV PATH="/xpack-arm-none-eabi-gcc-13.3.1-1.1/bin/:${PATH}"

# Set the working directory inside the container
WORKDIR /workspace

# Default command when the container starts
CMD ["/bin/bash"]
