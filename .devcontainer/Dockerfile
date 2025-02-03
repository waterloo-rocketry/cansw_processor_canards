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
    cmake \
    ninja-build \
    wget 

# Install clang-format-19
RUN echo "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main" >> /etc/apt/sources.list \
    && wget -qO - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - \
    && apt-get update && apt-get install -y clang-format-19 \
    && rm -rf /var/lib/apt/lists/*
ENV CLANG_FORMAT="/usr/bin/clang-format-19"

# Install arm-none-eabi by xpack
RUN wget -q https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.3.1-1.1/xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz \
    && tar -xzf /xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz \
    && rm -rf /xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz
ENV PATH="/xpack-arm-none-eabi-gcc-13.3.1-1.1/bin/:$PATH"

# install cppcheck 2.16
RUN wget -q https://github.com/danmar/cppcheck/archive/2.16.0.tar.gz \
    && tar -xzf /2.16.0.tar.gz \
    && cd /cppcheck-2.16.0 \
    && mkdir build & cd build \
    && cmake .. && cmake --build . -- -j16 \
    && make install \
    && cd / && rm -rf /cppcheck-2.16.0 /2.16.0.tar.gz

# Set the working directory inside the container
WORKDIR /workspace

# Default command when the container starts
CMD ["/bin/bash"]
