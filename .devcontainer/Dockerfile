# Use Ubuntu as the base image
FROM --platform=linux/amd64 ubuntu:22.04

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
    wget \
    nano

# Install latest clang-format
RUN echo "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main" >> /etc/apt/sources.list \
    && wget -qO - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - \
    && apt-get update && apt-get install -y clang-format-19 \
    && rm -rf /var/lib/apt/lists/* \
    && ln -sf /usr/bin/clang-format-19 /usr/local/bin/clang-format
ENV CLANG_FORMAT="/usr/bin/clang-format-19"

# Install arm-none-eabi by xpack
RUN wget -q https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.3.1-1.1/xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz \
    && tar -xzf /xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz \
    && rm -rf /xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz
ENV PATH="/xpack-arm-none-eabi-gcc-13.3.1-1.1/bin/:$PATH"

# Set the working directory inside the container
WORKDIR /workspace

# Default command when the container starts
CMD ["/bin/bash"]
