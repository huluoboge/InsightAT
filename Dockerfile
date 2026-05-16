# CUDA 11.8 + Ubuntu 22.04 — build InsightAT (default: PopSift, SiftGPU off).
# Build:  docker build -t insightat:cuda11.8 -f Dockerfile .
# Run:     docker run --rm --gpus all -it insightat:cuda11.8
# Pipeline: isat_sfm (see README)

FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

WORKDIR /workspace/insightat

# Ceres (apt) + SuiteSparse/METIS/TBB: required by the project; Boost: PopSift
# PopSift (third_party) requires CMake >= 3.24; jammy's cmake is 3.22 — install a newer one via pip.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    git \
    python3 \
    python3-pip \
    libeigen3-dev \
    libceres-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libopencv-dev \
    libgdal-dev \
    libglew-dev \
    libegl1-mesa-dev \
    libopengl0 \
    libsuitesparse-dev \
    libmetis-dev \
    libtbb-dev \
    libopenblas-dev \
    liblapack-dev \
    liblapacke-dev \
    libboost-filesystem-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    p7zip-full \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir "cmake>=3.24" \
    && cmake --version

COPY . .

# Broad GPU coverage; adjust for your deployment (e.g. only 80;86 for datacenter A100s)
# Updated to match compile-12.8-1060.sh script for consistency
ARG CMAKE_CUDA_ARCHITECTURES="60;61;70;75;80;86;89;90-virtual"
RUN cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES="${CMAKE_CUDA_ARCHITECTURES}" \
    -DINSIGHTAT_ENABLE_SIFTGPU=OFF \
    && cmake --build build -j"$(nproc)"

ENV PATH=/workspace/insightat/build:${PATH}

# Smoke: CLI loads (no GPU work)
RUN isat_sfm --help >/dev/null

CMD ["/bin/bash"]