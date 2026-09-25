# InsightAT release image: Ubuntu 22.04 + CUDA 12.8 + cuDSS 0.7 + Ceres hy.
# Produces AppImage + .deb (glibc 2.35 → runs on 22.04 and 24.04+).
#
# Local development does NOT need this image — use packaging/linux/build.sh
# with system Ceres (apt libceres-dev). This Dockerfile is for CI/release GPU BA.
#
#   docker build -t insightat:cuda12.8 -f packaging/Dockerfile .
#   ./packaging/docker-build.sh run

FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

WORKDIR /workspace/insightat

# cuDSS 0.7.1.4 from NVIDIA redistributable (not apt 0.8.x — breaks Ceres hy API).
ARG CUDSS_VERSION=0.7.1.4
ARG CUDSS_ARCHIVE=libcudss-linux-x86_64-${CUDSS_VERSION}_cuda12-archive
ARG CUDSS_URL=https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-x86_64/${CUDSS_ARCHIVE}.tar.xz
ENV CUDSS_ROOT=/opt/nvidia/cudss/${CUDSS_VERSION}
ENV LD_LIBRARY_PATH=${CUDSS_ROOT}/lib:${CUDSS_ROOT}/lib64:${LD_LIBRARY_PATH}
ENV CMAKE_PREFIX_PATH=${CUDSS_ROOT}

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    git \
    python3 \
    python3-pip \
    libeigen3-dev \
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
    p7zip-full \
    wget \
    xz-utils \
    file \
    fuse \
    libfuse2 \
    dpkg-dev \
    patchelf \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir "cmake>=3.24" \
    && cmake --version \
    && mkdir -p /opt/nvidia/cudss \
    && wget -q "${CUDSS_URL}" -O /tmp/cudss.tar.xz \
    && tar -xJf /tmp/cudss.tar.xz -C /opt/nvidia/cudss \
    && mv "/opt/nvidia/cudss/${CUDSS_ARCHIVE}" "${CUDSS_ROOT}" \
    && rm -f /tmp/cudss.tar.xz \
    && test -f "${CUDSS_ROOT}/include/cudss.h" \
    && (test -d "${CUDSS_ROOT}/lib/cmake/cudss" || test -d "${CUDSS_ROOT}/lib64/cmake/cudss") \
    && echo "cuDSS ${CUDSS_VERSION} installed at ${CUDSS_ROOT}"

# Custom Ceres-Solver (CUDA 12 + cuDSS 0.7) — release GPU BA only.
RUN git clone --branch hy --depth 1 --recurse-submodules https://github.com/huluoboge/ceres-solver.git /tmp/ceres-solver \
    && cd /tmp/ceres-solver \
    && chmod +x build_cuda12.sh \
    && CUDSS_CMAKE_DIR="$(find "${CUDSS_ROOT}" -type d -path '*/cmake/cudss' | head -1)" \
    && test -n "${CUDSS_CMAKE_DIR}" \
    && sed -i "s|CUDSS_CMAKE_DIR=\".*\"|CUDSS_CMAKE_DIR=\"${CUDSS_CMAKE_DIR}\"|" build_cuda12.sh \
    && ./build_cuda12.sh \
    && cd build_cuda128 \
    && make install \
    && rm -rf /tmp/ceres-solver

COPY . .

ARG CMAKE_CUDA_ARCHITECTURES="60;61;70;75;80;86;89;90-virtual"
RUN CUDSS_CMAKE_DIR="$(find "${CUDSS_ROOT}" -type d -path '*/cmake/cudss' | head -1)" \
    && cmake -S . -B build-release \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES="${CMAKE_CUDA_ARCHITECTURES}" \
    -DCeres_DIR="$HOME/.local/ceres-cuda128/lib/cmake/Ceres" \
    -Dcudss_DIR="${CUDSS_CMAKE_DIR}" \
    -DINSIGHTAT_BUILD_QT_UI=OFF \
    -DINSIGHTAT_ENABLE_SIFTGPU=ON \
    -DSIFTGPU_ENABLE_CUDA=ON \
    -DPopSift_BUILD_EXAMPLES=OFF \
    && cmake --build build-release -j"$(nproc)"

RUN build-release/isat_project --help 2>&1 | head -5 && echo "Smoke test OK"

ARG INSIGHTAT_VERSION=0.2.5-cuda12.8
ENV INSIGHTAT_BUILD_DIR=/workspace/insightat/build-release \
    CUDA_LIBS_DIR=/usr/local/cuda-12.8/lib64 \
    VERSION=${INSIGHTAT_VERSION} \
    APPIMAGE_OUT_DIR=/workspace/insightat/build-appimage \
    APPIMAGE_EXTRACT_AND_RUN=1 \
    DEB_OUTPUT_DIR=/workspace/insightat/build-deb

RUN cd /workspace/insightat && bash ./packaging/appimage/build.sh
RUN cd /workspace/insightat && bash ./packaging/deb/package.sh

CMD ["/bin/bash"]
