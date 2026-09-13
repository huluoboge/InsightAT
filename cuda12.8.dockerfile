# ─── InsightAT CUDA 12.8 Docker Build ─────────────────────────────────────────
#
# Builds InsightAT with Ubuntu's libceres-dev package and CUDA 12.8, then
# packages the result as an AppImage and Debian package inside the container.
#
# Base: nvidia/cuda:12.8.0-devel-ubuntu22.04
#   - Ubuntu 22.04 (glibc 2.35) → best AppImage compatibility before glibc
#     becomes too new for older distros.
#   - CUDA 12.8 toolkit
#
# Usage:
#   docker build -t insightat:cuda12.8 -f cuda12.8.dockerfile .
#   docker run --rm --gpus all -it insightat:cuda12.8
#
# Extract AppImage and Debian package:
#   docker create --name insightat-tmp insightat:cuda12.8
#   docker cp insightat-tmp:/workspace/insightat/build-appimage-cuda12.8 ./
#   docker cp insightat-tmp:/workspace/insightat/build-deb-cuda12.8 ./
#   docker rm insightat-tmp
#
# ─────────────────────────────────────────────────────────────────────────────

FROM nvidia/cuda:12.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

WORKDIR /workspace/insightat

# ── System dependencies ──────────────────────────────────────────────────────
# Use the distro Ceres package for the container build. A separately compiled
# Ceres can still be used by the local build/AppImage workflow.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    git \
    libceres-dev \
    python3 \
    python3-pip \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libopencv-dev \
    libgdal-dev \
    libglew-dev \
    libegl1-mesa-dev \
    libglu1-mesa-dev \
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
    dpkg-dev \
    patchelf \
    wget \
    file \
    fuse \
    libfuse2 \
    xz-utils \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir "cmake>=3.24" \
    && cmake --version \
    && dpkg-query -W -f='${Package} ${Version}\n' libceres-dev

# Ubuntu 22.04's apt Node.js is 12.x, which cannot parse the nullish
# coalescing syntax used by Electron 31's installer. Install a pinned Node 20
# toolchain instead of relying on the base image's JavaScript runtime.
ARG NODE_VERSION=20.19.4
RUN wget -q -O "/tmp/node-v${NODE_VERSION}-linux-x64.tar.xz" \
      "https://nodejs.org/dist/v${NODE_VERSION}/node-v${NODE_VERSION}-linux-x64.tar.xz" \
    && wget -q -O /tmp/node-SHASUMS256.txt \
      "https://nodejs.org/dist/v${NODE_VERSION}/SHASUMS256.txt" \
    && (cd /tmp && grep " node-v${NODE_VERSION}-linux-x64.tar.xz$" node-SHASUMS256.txt \
      | sha256sum -c -) \
    && tar -xJf "/tmp/node-v${NODE_VERSION}-linux-x64.tar.xz" \
      --strip-components=1 -C /usr/local \
    && node --version \
    && npm --version \
    && rm -f "/tmp/node-v${NODE_VERSION}-linux-x64.tar.xz" /tmp/node-SHASUMS256.txt

# ── Build InsightAT ──────────────────────────────────────────────────────────
COPY . .

ARG CMAKE_CUDA_ARCHITECTURES="60;61;70;75;80;86;89;90-virtual"
RUN cmake -S . -B build-cuda-12.8 \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES="${CMAKE_CUDA_ARCHITECTURES}" \
    -DINSIGHTAT_BUILD_RENDER_TESTS=OFF \
    -DSIFTGPU_ENABLE_CUDA=OFF \
    && cmake --build build-cuda-12.8 -j"$(nproc)"

# Smoke test: CLI loads (no GPU work)
RUN build-cuda-12.8/isat_project create -h 2>&1 | head -5 && echo "Smoke test OK"

# ── Package AppImage ─────────────────────────────────────────────────────────
ARG INSIGHTAT_VERSION=0.2.5-cuda12.8
ENV INSIGHTAT_BUILD_DIR=/workspace/insightat/build-cuda-12.8 \
    CUDA_LIBS_DIR=/usr/local/cuda-12.8/lib64 \
    VERSION=${INSIGHTAT_VERSION} \
    APPIMAGE_OUT_DIR=/workspace/insightat/build-appimage-cuda12.8 \
    APPIMAGE_RUNTIME_FILE=/workspace/insightat/build-appimage-cuda12.8/.tools/runtime-x86_64 \
    DEB_DEPENDS_EXTRA="libcudart12, libcublas12, libcusparse12, libcusolver11" \
    APPIMAGE_EXTRACT_AND_RUN=1

RUN mkdir -p /workspace/insightat/build-appimage-cuda12.8/.tools \
    && wget -q -O /workspace/insightat/build-appimage-cuda12.8/.tools/runtime-x86_64 \
       https://github.com/AppImage/type2-runtime/releases/download/continuous/runtime-x86_64 \
    && chmod +x /workspace/insightat/build-appimage-cuda12.8/.tools/runtime-x86_64

# Use ./ prefix so BASH_SOURCE resolves the directory correctly
RUN cd /workspace/insightat && bash ./scripts/package/compile_appimage-12.8.sh

# ── Package Debian artifact inside the same container ───────────────────────
ENV DEB_OUTPUT_DIR=/workspace/insightat/build-deb-cuda12.8
RUN cd /workspace/insightat && bash ./scripts/package/build_deb.sh

# Package the Electron GUI from the same native build. This script consumes
# build-cuda-12.8 and never reuses a host AppImage or rebuilds C++ code.
ENV GUI_APPIMAGE_OUT_DIR=/workspace/insightat/build-appimage-simple-gui \
    GUI_DEB_OUTPUT_DIR=/workspace/insightat/build-deb-simple-gui \
    GUI_LINUXDEPLOY_PATH=/workspace/insightat/build-appimage-cuda12.8/.tools/linuxdeploy-x86_64.AppImage
RUN cd /workspace/insightat && bash ./scripts/package/build_simple_gui.sh

CMD ["/bin/bash"]
