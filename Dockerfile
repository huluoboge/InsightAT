FROM nvidia/cuda:11.8.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive TZ=UTC

WORKDIR /workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3 \
    libeigen3-dev libceres-dev libopencv-dev libgdal-dev \
    libgoogle-glog-dev libglew-dev libegl1-mesa-dev \
    qtbase5-dev libqt5opengl5-dev \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:${PATH} LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# COPY . /workspace/insightat/
# WORKDIR /workspace/insightat/build
# RUN cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

# ENV PATH=/workspace/insightat/build:${PATH}
CMD ["/bin/bash"]
