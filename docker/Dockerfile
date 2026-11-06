# LIGO: Tightly Coupled LiDAR-Inertial-GNSS Odometry
# Ubuntu 20.04 + ROS Noetic, with Ceres 2.1, GTSAM 4.0, Livox driver and gnss_comm.
#
# Build (from the repository root):
#   docker build -f docker/Dockerfile -t ligo:noetic .

FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG JOBS=8
ARG CERES_VERSION=2.1.0
ARG LIVOX_SDK_VERSION=v2.3.0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ---------------------------------------------------------------------------
# System dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget ca-certificates software-properties-common \
        libboost-all-dev libeigen3-dev libgoogle-glog-dev libgflags-dev \
        libatlas-base-dev libsuitesparse-dev libtbb-dev \
        python3-dev \
        ros-noetic-sophus ros-noetic-pcl-ros ros-noetic-cv-bridge \
        ros-noetic-eigen-conversions ros-noetic-tf \
    && add-apt-repository -y ppa:borglab/gtsam-release-4.0 \
    && apt-get update && apt-get install -y --no-install-recommends \
        libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Ceres Solver (>= 2.1 is required for ceres::Manifold)
# ---------------------------------------------------------------------------
RUN git clone --depth 1 --branch ${CERES_VERSION} https://github.com/ceres-solver/ceres-solver.git /tmp/ceres \
    && cmake -S /tmp/ceres -B /tmp/ceres/build \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
    && cmake --build /tmp/ceres/build -j${JOBS} --target install \
    && rm -rf /tmp/ceres

# ---------------------------------------------------------------------------
# Livox SDK (needed by livox_ros_driver)
# ---------------------------------------------------------------------------
RUN git clone --depth 1 --branch ${LIVOX_SDK_VERSION} https://github.com/Livox-SDK/Livox-SDK.git /tmp/livox_sdk \
    && cmake -S /tmp/livox_sdk -B /tmp/livox_sdk/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/livox_sdk/build -j${JOBS} --target install \
    && rm -rf /tmp/livox_sdk \
    && ldconfig

# ---------------------------------------------------------------------------
# Catkin workspace: third-party ROS packages
# ---------------------------------------------------------------------------
ENV CATKIN_WS=/catkin_ws
WORKDIR ${CATKIN_WS}/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git \
    && git clone --depth 1 https://github.com/HKUST-Aerial-Robotics/gnss_comm.git

WORKDIR ${CATKIN_WS}
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j${JOBS} --pkg livox_ros_driver gnss_comm

# ---------------------------------------------------------------------------
# LIGO
# ---------------------------------------------------------------------------
COPY . ${CATKIN_WS}/src/ligo
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j${JOBS}

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "ligo", "mapping_avia.launch"]
