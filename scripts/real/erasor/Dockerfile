# Noetic + FAST_LIO + SC-A-LOAM (loop closure) + ERASOR (dynamic removal)
# Uses livox_ros_driver (singular, v1) — our bags are pre-converted via
# scripts/real/erasor/convert_bag_for_erasor.py before running here.
#
# Build:  docker build -t erasor:noetic -f scripts/real/erasor/Dockerfile scripts/real/erasor/
# Run:    bash scripts/real/erasor/docker_run.sh
#
FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

# ── system deps ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git wget curl ca-certificates cmake \
      software-properties-common gnupg \
      libeigen3-dev libpcl-dev libomp-dev libtbb-dev \
      libboost-all-dev libgoogle-glog-dev libgflags-dev \
      libceres-dev \
      libsuitesparse-dev libmetis-dev \
      python3-pip python3-catkin-tools \
      ros-noetic-pcl-ros ros-noetic-pcl-conversions \
      ros-noetic-jsk-recognition-msgs ros-noetic-jsk-rviz-plugins \
      ros-noetic-tf2-sensor-msgs ros-noetic-tf-conversions \
      ros-noetic-rviz \
      vim less locales sudo \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ── GTSAM 4.0.3 from BorgLab PPA (SC-A-LOAM + ERASOR need 4.0.x) ────────
RUN add-apt-repository ppa:borglab/gtsam-release-4.0 \
    && apt-get update && apt-get install -y --no-install-recommends \
       libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

# ── workspace ────────────────────────────────────────────────────────
RUN mkdir -p /ws/src
WORKDIR /ws/src

# livox_ros_driver (v1) — provides livox_ros_driver/CustomMsg
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git

# FAST_LIO (HKU master uses livox_ros_driver v1 by default)
RUN git clone https://github.com/hku-mars/FAST_LIO.git \
    && cd FAST_LIO && git submodule update --init --recursive

# SC-A-LOAM (gisbi-kim's loop closure backend) — patch OpenCV 3→4 compat:
#   - scanRegistration.cpp:  drop unused `#include <opencv/cv.h>`
#   - kittiHelper.cpp:       CV_LOAD_IMAGE_GRAYSCALE → cv::IMREAD_GRAYSCALE
RUN git clone https://github.com/gisbi-kim/SC-A-LOAM.git \
    && sed -i 's|^#include <opencv/cv.h>|// removed (OpenCV4): #include <opencv/cv.h>|' \
         SC-A-LOAM/src/scanRegistration.cpp \
    && sed -i 's|CV_LOAD_IMAGE_GRAYSCALE|cv::IMREAD_GRAYSCALE|g' \
         SC-A-LOAM/src/kittiHelper.cpp

# ERASOR (dynamic object removal — separate offline batch tool)
RUN git clone https://github.com/LimHyungTae/ERASOR.git \
    && cd ERASOR && git submodule update --init --recursive

# ── build ────────────────────────────────────────────────────────────
WORKDIR /ws
RUN /bin/bash -c "set -eo pipefail; source /opt/ros/noetic/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc)"

# Source setup in bashrc for convenience
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "source /ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /ws
CMD ["bash"]
