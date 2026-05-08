FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libboost-all-dev \
    libeigen3-dev \
    libtbb-dev \
    python3-catkin-tools \
    python3-rosdep \
    unzip \
    wget \
    ros-noetic-geometry-msgs \
    ros-noetic-message-filters \
    ros-noetic-nav-msgs \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-rosbag \
    ros-noetic-rosbash \
    ros-noetic-roscpp \
    ros-noetic-roslaunch \
    ros-noetic-roslib \
    ros-noetic-rosmsg \
    ros-noetic-rostopic \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-visualization-msgs \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN wget -q -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip && \
    unzip -q gtsam.zip && \
    cmake -S gtsam-4.1.1 -B gtsam-4.1.1/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_UNSTABLE=OFF \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    cmake --build gtsam-4.1.1/build --target install -- -j"$(nproc)" && \
    ldconfig && \
    rm -rf gtsam.zip gtsam-4.1.1

WORKDIR /catkin_ws/src
COPY src/vendor/sc_pgo/fast_lio_sam fast_lio_sam
RUN sed -i 's/fast_lio_sam_qn/fast_lio_sam/g' \
    fast_lio_sam/src/fast_lio_sam.cpp \
    fast_lio_sam/include/fast_lio_sam.h

WORKDIR /catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release

COPY docker/ros1_scpgo/scpgo_no_save.yaml /opt/scpgo/scpgo_no_save.yaml
COPY docker/ros1_scpgo/run_dual_scpgo.sh /opt/scpgo/run_dual_scpgo.sh
RUN chmod +x /opt/scpgo/run_dual_scpgo.sh

ENTRYPOINT ["/opt/scpgo/run_dual_scpgo.sh"]
