FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libceres-dev \
    libfmt-dev \
    libopencv-dev \
    python3-dev \
    python3-catkin-tools \
    python3-rosdep \
    python3-vcstool \
    ros-noetic-cv-bridge \
    ros-noetic-eigen-conversions \
    ros-noetic-gtsam \
    ros-noetic-image-transport \
    ros-noetic-jsk-recognition-msgs \
    ros-noetic-laser-geometry \
    ros-noetic-mavros-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-rosbag \
    ros-noetic-roscpp \
    ros-noetic-rospy \
    ros-noetic-rosfmt \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-topic-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

COPY docker/ros1_hybrid_slam/ros1_hybrid_entrypoint.sh /ros1_hybrid_entrypoint.sh
COPY docker/ros1_hybrid_slam/swarm_lio2_synthetic_contract_feeder.py /swarm_lio2_synthetic_contract_feeder.py
RUN chmod +x /ros1_hybrid_entrypoint.sh /swarm_lio2_synthetic_contract_feeder.py

ENTRYPOINT ["/ros1_hybrid_entrypoint.sh"]
CMD ["idle"]
