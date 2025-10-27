FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    curl git vim python3-pip \
    build-essential \
    cmake make g++ gcc spi-tools \
    nlohmann-json3-dev

RUN apt-get update && apt-get install -y \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-diff-drive-controller \
    ros-noetic-controller-manager \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf \
    ros-noetic-sensor-msgs \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-xacro \
    ros-noetic-serial \
    ros-noetic-foxglove-bridge \
    ros-noetic-costmap-2d \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-move-base \
    ros-noetic-laser-scan-matcher \
    ros-noetic-teb-local-planner
