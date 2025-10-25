FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    curl git vim python3-pip \
    build-essential \
    cmake make g++ gcc \
	nlohmann-json3-dev


RUN apt-get update && apt-get install -y \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-diff-drive-controller \
    ros-noetic-controller-manager \
    ros-noetic-teleop-twist-keyboard \
	ros-noetic-teleop-twist-joy \
	ros-noetic-joy \
	ros-noetic-tf \
	ros-noetic-tf2-geometry-msgs \
    ros-noetic-xacro \
	ros-noetic-serial \
	ros-noetic-foxglove-bridge

COPY catkin_ws /root/catkin_ws
WORKDIR /root/catkin_ws

RUN echo "source /opt/ros/noetic/setup.sh" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.sh" >> /root/.bashrc

CMD ["bash"]
