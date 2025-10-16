FROM ros:noetic-ros-core

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake make g++ gcc \
	ros-noetic-tf

COPY catkin_ws /root/catkin_ws
WORKDIR /root/catkin_ws

RUN echo "source /opt/ros/noetic/setup.sh" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.sh" >> /root/.bashrc

CMD ["bash"]
