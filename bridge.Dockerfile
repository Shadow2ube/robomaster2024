FROM osrf/ros:noetic-desktop-full
#FROM arm64v8/ros:noetic-ros-core

RUN apt-get update

RUN apt-get -y install ros-noetic-foxglove-bridge

ENTRYPOINT ["/ros_entrypoint.sh"]

#CMD ["sleep", "infinity"]
