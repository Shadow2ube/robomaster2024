#FROM librealsense/librealsense
FROM arm64v8/ros:noetic-ros-core

RUN apt-get update -y

RUN apt-get install -y ros-noetic-realsense2-*
RUN apt-get install -y ros-noetic-librealsense2*

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "bash" ]