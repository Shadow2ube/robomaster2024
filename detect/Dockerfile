FROM osrf/ros:noetic-desktop-full
#FROM arm64v8/ros:noetic-ros-core
#FROM nvcr.io/nvidia/l4t-base:r35.1.0


RUN apt-get update
RUN apt-get -y install pip

ENV PATH="${PATH}:/home/dockeruser/.local/bin"

RUN groupmod --gid 985 video
RUN useradd -m --uid 1000 dockeruser
RUN usermod -a -G video dockeruser
RUN mkdir -p /opt/extra
RUN chown dockeruser /opt/extra
USER dockeruser

RUN pip3 install numpy
RUN pip3 install opencv-python
RUN pip3 install pyrealsense2
RUN pip3 install ultralytics
RUN pip3 install rospy2
RUN pip3 install cv-bridge

USER root

RUN apt-get -y install libgl1-mesa-glx libglib2.0-dev
RUN apt-get -y install ros-noetic-std-msgs

RUN mkdir -p /root/test
RUN mkdir -p /opt/detect

RUN apt-get -y install python3-opencv

USER dockeruser

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/ros_entrypoint.sh"]
