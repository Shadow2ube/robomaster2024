# Adapted from: https://github.com/2b-t/realsense-ros2-docker/blob/main/docker/Dockerfile
#FROM osrf/ros:noetic-desktop-full
#
#RUN apt-get update
#
#    # Utilitaires
#RUN apt-get install -y
#RUN apt-get install -y git
#RUN apt-get install -y cmake
#RUN apt-get install -y wget
#RUN apt-get install -y tar
#RUN apt-get install -y libx11-dev
#RUN apt-get install -y xorg-dev
#RUN apt-get install -y libssl-dev
#RUN apt-get install -y build-essential
#RUN apt-get install -y libusb-1.0-0-dev
#RUN apt-get install -y libglu1-mesa-dev
#
#    # OpenGL
#RUN apt-get install -qq --no-install-recommends
#RUN apt-get install -qq --no-install-recommends libglvnd0
#RUN apt-get install -qq --no-install-recommends libgl1
#RUN apt-get install -qq --no-install-recommends libglx0
#RUN apt-get install -qq --no-install-recommends libegl1
#RUN apt-get install -qq --no-install-recommends libxext6
#RUN apt-get install -qq --no-install-recommends libx11-6
#RUN rm -rf /var/lib/apt/lists/*
#
#    # Get folder ready
#RUN mkdir /app /dpds
#WORKDIR dpds
#RUN #cd dpds
#
#    # Realsense SDK
#RUN wget --no-check-certificate https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.54.2.tar.gz
#RUN tar -xzf v2.54.2.tar.gz
#RUN rm v2.54.2.tar.gz
#WORKDIR librealsense-2.54.2/
#RUN mkdir build
#WORKDIR build
#RUN cmake .. && make
#RUN apt-get -y install udev
#RUN make install
#
#ENV NVIDIA_VISIBLE_DEVICES all
#ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
#
#RUN apt-get update

#FROM librealsense/librealsense
FROM arm64v8/ros:noetic-ros-core

RUN apt-get update -y
RUN apt-get install -y pip

RUN pip3 install opencv-python
RUN pip3 install pyrealsense2

RUN apt-get install -y libgl1-mesa-gl

RUN #apt-get install librealsense-dkms