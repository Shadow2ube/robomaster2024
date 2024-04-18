#FROM osrf/ros:noetic-desktop-full
FROM arm64v8/ros:noetic-ros-core

RUN apt-get update
RUN apt-get -y install curl python3 pip lsb-release apt-transport-https
RUN apt-get update

ENV PATH="${PATH}:/home/dockeruser/.local/bin"

RUN groupmod --gid 985 video
RUN useradd -m --uid 1000 dockeruser
RUN usermod -a -G video dockeruser
USER dockeruser

RUN pip3 install rospy2

USER root

RUN apt-get -y install ros-noetic-std-msgs

RUN mkdir -p /opt/detect

#RUN apt-get install -y wget udev
#RUN yes | wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
#RUN chmod +x ./libuvc_installation.sh
#RUN ./libuvc_installation.sh

USER dockeruser

WORKDIR "/home/dockeruser"

#ENTRYPOINT ["/bin/bash" , "-l", "-c"]
ENTRYPOINT ["/ros_entrypoint.sh"]

#CMD ["sleep", "infinity"]
