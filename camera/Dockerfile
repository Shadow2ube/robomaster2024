ARG base_image
FROM arm64v8/ros:noetic-ros-core

RUN apt-get update
RUN apt-get -y install curl python3 pip lsb-release apt-transport-https

ENV PATH="${PATH}:/home/dockeruser/.local/bin"

RUN groupmod --gid 985 video
RUN useradd -m --uid 1000 dockeruser
RUN usermod -a -G video dockeruser
USER dockeruser

RUN #pip3 install opencv-python
RUN #pip3 install pyrealsense2
RUN pip3 install rospy2

USER root

RUN apt-get -y install ros-noetic-std-msgs
RUN apt-get -y install ros-noetic-realsense2-camera
RUN #apt-get -y install libgl1-mesa-glx libglib2.0-dev
RUN #apt-get -y install librealsense2-dkms

RUN #mkdir -p /root/test
RUN mkdir -p /opt/detect

USER dockeruser
RUN pip3 install cv-bridge
USER root

#RUN apt-get install -y wget udev
#RUN yes | wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
#RUN chmod +x ./libuvc_installation.sh
#RUN ./libuvc_installation.sh

ARG GROUP_ID=44
RUN groupmod --gid ${GROUP_ID} video
USER dockeruser

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/ros_entrypoint.sh"]