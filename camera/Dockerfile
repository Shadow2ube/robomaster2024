FROM osrf/ros:noetic-desktop-full

RUN apt-get update
RUN apt-get -y install curl python3 pip lsb-release apt-transport-https

RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
    | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" \
    | tee /etc/apt/sources.list.d/librealsense.list

RUN apt-get update

ENV PATH="${PATH}:/home/dockeruser/.local/bin"

RUN groupmod --gid 985 video
RUN useradd -m --uid 1000 dockeruser
RUN usermod -a -G video dockeruser
USER dockeruser

RUN #pip3 install opencv-python
RUN pip3 install pyrealsense2
RUN #pip3 install ultralytics
RUN #pip3 install numpy
RUN pip3 install rospy2

USER root

RUN apt-get -y install ros-noetic-std-msgs
RUN #apt-get -y install ros-noetic-foxglove-bridge
RUN apt-get -y install ros-noetic-realsense2-camera
RUN #apt-get -y install libgl1-mesa-glx libglib2.0-dev
RUN #apt-get -y install librealsense2-dkms

RUN #mkdir -p /root/test
RUN mkdir -p /opt/detect

RUN apt-get install -y wget udev
RUN yes | wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
RUN chmod +x ./libuvc_installation.sh
RUN ./libuvc_installation.sh

USER dockeruser

WORKDIR "/home/dockeruser"

#ENTRYPOINT ["/bin/bash" , "-l", "-c"]
ENTRYPOINT ["/ros_entrypoint.sh"]

#CMD ["sleep", "infinity"]
