FROM nvcr.io/nvidia/l4t-pytorch:r32.7.1-pth1.9-py3

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# region update from 18.04 to 20.04

RUN apt-key adv --keyserver 'keyserver.ubuntu.com' --recv-key 1A127079A92F09ED
RUN apt-get update && apt-get upgrade -y && apt-get install -y update-manager-core
RUN yes | do-release-upgrade

# endregion updte from 18.04 to 20.04

RUN apt-get update \
 && apt-get install -y locales lsb-release
ENV DEBIAN_FRONTEND=noninteractive PATH=${PATH}:/home/dockeruser/.local/bin
RUN dpkg-reconfigure locales

RUN apt-get -y install pip

# Install ROS Noetic

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get -y install \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-rospy
RUN apt-get install -y python3-rosdep
RUN rosdep init \
    && rosdep fix-permissions \
    && rosdep update \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# endregion Install ROS noetic

RUN echo "Package: python3-opencv\nPin: release *\nPin-Priority: -1" >> /etc/apt/preferences && apt-get update \
    && mkdir -p /opt/detect \
    && echo "#!/bin/bash\nset -e\nsource /opt/ros/noetic/setup.bash --\nexec \"\$@\"" > /entrypoint.sh \
    && chmod +x /entrypoint.sh \
    && groupmod --gid 985 video \
    && useradd -m --uid 1000 dockeruser && usermod -a -G video dockeruser

USER dockeruser

RUN pip3 install ultralytics
RUN pip3 install pyrealsense2
RUN pip3 install cv-bridge

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
