FROM federicolanzani/opencv-cuda-jetson:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# region Update to focal

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y update-manager-core
RUN yes | do-release-upgrade

# endregion Update to focal

RUN #echo 'Etc/UTC' > /etc/timezone
RUN #ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN echo "Package: python3-opencv\nPin: release *\nPin-Priority: -1" >> /etc/apt/preferences

RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

RUN apt-get -y install pip

RUN groupmod --gid 985 video
RUN useradd -m --uid 1000 dockeruser
RUN usermod -a -G video dockeruser
RUN mkdir -p /opt/extra
RUN chown dockeruser /opt/extra

USER dockeruser

RUN pip3 install ultralytics
RUN pip3 install pyrealsense2
RUN pip3 install --upgrade numpy
RUN pip3 install rospy2
RUN pip3 install cv-bridge

USER root

# Install ROS Noetic

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update
RUN apt-get -y install ros-noetic-std-msgs \
    && ros-noetic-sensor-msgs \
    && ros-noetic-geometry-msgs
RUN apt-get install -y python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# endregion Install ROS noetic


RUN mkdir -p /root/test
RUN mkdir -p /opt/detect

ENV PATH=${PATH}:/home/dockeruser/.local/bin:/usr/local/cuda-10.2/bin

RUN echo "#!/bin/bash\nset -e\nsource /opt/ros/noetic/setup.bash --\nexec \"\$@\"" > /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER dockeruser

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
