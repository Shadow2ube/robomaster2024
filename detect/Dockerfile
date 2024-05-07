FROM nvcr.io/nvidia/l4t-pytorch:r32.7.1-pth1.9-py3

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

## region update from 18.04 to 20.04
#
RUN apt-key adv --keyserver 'keyserver.ubuntu.com' --recv-key 1A127079A92F09ED
#RUN apt-get update && apt-get upgrade -y && apt-get install -y update-manager-core
#RUN yes | do-release-upgrade
#
## endregion updte from 18.04 to 20.04

RUN apt-get update \
 && apt-get install -y locales lsb-release
ENV DEBIAN_FRONTEND=noninteractive \
    PATH=${PATH}:/home/dockeruser/.local/bin \
    ROS_DISTRO=melodic

# region Install ROS

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get -y install \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-rospy
RUN apt-get install -y python3-rosdep
RUN rosdep init \
    && rosdep fix-permissions \
    && rosdep update \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# endregion Install ROS

RUN mkdir -p /opt/detect \
    && echo "#!/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash --\nexec \"\$@\"" > /entrypoint.sh \
    && chmod +x /entrypoint.sh \
    && groupmod --gid 985 video \
    && useradd -m --uid 1000 dockeruser && usermod -a -G video dockeruser

RUN apt-get update && apt-get -y install python3.8 python3-pip

WORKDIR /tmp
RUN git clone --recursive --branch v1.9.0 http://github.com/pytorch/pytorch \
    && cd pytorch \
    && python3.8 -m pip install -r requirements.txt \
    && python3.8 setup.py install


USER dockeruser

RUN pip3 install ultralytics
RUN pip3 install pyrealsense2
RUN pip3 install cv-bridge

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
