FROM nvcr.io/nvidia/l4t-pytorch:r35.2.0-pth2.0-py3

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update \
 && apt-get install -y locales lsb-release
ENV DEBIAN_FRONTEND=noninteractive \
    PATH=${PATH}:/home/dockeruser/.local/bin \
    ROS_DISTRO=noetic

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

RUN apt-get update && apt-get -y install python3.8 python3-pip python-dev python3-dev

WORKDIR /tmp

USER dockeruser

RUN git clone --recursive --branch v1.9.0 https://github.com/pytorch/pytorch
WORKDIR /tmp/pytorch
RUN python3.8 -m pip install "cython<3.0.0" && python3 -m pip install pyyaml==6.0
RUN python3.8 -m pip install -r requirements.txt
RUN python3.8 setup.py install



RUN pip3 install ultralytics
RUN pip3 install pyrealsense2
RUN pip3 install cv-bridge

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
