FROM nvcr.io/nvidia/l4t-ml:r35.2.1-py3

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

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

# region reinstall torch with cuda
ENV TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v51/pytorch/torch-2.0.0a0+8aa34602.nv23.03-cp38-cp38-linux_aarch64.whl
RUN pip3 install --no-cache ${TORCH_INSTALL}
# endregion reinstall torch with cuda

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
