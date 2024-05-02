FROM nvcr.io/nvidia/l4t-base:r35.1.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y --autoremove

# Get python3.10.11

# Install tzdata
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata

# Install dependencies
RUN apt-get update && apt-get upgrade && apt-get -y install \
    git \
    git-buildpackage \
    python3.8 \
    debhelper \
    autoconf \
    libncursesw5-dev

# Build python
RUN apt-get install --reinstall ca-certificates -y \
    && git clone https://github.com/JetsonHacksNano/build_python.git \
    && cd build_python \
    && sed -i 's/^sudo //' build_python3.sh \
    && bash ./build_python3.sh --version 3.10; exit 0

# Make local repository
RUN cd build_python  \
    && sed -i 's/^sudo //' make_apt_repository.sh \
    && bash ./make_apt_repository.sh --version 3.10

# Install python
RUN apt-get install -y python3.10-full \
    python3-testresources \
    && python3.10 -m ensurepip --upgrade

# endregion get python3.10.11

RUN apt-get update && apt-get install -y \
    libtesseract4 \
    libatlas3-base \
    wget

RUN pip3 install numpy

RUN cd /tmp \
    && wget https://github.com/lanzani/opencv-cuda-jetson-docker/raw/main/ubuntu18.04-py3.10.11-opencv4.8.0/installer/OpenCV-4.8.0-aarch64.sh \
    && ./OpenCV-4.8.0-aarch64.sh --prefix=/usr/local --skip-license --exclude-subdir

## region Update to focal
#
#RUN apt-get update \
#    && apt-get upgrade -y \
#    && apt-get install -y update-manager-core
#RUN yes | do-release-upgrade
#
## endregion Update to focal

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update \
 && apt-get install -y locales lsb-release
ENV DEBIAN_FRONTEND=noninteractive PATH=${PATH}:/home/dockeruser/.local/bin
RUN dpkg-reconfigure locales

RUN apt-get -y install pip

## Install ROS Noetic
#
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
#RUN apt-get update && apt-get -y install \
#    ros-noetic-std-msgs \
#    ros-noetic-sensor-msgs \
#    ros-noetic-geometry-msgs
#RUN apt-get install -y python3-rosdep
#RUN rosdep init \
# && rosdep fix-permissions \
# && rosdep update
#RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
#
## endregion Install ROS noetic
#
#RUN echo "Package: python3-opencv\nPin: release *\nPin-Priority: -1" >> /etc/apt/preferences && apt-get update
#RUN mkdir -p /opt/detect
#
#RUN echo "#!/bin/bash\nset -e\nsource /opt/ros/noetic/setup.bash --\nexec \"\$@\"" > /entrypoint.sh
#RUN chmod +x /entrypoint.sh
#
#RUN groupmod --gid 985 video \
#    && useradd -m --uid 1000 dockeruser \
#    && usermod -a -G video dockeruser
#
USER dockeruser

#RUN pip3 install rospy2
#RUN pip3 install ultralytics
#RUN pip3 install pyrealsense2
#RUN pip3 install --upgrade numpy
#RUN pip3 install cv-bridge

WORKDIR "/home/dockeruser"

#ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
