FROM nvcr.io/nvidia/l4t-base:r35.1.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y --autoremove

RUN software-properties-common \
    && yes | apt-add-repository universe
RUN apt-get update && apt-get install -y \
    libglew-dev \
    libtiff5-dev \
    zlib1g-dev \
    libjpeg-dev \
    libpng12-dev \
    libjasper-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libpostproc-dev \
    libswscale-dev \
    libeigen3-dev \
    libtbb-dev \
    libgtk2.0-dev \
    pkg-config \
    git \
    python-dev \
    python-numpy \
    python-py \
    python-pytest \
    python3-dev \
    python3-numpy \
    python3-py \
    python3-pytest

RUN mkdir -p /opt/opencv_install
WORKDIR /opt/opencv_install

RUN git clone --recursive https://github.com/opencv/opencv-python.git

ENV CMAKE_ARGS="${CMAKE_ARGS} -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DBUILD_PNG=OFF \
    -DBUILD_TIFF=OFF \
    -DBUILD_TBB=OFF \
    -DBUILD_JPEG=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_ZLIB=OFF \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_JAVA=OFF \
    -DBUILD_opencv_python2=ON \
    -DBUILD_opencv_python3=OFF \
    -DENABLE_NEON=ON \
    -DWITH_OPENCL=OFF \
    -DWITH_OPENMP=OFF \
    -DWITH_FFMPEG=ON \
    -DWITH_GSTREAMER=OFF \
    -DWITH_GSTREAMER_0_10=OFF \
    -DWITH_CUDA=ON \
    -DWITH_GTK=ON \
    -DWITH_VTK=OFF \
    -DWITH_TBB=ON \
    -DWITH_1394=OFF \
    -DWITH_OPENEXR=OFF \
    -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2 \
    -DCUDA_ARCH_BIN=6.2 \
    -DCUDA_ARCH_PTX=\"\" \
    -DINSTALL_C_EXAMPLES=ON \
    -DINSTALL_TESTS=OFF" ENABLE_CONTRIB=1
RUN pip wheel . --verbose


#ENV LANG=C.UTF-8
#ENV LC_ALL=C.UTF-8

#RUN apt-get update \
# && apt-get install -y locales lsb-release
#ENV DEBIAN_FRONTEND=noninteractive PATH=${PATH}:/home/dockeruser/.local/bin
#RUN dpkg-reconfigure locales

#RUN apt-get -y install pip

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

RUN groupmod --gid 985 video \
    && useradd -m --uid 1000 dockeruser \
    && usermod -a -G video dockeruser

USER dockeruser

#RUN pip3 install rospy2
#RUN pip3 install ultralytics
#RUN pip3 install pyrealsense2
#RUN pip3 install --upgrade numpy
#RUN pip3 install cv-bridge

WORKDIR "/home/dockeruser"

#ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
