FROM nvcr.io/nvidia/l4t-base:35.3.1

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y --autoremove

RUN apt-get install -y wget sudo gcc-8 g++-8

RUN mkdir -p /opt/opencv_install
WORKDIR /opt/opencv_install

ENV CC=gcc-8 CXX=g++-8

RUN wget https://raw.githubusercontent.com/mdegans/nano_build_opencv/master/build_opencv.sh \
    && sed -i \
      -e 's/local CMAKEFLAGS="/local CMAKEFLAGS="-D CUDA_TOOLKIT_ROOT_DIR=\/usr\/local\/cuda-10.2 -D CUDA_BIN_PATH=\/usr\/local\/cuda-10.2\/bin/g' \
      -e 's/7\.2,8\.7/7\.2/g' \
      build_opencv.sh \
    && chmod +x build_opencv.sh \
    && echo n | ./build_opencv.sh

RUN groupmod --gid 985 video \
    && useradd -m --uid 1000 dockeruser \
    && usermod -a -G video dockeruser

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

#RUN pip3 install rospy2
#RUN pip3 install ultralytics
#RUN pip3 install pyrealsense2
#RUN pip3 install --upgrade numpy
#RUN pip3 install cv-bridge


USER dockeruser
WORKDIR "/home/dockeruser"

#ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
