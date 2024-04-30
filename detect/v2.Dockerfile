#FROM osrf/ros:noetic-desktop-full
#FROM arm64v8/ros:noetic-ros-core
FROM nvcr.io/nvidia/l4t-base:r35.1.0

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN #echo 'Etc/UTC' > /etc/timezone
RUN #ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

# region ROS install

# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y ros-noetic-desktop-full
RUN apt-get install -y python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# endregion Install ROS noetic

RUN apt-get update && apt-get install -y --no-install-recommends git cmake make wget

# region Install opencvwith cuda

RUN mkdir -p /opt/opencv/build

WORKDIR /opt/opencv

RUN wget https://raw.githubusercontent.com/innerlee/setup/master/zzopencv.sh
RUN chmod +x zzopencv.sh
RUN ./zzopencv.sh

# endregion ROS install

#RUN git clone https://github.com/opencv/opencv && \
#    git clone https://github.com/opencv/opencv_contrib
#
#WORKDIR /opt/opencv/build
#
#RUN apt-get update
#RUN apt-get -y install openjdk-8-jre libvtk7-dev libgflags2.2 vtk7
#
#ENV PATH=${PATH}:/usr/local:/usr/local/cuda-10.2:/usr/local/cuda-10.2/bin
#ENV CUDA_BIN_PATH=/usr/local/cuda-10.2/bin
#RUN ln -s /usr/local/cuda-10.2/ /usr/local/cuda
#
#RUN cmake -DOPENCV_EXTRA_MODULES_PATH=/opt/opencv/opencv_contrib/modules  \
##   -DBUILD_SHARED_LIBS=OFF \
#   -DBUILD_TESTS=OFF \
#   -DBUILD_PERF_TESTS=OFF \
#   -DBUILD_EXAMPLES=OFF \
#   -DWITH_OPENEXR=OFF \
#   -DWITH_CUDA=ON \
#   -DWITH_CUBLAS=ON \
#   -DWITH_CUDNN=ON \
#   -DOPENCV_DNN_CUDA=ON \
#   -GNinja \
#   /opt/opencv/opencv
#
#RUN ninja -j4 install

# endregion

RUN apt-get -y install pip


RUN groupmod --gid 985 video
RUN useradd -m --uid 1000 dockeruser
RUN usermod -a -G video dockeruser
RUN mkdir -p /opt/extra
RUN chown dockeruser /opt/extra
USER dockeruser

RUN pip3 install ultralytics
RUN pip3 install numpy
RUN pip3 install opencv-python
RUN pip3 install pyrealsense2
RUN pip3 install rospy2
RUN pip3 install cv-bridge

USER root

RUN apt-get -y install libgl1-mesa-glx libglib2.0-dev
RUN apt-get -y install ros-noetic-std-msgs

RUN mkdir -p /root/test
RUN mkdir -p /opt/detect

RUN apt-get -y install python3-opencv

ENV PATH=${PATH}:/home/dockeruser/.local/bin

RUN echo "#!/bin/bash\nset -e\nsource /opt/ros/noetic/setup.bash --\nexec \"\$@\"" > entrypoint.sh
RUN chmod +x /entrypoint.sh

USER dockeruser

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
