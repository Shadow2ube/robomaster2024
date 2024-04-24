FROM osrf/ros:noetic-desktop-full
#FROM arm64v8/ros:noetic-ros-core
FROM nvcr.io/nvidia/l4t-base:r35.1.0

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN echo 'Etc/UTC' > /etc/timezone
RUN ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime


# region Install ROS noetic

RUN apt-get update && apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

RUN apt-get update && apt-get install --no-install-recommends -y \
    dirmngr \
    gnupg2 \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rosdep init  \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/* \

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.4.1-0* \
    ros-noetic-robot=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

COPY scripts/ros_entrypoint.sh /

# endregion Install ROS noetic

RUN apt-get update && apt-get install -y --no-install-recommends git cmake make \
    && rm -rf /var/lib/apt/lists/*

# region Install opencvwith cuda

RUN mkdir -p /opt/opencv/build

WORKDIR /opt/opencv

RUN git clone https://github.com/opencv/opencv && \
    git clone https://github.com/opencv/opencv_contrib

WORKDIR /opt/opencv

RUN cmake -DOPENCV_EXTRA_MODULES_PATH=/opt/opencv/opencv_contrib/modules \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_TESTS=OFF \
    -BUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DWITH_OPENEXR=OFF \
    -DWITH_CUDA=ON \
    -DWITH_CUBLAS=ON \
    -DWITH_CUDNN=ON \
    -DOPENCV_DNN_CUDA=ON \
    /opt/opencv/opencv

RUN make -j4 install

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

USER dockeruser

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
