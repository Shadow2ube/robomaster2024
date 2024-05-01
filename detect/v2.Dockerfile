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
RUN apt-get install -y build-essential


RUN apt-get -y install pip

RUN groupmod --gid 985 video
RUN useradd -m --uid 1000 dockeruser
RUN usermod -a -G video dockeruser
RUN mkdir -p /opt/extra
RUN chown dockeruser /opt/extra
USER dockeruser

RUN pip3 install ultralytics
RUN pip3 install --upgrade numpy
RUN pip3 install pyrealsense2
RUN pip3 install rospy2
RUN pip3 install cv-bridge

USER root

RUN rm -r /home/dockeruser/.local/lib/python3.8/site-packages/cv2/

RUN apt-get -y install libgl1-mesa-glx libglib2.0-dev
RUN apt-get -y install ros-noetic-std-msgs

RUN mkdir -p /root/test
RUN mkdir -p /opt/detect

ENV PATH=${PATH}:/home/dockeruser/.local/bin:/usr/local/cuda-10.2/bin

# region Install opencv with cuda

RUN mkdir -p /opt/opencv
WORKDIR /opt/opencv

#RUN echo gotta recompile
#RUN wget https://raw.githubusercontent.com/Shadow2ube/robomaster2024/main/opencv.sh
#RUN chmod +x opencv.sh
#RUN ./opencv.sh

RUN mkdir -p src/opencv src/opencv_contrib
RUN wget https://github.com/opencv/opencv/archive/4.4.0.tar.gz -O opencv
RUN wget https://github.com/opencv/opencv_contrib/archive/4.4.0.tar.gz -O opencv_contrib
RUN tar -xf opencv -C src/opencv --strip-components 1
RUN tar -xf opencv_contrib -C src/opencv_contrib --strip-components 1
RUN mkdir /opt/opencv/src/opencv/build
WORKDIR /opt/opencv/src/opencv/build
ENV PKG_CONFIG_PATH="$ROOTDIR"/lib/pkgconfig:$PKG_CONFIG_PATH
RUN apt-get install -y --no-install-recommends \
  python3-pip \
  python3-distutils \
  python3-dev \
  python3-setuptools \
  python3-matplotlib \
  build-essential \
  gfortran \
  git \
  cmake \
  curl \
  unzip \
  gnupg \
  libopencv-dev \
  libopenblas-dev \
  liblapack-dev \
  libblas-dev \
  libhdf5-serial-dev \
  hdf5-tools \
  libhdf5-dev \
  zlib1g-dev \
  zip \
  pkg-config \
  libavcodec-dev \
  libavformat-dev \
  libswscale-dev \
  libtbb2 \
  libtbb-dev \
  libjpeg-dev \
  libpng-dev \
  libtiff-dev \
  libv4l-dev \
  v4l-utils \
  libdc1394-22-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgtk2.0-dev \
  libjpeg8-dev \
  libopenmpi-dev \
  openmpi-bin \
  openmpi-common \
  protobuf-compiler \
  libprotoc-dev \
  llvm-9 \
  llvm-9-dev \
  && apt-get -y purge *libopencv*

RUN cmake \
  -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local/opencv \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
  -D WITH_VTK=OFF \
  -D WITH_QT=OFF \
  -D WITH_GTK=OFF \
  -D WITH_GSTREAMER=ON \
  -D WITH_LIBV4L=ON \
  -D WITH_CUDA=ON \
  -D WITH_CUDNN=ON \
  -D CUDA_ARCH_BIN="5.3,6.2,7.2" \
  -D CUDA_ARCH_PTX="" \
  -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.2 \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_TESTS=OFF \
  -D BUILD_PERF_TESTS=OFF \
  -D BUILD_opencv_viz=OFF \
  -D BUILD_opencv_python2=OFF \
  -D BUILD_opencv_python3=ON \
  ..

RUN make -j4
RUN make install
ENV OpenCV_DIR=/opt/opencv

# endregion Install opencv with cuda

RUN echo "#!/bin/bash\nset -e\nsource /opt/ros/noetic/setup.bash --\nexec \"\$@\"" > /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER dockeruser

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
