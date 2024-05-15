FROM ubuntu:focal as pt_to_onnx

ENV DEBIAN_FRONTEND=noninteractive

ARG MODEL_PATH=./model.pt
ADD ${MODEL_PATH} /model.pt

RUN apt-get update \
    && apt-get install -y python3 python3-pip ffmpeg libsm6 libxext6
RUN pip3 install ultralytics
RUN yolo export model=model.pt format=onnx

FROM nvcr.io/nvidia/l4t-ml:r32.7.1-py3

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y locales lsb-release curl
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic
RUN dpkg-reconfigure locales

# region Install ROS

#RUN echo "\nPackage: libopencv-dev\nPin: release *\nPin-Priority: -1\n" >> /etc/apt/preferences
#RUN apt-mark hold libopencv-dev opencv-dev opencv-libs opencv-main opencv-scripts opencv-python opencv-licenses \

RUN echo 'Etc/UTC' > /etc/timezone \
    && rm /etc/localtime && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends dirmngr gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

# setup sources.list
RUN echo "deb http://snapshots.ros.org/melodic/final/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-snapshots.list

ENV ROS_DISTRO=melodic
RUN apt-get update && apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-overwrite" \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-cv-bridge
RUN apt-get install -y python3-rosdep
RUN rosdep init \
    && rosdep fix-permissions \
    && rosdep update \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# endregion Install ROS

RUN mkdir -p /opt/detect

COPY --from=pt_to_onnx /model.onnx /opt/detect/

RUN useradd -m --uid 1000 dockeruser \
    && chown dockeruser:dockeruser /opt/detect -R  \
    && echo "#!/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash --\nsource /ws/devel/setup.bash\nexec \"\$@\"" > /entrypoint.sh \
    && chmod +x /entrypoint.sh \
    && groupmod --gid 985 video \
    && usermod -a -G video dockeruser

RUN apt-get install -y cmake make gcc g++ python3-catkin-tools

ADD ./ws /ws

WORKDIR /ws

USER dockeruser

RUN /usr/bin/catkin build

ENTRYPOINT ["/entrypoint.sh"]

ADD ./ /opt/detect

CMD ["bash"]
