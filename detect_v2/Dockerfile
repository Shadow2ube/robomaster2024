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

RUN echo "libopencv-dev hold" | sudo dpkg --set-selections \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update
ENV ROS_DISTRO=melodic
RUN apt-get -y install \
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

RUN chown dockeruser:dockeruser /opt/detect -R  \
    && echo "#!/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash --\nsource /ws/devel/setup.bash\nexec \"\$@\"" > /entrypoint.sh \
    && chmod +x /entrypoint.sh \
    && groupmod --gid 985 video \
    && useradd -m --uid 1000 dockeruser && usermod -a -G video dockeruser

RUN apt-get install -y cmake cmake gcc g++

USER dockeruser


ADD ./ws /

WORKDIR ws

RUN catkin build

ENTRYPOINT ["/entrypoint.sh"]

ADD ./ /opt/detect

CMD ["bash"]
