FROM ubuntu:focal as pt_to_onnx

ARG MODEL_PATH=./model.pt
ADD ${MODEL_PATH} /model.pt

RUN apt-get update \
    && apt-get install python3 python3-pip \
    && pip3 install ultralytics \
    && yolo export model=model.pt format=onnx

FROM nvcr.io/nvidia/l4t-cuda:10.2.460-runtime


ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update \
 && apt-get install -y locales lsb-release curl
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=noetic

# region Install ROS

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update
RUN apt-get -y install \
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

RUN mkdir -p /opt/detect

COPY --from=pt_to_onnx /model.onnx /opt/detect/

RUN chown dockeruser:dockeruser /opt/detect -R  \
    && echo "#!/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash --\nsource /home/dockeruser/.venv/bin/activate\nexec \"\$@\"" > /entrypoint.sh \
    && chmod +x /entrypoint.sh \
    && groupmod --gid 985 video \
    && useradd -m --uid 1000 dockeruser && usermod -a -G video dockeruser

USER dockeruser

WORKDIR /home/dockeruser

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
