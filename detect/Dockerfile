FROM nvcr.io/nvidia/l4t-pytorch:r32.7.1-pth1.9-py3

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-key adv --keyserver 'keyserver.ubuntu.com' --recv-key 1A127079A92F09ED

RUN apt-get update \
 && apt-get install -y locales lsb-release curl
ENV DEBIAN_FRONTEND=noninteractive \
    PATH=${PATH}:/home/dockeruser/.local/bin \
    ROS_DISTRO=melodic

# region Install ROS

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
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

RUN apt-get update && apt-get -y install python3.8
ENV VENV_LOCATION=/home/dockeruser
RUN python3.8 -m pip install venv && python3.8 -m venv ${VENV_LOCATION}/.venv

RUN mkdir -p /opt/detect \
    && echo "#!/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash --\nsource /home/dockeruser/.venv/bin/activate\nexec \"\$@\"" > /entrypoint.sh \
    && chmod +x /entrypoint.sh \
    && groupmod --gid 985 video \
    && useradd -m --uid 1000 dockeruser && usermod -a -G video dockeruser

USER dockeruser
WORKDIR /home/dockeruser

#RUN python3.8 -m pip install "cython<3.0.0" && python3 -m pip install pyyaml==6.0
RUN git clone --recursive --branch v2.3.0 https://github.com/pytorch/pytorch
RUN source ${VENV_LOCATION}/.venv/bin/activate && python3.8 -m pip install -r requirements.txt
RUN source ${VENV_LOCATION}/.venv/bin/activate && python3.8 setup.py install

RUN source ${VENV_LOCATION}/.venv/bin/activate && pip3 install ultralytics
RUN source ${VENV_LOCATION}/.venv/bin/activate && pip3 install pyrealsense2
RUN source ${VENV_LOCATION}/.venv/bin/activate && pip3 install cv-bridge

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
