FROM federicolanzani/opencv-cuda-jetson:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# region Update to focal

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y update-manager-core
RUN yes | do-release-upgrade

# endregion Update to focal

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

RUN pip3 install --upgrade numpy

USER root

RUN rm -r /home/dockeruser/.local/lib/python3.8/site-packages/cv2/

RUN apt-get -y install libgl1-mesa-glx libglib2.0-dev
RUN apt-get -y install ros-noetic-std-msgs

RUN mkdir -p /root/test
RUN mkdir -p /opt/detect

ENV PATH=${PATH}:/home/dockeruser/.local/bin:/usr/local/cuda-10.2/bin

RUN echo "#!/bin/bash\nset -e\nsource /opt/ros/noetic/setup.bash --\nexec \"\$@\"" > /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER dockeruser

RUN pip3 install ultralytics
RUN pip3 install pyrealsense2
RUN pip3 install rospy2
RUN pip3 install cv-bridge

WORKDIR "/home/dockeruser"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
