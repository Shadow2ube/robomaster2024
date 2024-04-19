#FROM osrf/ros:noetic-desktop-full
FROM arm64v8/ros:noetic-ros-core

RUN apt-get update
RUN apt-get -y install curl python3 pip lsb-release apt-transport-https

RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
    | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" \
    | tee /etc/apt/sources.list.d/librealsense.list

RUN apt-get update

ENV PATH="${PATH}:/home/dockeruser/.local/bin"

RUN apt-get -y install python3-pip libopenblas-base libopenmpi-dev libomp-dev
RUN apt-get -y install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev

RUN groupmod --gid 985 video
RUN useradd -m --uid 1000 dockeruser
RUN usermod -a -G video dockeruser
USER dockeruser

RUN pip3 install 'Cython<3'
RUN curl https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -o pytorch.whl \
    && pip3 install numpy pytorch.whl
RUN git clone --branch v0.3.0 https://github.com/pytorch/vision torchvision \
    && cd torchvision \
    && export BUILD_VERSION=0.3.0 \
    && python3 setup.py install --user \
    && cd ../ \
    && pip install 'pillow<7'

RUN pip3 install opencv-python
RUN pip3 install pyrealsense2
RUN pip3 install ultralytics
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
