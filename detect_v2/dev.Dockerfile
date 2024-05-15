FROM osrf/ros:melodic-desktop-full

RUN apt-get update
RUN apt-get install -y python3-pip
RUN apt-get install -y cmake make gcc g++ python3-catkin-tools

RUN apt-get install -y ros-melodic-cv-bridge
RUN apt-get install -y libopencv-dev

RUN useradd -m --uid 1000 dockeruser && usermod -a -G video dockeruser


#ENV OLD_PYTHONPATH=${PYTHONPATH} PYTHONPATH="${PYTHONPATH}/usr/lib/python2.7/dist-packages"
#ENV PYTHONPATH=""

#USER dockeruser

ENTRYPOINT ["/ros_entrypoint.sh"]
#ENV PYTHONPATH="${OLD_PYTHONPATH}/usr/lib/python3.6"
CMD ["bash"]
