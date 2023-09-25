FROM ros:humble-perception-jammy

ARG USER_NAME
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd ${USER_NAME} --gid ${USER_ID}\
    && useradd -l -m ${USER_NAME} -u ${USER_ID} -g ${USER_ID} -s /bin/bash

USER root

ARG ROS_DISTRO=humble
RUN apt-get update 
RUN apt-get install -y \
    vim

# Install rmf
RUN apt-get update && sudo apt install -y \
    python3-pip \
    curl \
    python3-colcon-mixin \
    ros-dev-tools

WORKDIR /root/navigation_ws/

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
