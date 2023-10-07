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
   
# Install rviz2 and rqt tools
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt* 


# Install navigation packages
RUN apt-get update && sudo apt install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup 

# Build cartographer 
WORKDIR /
RUN git clone https://github.com/cartographer-project/cartographer.git
RUN /cartographer/scripts/install_debs_cmake.sh && rm -rf /var/lib/apt/lists/*
RUN /cartographer/scripts/install_abseil.sh && rm -rf /var/lib/apt/lists/*
RUN /cartographer/scripts/install_cartographer_cmake.sh

# Install cartographer_ros
RUN apt-get update && sudo apt install -y \
    ros-${ROS_DISTRO}-cartographer-ros

WORKDIR /root/navigation_ws/

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
