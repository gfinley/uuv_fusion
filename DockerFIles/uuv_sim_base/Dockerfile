# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
FROM ros:melodic

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get clean
RUN apt-get update && apt-get install -y \
    lsb  \
    unzip \
    wget \
    curl \
    sudo \
    python-vcstool \
    python-rosinstall \
    python3-colcon-common-extensions \
    ros-melodic-rviz \
    ros-melodic-rqt \
    ros-melodic-rqt-common-plugins \
    devilspie \
    ros-melodic-uuv-simulator \
    python3-pip \
    xfce4-terminal \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control
    
    

    

ENV QT_X11_NO_MITSHM=1

RUN pip3 install requests
RUN pip3 install boto3

RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -; \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN apt-get update && apt-get install -y gazebo9
RUN apt-get upgrade -y  libignition-math2

ARG USERNAME=robomaker
RUN groupadd $USERNAME
RUN useradd -ms /bin/bash -g $USERNAME $USERNAME
RUN sh -c 'echo "$USERNAME ALL=(root) NOPASSWD:ALL" >> /etc/sudoers'
USER $USERNAME

RUN sh -c 'cd /home/$USERNAME'

# Download and build our Robot and Simulation application
RUN sh -c 'mkdir -p /home/robomaker/'
RUN sh -c 'cd /home/robomaker/ && git clone https://github.com/gfinley/uuv_fusion.git'




RUN sudo rosdep fix-permissions
RUN rosdep update

