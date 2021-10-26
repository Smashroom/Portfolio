# https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/
FROM dorowu/ubuntu-desktop-lxde-vnc:bionic

# install packages
RUN apt-get update && \
    apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    build-essential \
    gcc \
    g++ \
    ssh \
    tmux \
    screen

# Getting supervisorD to look after ssh
RUN echo "[program:sshd]" >> /etc/supervisor/conf.d/supervisord.conf
RUN echo "command=/usr/sbin/sshd -D" >> /etc/supervisor/conf.d/supervisord.conf

## -------- Pulled from  https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/bionic/ros-core/Dockerfile
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list for ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# setup environment vars
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=melodic

# install ros packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    ros-${ROS_DISTRO}-desktop-full=1.4.1-0* \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

# -------- My Stuff

# Define variables
ARG USERNAME=ubuntu
ARG WORKDIR=/home/ubuntu

ENV USERNAME=${USERNAME}
ENV WORKDIR=${WORKDIR}


# Forces this user :D
ENV USER=${USERNAME}

# Create user and give it sudo
RUN useradd -ms /bin/bash $USERNAME
RUN usermod -aG sudo ${USERNAME}

# Working dir
WORKDIR ${WORKDIR}

# Get SSH running
RUN service ssh restart
RUN update-rc.d ssh defaults
RUN update-rc.d ssh enable 2 3 4

# Copy files across
ARG WORKSPACE=/home/ubuntu/catkin_ws
ENV WORKSPACE=${WORKSPACE}


ENV VNC_PASSWORD=ubuntu
ENV PASSWORD=${VNC_PASSWORD}
ENV RESOLUTION=1920x1080


