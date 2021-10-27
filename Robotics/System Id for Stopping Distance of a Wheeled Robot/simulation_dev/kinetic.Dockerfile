# https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc/
FROM dorowu/ubuntu-desktop-lxde-vnc:xenial

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

# setup environment vars
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=Kinetic

# Kinetic installer
COPY env_files/install_ros_kinetic.sh /root/install_ros_kinetic.sh
RUN chmod 755 /root/install_ros_kinetic.sh
RUN bash /root/install_ros_kinetic.sh

# packages
RUN apt-get install -y ros-kinetic-joy ros-kinetic-teleop-twist-joy \
    ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch \
    ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino \
    ros-kinetic-rosserial-python ros-kinetic-rosserial-server \
    ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs \
    ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf \
    ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view \
    ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers \
    python-rosdep python-rosinstall python-vcstools \
    ros-kinetic-moveit

# Working dir
ARG WORKDIR=/home/ubuntu
ENV WORKDIR=${WORKDIR}
WORKDIR ${WORKDIR}
ARG WORKSPACE=/home/ubuntu/catkin_ws
ENV WORKSPACE=${WORKSPACE}


ENV RESOLUTION=1920x1080
ENV VNC_PASSWORD=ubuntu
ENV PASSWORD=${VNC_PASSWORD}

ARG USERNAME=ubuntu
ENV USERNAME=${USERNAME}
ENV USER=${USERNAME}

# Create user and give it sudo
RUN useradd -ms /bin/bash $USERNAME
RUN usermod -aG sudo ${USERNAME}

# Get SSH running
RUN service ssh restart
RUN update-rc.d ssh defaults
RUN update-rc.d ssh enable 2 3 4

# profiledir
RUN chown -R ${USERNAME}  ${WORKDIR}/.ros
USER ${USERNAME}

COPY env_files/bash_profile /home/ubuntu/.bashrc
COPY env_files/vimrc /home/ubuntu/.vimrc

# bootstrap rosdep
RUN rosdep update

# the startup script needs to run as root
USER root
