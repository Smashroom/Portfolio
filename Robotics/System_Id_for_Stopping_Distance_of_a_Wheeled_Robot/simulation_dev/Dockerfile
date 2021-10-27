FROM misterkoz/ros-melodic-with-vnc-and-gazebo
ARG USERNAME=ubuntu
ARG WORKSPACE=/home/ubuntu/catkin_ws

COPY project_root ${WORKSPACE}
RUN chown -R ${USERNAME}:${USERNAME} ${WORKSPACE}

RUN rosdep init
RUN rosdep fix-permissions

# Assume Ubuntu user - we don't want ROS running as root!
USER ${USERNAME}

# bootstrap rosdep
RUN rosdep update

RUN bash -c "source /opt/ros/melodic/setup.bash && \
             cd ${WORKSPACE}/src && \
             catkin_init_workspace && \
             cd .. && \
             catkin_make && \
             rosdep update"

COPY env_files/bash_profile /home/ubuntu/.bashrc
COPY env_files/vimrc /home/ubuntu/.vimrc

# the startup script needs to run as root
USER root
