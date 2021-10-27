#!/bin/bash
cd ${WORKSPACE}/src
catkin_init_workspace
cd ..
catkin_make
rosdep update

source /home/ubuntu/catkin_ws/devel/setup.bash
