#!/usr/bin/env zsh

sudo -H pip3 install -U rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg

mkdir catkin3_ws
cd catkin3_ws || exit

sudo rosdep init
rosdep update

catkin config --init -DCMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3 --blacklist rqt_rviz rviz_plugin_tutorials librviz_tutorial --install

rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall

wstool init -j8 src melodic-ros_comm-wet.rosinstall

wstool update -j4 -t src

export ROS_PYTHON_VERSION=3

rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster
