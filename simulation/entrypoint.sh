#!/bin/bash
set -e
source "/usr/share/gazebo/setup.sh" --
source "/opt/ros/noetic/setup.bash" --
source "/root/robot_ws/devel/setup.bash" --

chmod -R +rx /root/robot_ws/src/multirobot_sim
roscore & roslaunch --wait /root/robot_ws/src/multirobot_sim/launch/world-map-docker.launch & npm start