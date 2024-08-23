#!/bin/bash
set -e
source "/usr/share/gazebo/setup.sh" --
source "/opt/ros/noetic/setup.bash" --
source "/root/robot_ws/devel/setup.bash" --
npm run deploy --- -m
cp -R /root/robot_ws/src/turtlebot3/turtlebot3_description /root/gzweb/http/client/assets/turtlebot3_description
cp -R /root/robot_ws/src/rotor_simulations/rotors_description /root/gzweb/http/client/assets/rotors_description
chmod -R +rx /root/robot_ws/src/multirobot_sim
roscore & roslaunch --wait /root/robot_ws/src/multirobot_sim/launch/world-map-docker.launch & npm start