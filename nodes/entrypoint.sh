#!/bin/bash
set -e
#change permissions
chmod +x -R /robot_ws/src/multirobot_sim/scripts
# setup ros environment
source "/opt/ros/noetic/setup.bash" --
source "/robot_ws/devel/setup.bash" --
#run the startup script
python3 /robot_ws/src/multirobot_sim/scripts/roschain/startup.py
#roslaunch rotors_gazebo uav_launch.launch
roslaunch --local --screen /robot_ws/src/multirobot_sim/launch/blockchain.launch > /dev/stdout 2> /dev/stderr

exec "$@"