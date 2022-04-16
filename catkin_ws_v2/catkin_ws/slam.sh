#!/bin/bash
sleep 5

source /home/slam/catkin_ws_v2/catkin_ws/devel/setup.bash
gnome-terminal -- roslaunch server launch.launch
