#!/bin/bash

cd /home/robomaker/uuv_fusion/simulation_ws/
source /opt/ros/melodic/setup.bash
source /usr/share/gazebo-9/setup.sh
source ./install/setup.sh
printenv



exec "${@:1}"
