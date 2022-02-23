
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
FROM uuv_sim_base:latest

ENV GAZEBO_MASTER_URI = http://localhost:11311

ENV DISPLAY = :0
# Build the Simulation application

#RUN sh -c 'rm -rf /home/robomaker/uuv_fusion/simulation_ws/src/uuv_simulator'
RUN cd /home/robomaker/uuv_fusion/simulation_ws && /bin/bash -c "source /opt/ros/melodic/setup.bash &&  rosdep install --rosdistro melodic --from-paths src --ignore-src -r -y && colcon build"
    

COPY simulation-entrypoint.sh /home/robomaker/uuv_fusion/simulation_ws/simulation-entrypoint.sh

RUN sh -c 'sudo chmod +x /home/robomaker/uuv_fusion/simulation_ws/simulation-entrypoint.sh'
RUN sh -c 'sudo chown robomaker:robomaker /home/robomaker/uuv_fusion/simulation_ws/simulation-entrypoint.sh'

#maybe only used for local launch and testing? overwirtten when launched from robomaker, maybe
CMD roslaunch aws_int aws_int.launch

ENTRYPOINT [ "/home/robomaker/uuv_fusion/simulation_ws/simulation-entrypoint.sh" ]
