
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
FROM uuv_sim_base:latest

# Build the Simulation application
RUN cd /home/robomaker/uuv_fusion/fusion_ws && /bin/bash -c "source /opt/ros/melodic/setup.bash && rosdep update && rosdep install --rosdistro melodic --from-paths src --ignore-src -r -y && colcon build"
    

COPY robot-entrypoint.sh /home/robomaker/uuv_fusion/fusion_ws/robot-entrypoint.sh

RUN sh -c 'sudo chmod +x /home/robomaker/uuv_fusion/fusion_ws/robot-entrypoint.sh'
RUN sh -c 'sudo chown robomaker:robomaker /home/robomaker/uuv_fusion/fusion_ws/robot-entrypoint.sh'

CMD  roslaunch aws_int aws_int.launch 
ENTRYPOINT [ "/home/robomaker/uuv_fusion/fusion_ws/robot-entrypoint.sh" ]
