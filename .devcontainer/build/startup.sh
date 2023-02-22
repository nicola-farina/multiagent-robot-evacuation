#!/bin/bash
export ROS_DOMAIN_ID=3

# Source ROS2
source /opt/ros/humble/setup.bash

# Install the shelfino packages if available
if [ -f root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash;
fi

# Install the workspace packages if available
if [ -f ${WORKSPACE}/install/setup.bash ]; then
    source ${WORKSPACE}/install/setup.bash;
fi
