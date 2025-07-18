#!/bin/bash
# Setup testing environment
echo "=== Setting up test environment ==="
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
echo "Test environment ready!"
echo "Current directory: $(pwd)"
