#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# If workspace has a setup.bash, source it
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# Export display for GUI applications
export DISPLAY=${DISPLAY:-:0}

# Create directories if they don't exist
mkdir -p /recordings
mkdir -p /config

# Note: API credentials are optional as DiaROS now uses local models by default

# Print ROS2 environment info
echo "==================================="
echo "DiaROS Docker Environment"
echo "==================================="
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "WORKSPACE: /workspace"
echo "RECORDINGS: /recordings"
echo "==================================="

# Execute the command passed to docker run
exec "$@"