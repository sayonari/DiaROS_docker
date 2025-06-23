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

# Check if API credentials are present
if [ ! -f "$GOOGLE_APPLICATION_CREDENTIALS" ]; then
    echo "Warning: Google Cloud credentials not found at $GOOGLE_APPLICATION_CREDENTIALS"
    echo "Please mount your credentials file to /config/google_credentials.json"
fi

if [ ! -f "$A3RT_APIKEY" ]; then
    echo "Warning: A3RT API key not found at $A3RT_APIKEY"
    echo "Please mount your API key file to /config/a3rt_apikey.data"
fi

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