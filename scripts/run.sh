#!/bin/bash

# DiaROS Docker Run Script

set -e

# Check if container is already running
if docker ps | grep -q diaros_container; then
    echo "DiaROS container is already running."
    echo "Attaching to existing container..."
    docker exec -it diaros_container bash
else
    echo "Starting DiaROS container..."
    
    # Check for macOS and XQuartz
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # Check if XQuartz is running
        if ! pgrep -x "XQuartz" > /dev/null; then
            echo "Warning: XQuartz is not running. GUI applications may not work."
            echo "Please start XQuartz and run: xhost +localhost"
        fi
    fi
    
    # Start the container using docker-compose
    docker-compose up -d
    
    # Wait a moment for container to start
    sleep 2
    
    # Attach to the container
    docker exec -it diaros_container bash
fi