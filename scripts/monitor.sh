#!/bin/bash

# DiaROS Docker Monitoring Script

set -e

# Function to display menu
show_menu() {
    echo ""
    echo "==================================="
    echo "DiaROS Monitoring Tools"
    echo "==================================="
    echo "1. rqt (Full GUI Dashboard)"
    echo "2. rqt_graph (Node Communication Graph)"
    echo "3. rqt_plot (Real-time Data Plotting)"
    echo "4. rqt_topic (Topic Monitor)"
    echo "5. rqt_bag (Bag File Viewer)"
    echo "6. rqt_console (Log Console)"
    echo "7. ros2 topic list (Command Line)"
    echo "8. ros2 bag record (Start Recording)"
    echo "9. Exit"
    echo "==================================="
    echo -n "Select option [1-9]: "
}

# Check if container is running
if ! docker ps | grep -q diaros_container; then
    echo "Error: DiaROS container is not running."
    echo "Please run ./scripts/run.sh first."
    exit 1
fi

# Check for XQuartz on macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    if ! command -v xquartz &> /dev/null && ! pgrep -x "XQuartz" > /dev/null; then
        echo "Warning: XQuartz may not be running."
        echo "Please ensure XQuartz is installed and running:"
        echo "  1. Install XQuartz from https://www.xquartz.org/"
        echo "  2. Open XQuartz"
        echo "  3. In XQuartz preferences, go to Security tab"
        echo "  4. Check 'Allow connections from network clients'"
        echo "  5. Run: xhost +localhost"
        echo ""
    fi
fi

# Main loop
while true; do
    show_menu
    read -r choice
    
    case $choice in
        1)
            echo "Starting rqt..."
            docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && rqt"
            ;;
        2)
            echo "Starting rqt_graph..."
            docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && rqt_graph"
            ;;
        3)
            echo "Starting rqt_plot..."
            docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && rqt_plot"
            ;;
        4)
            echo "Starting rqt_topic..."
            docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && rqt_topic"
            ;;
        5)
            echo "Starting rqt_bag..."
            docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && rqt_bag"
            ;;
        6)
            echo "Starting rqt_console..."
            docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && rqt_console"
            ;;
        7)
            echo "Listing ROS2 topics..."
            docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && ros2 topic list -v"
            echo ""
            echo "Press Enter to continue..."
            read -r
            ;;
        8)
            echo "Starting bag recording..."
            echo "Enter topics to record (space-separated, or 'all' for all topics):"
            read -r topics
            
            if [ "$topics" = "all" ]; then
                echo "Recording all topics to /recordings/diaros_$(date +%Y%m%d_%H%M%S)"
                docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && cd /recordings && ros2 bag record -a -o diaros_$(date +%Y%m%d_%H%M%S)"
            else
                echo "Recording specified topics to /recordings/diaros_$(date +%Y%m%d_%H%M%S)"
                docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && cd /recordings && ros2 bag record $topics -o diaros_$(date +%Y%m%d_%H%M%S)"
            fi
            ;;
        9)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid option. Please try again."
            ;;
    esac
done