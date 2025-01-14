#!/bin/bash

# Ensure the script is executable before running it: chmod +x run-script.sh

# Set the Docker image name
IMAGE_NAME="ros1_ros2_bridge:latest"

# Run the Docker container with host networking
docker run -it --rm \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    $IMAGE_NAME
