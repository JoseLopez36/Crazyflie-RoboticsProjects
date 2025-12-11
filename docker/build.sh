#!/bin/bash

# Script to build the Crazyflie Robotics Projects container

set -e

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Get the project root (parent directory of docker/)
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

export DOCKER_BUILDKIT=1

# Build the Docker image
echo "Building the Docker image 'crazyflie-robotics-projects-img'..."
if docker build --rm -t crazyflie-robotics-projects-img --file $SCRIPT_DIR/Dockerfile $PROJECT_ROOT; then
    echo "Image 'crazyflie-robotics-projects-img' built successfully."
else
    echo "Failed to build the Docker image 'crazyflie-robotics-projects-img'."
    exit 1
fi