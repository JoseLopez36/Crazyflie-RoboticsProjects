#!/bin/bash

# Script to stop the Crazyflie Robotics Projects container

set -e

CONTAINER_NAME="crazyflie-robotics-projects"

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Stopping container ${CONTAINER_NAME}..."
    docker stop ${CONTAINER_NAME}
    echo "Container stopped."
else
    echo "Container ${CONTAINER_NAME} is not running."
fi