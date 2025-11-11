#!/bin/bash

# Script to remove the Crazyflie Robotics Projects container

set -e

CONTAINER_NAME="crazyflie-robotics-projects"

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "Container is running. Stopping it first..."
        docker stop ${CONTAINER_NAME}
    fi
    echo "Removing container ${CONTAINER_NAME}..."
    docker rm ${CONTAINER_NAME}
    echo "Container removed."
else
    echo "Container ${CONTAINER_NAME} does not exist."
fi