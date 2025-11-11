#!/bin/bash

# Script to run the Crazyflie Robotics Projects container
# Mounts the repository from host for development

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CONTAINER_NAME="crazyflie-robotics-projects"
IMAGE_NAME="crazyflie-robotics-projects"

# Set up X11 forwarding
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY is not set. GUI applications may not work."
    XAUTH=""
else
    XAUTH=$(mktemp -u /tmp/.docker.xauth.XXXXXX)
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - 2>/dev/null || true
fi

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container ${CONTAINER_NAME} already exists."
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "Container is already running. Attaching to it..."
        docker exec -it ${CONTAINER_NAME} /bin/bash
        exit 0
    else
        echo "Starting existing container..."
        docker start ${CONTAINER_NAME}
        docker exec -it ${CONTAINER_NAME} /bin/bash
        exit 0
    fi
fi

# Build image if it doesn't exist
if ! docker images --format '{{.Repository}}' | grep -q "^${IMAGE_NAME}$"; then
    echo "Building Docker image..."
    docker build -t ${IMAGE_NAME} -f ${PROJECT_ROOT}/docker/Dockerfile ${PROJECT_ROOT}
fi

# Run container with host mount
echo "Starting container ${CONTAINER_NAME}..."

# Build docker run command with conditional mounts
DOCKER_RUN_CMD="docker run -it \
    --name ${CONTAINER_NAME} \
    --net=host \
    --privileged \
    --env=\"DISPLAY=$DISPLAY\" \
    --env=\"QT_X11_NO_MITSHM=1\" \
    --env=\"TERM=xterm-256color\" \
    --env=\"NVIDIA_VISIBLE_DEVICES=all\" \
    --env=\"NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute\" \
    --device=/dev/ttyUSB0 \
    --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" \
    --volume=\"/dev:/dev\" \
    --volume=\"/var/run/dbus/:/var/run/dbus/:z\" \
    --volume=\"${CRAZYFLIE_ROBOTICS_PROJECTS}:/root/Crazyflie-RoboticsProjects:rw\""

# Add XAUTH if DISPLAY is set
if [ -n "$XAUTH" ]; then
    DOCKER_RUN_CMD="$DOCKER_RUN_CMD --env=\"XAUTHORITY=$XAUTH\" --volume=\"$XAUTH:$XAUTH:rw\""
fi

# Add SSH agent forwarding if socket exists
if [ -S "${HOME}/.ssh/ssh_auth_sock" ]; then
    DOCKER_RUN_CMD="$DOCKER_RUN_CMD --volume=\"${HOME}/.ssh/ssh_auth_sock:/ssh-agent\" --env=\"SSH_AUTH_SOCK=/ssh-agent\""
fi

DOCKER_RUN_CMD="$DOCKER_RUN_CMD ${IMAGE_NAME}"

# Execute the command
eval $DOCKER_RUN_CMD