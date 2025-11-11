# Dockerfile for Crazyflie ROS2 Gazebo Simulation
# Based on sim_cf2: https://github.com/CrazyflieTHI/sim_cf2

# Ubuntu 22.04
FROM ubuntu:jammy

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS2_DISTRO=humble
ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ARG HOME=/root

# Tools useful during development
RUN apt-get update \
 && apt install -y \
        build-essential \
        atop \
        clang \
        cmake \
        cppcheck \
        expect \
        gdb \
        git \
        gnutls-bin \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
        libfcl-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libspdlog-dev \
        libusb-dev \
        libboost-thread-dev \
        libboost-system-dev \
        libboost-filesystem-dev \
        libboost-regex-dev \
        libboost-program-options-dev \
        libconsole-bridge-dev \
        libpoco-dev \
        libtinyxml2-dev \
        liblz4-dev \
        libbz2-dev \
        uuid-dev \
        liblog4cxx-dev \
        libgpgme-dev \
        libgtest-dev \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-setuptools \
        python3-pip \
        python3-venv \
        python3-nose \
        python3-pycryptodome \
        python3-defusedxml \
        python3-mock \
        python3-netifaces \
        python3-gnupg \
        python3-psutil \
        vim \
        tmux \
        tmuxinator \
        nano \
        net-tools \
        iputils-ping \
        xvfb \
        curl \
        jq \
        wget \
        ranger \
        htop \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        rfkill \
        sudo \
        usbutils \
        software-properties-common \
        genromfs \
        ninja-build \
        protobuf-compiler \
        libeigen3-dev \
        libxml2-utils \
        lsb-release \
        ca-certificates \
        gnupg \
        libncurses5-dev \
 && add-apt-repository universe \
 && apt-get clean -qq

RUN apt-get update \
 && apt install -y \
        python3-flake8 \
        python3-flake8-blind-except \
        python3-flake8-builtins \
        python3-flake8-class-newline \
        python3-flake8-comprehensions \
        python3-flake8-deprecated \
        python3-flake8-docstrings \
        python3-flake8-import-order \
        python3-flake8-quotes \
        python3-pytest \
        python3-pytest-cov \
        python3-pytest-repeat \
        python3-pytest-rerunfailures

# Install SSH client for git operations
ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v"
USER root

RUN --mount=type=ssh id=default \
    mkdir -p -m 0600 ~/.ssh/ && \
    ssh-keyscan github.com >> ~/.ssh/known_hosts

#----- ROS2 humble installation -----#
RUN apt update && sudo apt install locales \
  &&  locale-gen en_US en_US.UTF-8 \
  &&  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && export LANG=en_US.UTF-8

# Set up ROS2 repository
RUN /bin/sh -c 'sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'

RUN apt update && sudo apt install -y \
   python3-rosdep \
   ros-dev-tools \
   libboost-program-options-dev \
   libusb-1.0-0-dev \
   python-is-python3

RUN apt-get upgrade -y && sudo apt install ros-${ROS2_DISTRO}-desktop -y

RUN apt-get install -y python3-colcon-common-extensions
#----- End of ROS2 humble installation -----#

# Install ROS2 packages required for sim_cf2
RUN apt-get update && apt-get install -y \
    ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS2_DISTRO}-xacro

# Install SLAM and Navigation packages
RUN apt-get update && apt-get install -y \
    ros-${ROS2_DISTRO}-slam-toolbox \
    ros-${ROS2_DISTRO}-navigation2 \
    ros-${ROS2_DISTRO}-nav2-bringup

# Install Gazebo 11
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo-dev

# Install ROS2 Gazebo integration (provides gazebo_msgs)
RUN apt-get update && apt-get install -y \
    ros-${ROS2_DISTRO}-gazebo-ros

# Add alias for sourcing ROS2
RUN echo "alias source_ros2='source /opt/ros/${ROS2_DISTRO}/setup.bash'" >> $HOME/.bashrc

# Setup ROS2 environment variables
RUN echo "export ROS_LOCALHOST_ONLY=1" >> $HOME/.bashrc
RUN echo "export ROS_DOMAIN_ID=0" >> $HOME/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $HOME/.bashrc

# Source ROS2 in bashrc
RUN echo "source_ros2" >> $HOME/.bashrc

#----- Install GNU Scientific Library (GSL) for crazyflie-firmware SITL -----#
# GSL is required for the firmware SITL build
RUN cd /tmp && \
    wget https://ftp.eq.uc.pt/software/unix/gnu/gsl/gsl-2.8.tar.gz && \
    tar -xzf gsl-2.8.tar.gz && \
    cd gsl-* && \
    ./configure && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && \
    rm -rf /tmp/gsl-* /tmp/gsl-2.8.tar.gz

#----- Clone and build crazyflie-firmware -----#
WORKDIR $HOME
RUN git clone --recursive https://github.com/CrazyflieTHI/crazyflie-firmware.git && \
    cd crazyflie-firmware && \
    git submodule update --init --recursive

#----- Clone and install crazyflie-lib-python -----#
WORKDIR $HOME
RUN git clone https://github.com/CrazyflieTHI/crazyflie-lib-python.git && \
    cd crazyflie-lib-python && \
    (pip3 install -r requirements.txt 2>/dev/null || true) && \
    pip3 install posix-ipc && \
    pip3 install -e .

#----- Set user root and start bash -----#
USER root
WORKDIR $HOME
CMD ["/bin/bash"]
