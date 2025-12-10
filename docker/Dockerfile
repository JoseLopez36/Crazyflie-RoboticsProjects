# Ubuntu 22.04 - ROS2 Humble
FROM ros:humble-ros-base

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_AL=C.UTF-8
ENV ROS2_DISTRO=humble
ENV GZ_RELEASE=garden
ARG DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV TZ=Europe/Zagreb
ARG HOME=/root
ENV DOCKER_BUILDKIT=1
USER root

# Install dependencies
RUN apt-get update \
  && apt-get install -y \
    python3-pip \
    python-is-python3 \
    curl \
    jq \
    git \
    tmux \
    tmuxinator \
    nano

# Install Rviz2
RUN apt-get install -y ros-${ROS2_DISTRO}-rviz2

# Install Gazebo Garden
RUN /bin/sh -c ' curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'
RUN  apt-get update \
  && apt-get install -y gz-${GZ_RELEASE}

# Install CrazySim
WORKDIR $HOME
RUN  git clone https://github.com/gtfactslab/CrazySim.git --recursive \
    && cd $HOME/CrazySim/crazyflie-lib-python \
    &&  pip install -e .

RUN pip install Jinja2
RUN cd $HOME/CrazySim/crazyflie-firmware \
    &&  mkdir -p sitl_make/build && cd sitl_make/build \
    &&  cmake .. \
    &&  make all

# Install ROS2 dependencies
RUN pip3 install rowan transforms3d nicegui
RUN apt-get install -y ros-${ROS2_DISTRO}-tf-transformations \
                        ros-${ROS2_DISTRO}-nav2-map-server \
                        ros-${ROS2_DISTRO}-motion-capture-tracking \
                        ros-${ROS2_DISTRO}-nav2-lifecycle-manager \
                        ros-${ROS2_DISTRO}-rosbridge-suite \
                        ros-${ROS2_DISTRO}-rosbag2-storage-mcap \
                        ros-${ROS2_DISTRO}-vision-msgs
RUN VERSION="releases/mcap-cli/v0.0.55" && \
    RELEASE_URL=$(curl -s https://api.github.com/repos/foxglove/mcap/releases | jq -r --arg VERSION "$VERSION" '.[] | select(.tag_name == $VERSION) | .assets[0].browser_download_url') && \
    echo "Downloading release $VERSION from: $RELEASE_URL" && \
    curl -L -o /bin/mcap "$RELEASE_URL" && \
    cd /bin && chmod +x mcap

# Add aliases for sourcing for ros2 and ros2 workspace
RUN echo "alias ros2_ws='source $HOME/ros2_ws/install/setup.bash'" >> $HOME/.bashrc
RUN echo "alias source_ros2='source /opt/ros/${ROS2_DISTRO}/setup.bash'" >> $HOME/.bashrc
RUN echo "export PATH='$HOME/.local/bin:$PATH'" >> $HOME/.bashrc

WORKDIR $HOME/ros2_ws

RUN echo "ros2_ws" >> $HOME/.bashrc
RUN echo "source_ros2" >> $HOME/.bashrc

WORKDIR $HOME
COPY ../to_copy/aliases $HOME/.bash_aliases
COPY ../to_copy/nanorc $HOME/.nanorc
COPY ../to_copy/tmux $HOME/.tmux.conf
COPY ../to_copy/ranger $HOME/.config/ranger/rc.conf

USER root
WORKDIR $HOME
CMD ["/bin/bash"]