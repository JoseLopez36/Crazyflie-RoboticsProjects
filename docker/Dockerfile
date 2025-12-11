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
        python-is-python3 \
        vim \
        tmux \
        tmuxinator \
        nano \
        net-tools \
        iputils-ping \
        xvfb \
        curl\
        jq\
        wget\
        ranger \
        htop \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        rfkill \
        sudo \
        usbutils \
        software-properties-common \
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

ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v"
USER root

RUN --mount=type=ssh id=default \
    mkdir -p -m 0600 ~/.ssh/ && \
    ssh-keyscan github.com >> ~/.ssh/known_hosts

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

# Install crazyswarm2
RUN mkdir -p $HOME/ros2_ws/src \
  && cd $HOME/ros2_ws/src \
  && git clone https://github.com/IMRCLab/crazyswarm2 --recursive

# Replace crazyflie_server.py adapted for sitl
RUN rm $HOME/ros2_ws/src/crazyswarm2/crazyflie/scripts/crazyflie_server.py
COPY docker/to_copy/crazyflie_server.py $HOME/ros2_ws/src/crazyswarm2/crazyflie/scripts/
RUN chmod +x $HOME/ros2_ws/src/crazyswarm2/crazyflie/scripts/crazyflie_server.py

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

# Install DARP dependencies
RUN pip3 install --no-cache-dir --upgrade \
    numpy==1.26.4 \
    scipy \
    scikit-learn \
    opencv-python \
    pygame \
    numba \
    coverage>=7.0.0
    
# Build crazyswarm2
WORKDIR $HOME/ros2_ws
RUN bash -c "source /opt/ros/${ROS2_DISTRO}/setup.bash; colcon build --symlink-install --merge-install --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON";

# Add aliases for sourcing for ros2 and ros2 workspace
RUN echo "alias ros2_ws='source $HOME/Crazyflie-RoboticsProjects/ros2_ws/install/setup.bash && source $HOME/ros2_ws/install/setup.bash'" >> $HOME/.bashrc
RUN echo "alias source_ros2='source /opt/ros/${ROS2_DISTRO}/setup.bash'" >> $HOME/.bashrc
RUN echo "export PATH='$HOME/.local/bin:$PATH'" >> $HOME/.bashrc

WORKDIR $HOME

RUN echo "ros2_ws" >> $HOME/.bashrc
RUN echo "source_ros2" >> $HOME/.bashrc

WORKDIR $HOME
COPY docker/to_copy/aliases $HOME/.bash_aliases
COPY docker/to_copy/nanorc $HOME/.nanorc
COPY docker/to_copy/tmux $HOME/.tmux.conf
COPY docker/to_copy/ranger $HOME/.config/ranger/rc.conf

USER root
WORKDIR $HOME
CMD ["/bin/bash"]