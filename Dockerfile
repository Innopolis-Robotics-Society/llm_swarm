FROM nvidia/cuda:12.9.0-devel-ubuntu22.04

# Set ROS distro
ENV ROS_DISTRO=humble

# Set arguments for user creation
ARG USERNAME=fabian
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
 && rm -rf /var/lib/apt/lists/*

#Create USER
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Update and install necessary packages
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y sudo curl gnupg2 lsb-release net-tools python3-pip \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Build deps for pip packages (pyaudio needs portaudio headers)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev python3-venv \
    build-essential pkg-config \
    portaudio19-dev libasound2-dev \
    libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# Force protobuf from PyPI so onnxruntime can resolve
RUN pip install --no-cache-dir -U pip setuptools wheel \
 && pip install --no-cache-dir --index-url https://pypi.org/simple \
      "protobuf>=4.21.12,<6"

# Common python libs
RUN python3 -m pip install --no-cache-dir --upgrade pip \
 && python3 -m pip install --no-cache-dir \ 
    onnxruntime==1.18.1 \
    ultralytics \
    "numpy<2" \
    sounddevice \
    vosk \
    opencv-python \
    pyaudio \
    open3d \
    matplotlib \
    pyyaml \
    rapidfuzz \
    torch \
    torchvision \
    segmentation-models-pytorch 

# System tools
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    sudo \
    git \
    wget \
    nano \
    tree \
    iputils-ping \
    python3-pip \
    --fix-missing

# Extra libs
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    at-spi2-core x11-apps xauth \
    libgflags-dev \
    libdw-dev \
    nlohmann-json3-dev  \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    at-spi2-core \
    x11-apps \
    xauth \
    alsa-utils \
    pulseaudio \
    portaudio19-dev \
    --fix-missing

# Install slcan-utils from source if not available
RUN git clone https://github.com/linux-can/can-utils.git \
    && cd can-utils \
    && make \
    && make install

# Common ROS2 tools
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-urdfdom \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    --fix-missing

# ROS2 img processing
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \ 
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-camera-calibration-parsers \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-image-view \
    --fix-missing

# ROS Navigation
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-rviz-plugins \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    --fix-missing

# ROS pkgs
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    #ros-${ROS_DISTRO}-diagnostic-updater \ 
    #ros-${ROS_DISTRO}-diagnostic-msgs \
    #ros-${ROS_DISTRO}-statistics-msgs \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-transmission-interface \
    ros-${ROS_DISTRO}-urdfdom-headers \
    ros-${ROS_DISTRO}-urdf-tutorial \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-topic-tools \
    --fix-missing

# ROS Gazebo
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-plugins \
    --fix-missing

# Extras for Azure download
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget curl ca-certificates gnupg \
    build-essential cmake pkg-config ninja-build \
    libusb-1.0-0-dev libgl1-mesa-dev \
    debconf-utils \
    python3-rosdep python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Azure Kinect Sensor SDK (libk4a 1.4.1)
RUN set -eux; \
    echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections; \
    echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | debconf-set-selections; \
    wget -qO /tmp/libk4a1.4.deb     https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb; \
    wget -qO /tmp/libk4a1.4-dev.deb https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb; \
    apt-get update; \
    apt-get install -y --no-install-recommends /tmp/libk4a1.4.deb /tmp/libk4a1.4-dev.deb; \
    rm -f /tmp/libk4a1.4*.deb; \
    rm -rf /var/lib/apt/lists/*

# Clean upros2 topic pub --once /voice/command std_msgs/msg/String 'data: "Maf"'
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (run as user)
RUN sudo rosdep init || true \
    && rosdep update

# COPY extras/python_api_1.4.1.zip /tmp/python_api_1.4.1.zip
# RUN pip3 install /tmp/python_api_1.4.1.zip && \
#     sudo rm /tmp/python_api_1.4.1.zip

# Stage sim
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ros-humble-slam-toolbox \
        ros-humble-rmw-cyclonedds-cpp \
        build-essential cmake git \
        libfltk1.3-dev libpng-dev libjpeg-dev \
        libglu1-mesa-dev libltdl-dev;\
    rm -rf /var/lib/apt/lists/*; \
    cd /tmp; \
    git clone https://github.com/rtv/Stage.git; \
    cd Stage; \
    mkdir -p build; \
    cd build; \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local; \
    make -j"$(nproc)"; \
    make install; \
    ldconfig; \
    rm -rf /tmp/Stage

RUN bash -lc '\
  set -ex; \
  mkdir -p /home/'"$USERNAME"'/extras_ws/src; \
  cd /home/'"$USERNAME"'/extras_ws/src; \
  git clone --depth 1 -b humble https://github.com/tuw-robotics/stage_ros2.git; \
  cd /home/'"$USERNAME"'/extras_ws; \
  source /opt/ros/humble/setup.bash; \
  colcon build --symlink-install \
' && chown -R $USERNAME:$USERNAME /home/$USERNAME/extras_ws

# AI python libs
RUN python3 -m pip install --no-cache-dir --upgrade pip \
 && python3 -m pip install --no-cache-dir \ 
    aiohttp

# rosbridge_server (apt) + MCP Python SDK + uv/uvx system-wide so the chat
# server (running as $USERNAME) can spawn `uvx ros-mcp --transport=stdio` for
# the mcp_readonly context provider. Installing uv via pip lands uvx in
# /usr/local/bin, which is on every user's PATH — the upstream curl installer
# would put it in /root/.local/bin and be invisible to $USERNAME.
RUN (ros2 pkg prefix rosbridge_server || (apt update && apt install -y ros-humble-rosbridge-server)) \
    && python3 -m pip install --no-cache-dir --break-system-packages mcp uv

# Add source in .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/$USERNAME/extras_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN mkdir -p /home/$USERNAME/ros2_ws/src

CMD ["bash"]
#docker build -t fabook/iros:v0.0.<> -f Dockerfile .
