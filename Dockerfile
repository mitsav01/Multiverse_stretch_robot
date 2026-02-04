# --------------------------
# Base image
# --------------------------
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive

# --------------------------
# Install base packages
# --------------------------
RUN apt-get update && apt-get install -y \
    sudo git wget curl vim \
    build-essential cmake ninja-build pkg-config \
    python3 python3-pip python3-venv python3-dev \
    libgl1-mesa-dev libglew-dev libx11-dev libxext-dev \
    libegl1 libgles2-mesa-dev libglfw3 libglfw3-dev \
    x11-apps gnupg lsb-release software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# --------------------------
# Create a working user
# --------------------------
RUN useradd -m -s /bin/bash developer && \
    echo 'developer ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER developer
WORKDIR /home/developer
ENV USER=developer
ENV HOME=/home/developer

# --------------------------
# Install ROS 2 Jazzy (root needed)
# --------------------------
USER root
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions ros2-control ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

USER developer
# Source ROS automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# --------------------------
# Clone Multiverse repository
# --------------------------
RUN git clone --recursive https://github.com/mitsav01/Multiverse_stretch_robot.git ~/multiverse

# --------------------------
# Build Multiverse ROS workspace
# --------------------------
RUN cd ~/multiverse/Multiverse-ROS-Connector/ros_ws/multiverse_ws2 && \
    rm -rf build install log && \
    colcon build --symlink-install && \
    echo "source ~/multiverse/Multiverse-ROS-Connector/ros_ws/multiverse_ws2/install/setup.bash" >> ~/.bashrc

# --------------------------
# Set MuJoCo environment
# --------------------------
ENV MUJOCO_HOME=/home/developer/multiverse/mujoco-3.3.5
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_HOME/lib
ENV MUJOCO_GL=egl

RUN echo "export MUJOCO_GL=egl" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$MUJOCO_HOME/lib" >> ~/.bashrc

# --------------------------
# Add convenient aliases
# --------------------------
RUN echo "alias simulate='~/multiverse/mujoco-3.3.5/bin/simulate'" >> ~/.bashrc && \
    echo "alias multiverse_server='~/multiverse/Multiverse-ServerClient/bin/multiverse_server'" >> ~/.bashrc && \
    echo "alias multiverse_ros='python3 ~/multiverse/Multiverse-ROS-Connector/scripts/multiverse_ros_run.py'" >> ~/.bashrc

# --------------------------
# Set working directory
# --------------------------
WORKDIR /home/developer

# --------------------------
# Keep container alive for interactive use
# --------------------------
CMD ["bash"]
