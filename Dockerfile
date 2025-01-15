FROM ros:noetic-ros-base

# Avoid interactive dialog during package installation
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 DEBIAN_FRONTEND=noninteractive

# Install necessary tools and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    software-properties-common \
    curl \
    python3-pip \
    build-essential \
    cmake \
    git \
    wget \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt-get clean

# Install vcstool via pip
RUN python3 -m pip install vcstool
# Set locale
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# Add ROS 2 repository key and source
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update

# Install ROS 2 development tools and dependencies
RUN apt-get install -y --no-install-recommends \
    python3-flake8-docstrings \
    python3-pytest-cov \
    ros-dev-tools \
    && python3 -m pip install -U \
        flake8-blind-except \
        flake8-builtins \
        flake8-class-newline \
        flake8-comprehensions \
        flake8-deprecated \
        flake8-import-order \
        flake8-quotes \
        "pytest>=5.3" \
        pytest-repeat \
        pytest-rerunfailures

# Set up ROS 2 workspace
RUN mkdir -p /root/ros2_humble/src \
    && cd /root/ros2_humble \
    && vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Install ROS 2 dependencies
RUN apt-get upgrade -y \
    && if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rm /etc/ros/rosdep/sources.list.d/20-default.list; fi \
    && rosdep init \
    && rosdep update \
    && rosdep install --from-paths /root/ros2_humble/src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

    # Build ROS 2 workspace
RUN cd /root/ros2_humble \
    && colcon build --symlink-install


RUN cd /root \
    && mkdir -p /root/bridge_ws/src \
    && cd /root/bridge_ws/src \
    && git clone https://github.com/ros2/ros1_bridge.git \
    && cd /root/bridge_ws \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/ros2_humble/install/setup.bash && colcon build --symlink-install --packages-skip ros1_bridge"

# Source ROS1 and ROS2 environments and build ros1_bridge
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/ros2_humble/install/setup.bash && cd /root/bridge_ws && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

CMD ["bash"]
