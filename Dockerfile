# Use the base image
FROM ghcr.io/kronton/ros1-ros-humble-bridge-docker:base-image

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i '/source \/root\/ros2_humble\/install\/local_setup.bash/d' ~/.bashrc


# Clone and build the ros1_bridge package without sourcing in .bashrc
RUN cd /root \
    && mkdir -p /root/bridge_ws/src \
    && cd /root/bridge_ws/src \
    && git clone https://github.com/ros2/ros1_bridge.git \
    && cd /root/bridge_ws \
    && colcon build --symlink-install --packages-skip ros1_bridge

# Build ros1_bridge selectively without sourcing
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/ros2_humble/install/setup.bash && cd /root/bridge_ws && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

# Set working directory (optional)
WORKDIR /root

# Default command
CMD ["bash"]
