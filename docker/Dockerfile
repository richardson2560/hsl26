# Stage 1: Base hardware environment inherited from HSL25
FROM osrf/ros:humble-desktop AS kobuki-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    libudev-dev \
    libusb-1.0-0-dev \
    libyaml-cpp-dev \
    python3-pip \
    iproute2 \
    iputils-ping \
    net-tools \
    git \
    && rm -rf /var/lib/apt/lists/*

# Build the Kobuki and Livox hardware drivers once in the base stage.
WORKDIR /workspace_kobuki
COPY kobuki/workspace/src src
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Stage 2: HSL26 competition software
FROM kobuki-base AS hsl26-final

# Install the numerical and testing dependencies used by hsl_core.
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    open3d \
    pytest

# Install hsl_core in editable mode for development.
WORKDIR /workspace_hsl26
COPY hsl_core /workspace_hsl26/hsl_core
RUN pip3 install -e /workspace_hsl26/hsl_core

# Build the HSL26 ROS 2 interfaces and Python nodes.
COPY ros_ws/src /workspace_hsl26/src
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /workspace_kobuki/install/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Chain the ROS 2 environments when the container starts.
COPY docker/entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
