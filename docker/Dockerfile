# Stage 1: HSL25 hardware environment resolved by R0-A for Linux/amd64.
# Override only with another reviewed digest and regenerate the affected R0
# evidence. A tag is deliberately not an accepted default.
ARG BASE_IMAGE=nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619
FROM ${BASE_IMAGE} AS kobuki-base

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

# Rebuild the inherited workspace against the pinned HSL25 toolchain. The
# base image already contains Livox-SDK2 and the Kobuki/ecl dependencies.
WORKDIR /workspace_kobuki
COPY kobuki/workspace/src src
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Stage 2: HSL26 development software. The release/acceptance image remains a
# separate R0 deliverable because its apt layer is not yet vendored.
FROM kobuki-base AS hsl26-final

# Install the reviewed target-platform Python closure without accessing PyPI.
# The wheelhouse itself is an R0 build input, kept outside Git due to its size.
COPY docker/r0/requirements.lock /opt/hsl26-r0/requirements.lock
COPY docker/r0/wheelhouse/ /opt/hsl26-r0/wheelhouse/
RUN pip3 install --no-index --require-hashes \
    --find-links /opt/hsl26-r0/wheelhouse \
    -r /opt/hsl26-r0/requirements.lock && \
    python3 -c "import setuptools; major = int(setuptools.__version__.split('.', 1)[0]); assert major < 80, f'setuptools {setuptools.__version__} is incompatible with colcon --symlink-install; regenerate the R0 wheelhouse/lock with setuptools==79.0.1'" && \
    python3 -c "import pytest; major = int(pytest.__version__.split('.', 1)[0]); assert major < 8, f'pytest {pytest.__version__} is incompatible with ROS Humble launch_testing; regenerate the R0 wheelhouse/lock with pytest==7.4.4'" && \
    pip3 check

# Install hsl_core as a regular wheel using the locked Setuptools already
# installed above. This avoids PEP-660 editable semantics and isolated-build
# downloads, both inappropriate for a self-contained container image.
WORKDIR /workspace_hsl26
COPY hsl_core /workspace_hsl26/hsl_core
RUN pip3 install --no-index --no-deps --no-build-isolation \
    /workspace_hsl26/hsl_core && \
    python3 -c "import hsl_core; print(hsl_core.__file__)"

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
