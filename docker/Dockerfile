FROM ros:jazzy

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace

# Copy source
COPY src/ src/

# Build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Default environment
ENV TELEMETRY_LOG_DIR=/data

# Source overlays on container start
COPY docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]