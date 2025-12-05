# Use ROS2 Humble base image
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-tk \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install pyserial for XV11 lidar
RUN pip3 install pyserial

# Create workspace directory
WORKDIR /workspace

# Copy source files
COPY src/ /workspace/src/

# Install dependencies using rosdep
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source ROS2 and workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["/bin/bash"]
