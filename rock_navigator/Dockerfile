FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-turtlesim \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    numpy \
    websocket-client \
    websockets

# Fix permissions for /tmp to avoid QStandardPaths warning
RUN chmod 700 /tmp

# Create workspace
WORKDIR /root/ros2_ws

# Copy scripts
COPY ./scripts/rock_navigator.py /root/ros2_ws/rock_navigator.py
RUN chmod +r /root/ros2_ws/rock_navigator.py

# Set environment variables for Qt
ENV QT_LOGGING_RULES="qt5ct.debug=false"

# Source ROS2 and run TurtleSim and node (command will be overridden by setup.sh)
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node & python3 /root/ros2_ws/rock_navigator.py --mock"]
