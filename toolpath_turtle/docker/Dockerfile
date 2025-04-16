ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO-ros-base

ARG ROS_DISTRO

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color

# Install system dependencies, including python3-tk for tkinter
RUN apt-get update && apt-get install -y --no-install-recommends \
    ssh \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    python3-pip \
    python3-flake8 \
    python3-tk \
    terminator \
    vim \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-plugins \
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-rqt-gui \
    ros-$ROS_DISTRO-rqt-gui-py \
    ros-$ROS_DISTRO-rqt-py-common \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-ecl-threads \
    ros-$ROS_DISTRO-ecl-geometry \
    ros-$ROS_DISTRO-ecl-streams \
    ros-$ROS_DISTRO-diagnostics \
    ros-$ROS_DISTRO-turtlesim \
    ros-$ROS_DISTRO-ros-tutorials \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for Toolpath_clustering with specific versions
RUN pip3 install --no-cache-dir \
    numpy==1.23.5 \
    Pillow==10.4.0 \
    scikit-learn==1.3.2 \
    scipy==1.10.1 \
    matplotlib==3.7.5

WORKDIR /
RUN mkdir -p dev_ws/src

# Copy repository (includes csv_turtle with Toolpath_clustering inside)
COPY . /dev_ws/src/

WORKDIR /dev_ws
# Build ROS workspace
RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build"]

# Copy and set permissions for entrypoint and setup scripts
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.sh /entrypoint.sh && chmod 777 /entrypoint.sh"]
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]

ENTRYPOINT ["bash", "/entrypoint.sh"]
CMD ["bash"]
