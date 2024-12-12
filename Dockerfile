# Use the full desktop version of ROS Noetic
FROM osrf/ros:noetic-desktop-full

# Install additional utilities and ensure system dependencies are up-to-date
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    git \
    ros-noetic-navigation\
    ros-noetic-gmapping\
    && apt-get clean

RUN apt-get install -y\
    ros-noetic-turtlebot3\
    ros-noetic-turtlebot3-simulations\
    ros-noetic-turtlebot3-gazebo\
    && apt-get clean

# Ensure rosdep is initialized only if necessary
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi && \
    rosdep update

# Set up the catkin workspace
RUN mkdir -p /root/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make"

# Set environment variables for convenience
ENV ROS_WS=/root/catkin_ws
ENV DISPLAY=:0
WORKDIR /root/catkin_ws
