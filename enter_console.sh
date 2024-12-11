#!/bin/bash

# Ensure the container is running
docker compose up -d

# Enter the container and handle missing setup.bash
docker exec -it ros_noetic bash -c "
  source /opt/ros/noetic/setup.bash &&
  if [ -f /root/catkin_ws/devel/setup.bash ]; then
    source /root/catkin_ws/devel/setup.bash;
  else
    echo 'setup.bash not found. Building workspace...';
    mkdir -p /root/catkin_ws/src &&
    cd /root/catkin_ws && catkin_make;
    source /root/catkin_ws/devel/setup.bash;
  fi &&
  exec bash
"
