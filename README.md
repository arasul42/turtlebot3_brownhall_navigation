Turtlebot3 Navigation in virtual Brownhall Map. 

The package is based on turtlebot_3 navigation stack. 

The multi_goal_mission.py script publishes 3 goal points sequentially. 

I am running on Ubuntu 22.04, therefore used docker for ROS Noetic setup.To use this repository please follor the steps.  

#git Clone with SSH Link

1. git clone -b master --recurse-submodules git@github.com:arasul42/turtlebot3_brownhall_navigation.git

2. cd turtlebot3_brownhall_navigation/

#bash Script for building the docker image and running the container with ros noetic environment. 

3. ./enter_console.sh

4. cd ~/catkin_ws 

5. catkin_make 

6. source /opt/ros/noetic/setup.bash

7. source devel/setup.bash

8. 

