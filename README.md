## Turtlebot3 Navigation in virtual Brownhall Map. 

The package is based on turtlebot_3 navigation stack. 

The multi_goal_mission.py script publishes 3 goal points sequentially. 

I am running on Ubuntu 22.04, therefore used docker for ROS Noetic setup.To use this repository please follor the steps.  

### git Clone with SSH Link

1.  ```
    git clone -b master --recurse-submodules git@github.com:arasul42/turtlebot3_brownhall_navigation.git
    ```

2. ```cd turtlebot3_brownhall_navigation/```

### bash Script for building the docker image and running the container with ros noetic environment. 

3. ./enter_console.sh


4.  source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash

5. echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc

    echo "export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/turtlebot3_brownhall/lib:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
    echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/turtlebot3_brownhall/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
    echo "export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/turtlebot3_brownhall/models:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc

6. roscd turtlebot3_navigation && cd launch

 ### This step is to automatic update of 2D pose estimate and map value. 

7. nano turtlebot3_navigation.launch to edit the file according to the refernce in this repository. 

8. source ~/.bashrc

9. roslaunch turtlebot3_brownhall turtlebot3_brownhall.launch 

# This should Run the turtlebot3 on a mission to chase 3 goal points.  



