#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
import tf
from nav_msgs.msg import OccupancyGrid


class Explorer:
    def __init__(self):
        rospy.init_node('explore_map')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server.")
        rospy.loginfo("Waiting for map data...")
        self.map_data = rospy.wait_for_message('/map', OccupancyGrid)
        rospy.loginfo("Map data received.")
        self.map_boundaries = self.calculate_map_boundaries()
        self.waypoints = self.generate_waypoints()

    def calculate_map_boundaries(self):
        """Calculate map boundaries based on map metadata."""
        map_metadata = self.map_data.info
        resolution = map_metadata.resolution
        width = map_metadata.width
        height = map_metadata.height
        origin_x = map_metadata.origin.position.x
        origin_y = map_metadata.origin.position.y

        # Calculate boundaries
        x_start = origin_x
        x_end = origin_x + width * resolution
        y_start = origin_y
        y_end = origin_y + height * resolution

        rospy.loginfo(f"Map boundaries calculated: x_start={x_start}, x_end={x_end}, y_start={y_start}, y_end={y_end}")
        return x_start, x_end, y_start, y_end

    def generate_waypoints(self):
        """Generate a grid of waypoints based on map boundaries."""
        x_start, x_end, y_start, y_end = self.map_boundaries
        waypoints = []
        x_step = 1.0  # Adjust for density
        y_step = 1.0

        for x in range(int(x_start), int(x_end + x_step), int(x_step)):
            for y in range(int(y_start), int(y_end + y_step), int(y_step)):
                waypoints.append((x, y, 0.0))  # Orientation yaw = 0
        rospy.loginfo(f"Generated {len(waypoints)} waypoints for exploration.")
        return waypoints

    def move_to_goal(self, x, y, yaw, max_retries=3):
        """Send the robot to a specific goal with retries."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        for attempt in range(max_retries):
            rospy.loginfo(f"Attempting goal: x={x}, y={y}, yaw={yaw}, try {attempt+1}")
            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached!")
                return True
            rospy.logwarn("Goal failed. Retrying...")

        rospy.logwarn("Max retries reached. Skipping waypoint.")
        return False

    def explore(self):
        """Explore all waypoints."""
        for waypoint in self.waypoints:
            x, y, yaw = waypoint
            self.move_to_goal(x, y, yaw)

if __name__ == "__main__":
    try:
        explorer = Explorer()
        explorer.explore()
        rospy.loginfo("Exploration complete!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Exploration interrupted.")
