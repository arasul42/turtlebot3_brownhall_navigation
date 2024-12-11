#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class GoalPublisher:
    def __init__(self):
        rospy.init_node("goal_publisher", anonymous=True)

        # Publishers and Subscribers
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)

        # Goal coordinates and orientation
        self.goal_x = -3.6  # Set desired goal x-coordinate
        self.goal_y = -16.78  # Set desired goal y-coordinate
        self.goal_orientation_z = 0.68  # Desired goal orientation (quaternion z)
        self.goal_orientation_w = 0.72  # Desired goal orientation (quaternion w)
        self.goal_tolerance = 0.2  # Tolerance for position
        self.orientation_tolerance = 0.1  # Tolerance for orientation (z, w)

        # Robot's current pose and orientation
        self.current_pose_x = None
        self.current_pose_y = None
        self.current_orientation_z = None
        self.current_orientation_w = None

        rospy.sleep(1)  # Wait for topics to be ready
        self.send_goal()

    def send_goal(self):
        """Send goal to the /move_base/goal topic."""
        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        # Define goal pose
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = self.goal_x
        goal.goal.target_pose.pose.position.y = self.goal_y
        goal.goal.target_pose.pose.orientation.z = self.goal_orientation_z
        goal.goal.target_pose.pose.orientation.w = self.goal_orientation_w

        rospy.loginfo("Publishing goal: x = %.2f, y = %.2f, orientation_z = %.3f, orientation_w = %.3f", 
                      self.goal_x, self.goal_y, self.goal_orientation_z, self.goal_orientation_w)
        self.goal_pub.publish(goal)

    def amcl_pose_callback(self, msg):
        """Callback for /amcl_pose to monitor robot's current position and orientation."""
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y
        self.current_orientation_z = msg.pose.pose.orientation.z
        self.current_orientation_w = msg.pose.pose.orientation.w

        # Check if the robot is within tolerance of the goal
        if self.is_goal_reached():
            rospy.loginfo("Goal reached!")
            rospy.signal_shutdown("Goal reached.")

    def is_goal_reached(self):
        """Check if the robot has reached the goal within tolerance."""
        if (self.current_pose_x is None or self.current_pose_y is None or
                self.current_orientation_z is None or self.current_orientation_w is None):
            return False

        # Calculate distance to the goal
        distance = math.sqrt(
            (self.goal_x - self.current_pose_x) ** 2 +
            (self.goal_y - self.current_pose_y) ** 2
        )

        # Check orientation differences (z and w)
        orientation_z_diff = abs(self.goal_orientation_z - self.current_orientation_z)
        orientation_w_diff = abs(self.goal_orientation_w - self.current_orientation_w)

        return (distance <= self.goal_tolerance and
                orientation_z_diff <= self.orientation_tolerance and
                orientation_w_diff <= self.orientation_tolerance)


if __name__ == "__main__":
    try:
        GoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal Publisher node terminated.")
