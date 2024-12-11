#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker

class MultiGoalMission:
    def __init__(self):
        rospy.init_node("multi_goal_mission", anonymous=True)

        # Publishers
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)
        self.marker_pub = rospy.Publisher("/goal_markers", Marker, queue_size=10)

        # Subscriber
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)

        # Mission goals (x, y, z, w)
        self.goals = [

            {"x": 5.9, "y": 12.9, "z": 1.0, "w": 0.0},
            {"x": 6.58, "y": 5.57, "z": 0.0, "w": 1.0},
            {"x": -5.6, "y": -7.0, "z": 1.0, "w": 0.0},
        ]
        self.current_goal_index = 0
        self.goal_reached = False
        self.current_goal_id = None  # Track the current goal ID

        rospy.sleep(1)

        # clear all marker at the beginning for a clean start    

        self.clear_all_markers()

        # Register shutdown hook
        # rospy.on_shutdown(self.clear_all_markers)





        # Wait for topics to be ready
        rospy.sleep(1)
        # self.publish_marker(0, 0, -1, "Waiting for RViz...")  # Inform RViz of the first marker
        # rospy.sleep(2)  # Allow RViz time to load the marker topic
        self.send_next_goal()

    def send_next_goal(self):
        """Send the next goal to the /move_base/goal topic."""
        if self.current_goal_index < len(self.goals):




            while self.marker_pub.get_num_connections() == 0:
                rospy.loginfo("Waiting for RViz to subscribe to /goal_markers...")
                rospy.sleep(0.5)

            goal = self.goals[self.current_goal_index]
            goal_msg = MoveBaseActionGoal()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = "map"

            # Define goal pose
            goal_msg.goal.target_pose.header.frame_id = "map"
            goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
            goal_msg.goal.target_pose.pose.position.x = goal["x"]
            goal_msg.goal.target_pose.pose.position.y = goal["y"]
            goal_msg.goal.target_pose.pose.orientation.z = goal["z"]
            goal_msg.goal.target_pose.pose.orientation.w = goal["w"]

            # Generate a unique goal ID
            self.current_goal_id = goal_msg.goal_id.id = f"goal_{self.current_goal_index}"

            rospy.loginfo("Sending goal %d: x=%.2f, y=%.2f", self.current_goal_index + 1, goal["x"], goal["y"])
            self.publish_marker(goal["x"], goal["y"], self.current_goal_index + 1)
            self.goal_pub.publish(goal_msg)
            self.goal_reached = False
        else:
            rospy.loginfo("Mission complete! All goals reached.")
            self.publish_marker(-1.36, 0, -1, "Mission Complete!")  # Final marker

            rospy.sleep(3)

            rospy.signal_shutdown("Mission complete.")

    def status_callback(self, msg):
        """Callback for /move_base/status to monitor goal status."""
        for status in msg.status_list:
            # Ensure status corresponds to the current goal
            if status.goal_id.id == self.current_goal_id and status.status == 3:  # Status 3 indicates "Goal reached"
                if not self.goal_reached:
                    rospy.loginfo("Goal %d reached!", self.current_goal_index + 1)
                    self.goal_reached = True
                    rospy.sleep(5)  # Pause for 5 seconds
                    self.current_goal_index += 1
                    self.send_next_goal()

    def publish_marker(self, x, y, goal_index, text="Goal", orientation_z=0.0, orientation_w=1.0):
        """Publish markers (text and arrow) in RViz."""
        # Text marker
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "goal_markers"
        text_marker.id = goal_index
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = 1.0  # Place text slightly above the arrow

        # text_marker.pose.orientation.x = 0.0
        # text_marker.pose.orientation.y = 0.0
        # text_marker.pose.orientation.z = 1
        text_marker.pose.orientation.w = 1

        text_marker.scale.z = 2  if text=="Mission complete." else 0.5
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0 if goal_index > 0 else 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = f"{text} {goal_index}" if goal_index > 0 else text

        # Arrow marker
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "map"
        arrow_marker.header.stamp = rospy.Time.now()
        arrow_marker.ns = "goal_arrows"
        arrow_marker.id = goal_index
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position.x = x
        arrow_marker.pose.position.y = y
        arrow_marker.pose.position.z = 0.0  # Base of the arrow
        arrow_marker.pose.orientation.z = orientation_z  # Set the orientation from the goal
        arrow_marker.pose.orientation.w = orientation_w  # Set the orientation from the goal
        arrow_marker.scale.x = 1.0  # Arrow shaft length
        arrow_marker.scale.y = 0.1  # Arrow shaft diameter
        arrow_marker.scale.z = 0.1  # Arrow head diameter
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        # Publish markers
        self.marker_pub.publish(text_marker)
        self.marker_pub.publish(arrow_marker)

    def clear_all_markers(self):

        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "goal_markers"  # Namespace of the markers to clear
        delete_marker.action = Marker.DELETEALL  # Delete all markers in this namespace
        self.marker_pub.publish(delete_marker)
        rospy.loginfo("All markers cleared from RViz.")



if __name__ == "__main__":
    try:
        MultiGoalMission()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mission node terminated.")
