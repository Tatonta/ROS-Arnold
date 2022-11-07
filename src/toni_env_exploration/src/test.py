#!/usr/bin/env python

# license removed for brevity

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from darknet_ros_msgs.msg import BoundingBoxes
import rospy
from geometry_msgs.msg import Twist

position = []

def callback_function(data):
    client.wait_for_server()
    for element in data.bounding_boxes:
        print(element.Class)
        if element.Class == "diningtable":
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "camera_link"
            goal.target_pose.header.stamp = rospy.Time.now()
        # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
            goal.target_pose.pose.position.x = 1
        # No rotation of the mobile base frame w.r.t. map frame
            goal.target_pose.pose.orientation.w = 1
            # Sends the goal to the action server.
            client.send_goal(goal)
        # Waits for the server to finish performing the action.
            wait = client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
            # Result of executing the action
                return client.get_result()   

def odom_extractor(odom_data):
    position.append(odom_data.linear.x)
    position.append(odom_data.linear.y)

def getting_data():
    odometry_sub = rospy.Subscriber("/kbot/base_controller/odom", Twist, odom_extractor)
    img_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback_function)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("darknet_subscriber_test")
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    try:
        getting_data()
    except KeyboardInterrupt:
        print("Shutting down")