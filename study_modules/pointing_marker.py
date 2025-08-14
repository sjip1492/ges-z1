#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import PoseStamped

def processFeedback(feedback):
    pose = PoseStamped()
    pose.header = feedback.header
    pose.pose = feedback.pose
    pose_pub.publish(pose)

if __name__ == "__main__":
    rospy.init_node("pointing_goal_marker")

    server = InteractiveMarkerServer("pointing_marker_server")
    global pose_pub
    pose_pub = rospy.Publisher("/pointing_goal", PoseStamped, queue_size=10)

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.name = "pointing_goal"
    int_marker.description = "Pointing Goal"
    int_marker.scale = 0.2

    control = InteractiveMarkerControl()
    control.name = "move_3D"
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D

    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = 1.0
    marker.color.g = 0.2
    marker.color.b = 0.2
    marker.color.a = 1.0

    control.markers.append(marker)
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    server.applyChanges()

    rospy.spin()
