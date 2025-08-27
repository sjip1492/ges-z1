import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def compute_look_at_orientation(from_pos, to_pos):
    # Create a rotation where Z-axis points from `from_pos` to `to_pos`
    z_axis = normalize(to_pos - from_pos)

    # Pick a reasonable up direction that's not colinear with Z
    up = np.array([0, 0, 1]) if abs(z_axis[2]) < 0.95 else np.array([1, 0, 0])

    x_axis = normalize(np.cross(up, z_axis))
    y_axis = np.cross(z_axis, x_axis)

    rot_matrix = np.eye(4)
    rot_matrix[0:3, 0] = x_axis
    rot_matrix[0:3, 1] = y_axis
    rot_matrix[0:3, 2] = z_axis

    quat = tf.transformations.quaternion_from_matrix(rot_matrix)
    return quat

def marker_callback(msg):
    rospy.loginfo("Received new pointing goal")

    # get the marker’s position from the PoseStamped (msg).
    target = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    rospy.loginfo("Marker position: %s", target)

    # get the current end-effector (EE) pose
    current_pose = group.get_current_pose().pose
    ee_position = np.array([
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z
    ])
    rospy.loginfo("EE position: %s", ee_position)

    # calculates the vector from the EE to the marker by subtracting positions
    direction = normalize(target - ee_position)
    rospy.loginfo("Direction vector (normalized): %s", direction)

    quat = compute_look_at_orientation(ee_position, target)
    rospy.loginfo("Computed orientation (quaternion): %s", quat)

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "world"
    pose_goal.pose.position.x = ee_position[0]
    pose_goal.pose.position.y = ee_position[1]
    pose_goal.pose.position.z = ee_position[2]
    pose_goal.pose.orientation.x = quat[0]
    pose_goal.pose.orientation.y = quat[1]
    pose_goal.pose.orientation.z = quat[2]
    pose_goal.pose.orientation.w = quat[3]

    group.set_pose_target(pose_goal)
    plan = group.plan()
    if plan and hasattr(plan[0], 'joint_trajectory') and plan[0].joint_trajectory.points:
        rospy.loginfo("Planning successful, executing")
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
    else:
        rospy.logwarn("Planning failed or empty plan returned")


if __name__ == "__main__":
    roscpp_initialize([])

    # starting new ROS node, can add multiple instances (assigns new id w/ anon param)
    rospy.init_node("pointing_arm_planner", anonymous=True)

    # MoveIt api - lets you check joint names, link names, current states, etc.
    robot = RobotCommander()

    # PlanningSceneInterface object, lets you add or remove collision objects in the world that MoveIt considers during planning
    scene = PlanningSceneInterface()

    # use this object to set poses, plan trajectories and execute them
    group = MoveGroupCommander("manipulator")

    # Subscribes to the /pointing_goal topic (the one published by our interactive marker script).
    # Every time a new marker pose is published, marker_callback is called.
    # This is where we compute the orientation to point toward the marker and tell MoveIt to plan & execute.

    # marker_callback is called every time a new PoseStamped is published to /pointing_goal
    # PoseStamped part means the pose is tied to a specific time and reference frame
    rospy.Subscriber("/pointing_goal", PoseStamped, marker_callback)

    rospy.loginfo("Ready to receive pointing goal")
    rospy.spin()


# move (a little/lot, left/right/up/down)