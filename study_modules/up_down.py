#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

rospy.init_node("z1_moveit_updown", anonymous=True)

# Initialize MoveIt commander
moveit_commander.roscpp_initialize(sys.argv)
arm = moveit_commander.MoveGroupCommander("manipulator")
arm.set_max_velocity_scaling_factor(1.0)
arm.set_max_acceleration_scaling_factor(1.0)
# -------------------
# 1. Move to start pose
# -------------------
start_pose = geometry_msgs.msg.Pose()
start_pose.position.x = 0.0
start_pose.position.y = 0.0
start_pose.position.z = 0.4
start_pose.orientation.w = 1.0   # neutral orientation

arm.set_pose_target(start_pose)
arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()

rospy.sleep(1.0)  # short pause to settle

# -------------------
# 2. Repetitive up-down movement
# -------------------
amplitude   = 0.15   # meters (5 cm)
repetitions = 5      # number of cycles
hold_time   = 0   # pause at top/bottom

for i in range(repetitions):
    # Up
    pose_up = geometry_msgs.msg.Pose()
    pose_up.position.x = start_pose.position.x
    pose_up.position.y = start_pose.position.y
    pose_up.position.z = start_pose.position.z + amplitude
    pose_up.orientation = start_pose.orientation

    arm.set_pose_target(pose_up)
    arm.go(wait=True)
    #arm.stop()
    arm.clear_pose_targets()
    rospy.sleep(hold_time)

    # Down
    pose_down = geometry_msgs.msg.Pose()
    pose_down.position.x = start_pose.position.x
    pose_down.position.y = start_pose.position.y
    pose_down.position.z = start_pose.position.z - amplitude
    pose_down.orientation = start_pose.orientation

    arm.set_pose_target(pose_down)
    arm.go(wait=True)
    #arm.stop()
    arm.clear_pose_targets()
    rospy.sleep(hold_time)

# -------------------
# 3. Return to start pose
# -------------------
arm.set_pose_target(start_pose)
arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()