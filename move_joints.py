#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_commander.planning_scene_interface import PlanningSceneInterface

def move_joints_small_delta():
	roscpp_initialize([])
	rospy.init_node("z1_simple_joint_mover", anonymous=True)
	
	robot = RobotCommander()
	group = MoveGroupCommander("manipulator")
	
	current_joints = group.get_current_joint_values()
	print("Current joints:", current_joints)
	
	target_joints = [j + 0.0005 for j in current_joints]
	
	group.set_joint_value_target(target_joints)
	
	plan = group.plan()
	print(plan)
	if plan and len(plan[1].joint_trajectory.points) > 0 :
		group.go(wait=True)
		group.stop()
		group.clear_pose_targets()
		
	else:
		rospy.logwarn("Planning failed.")
	roscpp_shutdown()
    	
if __name__ == "__main__":
    move_joints_small_delta()    	
    	

