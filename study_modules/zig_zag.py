#!/usr/bin/env python3
import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

"""
MoveIt velocity-driven wave:
1. Move to Cartesian pose (0,0,0.4) facing forward
2. Then run sine/cosine velocity waves on joints
"""

# ---------------- Parameters ----------------
freq_hz   = 0.3       # main frequency for joints 1–3 (Hz)
side_freq = 0.05      # slower sway for joint 0 (Hz)
cycles    = 5         # number of oscillations
amp_j0    = 0.2       # rad/s max velocity for joint 0
amp_j1    = 0.3       # rad/s max velocity for joint 1
amp_j2    = 0.7      # rad/s max velocity for joint 2
amp_j3    = 0.5       # rad/s max velocity for joint 3
amp_j4    = 0.0
dt        = 0.02      # seconds between waypoints (50 Hz)
# --------------------------------------------

rospy.init_node("z1_moveit_vel_sine", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
arm = moveit_commander.MoveGroupCommander("manipulator")
arm.set_max_velocity_scaling_factor(1.0)
arm.set_max_acceleration_scaling_factor(1.0)
home_joints = arm.get_current_joint_values()

# -------------------
# 1) Move to start Cartesian pose (0,0,0.4)
# -------------------
start_pose = geometry_msgs.msg.Pose()
start_pose.position.x = 0.0
start_pose.position.y = 0.0
start_pose.position.z = 0.5
start_pose.orientation.w = 1.0  # neutral orientation

arm.set_pose_target(start_pose)
arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()
rospy.sleep(1.0)

# -------------------
# 2) Get baseline joint configuration
# -------------------
joint_names = arm.get_active_joints()
q_curr = arm.get_current_joint_values()

traj = JointTrajectory()
traj.joint_names = joint_names

# Add current state as first trajectory point
pt0 = JointTrajectoryPoint()
pt0.positions = list(q_curr)
pt0.velocities = [0.0] * len(joint_names)
pt0.time_from_start = rospy.Duration.from_sec(0.0)
traj.points.append(pt0)
# -------------------
# 3) Generate velocity wave trajectory
# -------------------
T_total = float(cycles) / float(freq_hz)
N = int(T_total / dt)


t = 0.0
for i in range(N):
    # Phases
    phase      = 2.0 * math.pi * freq_hz * t
    phase_side = 2.0 * math.pi * side_freq * t

    # Velocity commands (rad/s) — pure sine/cosine
    dq0 = amp_j0 * math.cos(phase_side)
    dq1 = amp_j1 * math.cos(phase)
    dq2 = amp_j2 * math.cos(phase-math.pi)
    dq3 = amp_j3 * math.cos(phase-math.pi)
    dq4 = -amp_j4 * math.cos(phase_side)
    dq5 = 0.0

    # Integrate velocities into positions for trajectory
    q_curr[0] += dq0 * dt
    q_curr[1] += dq1 * dt
    q_curr[2] += dq2 * dt
    q_curr[3] += dq3 * dt
    q_curr[4] += dq4 *dt
    # q_curr[4], q_curr[5] unchanged

    pt = JointTrajectoryPoint()
    pt.positions = list(q_curr)
    pt.velocities = [dq0, dq1, dq2, dq3, dq4, dq5]
    pt.time_from_start = rospy.Duration.from_sec((i + 1) * dt)
    traj.points.append(pt)

    t += dt

# -------------------
# 4) Execute trajectory
# -------------------
robot_traj = RobotTrajectory()
robot_traj.joint_trajectory = traj

arm.execute(robot_traj, wait=True)


arm.set_joint_value_target(start_pose)
arm.go(wait=True)
arm.stop()

# -------------------
# 5) Return to saved 'home' joints
 # -------------------
arm.set_joint_value_target(home_joints)
arm.go(wait=True)
arm.stop()