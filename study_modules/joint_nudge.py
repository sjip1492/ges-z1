#!/usr/bin/env python3
"""
Nudge Unitree Z1 joints by small/medium/large increments.

Usable from CLI:
  rosrun YOUR_PKG joint_nudge.py --joint joint2 --dir + --inc small
  rosrun YOUR_PKG joint_nudge.py --joint joint3 --dir - --inc large

Usable from code:
  from joint_nudge import Z1Nudger
  nudger = Z1Nudger()
  nudger.nudge('joint2', direction='+', inc='medium')
  nudger.goto_state({'joint1': 10.0, 'joint2': -5.0})  # degrees
"""

import sys
import math
import argparse
import rospy
import moveit_commander

# Tune these if you prefer different step sizes (degrees)
INCREMENTS_DEG = {
    'small': 2.0,
    'medium': 5.0,
    'large': 10.0,
}

# If your MoveIt group is named differently, change this:
MOVE_GROUP_NAME = 'manipulator'

# Optionally pin the Z1 joint list if MoveIt exposes virtual joints on your setup.
DEFAULT_JOINT_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


def deg2rad(d):
    return d * math.pi / 180.0


class Z1Nudger:
    def __init__(self, group_name=MOVE_GROUP_NAME, joint_order=None):
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.core.is_initialized():
            rospy.init_node('z1_joint_nudger', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)
        self.joint_order = joint_order or self._derive_joint_order()

    def _derive_joint_order(self):
        # Filter to typical Z1 joint names only
        names = [j for j in self.group.get_active_joints() if j.startswith('joint')]
        # Fallback to DEFAULT_JOINT_ORDER if that seems safer
        if len(names) != 6:
            return DEFAULT_JOINT_ORDER
        return names

    def _current_joint_map(self):
        vals = self.group.get_current_joint_values()  # radians in same order as group.get_active_joints()
        active = self.group.get_active_joints()
        m = dict(zip(active, vals))
        return {j: m[j] for j in self.joint_order if j in m}

    def nudge(self, joint, direction='+', inc='small'):
        if joint not in self.joint_order:
            raise ValueError(f"Unknown joint '{joint}'. Known: {self.joint_order}")

        step_deg = INCREMENTS_DEG.get(inc)
        if step_deg is None:
            raise ValueError(f"Unknown increment '{inc}'. Choose from {list(INCREMENTS_DEG)}.")

        sign = +1.0 if direction.strip() in ['+', 'plus', 'pos', 'cw', 'up'] else -1.0
        step_rad = sign * deg2rad(step_deg)

        current = self._current_joint_map()
        # target = [current[j] + (step_rad if j == joint else 0.0) for j in self.joint_order]

        # Build a {joint_name: target_rad} dict; only bump the requested joint
        target_map = current.copy()
        target_map[joint] = current[joint] + step_rad
        # Send to MoveIt and execute
        self.group.set_joint_value_target(target_map)
        ok = self.group.go(wait=True)
        rospy.sleep(10)
        self.group.stop()

        return bool(ok)

    def goto_state(self, state_deg_map):
        """
        Move to an absolute joint state in DEGREES.
        Example: {'joint1': 0.0, 'joint2': 15.0, ...}
        Unspecified joints hold their current angle.
        """
        current = self._current_joint_map()
        target = []
        for j in self.joint_order:
            if j in state_deg_map:
                target.append(deg2rad(state_deg_map[j]))
            else:
                target.append(current[j])  # keep current for unspecified joints

        self.group.set_joint_value_target(target, self.joint_order)
        plan = self.group.plan()
        ok = self.group.execute(plan, wait=True)
        rospy.sleep(0.1)
        self.group.stop()
        return bool(ok)


def _parse_args():
    p = argparse.ArgumentParser(description="Nudge Unitree Z1 joints via MoveIt.")
    p.add_argument('--joint', default='joint2', help='Joint name (e.g., joint2')
    p.add_argument('--dir', default='+', help="Direction '+' or '-'")
    p.add_argument('--inc', default='large', choices=['small', 'medium', 'large'])
    return p.parse_args()


def main():
    args = _parse_args()
    nudger = Z1Nudger()
    ok = nudger.nudge(args.joint, direction=args.dir, inc=args.inc)
    if not ok:
        rospy.logerr("Execution failed.")
        sys.exit(2)
    rospy.loginfo("Done.")


if __name__ == '__main__':
    main()