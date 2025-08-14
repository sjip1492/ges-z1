#!/usr/bin/env python3
"""
Save/load named Z1 joint states (in DEGREES) to a YAML file.

CLI:
  # Save the robot's CURRENT pose as 'ready'
  rosrun YOUR_PKG state_manager.py --save ready

  # Load a named pose and move the arm there
  rosrun YOUR_PKG state_manager.py --load ready

From code:
  from state_manager import Z1StateManager
  sm = Z1StateManager()
  sm.save('ready')                      # uses current arm state
  sm.load_and_move('ready')             # moves to saved state
  sm.write_state('look_left', {'joint1': 30, 'joint2': 0, 'joint3': 0, 'joint4': 0, 'joint5': 0, 'joint6': 0})
"""
import os
import sys
import yaml
import math
import argparse
import rospy
import moveit_commander

MOVE_GROUP_NAME = 'manipulator'
DEFAULT_JOINT_ORDER = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
DEFAULT_DB = os.path.expanduser('/home/tangentlab/z1_ws/src/z1_ros/z1_examples/examples/ges-z1/study_modules/z1_states/saved_states.yaml')


def rad2deg(r):
    return r * 180.0 / math.pi


def deg2rad(d):
    return d * math.pi / 180.0


class Z1StateManager:
    def __init__(self, db_path=DEFAULT_DB, group_name=MOVE_GROUP_NAME, joint_order=None):
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.core.is_initialized():
            rospy.init_node('z1_state_manager', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_max_velocity_scaling_factor(0.2)
        self.group.set_max_acceleration_scaling_factor(0.2)
        self.db_path = db_path
        self.joint_order = joint_order or self._derive_joint_order()
        self._ensure_db()

    def _derive_joint_order(self):
        names = [j for j in self.group.get_active_joints() if j.startswith('joint')]
        if len(names) != 6:
            return DEFAULT_JOINT_ORDER
        return names

    def _ensure_db(self):
        if not os.path.exists(self.db_path):
            with open(self.db_path, 'w') as f:
                yaml.safe_dump({'states': {}}, f)

    def _read_all(self):
        with open(self.db_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        data.setdefault('states', {})
        return data

    def _write_all(self, data):
        with open(self.db_path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=True)

    def current_state_deg(self):
        vals = self.group.get_current_joint_values()  # radians
        active = self.group.get_active_joints()
        m = dict(zip(active, vals))
        return {j: round(rad2deg(m[j]), 4) for j in self.joint_order if j in m}

    def save(self, name):
        data = self._read_all()
        data['states'][name] = self.current_state_deg()
        self._write_all(data)
        rospy.loginfo(f"Saved state '{name}' to {self.db_path}")

    def write_state(self, name, state_deg_map):
        """
        Write a provided state (DEGREES) into the DB without moving the robot.
        """
        # Normalize ordering and keep only known joints
        norm = {j: float(state_deg_map[j]) for j in self.joint_order if j in state_deg_map}
        data = self._read_all()
        data['states'][name] = norm
        self._write_all(data)
        rospy.loginfo(f"Wrote state '{name}' to {self.db_path}")

    def load(self, name):
        data = self._read_all()
        if name not in data['states']:
            raise KeyError(f"State '{name}' not found in {self.db_path}")
        return data['states'][name]  # degrees

    def load_and_move(self, name):
        state_deg = self.load(name)
        # Fill unspecified joints with current angles (defensive, though we saved full states)
        current = self.current_state_deg()
        target_deg = {j: state_deg.get(j, current[j]) for j in self.joint_order}

        target_map = {j: deg2rad(target_deg[j]) for j in self.joint_order}
        self.group.set_joint_value_target(target_map)
        ok = self.group.go(wait=True)
        rospy.sleep(0.1)
        self.group.stop()

        return bool(ok)


def _parse_args():
    p = argparse.ArgumentParser(description="Save/load Unitree Z1 joint states (degrees) to YAML.")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--save', metavar='NAME', help='Save CURRENT arm state as NAME')
    g.add_argument('--load', metavar='NAME', help='Load NAME and move arm there')
    g.add_argument('--write', nargs='+', metavar='K=V', help='Write a state without moving, provide NAME and joint degs. Example: --write look_left joint1=30 joint2=0 ...')
    p.add_argument('--db', default=DEFAULT_DB, help=f"Path to YAML DB (default: {DEFAULT_DB})")
    return p.parse_args()


def main():
    args = _parse_args()
    sm = Z1StateManager(db_path=args.db)

    if args.save:
        sm.save(args.save)
        return

    if args.load:
        ok = sm.load_and_move(args.load)
        if not ok:
            rospy.logerr("Execution failed.")
            sys.exit(2)
        rospy.loginfo("Done.")
        return

    if args.write:
        name = args.write[0]
        kvs = args.write[1:]

        if len(args.write) == 2 and kvs[0].lower() in ["current", "current_position", "currentpos"]:
            # Special case: write current joint positions
            state = sm.current_state_deg()
            sm.write_state(name, state)
            return

        if len(args.write) < 2:
            raise SystemExit("Usage: --write NAME joint1=... joint2=... ... OR --write NAME current")

        # Normal case: parse key=value pairs
        state = {}
        for kv in kvs:
            k, v = kv.split('=', 1)
            state[k] = float(v)
        sm.write_state(name, state)



if __name__ == '__main__':
    main()
