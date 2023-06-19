#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander

# Initialize the moveit_commander and rospy nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_movement_script', anonymous=True)

# Create a MoveGroupCommander for the arm_noend group
group = MoveGroupCommander("arm_noend")

# Set the planning time
group.set_planning_time(10)

# Set the target joint values for the initial pose
init_pose = [0, 0, 0, 0, 0, 0]
group.set_joint_value_target(init_pose)

# Plan and execute the trajectory to the initial pose
group.go(wait=True)

# Set the target joint values for the final pose (mov_to_belt)
final_pose = [0, -0.6981, 0, 0.8726, -1.571, -1.5708]
group.set_joint_value_target(final_pose)

# Plan and execute the trajectory to the final pose
group.go(wait=True)

# Shutdown moveit_commander and rospy nodes
moveit_commander.roscpp_shutdown()
rospy.signal_shutdown("Done")


