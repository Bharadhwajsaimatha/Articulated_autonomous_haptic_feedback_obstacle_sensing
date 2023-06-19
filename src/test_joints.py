#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def control_joints():
    rospy.init_node('joint_control')
    pub = rospy.Publisher('/robot_arm/joint_command', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Generate random joint positions for revolute joints
        revolute_positions = [1.0, 0.5, -0.2, 0.0, 0.3]

        # Set a constant joint velocity for the continuous joint
        continuous_velocity = 0.1

        # Create Float64MultiArray message
        joint_command = Float64MultiArray()

        # Assign joint positions for revolute joints
        joint_command.data = revolute_positions

        # Append joint velocity for the continuous joint
        joint_command.data.append(continuous_velocity)

        # Publish the joint command
        pub.publish(joint_command)

        rate.sleep()

if __name__ == '__main__':
    try:
        control_joints()
    except rospy.ROSInterruptException:
        pass

