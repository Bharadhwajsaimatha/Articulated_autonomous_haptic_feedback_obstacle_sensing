#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped

def force_torque_callback(msg):
    # Extract the force and torque values from the received message
    force_x = msg.wrench.force.x
    force_y = msg.wrench.force.y
    force_z = msg.wrench.force.z
    torque_x = msg.wrench.torque.x
    torque_y = msg.wrench.torque.y
    torque_z = msg.wrench.torque.z

    # Check if any of the force or torque values exceed the threshold
    if (force_x > 50 or force_y > 50 or force_z > 50 or
        torque_x > 50 or torque_y > 50 or torque_z > 50 or
        force_x < -50 or force_y < -50 or force_z < -50 or
        torque_x < -50 or torque_y < -50 or torque_z < -50):
        # Increment the count variable
        global count
        count += 1

    # Print the force and torque values for debugging
    rospy.loginfo("Force: %.2f %.2f %.2f", force_x, force_y, force_z)
    rospy.loginfo("Torque: %.2f %.2f %.2f", torque_x, torque_y, torque_z)

    # Print the count variable
    rospy.loginfo("Count: %d", count)

def sensor_data_processing_node():
    rospy.init_node("sensor_data_processing_node", anonymous=True)

    # Initialize the count variable
    global count
    count = 0

    # Subscribe to the force-torque sensor topic
    rospy.Subscriber("/assembly_updated/force_torque_values", WrenchStamped, force_torque_callback)

    # Enter the main loop
    rospy.spin()

if __name__ == '__main__':
    try:
        sensor_data_processing_node()
    except rospy.ROSInterruptException:
        pass


