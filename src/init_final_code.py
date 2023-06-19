#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped

# Threshold for detecting an obstacle
threshold = 20

# Flag to indicate if calibration is done
calibrated = False

# Variables for storing the calibration offsets
calibration_offsets = {
    "force_x": 0,
    "force_y": 0,
    "force_z": 0,
    "torque_x": 0,
    "torque_y": 0,
    "torque_z": 0
}

# Variables for storing maximum change in sensor values
max_change_force = 0
max_change_torque = 0

def force_torque_callback(msg):
    global calibrated, calibration_offsets
    global max_change_force, max_change_torque

    # Extract the force and torque values from the received message
    force_x = msg.wrench.force.x - calibration_offsets["force_x"]
    force_y = msg.wrench.force.y - calibration_offsets["force_y"]
    force_z = msg.wrench.force.z - calibration_offsets["force_z"]
    torque_x = msg.wrench.torque.x - calibration_offsets["torque_x"]
    torque_y = msg.wrench.torque.y - calibration_offsets["torque_y"]
    torque_z = msg.wrench.torque.z - calibration_offsets["torque_z"]

    # Perform calibration (tare the values) if not calibrated yet
    if not calibrated:
        calibration_offsets["force_x"] = msg.wrench.force.x
        calibration_offsets["force_y"] = msg.wrench.force.y
        calibration_offsets["force_z"] = msg.wrench.force.z
        calibration_offsets["torque_x"] = msg.wrench.torque.x
        calibration_offsets["torque_y"] = msg.wrench.torque.y
        calibration_offsets["torque_z"] = msg.wrench.torque.z
        calibrated = True
        rospy.loginfo("Sensor calibration completed.")

    # Calculate the change in force and torque values
    change_force_x = abs(force_x)
    change_force_y = abs(force_y)
    change_force_z = abs(force_z)
    change_torque_x = abs(torque_x)
    change_torque_y = abs(torque_y)
    change_torque_z = abs(torque_z)

    # Check if any of the change values exceed the threshold
    if (change_force_x > threshold or change_force_y > threshold or change_force_z > threshold or
            change_torque_x > threshold or change_torque_y > threshold or change_torque_z > threshold):
        max_change_force = max(change_force_x, change_force_y, change_force_z)
        max_change_torque = max(change_torque_x, change_torque_y, change_torque_z)
        rospy.loginfo("Obstacle sensed. Maximum change in force: %.2f, Maximum change in torque: %.2f",
                      max_change_force, max_change_torque)

def sensor_data_processing_node():
    rospy.init_node("sensor_data_processing_node", anonymous=True)

    # Subscribe to the force-torque sensor topic
    rospy.Subscriber("/assembly_updated/force_torque_values", WrenchStamped, force_torque_callback)

    # Enter the main loop
    rospy.spin()

if __name__ == '__main__':
    try:
        sensor_data_processing_node()
    except rospy.ROSInterruptException:
        pass

