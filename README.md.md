# Articulated configuration autonomous robot with haptic feedback for obstacle sensing

This project is a part of the case study Autonomous Systems (SS2023) under the guidance of Prof.Alunkal Ginu Paul. The objective is to detect/sense the presence of an object using a 6 DOF articulated robot arm using sensors compatible with giving haptic feedback.



## Project Description

This project aims to implement a haptic feedback system on 6 DOF articulated autonomous robots for obstacle-sensing applications. An articulated robot was designed, and a force/torque sensor plugin was incorporated at the end effector joint. A conveyer belt was spawned and the blocks move on it. When an obstacle/object comes in contact with the arm, the change in force and torque values are captured and these values can be used as the haptic feedback to drive any other end applications.

## Prerequisites
### Software
Ubuntu 20.04 

ROS Noetic (or any compatible distro)

Gazebo (for visualization)

Rviz (for visualisation)

Moveit package(for motion planning)

## Usage
For running the virtual emulation, first, create a workspace in your system.
For a detailed tutorial on how to create a workspace refer to the link below!

[Create a workspace for ROS-related work](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Once the workspace is created, create a package in your workspace named **'assembly_updated'.**

**Note**: If you want to create any custom name for your package, you need to change the package name in all the files that have a reference to it.

Once the workspace and package are created, copy and paste all the files from this repository and build your workspace.

Source your workspace using
```bash
~/name_workspace$ source /devel/setup.bash
```
Before going further, make sure that you have necessary controllers like PositionControllers, EffortControllers installed.

Additionally for the plugin files(available in urdf folder), download, and build the gazebo-ros-pkg. Find the tutorial below!

[Installing ros-gazebo-pkgs](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

Once the setup is ready, you can run the following commands to start the simulation.

```bash
roslaunch src/assembly_updated/launch/demo_gazebo.launch
```
This should launch the Rviz and Gazebo world with the robot, conveyer belt, and the red blocks. The red blocks were set to an initial speed by providing wrench force in the block_spawners.cpp file.

Next, launch the python script to move the robot to the conveyer belt.
```bash
rosrun assembly_updated sample_move_robot.py
```
This node should move the robot onto the conveyer belt and waits for the obstacle to come toward it. Now launch the sensors reading and final logic code using
```bash
rosrun assembly_updated init_final_code.py
```
This calibrates the sensor first and then when a block is sensed, it displays the output and the change in force and torque values. These values can be used as a haptic feedback value to do any further applications.
## Contributing
Contributions to this project are welcome. Please fork the repository and submit a pull request with your improvements.

## Additional information
The project can be further refined by modifying the code and further including test cases, which are scheduled to be done by the time of the final presentation.

## Acknowledgement
Thanks to this GitHub repository for the [Conveyer belt](https://github.com/lihuang3/ur5_ROS-Gazebo) 

Thanks to Prof. Ginu Paul for his support and guidance throughout the project.

## Contact
For any questions or inquiries, please contact [me](https://github.com/Bharadhwajsaimatha).


## License
yet to be verified and valuated."# casestudy_autonomoussystems_SS23" 
