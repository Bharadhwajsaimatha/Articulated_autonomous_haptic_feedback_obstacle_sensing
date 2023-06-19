cmake_minimum_required(VERSION 2.8.3)
project(assembly_updated)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs gazebo_msgs geometry_msgs std_srvs message_generation)

#add_message files
add_message_files(FILES blocks_poses.msg)
add_message_files(FILES Tracker.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

find_package(roslaunch)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(blocks_spawner src/blocks_spawner.cpp)
target_link_libraries(blocks_spawner ${catkin_LIBRARIES})

add_executable(blocks_poses_publisher src/blocks_poses_publisher.cpp)
target_link_libraries(blocks_poses_publisher ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
foreach(dir config launch meshes urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


