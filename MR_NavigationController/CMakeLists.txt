cmake_minimum_required(VERSION 2.8.3)
project(rsd_navigation_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

#generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/navigationControllerNode.cpp)

add_executable(rsd_navigation_controller_node ${SrcFiles})

target_link_libraries(rsd_navigation_controller_node ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

