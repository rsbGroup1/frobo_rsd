cmake_minimum_required(VERSION 2.8.3)
project(mr_navigation_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg)

#generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/navigationControllerNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

