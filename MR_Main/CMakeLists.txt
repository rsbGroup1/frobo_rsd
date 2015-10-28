cmake_minimum_required(VERSION 2.8.3)
project(mr_main)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg)

# Service files
add_service_files(
	FILES
	tip.srv
	performAction.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/mainNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

