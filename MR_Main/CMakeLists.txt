cmake_minimum_required(VERSION 2.8.3)
project(mr_main)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg message_generation mr_navigation_controller mr_tip_controller)

set(CMAKE_CXX_FLAGS "-std=c++11")

# Service files
add_service_files(
	DIRECTORY srv
	FILES
	run.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
	mr_navigation_controller
	mr_tip_controller
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/mainNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

MESSAGE(${PROJECT_NAME} "Done!")

