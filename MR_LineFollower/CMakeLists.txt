cmake_minimum_required(VERSION 2.8.3)
project(mr_line_follower)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp 
	rospy
	std_msgs 
	geometry_msgs 
	genmsg
	msgs
	message_generation
	mr_camera_processing
)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Generate services in the 'srv' folder
add_service_files(
	FILES
	followUntilQR.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs  
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(mr_line_follower src/lineFollowerNode.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mr_line_follower
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})
