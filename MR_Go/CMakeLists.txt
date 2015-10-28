cmake_minimum_required(VERSION 2.8.3)
project(mr_go)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	nav_msgs
	msgs
	geometry_msgs
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	angularMove.srv
	linearMove.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs 
)

catkin_package()

## Specify additional locations of header files std
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/goNode.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

