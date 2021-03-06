cmake_minimum_required(VERSION 2.8.3)
project(mr_navigation_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg actionlib mr_line_follower mr_go)

set(CMAKE_CXX_FLAGS "-std=c++11")

# Service files
add_service_files(
	DIRECTORY srv
	FILES
	performAction.srv
	setCurrentNode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
	mr_line_follower
	mr_go
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(
SOURCES 
	src/navigationControllerNode.cpp
	src/node.cpp
	src/vertex.cpp
	src/graph.cpp
	src/skills.cpp
)

set(
HEADERS
	src/node.h
	src/vertex.h
	src/skills.h
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

MESSAGE(${PROJECT_NAME} " Done!")

