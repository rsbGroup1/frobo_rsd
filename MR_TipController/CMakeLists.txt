cmake_minimum_required(VERSION 2.8.3)
project(mr_tip_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs serial genmsg)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	tip.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/tipControllerNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

