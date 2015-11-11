cmake_minimum_required(VERSION 2.8.3)

project(mr_hmi)

find_package(catkin REQUIRED COMPONENTS roscpp rospy msgs std_msgs genmsg)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
	tip.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
)

catkin_package()

