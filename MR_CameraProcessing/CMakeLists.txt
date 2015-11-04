cmake_minimum_required(VERSION 2.8.3)
project(mr_camera_processing)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	geometry_msgs 
	cv_bridge 
	genmsg 
	image_transport 
	sensor_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Generate services in the 'srv' folder
add_service_files(
	FILES
	enable.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
)


catkin_package(
	#INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
	#LIBRARIES 
)

# Use OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(SrcFiles src/cameraProcessingNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ZBAR_LIBRARIES} zbar)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

MESSAGE(${PROJECT_NAME} "Done!")
