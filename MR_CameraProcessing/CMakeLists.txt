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

catkin_package(
	#INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
	#LIBRARIES 
)

# Use OpenCV
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(SrcFiles src/cameraProcessingNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")
