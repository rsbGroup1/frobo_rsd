cmake_minimum_required(VERSION 2.8.3)
project(mr_camera)

set(OpenCV_DIR "/usr/share/OpenCV")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge genmsg image_transport sensor_msgs OpenCV)

#generate_messages(DEPENDENCIES sensor_msgs)

add_service_files(
	FILES
	enable.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	#INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
	#LIBRARIES 
)

# Use OpenCV
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(SrcFiles src/cameraNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

MESSAGE(${PROJECT_NAME} "Done!")

