cmake_minimum_required(VERSION 2.8.3)
project(mr_camera_false)

set(OpenCV_DIR "/usr/share/OpenCV")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge genmsg image_transport sensor_msgs OpenCV)

#generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

# Use OpenCV
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(SrcFiles src/mr_camera_false_node.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")
