cmake_minimum_required(VERSION 2.8.3)
project(mr_line_follower)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge genmsg image_transport sensor_msgs)

catkin_package()

# Use OpenCV
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(SrcFiles src/lineFollowerNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

