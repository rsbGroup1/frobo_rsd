cmake_minimum_required(VERSION 2.8.3)
project(mr_kalman_filter)

#set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED

find_package(catkin REQUIRED COMPONENTS	roscpp rospy std_msgs message_generation)

#generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

# Use OpenCV
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

SET(SrcFiles src/kalmanFilterNode.cpp)

add_executable(${PROJECT_NAME} src/kalmanFilterNode.cpp)

#add_dependencies(${PROJECT_NAME} kalman_filter_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

