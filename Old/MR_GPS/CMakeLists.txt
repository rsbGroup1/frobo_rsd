cmake_minimum_required(VERSION 2.8.3)
project(mr_gps)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/GPSNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(GPSLocalisation src/GPSLocalisationNode.cpp)
target_link_libraries(GPSLocalisation ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

