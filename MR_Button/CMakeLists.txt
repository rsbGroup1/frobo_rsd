cmake_minimum_required(VERSION 2.8.3)
project(mr_button)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs serial genmsg mr_main)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/buttonNode.cpp)

add_executable(${PROJECT_NAME} ${SrcFiles})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

MESSAGE(${PROJECT_NAME} "Done!")

