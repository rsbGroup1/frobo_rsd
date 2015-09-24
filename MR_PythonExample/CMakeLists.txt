cmake_minimum_required(VERSION 2.8.3)

project(mr_python_example)

find_package(catkin REQUIRED COMPONENTS msgs roscpp rospy)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frobit_demo
#  CATKIN_DEPENDS msgs roscpp rospy
#  DEPENDS system_lib
)

