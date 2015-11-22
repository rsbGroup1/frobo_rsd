cmake_minimum_required(VERSION 2.8.3)
project(mr_deadmanpriority)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS msgs roscpp rospy)

catkin_python_setup()

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

include_directories(
#  ${catkin_INCLUDE_DIRS}
)

