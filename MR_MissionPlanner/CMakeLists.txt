cmake_minimum_required(VERSION 2.8.3)
project(rsd_mission_planner)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs serial genmsg)

#generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

SET(SrcFiles src/missionPlannerNode.cpp)

add_executable(rsd_mission_planner_node ${SrcFiles})

target_link_libraries(rsd_mission_planner_node ${catkin_LIBRARIES})

MESSAGE(${PROJECT_NAME} "Done!")

