cmake_minimum_required(VERSION 2.8.3)
project(mr_bridgewalltracking)

find_package(catkin REQUIRED COMPONENTS msgs roscpp rospy cv_bridge image_transport)
find_package(OpenCV REQUIRED core imgproc highgui)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(BridgeWallTracking src/bridge_wall_tracking.cpp src/linreg.cpp)
add_executable(OdomAddCov src/covariance_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(BridgeWallTracking
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

target_link_libraries(OdomAddCov
   ${catkin_LIBRARIES}
)
