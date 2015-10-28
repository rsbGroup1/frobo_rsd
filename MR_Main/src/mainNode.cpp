// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include "mr_main/performAction.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

// Defines
#define M_PI                    3.14159265358979323846
#define DEGREETORAD             (M_PI/180.0)
#define RADTODEGREE             (180.0/M_PI)
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Global var
ros::ServiceClient _servicePerformAction;

// Functions
int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "MR_Main");
    ros::NodeHandle nh, pNh("~");

    // Get parameter names
    std::string performActionString;
    pNh.param<std::string>("lineFollowService", performActionString, "mrNavigationController/performAction");

    // Service
    _servicePerformAction = nh.serviceClient<mr_main::performAction>(performActionString);

    // ROS Spin: Handle callbacks
    while(ros::ok)
        ros::spinOnce();

    // Return
    return 0;
}
