// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include "mr_navigation_controller/performAction.h"
#include "mr_tip_controller/tip.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

// Defines
#define M_PI                    3.14159265358979323846
#define DEGREETORAD             (M_PI/180.0)
#define RADTODEGREE             (180.0/M_PI)
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Global var
ros::ServiceClient _servicePerformAction;
bool _buttonStatus, _hmiStatus;
boost::mutex _runMutex;

// Functions
void buttonCallback(std_msgs::Bool msg)
{
    boost::unique_lock<boost::mutex> lock(_runMutex);
    _buttonStatus = msg.data;
}

void hmiCallback(std_msgs::String msg)
{
    boost::unique_lock<boost::mutex> lock(_runMutex);

    if(msg.data == "run")
        _hmiStatus = true;
    else if(msg.data == "idle" || msg.data == "manual")
        _hmiStatus = false;
}

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
    std::string performActionString, buttonSub, hmiSub;
    pNh.param<std::string>("lineFollowService", performActionString, "mrNavigationController/performAction");
    pNh.param<std::string>("button_sub", buttonSub, "mrButton/run");
    pNh.param<std::string>("hmi_sub", hmiSub, "mrHMI/run");

    // Service
    _servicePerformAction = nh.serviceClient<mr_navigation_controller::performAction>(performActionString);

    // Topic
    ros::Subscriber buttonSubriber = nh.subscribe(buttonSub, 1, buttonCallback);
    ros::Subscriber hmiSubriber = nh.subscribe(hmiSub, 1, hmiCallback);

    // ROS Spin: Handle callbacks
    while(ros::ok)
        ros::spinOnce();

    // Return
    return 0;
}
