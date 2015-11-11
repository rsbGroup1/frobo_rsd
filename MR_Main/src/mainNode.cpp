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
#include "mr_mes_client/server.h"

// Defines
#define M_PI                    3.14159265358979323846
#define DEGREETORAD             (M_PI/180.0)
#define RADTODEGREE             (180.0/M_PI)
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Global var
ros::ServiceClient _servicePerformAction, _serviceTipper;
ros::Publisher _hmiPublisher, _mesPublisher, _buttonPublisher;
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

    if(msg.data == "start")
        _hmiStatus = true;
    else if(msg.data == "stop" || msg.data == "manual")
        _hmiStatus = false;
}

void mesCallback(mr_mes_client::server msg)
{
    msg.mobileRobot;
    msg.cell;
}

void navStatusCallback(std_msgs::String msg)
{

}

void navCurrentNodeCallback(std_msgs::String msg)
{

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
    std::string performActionString, navStatusSub, navCurrentnodeSub, buttonSub, buttonPub, hmiSub, tipperString, hmiPub, mesSub, mesPub;
    pNh.param<std::string>("nav_perferm_srv", performActionString, "mrNavigationController/performAction");
    pNh.param<std::string>("nav_status_sub", navStatusSub, "mrNavigationController/status");
    pNh.param<std::string>("nav_currentnode_sub", navCurrentnodeSub, "mrNavigationController/currentNode");
    pNh.param<std::string>("button_sub", buttonSub, "mrButton/run");
    pNh.param<std::string>("button_pub", buttonPub, "mrButton/status");
    pNh.param<std::string>("hmi_sub", hmiSub, "mrHMI/run");
    pNh.param<std::string>("hmi_pub", hmiPub, "mrHMI/status");
    pNh.param<std::string>("tipper_srv", tipperString, "mrTipController/tip");
    pNh.param<std::string>("mes_pub", mesPub, "mrMESClient/msgToServer");
    pNh.param<std::string>("mes_sub", mesSub, "mrMESClient/msgFromServer");

    // Service
    _servicePerformAction = nh.serviceClient<mr_navigation_controller::performAction>(performActionString);
    _serviceTipper = nh.serviceClient<mr_tip_controller::tip>(tipperString);

    // Publish
    _hmiPublisher = nh.advertise<std_msgs::String>(hmiPub, 10);
    _mesPublisher = nh.advertise<std_msgs::String>(mesPub, 10);
    _buttonPublisher = nh.advertise<std_msgs::Bool>(buttonPub, 10);

    // Subscribe
    ros::Subscriber buttonSubriber = nh.subscribe(buttonSub, 5, buttonCallback);
    ros::Subscriber hmiSubscriber = nh.subscribe(hmiSub, 5, hmiCallback);
    ros::Subscriber navStatusSubscriber = nh.subscribe(navStatusSub, 10, navStatusCallback);
    ros::Subscriber navCurrentSubscriber = nh.subscribe(navCurrentnodeSub, 10, navCurrentNodeCallback);
    ros::Subscriber mesSubscriber = nh.subscribe(mesSub, 10, mesCallback);

    // ROS Spin: Handle callbacks
    while(ros::ok)
        ros::spinOnce();

    // Return
    return 0;
}
