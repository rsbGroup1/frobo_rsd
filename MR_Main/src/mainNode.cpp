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


/*
* TODO 
* Change performAction by navigate_to
* Change the node to the class form
* Implement waitForHMIRequest
* 
*/

// Defines
#define M_PI			3.14159265358979323846
#define DEGREETORAD		(M_PI/180.0)
#define RADTODEGREE		(180.0/M_PI)
#define SSTR(x)			dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Global var
ros::ServiceClient _servicePerformAction, _serviceTipper;
ros::Publisher _hmiPublisher, _mesPublisher, _buttonPublisher;
bool _buttonStatus, _hmiStatus;
boost::mutex _runMutex;
int _destinationCell;

// Enum
enum ROBOT_POS
{
	nul = 0,
    box = 1,
    camera = 2,
    trackZone2 = 3,
    trackZone1 = 4,
    RC1 = 5,
    RC2 = 6,
    RC3 = 7
};

enum HMI_ICONS
{
	null = 00,
    tipper_up = 11,
	tipper_down = 12,
    lineFollowing_on = 21,
	lineFollowing_off = 22,
    gps_on = 31,
	gps_off = 32,
    collectingBricks_on = 41,
	collectingBricks_off = 42,
    insideBox_on = 51,
	insideBox_off = 52,
    charging_on = 61,
	charging_off = 62
};

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
    boost::unique_lock<boost::mutex> lock(_runMutex);
    if(msg.mobileRobot == 1)
    {
        _destinationCell = msg.cell;
        // START STUFF
        mr_navigation_controller::performAction obj;
        obj.request.action = "WTF";
		
        _servicePerformAction.call(obj);
    }
}

void navStatusCallback(std_msgs::String msg)
{

}

void navCurrentNodeCallback(std_msgs::String msg)
{
    if(msg.data == "conveyer")
    {
        mr_tip_controller::tip obj;
        obj.request.direction = true; // up
        _serviceTipper.call(obj);
        obj.request.direction = false; // down
        _serviceTipper.call(obj);
    }
}

void HMIUpdatePosition(ROBOT_POS pos)
{
    std_msgs::String obj;
    obj.data = "00" + SSTR(pos) + "0,,";
    _hmiPublisher.publish(obj);
}

void HMIUpdateIcons(HMI_ICONS state, bool value)
{
    std_msgs::String obj;
    obj.data = SSTR(state) + (value?"1":"0") + "00,,";
    _hmiPublisher.publish(obj);
}

void HMISendError(std::string msg)
{
    std_msgs::String obj;
    obj.data = "0003," + msg + ",";
    _hmiPublisher.publish(obj);
}

void HMISendInfo(std::string msg)
{
    std_msgs::String obj;
    obj.data = "0001," + msg + ",";
    _hmiPublisher.publish(obj);
}

void HMISendWarning(std::string msg)
{
    std_msgs::String obj;
    obj.data = "0002," + msg + ",";
    _hmiPublisher.publish(obj);
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
    pNh.param<std::string>("nav_perform_srv", performActionString, "/mrNavigationController/performAction");
    pNh.param<std::string>("nav_status_sub", navStatusSub, "/mrNavigationController/status");
    pNh.param<std::string>("nav_currentnode_sub", navCurrentnodeSub, "/mrNavigationController/currentNode");
    pNh.param<std::string>("button_sub", buttonSub, "/mrButton/run");
    pNh.param<std::string>("button_pub", buttonPub, "/mrButton/status");
    pNh.param<std::string>("hmi_sub", hmiSub, "/mrHMI/run");
    pNh.param<std::string>("hmi_pub", hmiPub, "/mrHMI/status");
    pNh.param<std::string>("tipper_srv", tipperString, "/mrTipController/tip");
    pNh.param<std::string>("mes_pub", mesPub, "/mrMESClient/msgToServer");
    pNh.param<std::string>("mes_sub", mesSub, "/mrMESClient/msgFromServer");

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

	
    ros::Rate rate(1);
    for(int i=0; i<3; i++)
	    rate.sleep();

	HMISendError("helloError");
	HMISendWarning("helloWarning");

    // ROS Spin: Handle callbacks
    while(ros::ok)
        ros::spinOnce();

    // Return
    return 0;
}



/*
 * mrNavigationController/status
 * following_line + (data) qr_name
 * linear_move + (data) distance
 * angular_move + (data) angle
 * free_navigation + (data) x + (data) y
 */

/* mrNavigationController/currentNode
 * line_start
 * line_stop
 * wc1
 * wc1_conveyor
 * wc1_robot
 * wc2
 * wc2_conveyor
 * wc2_robot
 * wc3
 * wc3_conveyor
 * wc3_robot
 * wc_exit
 * box
 * pre_charge
 * charge
 * pre_bricks
 * bricks
 */