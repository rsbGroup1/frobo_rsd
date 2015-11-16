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
*/

// Defines
#define M_PI			3.14159265358979323846
#define DEGREETORAD		(M_PI/180.0)
#define RADTODEGREE		(180.0/M_PI)
#define SSTR(x)			dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Global var


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

class MainNode{
public:
	MainNode() : 
		      _pNh("~")	
	{
		// Get parameter names
		_pNh.param<std::string>("nav_perform_srv", _performActionString, "/mrNavigationController/performAction");
		_pNh.param<std::string>("nav_status_sub", _navStatusSub, "/mrNavigationController/status");
		_pNh.param<std::string>("nav_currentnode_sub", _navCurrentnodeSub, "/mrNavigationController/currentNode");
		_pNh.param<std::string>("button_sub", _buttonSub, "/mrButton/run");
		_pNh.param<std::string>("button_pub", _buttonPub, "/mrButton/status");
		_pNh.param<std::string>("hmi_sub", _hmiSub, "/mrHMI/run");
		_pNh.param<std::string>("hmi_pub", _hmiPub, "/mrHMI/status");
		_pNh.param<std::string>("tipper_srv", _tipperString, "/mrTipController/tip");
		_pNh.param<std::string>("mes_pub", _mesPub, "/mrMESClient/msgToServer");
		_pNh.param<std::string>("mes_sub", _mesSub, "/mrMESClient/msgFromServer");
		
		// Services
		_servicePerformAction = _nh.serviceClient<mr_navigation_controller::performAction>(_performActionString);
		_serviceTipper = _nh.serviceClient<mr_tip_controller::tip>(_tipperString);
		
		// Publishers
		_hmiPublisher = _nh.advertise<std_msgs::String>(_hmiPub, 10);
		_mesPublisher = _nh.advertise<std_msgs::String>(_mesPub, 10);
		_buttonPublisher = _nh.advertise<std_msgs::Bool>(_buttonPub, 10);
		
		// Subscribers
		ros::Subscriber buttonSubriber = _nh.subscribe<std_msgs::Bool>(_buttonSub, 5, &MainNode::buttonCallback, this);
		ros::Subscriber hmiSubscriber = _nh.subscribe<std_msgs::String>(_hmiSub, 5, &MainNode::hmiCallback, this);
		ros::Subscriber navStatusSubscriber = _nh.subscribe<std_msgs::String>(_navStatusSub, 10, &MainNode::navStatusCallback, this);
		ros::Subscriber navCurrentSubscriber = _nh.subscribe<std_msgs::String>(_navCurrentnodeSub, 10, &MainNode::navCurrentNodeCallback, this);
		ros::Subscriber mesSubscriber = _nh.subscribe<mr_mes_client::server>(_mesSub, 10, &MainNode::mesCallback, this);
		
	}
	
	~MainNode()
	{
		//Nothing
	}
	
	
	/*
	 * 
	 * HMI
	 * 
	 */
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
	
	
	/*
	 * 
	 * Callbacks
	 * 
	 */
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
			mr_navigation_controller::performAction perform_action_obj;
			mr_tip_controller::tip tip_obj;
			
			// Send the robot to the correct wc conveyor
			if (msg.cell == 1)
				perform_action_obj.request.action = "wc1_conveyor";
			if (msg.cell == 2)
				perform_action_obj.request.action = "wc2_conveyor";
			if (msg.cell == 3)
				perform_action_obj.request.action = "wc3_conveyor";		
			_servicePerformAction.call(perform_action_obj);
			
			// Tip 
			tip_obj.request.direction = true;
			_serviceTipper.call(tip_obj);
			tip_obj.request.direction = false;
			_serviceTipper.call(tip_obj);
			
			// Go to the robot
			if (msg.cell == 1)
				perform_action_obj.request.action = "wc1_robot";
			if (msg.cell == 2)
				perform_action_obj.request.action = "wc2_robot";
			if (msg.cell == 3)
				perform_action_obj.request.action = "wc3_robot";		
			_servicePerformAction.call(perform_action_obj);
			
		}
	}
	
	void navStatusCallback(std_msgs::String msg)
	{
		
	}
	
	void navCurrentNodeCallback(std_msgs::String msg)
	{
		if (msg.data == "line_start")
			HMIUpdatePosition(trackZone1);
		else if (msg.data == "line_stop")
			HMIUpdatePosition(trackZone1);
		else if (msg.data == "wc1")
			HMIUpdatePosition(RC1);
		else if (msg.data == "wc1_conveyor")
			HMIUpdatePosition(RC1);
		else if (msg.data == "wc1_robot")
			HMIUpdatePosition(RC1);
		else if (msg.data == "wc2")
			HMIUpdatePosition(RC2);
		else if (msg.data == "wc2_conveyor")
			HMIUpdatePosition(RC2);
		else if (msg.data == "wc2_robot")
			HMIUpdatePosition(RC2);
		else if (msg.data == "wc3")
			HMIUpdatePosition(RC3);
		else if (msg.data == "wc3_conveyor")
			HMIUpdatePosition(RC3);
		else if (msg.data == "wc3_robot")
			HMIUpdatePosition(RC3);
		else if (msg.data == "wc_exit")
			HMIUpdatePosition(trackZone1);
		else if (msg.data == "box")
			HMIUpdatePosition(camera);
		else if (msg.data == "pre_charge")
			HMIUpdatePosition(box);
		else if (msg.data == "charge")
			HMIUpdatePosition(box);
		else if (msg.data == "pre_bricks")
			HMIUpdatePosition(box);
		else if (msg.data == "bricks")
			HMIUpdatePosition(box);
		else
			HMIUpdatePosition(nul);
	}
	
	
private:
	ros::NodeHandle _nh, _pNh;
	ros::ServiceClient _servicePerformAction, _serviceTipper;
	ros::Publisher _hmiPublisher, _mesPublisher, _buttonPublisher;
	bool _buttonStatus, _hmiStatus;
	boost::mutex _runMutex;
	std::string _performActionString, _navStatusSub, _navCurrentnodeSub, _buttonSub, _buttonPub, _hmiSub, _tipperString, _hmiPub, _mesSub, _mesPub;
};


/*
 * Main
 */
int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "MR_Main");
	
	// Main Node
	MainNode mn;
	
	// Rate
	ros::Rate rate(1);
	
    // ROS Spin: Handle callbacks
    while(ros::ok) {
        ros::spinOnce();
		rate.sleep();
	}

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