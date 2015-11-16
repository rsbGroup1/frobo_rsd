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
    null = 0,
    tipper = 1,
    lineFollowing = 2,
    gps = 3,
    collectingBricks = 4,
    fixedMovement = 5,
    charging = 6,
};

class MainNode {
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
	/**
	 * Updates the position in the HMI Map
	 */
    void HMIUpdatePosition(ROBOT_POS pos)
    {
        std_msgs::String obj;
        obj.data = "00" + SSTR(pos) + "0,,";
        _hmiPublisher.publish(obj);
    }

    /**
	 * Updates the icons that inform to the user what the robot is doing
	 */
    void HMIUpdateIcons(HMI_ICONS state)
    {
        std_msgs::String obj;
        obj.data = SSTR(state) + "00,,";
        _hmiPublisher.publish(obj);
    }
	
	/**
	 * Sends to the HMI a error message
	 */
    void HMISendError(std::string msg)
    {
        std_msgs::String obj;
        obj.data = "0003," + msg + ",";
        _hmiPublisher.publish(obj);
    }

    /**
	 * Sends to the HMI a info message
	 */
    void HMISendInfo(std::string msg)
    {
        std_msgs::String obj;
        obj.data = "0001," + msg + ",";
        _hmiPublisher.publish(obj);
    }

    /**
	 * Sends to the HMI a warning message
	 */
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
	/**
	 * When the botton is pressed, 
	 */
    void buttonCallback(std_msgs::Bool msg)
    {
        boost::unique_lock<boost::mutex> lock(_runMutex);
        _buttonStatus = msg.data;
    }

    /**
	 * Controls the mode of the robot from the HMI. Automatic or Manual
	 */
    void hmiCallback(std_msgs::String msg)
    {
        boost::unique_lock<boost::mutex> lock(_runMutex);

        if(msg.data == "start")
            _hmiStatus = true;
        else if(msg.data == "stop" || msg.data == "manual")
            _hmiStatus = false;
    }

    /**
	 * Reads from the MES server if there is a new order.
	 * If so:
	 * 1. Goes to pick some bricks to the dispenser
	 * 2. Goes to the desired workcell coveyor
	 * 3. Leave the bricks in the conveyor with the tipper
	 * 4. Moves to the robot position of the same workcell
	 */
    void mesCallback(mr_mes_client::server msg)
    {
        boost::unique_lock<boost::mutex> lock(_runMutex);
        if(msg.mobileRobot == 1)
        {
            mr_navigation_controller::performAction perform_action_obj;
            mr_tip_controller::tip tip_obj;
			
			// Go to the dispenser position
			perform_action_obj.request.action = "bricks";
			_servicePerformAction.call(perform_action_obj);

            // Send the robot to the correct wc conveyor
            if (msg.cell == 1)
                perform_action_obj.request.action = "wc1_conveyor";
            if (msg.cell == 2)
                perform_action_obj.request.action = "wc2_conveyor";
            if (msg.cell == 3)
                perform_action_obj.request.action = "wc3_conveyor";
            _servicePerformAction.call(perform_action_obj);

            // Tip Up
			HMIUpdateIcons(tipper);
            tip_obj.request.direction = true;
            _serviceTipper.call(tip_obj);
			// Tip Down
            tip_obj.request.direction = false;
            _serviceTipper.call(tip_obj);
			HMIUpdateIcons(null);

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

    /**
	 * Reads the navigation status and activate the icons in the HMI
	 */
    void navStatusCallback(std_msgs::String msg)
    {
		if (msg.data == "following_line")
			HMIUpdateIcons(lineFollowing);
		else if (msg.data == "linear_move")
			HMIUpdateIcons(fixedMovement);
		else if (msg.data == "angular_move")
			HMIUpdateIcons(fixedMovement);
		else if (msg.data == "free_navigation")
			HMIUpdateIcons(gps);
    }
    
    /**
	 * Reads the current robot position and send it to the HMI
	 */
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