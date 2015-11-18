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
#include "std_msgs/Float32.h"
#include "mr_mes_client/server.h"

// Defines
#define SSTR(x)			dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

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

enum HMI_SAFETY
{
    safe = 0,
    proximityAlert = 1,
    colliding = 2
};

class MainNode
{
public:
    MainNode() :
        _pNh ("~"),
        _batteryLevel(0),
		_newOrder(false)
    {
        // Get parameter names
        _pNh.param<std::string> ("nav_perform_srv", _performActionString, "/mrNavigationController/performAction");
        _pNh.param<std::string> ("nav_status_sub", _navStatusSub, "/mrNavigationController/status");
        _pNh.param<std::string> ("nav_currentnode_sub", _navCurrentnodeSub, "/mrNavigationController/currentNode");
        _pNh.param<std::string> ("button_sub", _buttonSub, "/mrButton/run");
        _pNh.param<std::string> ("button_pub", _buttonPub, "/mrButton/status");
        _pNh.param<std::string> ("hmi_sub", _hmiSub, "/mrHMI/run");
        _pNh.param<std::string> ("hmi_pub", _hmiPub, "/mrHMI/status");
        _pNh.param<std::string> ("tipper_srv", _tipperString, "/mrTipController/tip");
        _pNh.param<std::string> ("mes_pub", _mesPub, "/mrMESClient/msgToServer");
        _pNh.param<std::string> ("mes_sub", _mesSub, "/mrMESClient/msgFromServer");
        _pNh.param<std::string> ("obstacle_detector_sub", _obstacleDetectorSub, "/mrObstacleDetector/status");
        _pNh.param<std::string> ("battery_sub", _batterySub, "/fmInformation/battery");
        _pNh.param<double> ("battery_low", _batteryLow, 12.4);
        _pNh.param<double> ("battery_critic", _batteryCritic, 12.1);
        _pNh.param<double> ("desired_charge", _desiredCharge, 14.1);

        // Services
        _servicePerformAction = _nh.serviceClient<mr_navigation_controller::performAction> (_performActionString);
        _serviceTipper = _nh.serviceClient<mr_tip_controller::tip> (_tipperString);

        // Publishers
        _hmiPublisher = _nh.advertise<std_msgs::String> (_hmiPub, 10);
        _mesPublisher = _nh.advertise<std_msgs::String> (_mesPub, 10);
        _buttonPublisher = _nh.advertise<std_msgs::Bool> (_buttonPub, 10);

        // Subscribers
        _buttonSubriber = _nh.subscribe<std_msgs::Bool> (_buttonSub, 5, &MainNode::buttonCallback, this);
        _hmiSubscriber = _nh.subscribe<std_msgs::String> (_hmiSub, 5, &MainNode::hmiCallback, this);
        _navStatusSubscriber = _nh.subscribe<std_msgs::String> (_navStatusSub, 1, &MainNode::navStatusCallback, this);
        _navCurrentSubscriber = _nh.subscribe<std_msgs::String> (_navCurrentnodeSub, 1, &MainNode::navCurrentNodeCallback, this);
        _mesSubscriber = _nh.subscribe<mr_mes_client::server> (_mesSub, 1, &MainNode::mesCallback, this);
        _obstacleDetectorSubscriber = _nh.subscribe<std_msgs::String> (_obstacleDetectorSub, 10, &MainNode::obstacleCallback, this);
        _batterySubscriber = _nh.subscribe<std_msgs::Float32> (_batterySub, 1, &MainNode::_batteryCallback, this);

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
    void HMIUpdatePosition (ROBOT_POS pos)
    {
        std_msgs::String obj;
        obj.data = "0" + SSTR (pos) + "00,,";
        _hmiPublisher.publish (obj);
    }

    /**
     * Updates the icons that inform to the user what the robot is doing
     */
    void HMIUpdateIcons (HMI_ICONS state)
    {
        std_msgs::String obj;
        obj.data = "00" + SSTR (state) + "0,,";
        _hmiPublisher.publish (obj);
    }

    /**
     * Updates the icons that inform to the user the safety status of the robot
     */
    void HMIUpdateSafety (HMI_SAFETY state)
    {
        std_msgs::String obj;
        obj.data = "000" + SSTR (state) + ",,";
        _hmiPublisher.publish (obj);
    }

    /**
     * Sends to the HMI a error message
     */
    void HMISendError (std::string msg)
    {
        std_msgs::String obj;
        obj.data = "3000," + msg + ",";
        _hmiPublisher.publish (obj);
    }

    /**
     * Sends to the HMI a info message
     */
    void HMISendInfo (std::string msg)
    {
        std_msgs::String obj;
        obj.data = "1000," + msg + ",";
        _hmiPublisher.publish (obj);
    }

    /**
     * Sends to the HMI a warning message
     */
    void HMISendWarning (std::string msg)
    {
        std_msgs::String obj;
        obj.data = "2000," + msg + ",";
        _hmiPublisher.publish (obj);
    }


    /*
     *
     * Callbacks
     *
     */
    /**
     * When the botton is pressed,
     */
    void buttonCallback (std_msgs::Bool msg)
    {
        boost::unique_lock<boost::mutex> lock (_runMutex);
        _buttonAuto = msg.data;
    }

    /**
     * Controls the mode of the robot from the HMI. Automatic or Manual
     */
    void hmiCallback (std_msgs::String msg)
    {
        boost::unique_lock<boost::mutex> lock (_runMutex);

        if (msg.data == "start")
            _hmiAuto = true;
        else if (msg.data == "stop" || msg.data == "manual")
            _hmiAuto = false;
    }

    /**
	 * Copies the MES message to the internal msg
	 */
    void mesCallback (mr_mes_client::server msg)
    {
		boost::unique_lock<boost::mutex> lock (_new_MESmsg);
		_msg_last = msg;
		_newOrder = true;
    }
    
    /**
     * Based on the last MES message, if status is false, 
	 * which means that robotic arm hasn't finished the order do:
     * 1. Goes to pick some bricks to the dispenser
     * 2. Goes to the desired workcell coveyor
     * 3. Leave the bricks in the conveyor with the tipper
     * 4. Moves to the robot position of the same workcell
	 * 	 * 1. Goes to charge
	 * 2. Send an "Ok" to the MES Server indicating that we are ready
	 * for another order
     */
    void MESProcessOrder()
	{
		mr_mes_client::server msg;
		_new_MESmsg.lock();
		msg = _msg_last;
		bool newOrder = _newOrder;
		_new_MESmsg.unlock();
		
		_runMutex.lock();
		bool automode = _buttonAuto | _hmiAuto;
		_runMutex.unlock();
		
		if (msg.mobileRobot == 1 && automode && newOrder)
		{
			
			mr_navigation_controller::performAction perform_action_obj;
			mr_tip_controller::tip tip_obj;
			std::string action;
			
			// Stores the current position just in case the battery is in
			// the critic level
			action = _currentNode;
			// Checks if the battery is the critic level
			checkBattery (_batteryCritic, action);
			
			// Go to the dispenser position
			action = "bricks";
			perform_action_obj.request.action = action;
			_servicePerformAction.call (perform_action_obj);
			
			// Checks if the battery is the critic level
			checkBattery (_batteryCritic, action);
			
			// Send the robot to the correct wc conveyor
			if (msg.cell == 1)
				action = "wc1_conveyor";
			if (msg.cell == 2)
				action = "wc2_conveyor";
			if (msg.cell == 3)
				action = "wc3_conveyor";
			perform_action_obj.request.action = action;
			_servicePerformAction.call (perform_action_obj);
			
			// Tip Up
			HMIUpdateIcons (tipper);
			tip_obj.request.direction = true;
			_serviceTipper.call (tip_obj);
			while(tip_obj.response.status != true) 
				;; //Wait
				// Tip Down
				tip_obj.request.direction = false;
			_serviceTipper.call (tip_obj);
			while(tip_obj.response.status != true) 
				;; //Wait
				HMIUpdateIcons (null);
			
			// Checks if the battery is the critic level
			checkBattery (_batteryCritic, action);
			
			// Go to the robot
			if (msg.cell == 1)
				action = "wc1_robot";
			if (msg.cell == 2)
				action = "wc2_robot";
			if (msg.cell == 3)
				action = "wc3_robot";
			perform_action_obj.request.action = action;
			_servicePerformAction.call (perform_action_obj);
			
			_new_MESmsg.lock();
			msg = _msg_last;
			_new_MESmsg.unlock();
			while(msg.status != 1) {
				HMISendInfo("Waiting for robot to complete");
				_new_MESmsg.lock();
				msg = _msg_last;
				_new_MESmsg.unlock();
				usleep(5000);
			}
	
			// Go to charge position
			perform_action_obj.request.action = "charge";
			_servicePerformAction.call (perform_action_obj);
			
			std_msgs::String msg;
			msg.data = "Ok";
			_mesPublisher.publish(msg);
			
			// Charges the battery until the threshold
			chargeBattery (_batteryLow);
			
			_new_MESmsg.unlock();
			_newOrder = false;
			_new_MESmsg.lock();
		}
	}

    /**
     * Reads the navigation status and activate the icons in the HMI
     */
    void navStatusCallback (std_msgs::String msg)
    {
        if (msg.data == "following_line")
            HMIUpdateIcons (lineFollowing);
        else if (msg.data == "linear_move")
            HMIUpdateIcons (fixedMovement);
        else if (msg.data == "angular_move")
            HMIUpdateIcons (fixedMovement);
        else if (msg.data == "free_navigation")
            HMIUpdateIcons (gps);
    }

    /**
     * Reads the current robot position and send it to the HMI
     */
    void navCurrentNodeCallback (std_msgs::String msg)
    {
        _currentNode = msg.data;

        if (msg.data == "line_start")
            HMIUpdatePosition (trackZone1);
        else if (msg.data == "line_stop")
            HMIUpdatePosition (trackZone1);
        else if (msg.data == "wc1")
            HMIUpdatePosition (RC1);
        else if (msg.data == "wc1_conveyor")
            HMIUpdatePosition (RC1);
        else if (msg.data == "wc1_robot")
            HMIUpdatePosition (RC1);
        else if (msg.data == "wc2")
            HMIUpdatePosition (RC2);
        else if (msg.data == "wc2_conveyor")
            HMIUpdatePosition (RC2);
        else if (msg.data == "wc2_robot")
            HMIUpdatePosition (RC2);
        else if (msg.data == "wc3")
            HMIUpdatePosition (RC3);
        else if (msg.data == "wc3_conveyor")
            HMIUpdatePosition (RC3);
        else if (msg.data == "wc3_robot")
            HMIUpdatePosition (RC3);
        else if (msg.data == "wc_exit")
            HMIUpdatePosition (trackZone1);
        else if (msg.data == "box")
            HMIUpdatePosition (camera);
        else if (msg.data == "pre_charge")
            HMIUpdatePosition (box);
        else if (msg.data == "charge")
            HMIUpdatePosition (box);
        else if (msg.data == "pre_bricks")
            HMIUpdatePosition (box);
        else if (msg.data == "bricks")
            HMIUpdatePosition (box);
        else
            HMIUpdatePosition (nul);
    }

    /**
     * Check if the robot is close or colliding with something and
     * carry out the situation at the same time that shows the state
     * in the HMI
     */
    void obstacleCallback (std_msgs::String status)
    {
        // Checks if the safety status has changed
        // This avoids unnecesary messages
        if (status.data != safety_status_prev)
        {
            if (status.data == "safe")
                HMIUpdateSafety (safe);
            else if (status.data == "proximityAlert")
                HMIUpdateSafety (proximityAlert);
            else if (status.data == "colliding")
                HMIUpdateSafety (colliding);
        }

        safety_status_prev = status.data;
    }

    /**
     * Reads the level of the battery and store it
     */
    void _batteryCallback (std_msgs::Float32 battery)
    {
        _batteryLevel = battery.data;
    }

    /**
     * Check if the battery is under the given threshold. If so
     * goes to charge until it reaches the limit
     */
    void checkBattery (float threshold, std::string prev_pos)
    {
        mr_navigation_controller::performAction perform_action_obj;

        if (_batteryLevel == 0)
            std::cout << "No battery level! Waiting..." << std::endl;

        while (_batteryLevel == 0) // Wait
            ;;

        if (_batteryLevel < threshold)
        {
            perform_action_obj.request.action = "charge";
            _servicePerformAction.call (perform_action_obj);

            while (_batteryLevel < _desiredCharge)
                ; // Wait
        }

        perform_action_obj.request.action = prev_pos;
        _servicePerformAction.call (perform_action_obj);
    }
    
    /**
	 * Charge until it reaches the limit
	 */
	void chargeBattery (float threshold)
	{
		if (_batteryLevel == 0)
			std::cout << "No battery level! Waiting..." << std::endl;
		
		while (_batteryLevel == 0) // Wait
			;;
		
		if (_batteryLevel < threshold)
		{
			while (_batteryLevel < _desiredCharge)
				; // Wait
		}
	}


private:
    ros::NodeHandle _nh, _pNh;
    ros::ServiceClient _servicePerformAction, _serviceTipper;
    ros::Subscriber _buttonSubriber, _hmiSubscriber, _navStatusSubscriber, _navCurrentSubscriber,
        _mesSubscriber, _obstacleDetectorSubscriber, _batterySubscriber;
    ros::Publisher _hmiPublisher, _mesPublisher, _buttonPublisher;
    bool _buttonAuto, _hmiAuto, _newOrder;
    std::string safety_status_prev, _currentNode;
    double _batteryLevel, _batteryLow, _batteryCritic, _desiredCharge;
    boost::mutex _runMutex, _new_MESmsg;
    std::string _performActionString, _navStatusSub, _navCurrentnodeSub, _buttonSub, _buttonPub,
        _hmiSub, _tipperString, _hmiPub, _mesSub, _mesPub, _obstacleDetectorSub, _batterySub;
	mr_mes_client::server _msg_last;
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
    ros::init (argc, argv, "MR_Main");

    // Main Node
    MainNode* mn = new MainNode();

    // Rate
    ros::Rate rate (30);

    // Multithreading
    ros::AsyncSpinner spinner (0);

    int i = 0;
    // ROS Spin: Handle callbacks
    while (!ros::isShuttingDown())
    {
		mn->MESProcessOrder();
		
		// Spin
        spinner.start();
        rate.sleep();
    }

    // Return
    return 0;
}
