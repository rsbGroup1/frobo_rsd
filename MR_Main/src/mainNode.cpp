// Includes
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "msgs/BoolStamped.h"

#include "mr_navigation_controller/performAction.h"
#include "mr_tip_controller/tip.h"
#include "mr_mes_client/server.h"
#include "mr_main/run.h"

// Defines
#define SSTR(x)			dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()

// Enum
enum HMI_ROBOT_POS
{
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
    tipper = 1,
    lineFollowing = 2,
    gps = 3,
    collectingBricks = 4,
    fixedMovement = 5,
    charging = 6,
};

enum MODE
{
    IDLE = 0,
    AUTO = 1,
    MANUAL = 2
};

class MainNode
{
public:
    MainNode() :
        _pNh ("~"),
        _batteryLevel (0),
        _newOrder (false),
        _criticalFaultSignalRunning(false),
        _rate(30)
    {
        // Get parameter names
        _pNh.param<std::string> ("nav_perform_srv", _performActionString, "/mrNavigationController/performAction");
        _pNh.param<std::string> ("nav_status_sub", _navStatusSub, "/mrNavigationController/status");
        _pNh.param<std::string> ("nav_currentnode_sub", _navCurrentnodeSub, "/mrNavigationController/currentNode");
        _pNh.param<std::string> ("hmi_pub", _hmiPub, "/mrHMI/status");
        _pNh.param<std::string> ("tipper_srv", _tipperString, "/mrTipController/tip");
        _pNh.param<std::string> ("mes_pub", _mesPub, "/mrMESClient/msgToServer");
        _pNh.param<std::string> ("mes_sub", _mesSub, "/mrMESClient/msgFromServer");
        _pNh.param<std::string> ("obstacle_detector_sub", _obstacleDetectorSub, "/mrObstacleDetector/status");
        _pNh.param<std::string> ("critical_fault_pub", _criticalFaultSignalPub, "/fmSafe/critical_fault");
        _pNh.param<std::string> ("battery_sub", _batterySub, "/fmInformation/battery");
        _pNh.param<std::string> ("mode_pub", _modePub, "/mrMain/mode");
        _pNh.param<std::string> ("run_srv", _runSrv, "/mrMain/run");
		_pNh.param<bool> ("check_battery_low", _check_battery_low , true);
        _pNh.param<bool> ("check_battery_critic", _check_battery_critic , false);
        _pNh.param<double> ("battery_low", _batteryLow, 12.4);
        _pNh.param<double> ("battery_critic", _batteryCritic, 12.0);
        _pNh.param<double> ("desired_charge", _desiredCharge, 13.9);

        // Service subscribers
        _servicePerformAction = _nh.serviceClient<mr_navigation_controller::performAction> (_performActionString);
        _serviceTipper = _nh.serviceClient<mr_tip_controller::tip> (_tipperString);

        // Service providers
        _serviceSrvRun = _nh.advertiseService (_runSrv, &MainNode::runCallback, this);

        // Publishers
        _hmiPublisher = _nh.advertise<std_msgs::String> (_hmiPub, 10);
        _mesPublisher = _nh.advertise<std_msgs::String> (_mesPub, 10);
        _modePublisher = _nh.advertise<std_msgs::String> (_modePub, 10);
        _criticalFaultSignalPublisher = _nh.advertise<msgs::BoolStamped>(_criticalFaultSignalPub, 10);

        // Subscribers
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
    void HMIUpdatePosition (HMI_ROBOT_POS pos)
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

    /**
     * Sends to the HMI a error message
     */
    void HMISendError (std::string msg)
    {
        std_msgs::String obj;
        obj.data = "3000," + msg + ",";
        _hmiPublisher.publish (obj);
    }

    /*
     *
     * Callbacks
     *
     */
    /**
     * Run service (for changing mode)
     */
    bool runCallback (mr_main::run::Request& req, mr_main::run::Response& res)
    {
        if (req.state != _run_msg_last)
        {
            boost::unique_lock<boost::mutex> lock (_runMutex);
            if (req.state == "auto")
            {
                if(_criticalFaultSignalRunning == false)
                {
                    _criticalFaultSignalRunning = true;
                    _criticalFaultSignalThread = new boost::thread (&MainNode::enableCriticalFaultSignal, this);
                }
                HMISendInfo("Main: Auto mode!");
                _mode = AUTO;
            }
            else if (req.state== "manual")
            {
                if(_criticalFaultSignalRunning == false)
                {
                    _criticalFaultSignalRunning = true;
                    _criticalFaultSignalThread = new boost::thread (&MainNode::enableCriticalFaultSignal, this);
                }
                HMISendInfo("Main: Manual mode!");
                _mode = MANUAL;
            }
            else if (req.state == "idle")
            {
                if(_criticalFaultSignalRunning)
                {
                    _criticalFaultSignalRunning = false;
                    _criticalFaultSignalThread->interrupt();
                    delete _criticalFaultSignalThread;
                }
                HMISendInfo("Main: Idle mode!");
                _mode = IDLE;
            }

            std_msgs::String msgMode;
            if (_mode == AUTO)
                msgMode.data = "auto";
            else if (_mode == IDLE)
                msgMode.data = "idle";
            else
                msgMode.data = "manual";
            _modePublisher.publish(msgMode);

            _run_msg_last = req.state;
        }

        res.status = _run_msg_last;
        return true;
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
        MODE mode = _mode;
        _runMutex.unlock();
		
		ROS_INFO("MR waiting for an order");
		HMISendInfo("MR waiting for an order");
		ros::Rate (0.25).sleep();

        if (msg.mobileRobot == 1 && mode==AUTO && newOrder)
        {
			ROS_INFO("MR performing an order");
            mr_navigation_controller::performAction perform_action_obj;
            mr_tip_controller::tip tip_obj;
            std::string action;

            // Stores the current position just in case the battery is in
            // the critic level and checks if the battery is the critic level
            action = _currentNode;
            if (_check_battery_critic) 
                checkBattery (_batteryCritic, action);

            // Go to the dispenser position
            action = "bricks";
            perform_action_obj.request.action = action;
            _servicePerformAction.call (perform_action_obj);
	    
            // Checks if the battery is the critic level
            if (_check_battery_critic) 
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
	    
            // Starts the conveyor
            std_msgs::String msg_to_server;
            msg_to_server.data = "Ok";
            _mesPublisher.publish(msg_to_server);

            // Tip Up
            tip_obj.request.direction = true;
            _serviceTipper.call (tip_obj);
            ros::Rate (0.33).sleep();

            // Tip Down
            tip_obj.request.direction = false;
            _serviceTipper.call (tip_obj);

            // Checks if the battery is the critic level
            if (_check_battery_critic) 
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

            // Process the order
            msg_to_server.data = "Ok";
            _mesPublisher.publish(msg_to_server);
            
            // Checks if the battery is the critic level
            if (_check_battery_critic) 
	        checkBattery (_batteryCritic, action);

            _new_MESmsg.lock();
            msg = _msg_last;
            _new_MESmsg.unlock();
            HMISendInfo("Main: Waiting for robot to complete");
            while(msg.status != 1) 
            {      	
            	_new_MESmsg.lock();
            	msg = _msg_last;
            	_new_MESmsg.unlock();
				ros::spinOnce();
                _rate.sleep();
            }

            // Go to charge position
            perform_action_obj.request.action = "charge";
            _servicePerformAction.call (perform_action_obj);

            // Inform MES
            std_msgs::String msg;
            msg.data = "Ok";
            _mesPublisher.publish(msg);
            
            // Clear order
            _new_MESmsg.unlock();
            _newOrder = false;
            _new_MESmsg.lock();

            // Charges the battery until the threshold
            if (_check_battery_low)
                chargeBattery ();
        }
    }

    /**
     * Reads the navigation status and activate the icons in the HMI
     */
    void navStatusCallback(std_msgs::String msg)
    {
        //
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
            HMIUpdatePosition (trackZone2);
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
    }

    /**
     * Check if the robot is close or colliding with something and
     * carry out the situation at the same time that shows the state
     * in the HMI
     */
    void obstacleCallback (std_msgs::String status)
    {
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

        while (_batteryLevel == 0) { // Wait
			_rate.sleep();
			ros::spinOnce();
		}

        if (_batteryLevel < threshold)
        {
            perform_action_obj.request.action = "charge";
            _servicePerformAction.call (perform_action_obj);

            while (_batteryLevel < _desiredCharge){
				_rate.sleep();
				ros::spinOnce();
			}
		}

        perform_action_obj.request.action = prev_pos;
        _servicePerformAction.call (perform_action_obj);
    }

    /**
     * Charge until it reaches the limit
     */
    void chargeBattery()
    {
        // Update HMI
        //HMIUpdateIcons(charging);
		HMISendInfo("MR charging the battery");
		ROS_INFO("MR charging the battery");

        if (_batteryLevel == 0)
            std::cout << "No battery level! Waiting..." << std::endl;

        while (_batteryLevel == 0) { // Wait
			_rate.sleep();
			ros::spinOnce();
		}

		while (_batteryLevel < _desiredCharge) {
			ros::spinOnce();
			ros::Rate (0.25).sleep();
			HMISendInfo("MR battery: " + SSTR(_batteryLevel) +  "/" + SSTR(_desiredCharge));
			ROS_INFO("MR battery: %f/%f", _batteryLevel, _desiredCharge);
		}
		
        // Update HMI
        //HMIUpdateIcons(charging);
    }
    
    /**
     * Necessary to move the robot
     */
    void enableCriticalFaultSignal()
    {
        msgs::BoolStamped critical_fault;
        while (true)
        {
            try
            {
                critical_fault.data = true;
                critical_fault.header.stamp = ros::Time::now();
                _criticalFaultSignalPublisher.publish (critical_fault);

                // Sleep for 50 ms = 20Hz
                boost::this_thread::sleep_for (boost::chrono::milliseconds (50));

                // Signal interrupt point
                boost::this_thread::interruption_point();
            }
            catch (const boost::thread_interrupted&)
            {
                critical_fault.data = false;
                critical_fault.header.stamp = ros::Time::now();
                _criticalFaultSignalPublisher.publish (critical_fault);
                break;
            }
        }
    }


private:
    ros::NodeHandle _nh, _pNh;
    ros::ServiceServer _serviceSrvRun;
    ros::ServiceClient _servicePerformAction, _serviceTipper;
    ros::Subscriber _navStatusSubscriber, _navCurrentSubscriber,
        _mesSubscriber, _obstacleDetectorSubscriber, _batterySubscriber;
    ros::Publisher _hmiPublisher, _mesPublisher, _criticalFaultSignalPublisher, _modePublisher;
    bool _newOrder, _check_battery_low, _check_battery_critic;
    std::string safety_status_prev, _currentNode;
    double _batteryLevel, _batteryLow, _batteryCritic, _desiredCharge;
    boost::mutex _runMutex, _new_MESmsg;
    std::string _performActionString, _navStatusSub, _navCurrentnodeSub, _buttonPub,
        _runSrv, _tipperString, _hmiPub, _mesSub, _mesPub, _obstacleDetectorSub, _batterySub,
        _criticalFaultSignalPub, _modePub;
    mr_mes_client::server _msg_last;
    std::string _run_msg_last;
    MODE _mode;
    bool _criticalFaultSignalRunning;
    ros::Rate _rate;

    boost::thread* _criticalFaultSignalThread;
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

    // ROS Spin: Handle callbacks
    while (!ros::isShuttingDown())
    {
        // Spin
        spinner.start();
		
		// Start the MES Client
        mn->MESProcessOrder();

		// Sleep
        rate.sleep();
    }

    // Return
    return 0;
}
