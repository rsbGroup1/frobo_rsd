// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include "serial/serial.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()
#define DATA_LENGTH             10

// System mode enum
enum MODES
{
    SLOW = 0,
    ERROR,
    STOP,
    START,
    MANUAL
};

// Global var
bool _running = false;
serial::Serial *_serialConnection;
ros::Publisher _missionPlannerPublisher;
MODES _systemMode = STOP;
boost::mutex _serialMutex;
bool _debugMsg;

// Functions
void changeMode(MODES mode)
{
    _serialMutex.lock();
    std_msgs::String topicMsg;

    switch(mode)
    {
        case SLOW:
            //_serialConnection->write("slow\n");

	    topicMsg.data = "slowDown";

            if(_debugMsg)
                ROS_INFO("Slow");
            break;

        case START:
            _serialConnection->write("start\n");

	    topicMsg.data = "start";

            if(_debugMsg)
                ROS_INFO("Start");
	    _running = true;
            break;

        case ERROR:
            //_serialConnection->write("error\n");

	    topicMsg.data = "errorStop";

            if(_debugMsg)
                ROS_INFO("Error");
            break;

        case STOP:
            _serialConnection->write("stop\n");

	    topicMsg.data = "stop";

            if(_debugMsg)
                ROS_INFO("Stop");
	    _running = false;
            break;

        case MANUAL:
            _serialConnection->write("manual\n");

	    topicMsg.data = "manual";

            if(_debugMsg)
                ROS_INFO("Manual");
	    _running = false;
            break;

        default:
            break;
    }

    _serialMutex.unlock();
    _missionPlannerPublisher.publish(topicMsg);
    _systemMode = mode;	
}

void collisionCallback(std_msgs::String msg)
{
    if(_running)
    {
	    if(msg.data == "stop")
		changeMode(ERROR);
	    else if(msg.data == "slow")
		changeMode(SLOW);
	    else if(msg.data == "normal")
		changeMode(START);  
    }
}

void startStopCallback(std_msgs::String msg)
{
    if(msg.data == "start")
        changeMode(START);
    else if(msg.data == "stop")
        changeMode(STOP);
    else if(msg.data == "manual")
        changeMode(MANUAL);
}

bool compareMsg(char* msg, char* command)
{
    int i = 0;
    while(msg[i] != '\n' && command[i] != '\n')
    {
        if(tolower(msg[i]) != tolower(command[i]))
            return false;
        i++;
    }

    if(i == 0)
        return false;

    return true;
}

void readSerialThread()
{
    std::string tempString;
    char msg[DATA_LENGTH+1];
    msg[DATA_LENGTH] = '\n';
    int i = 0;

    while(true)
    {
        _serialMutex.lock();
        tempString = _serialConnection->read(1);
        _serialMutex.unlock();

        if(tempString.size() == 1)
        {
            msg[i] = tempString[0];

            if(msg[i] == '\n')
            {
                if(compareMsg(msg, "start\n"))
                    changeMode(START);
		else if(compareMsg(msg, "stop\n"))
                    changeMode(STOP);

                // Clear data
                i = 0;
            }
            else	// Wait for new character
                i++;
        }
        else
        {
            // Clear if buffer is full without newline
            if(i == DATA_LENGTH-1)
            {
                i = 0;
                for(int k=0; k<DATA_LENGTH; k++)
                    msg[k] = ' ';
                msg[DATA_LENGTH] = '\n';
            }
        }
    }

    // Close connection
    _serialConnection->close();
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_MissionPlanner_Node");
    ros::NodeHandle nh;

    // Topic names
    std::string obstaclePub, startStopSub, missionPlannerPub;
    nh.param<std::string>("/MR_MissionPlanner/MissionPlanner/mr_collision_status_sub", obstaclePub, "/mrObstacleDetector/status");
    nh.param<std::string>("/MR_MissionPlanner/MissionPlanner/mr_hmi_sub", startStopSub, "/mrHMI/start_stop");
    nh.param<std::string>("/MR_MissionPlanner/MissionPlanner/mr_missionplanner_pub", missionPlannerPub, "/mrMissionPlanner/status");

    // Publisher
    _missionPlannerPublisher = nh.advertise<std_msgs::String>(missionPlannerPub, 1);

    // Subscriber
    ros::Subscriber subCollision = nh.subscribe(obstaclePub, 1, collisionCallback);
    ros::Subscriber subStartStop = nh.subscribe(startStopSub, 1, startStopCallback);

    // Get serial data parameters
    int baudRate;
    std::string port;
    nh.param<bool>("/MR_MissionPlanner/MissionPlanner/debug", _debugMsg, false);
    nh.param<int>("/MR_MissionPlanner/MissionPlanner/baud_rate", baudRate, 115200);
    nh.param<std::string>("/MR_MissionPlanner/MissionPlanner/port", port, "/dev/serial/by-id/usb-Texas_Instruments_In-Circuit_Debug_Interface_0E203B83-if00");

    // Inform user
    std::string temp = "Connecting to '" + port + "' with baud '" + SSTR(baudRate) + "'";
    ROS_INFO(temp.c_str());

    // Open connection
    _serialConnection = new serial::Serial(port.c_str(), baudRate, serial::Timeout::simpleTimeout(50));

    // Check if connection is ok
    if(!_serialConnection->isOpen())
    {
        ROS_ERROR("Error opening connection!");
        _serialConnection->close();
        return 0;
    }
    else
        ROS_INFO("Successfully connected!");


    // Start serial read thread
    boost::thread serialThread(readSerialThread);

    // ROS Spin: Handle callbacks
    ros::spin();

    // Close connection
    serialThread.interrupt();
    _serialConnection->close();

    // Return
    return 0;
}
