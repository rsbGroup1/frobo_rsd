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
int _baudRate, _port;
serial::Serial *_serialConnection;
ros::Publisher _missionPlannerPublisher;
MODES _systemMode = STOP;
boost::mutex _serialMutex, _publishMutex;
bool _debugMsg;

// Functions
void changeMode(MODES mode)
{
    _serialMutex.lock();
    switch(mode)
    {
        case SLOW:
            _serialConnection->write("slow\n");
            if(_debugMsg)
                ROS_INFO("Slow");
            break;

        case START:
            _serialConnection->write("start\n");
            if(_debugMsg)
                ROS_INFO("Start");
            break;

        case ERROR:
            _serialConnection->write("error\n");
            if(_debugMsg)
                ROS_INFO("Error");
            break;

        case STOP:
            _serialConnection->write("stop\n");
            if(_debugMsg)
                ROS_INFO("Stop");
            break;

        case MANUAL:
            _serialConnection->write("manual\n");
            if(_debugMsg)
                ROS_INFO("Manual");
            break;

        default:
            break;
    }
    _serialMutex.unlock();

    _systemMode = mode;
}

void slowDownCallback(std_msgs::Bool value)
{
    if(value.data)
    {
        changeMode(SLOW);

        std_msgs::String topicMsg;
        topicMsg.data = "slowDown";
        _publishMutex.lock();
        _missionPlannerPublisher.publish(topicMsg);
        _publishMutex.unlock();
    }
}

void errorStopCallback(std_msgs::Bool value)
{
    if(value.data)
    {
        changeMode(ERROR);

        std_msgs::String topicMsg;
        topicMsg.data = "errorStop";
        _publishMutex.lock();
        _missionPlannerPublisher.publish(topicMsg);
        _publishMutex.unlock();
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
                if(compareMsg(msg, "start\n") || compareMsg(msg, "stop\n"))
                {
                    std::string stringMsg(msg);
                    stringMsg = stringMsg.substr(0, stringMsg.size()-1);
                    std_msgs::String topicMsg;
                    topicMsg.data = stringMsg;
                    _publishMutex.lock();
                    _missionPlannerPublisher.publish(topicMsg);
                    _publishMutex.unlock();

                    std::cout << "Publishing: " << stringMsg << std::endl;
                }

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
    std::string slowDownSub, errorStopSub, startStopSub, missionPlannerPub;
    nh.param<std::string>("/MR_MissionPlanner/MissionPlanner/mr_collision_slowdown_sub", slowDownSub, "/mrCollisionDetector/slow_down");
    nh.param<std::string>("/MR_MissionPlanner/MissionPlanner/mr_collision_stop_sub", errorStopSub, "/mrCollisionDetector/error_stop");
    nh.param<std::string>("/MR_MissionPlanner/MissionPlanner/mr_hmi_sub", startStopSub, "/mrHMI/start_stop");
    nh.param<std::string>("/MR_MissionPlanner/MissionPlanner/mr_missionplanner_pub", missionPlannerPub, "/mrMissionPlanner/status");

    // Publisher
    _missionPlannerPublisher = nh.advertise<std_msgs::String>(missionPlannerPub, 1);

    // Subscriber
    ros::Subscriber subSlowDown = nh.subscribe(slowDownSub, 10, slowDownCallback);
    ros::Subscriber subErrorStop = nh.subscribe(errorStopSub, 10, errorStopCallback);
    ros::Subscriber subStartStop = nh.subscribe(startStopSub, 10, startStopCallback);

    // Get serial data parameters
    nh.param<bool>("/MR_MissionPlanner/MissionPlanner/debug", _debugMsg, false);
    nh.param<int>("/MR_MissionPlanner/MissionPlanner/baud_rate", _baudRate, 115200);
    nh.param<int>("/MR_MissionPlanner/MissionPlanner/port", _port, 0);
    std::string port = "/dev/ttyACM" + SSTR(_port);

    // Inform user
    std::string temp = "Connecting to '" + port + "' with baud '" + SSTR(_baudRate) + "'";
    ROS_INFO(temp.c_str());

    // Open connection
    _serialConnection = new serial::Serial(port.c_str(), _baudRate, serial::Timeout::simpleTimeout(50));

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
