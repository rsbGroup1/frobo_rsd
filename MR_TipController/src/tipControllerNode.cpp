// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include "serial/serial.h"
//#include <boost/thread/mutex.hpp>
//#include <boost/thread.hpp>
#include "std_msgs/String.h"

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()
#define DATA_LENGTH             10

// Global var
int _baudRate, _port;
serial::Serial *_serialConnection;
bool _isDown = true;
bool _debugMsg;

// Functions
void tipControlCallback(std_msgs::String msg)
{
    if(msg.data == "up" && _isDown == true)
    {
        if(_debugMsg)
            ROS_INFO("Tipper goes UP");

        _serialConnection->write("1111");
        _isDown = false;
    }
    else if(msg.data == "down" && _isDown == false)
    {
        if(_debugMsg)
            ROS_INFO("Tipper goes DOWN");

        _serialConnection->write("d");
        _isDown = true;
    }
}

/*bool compareMsg(char* msg, char* command)
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
}*/

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_TipController_Node");
    ros::NodeHandle nh;

    // Topic names
    std::string tipControlSub;
    nh.param<std::string>("/MR_TipController/TipController/mr_maincontroller_tipper_pub", tipControlSub, "/mrMainController/tipper");

    // Subscriber
    ros::Subscriber subTipControl = nh.subscribe(tipControlSub, 10, tipControlCallback);

    // Get serial data parameters
    nh.param<bool>("/MR_TipController/TipController/debug", _debugMsg, false);
    nh.param<int>("/MR_TipController/TipController/baud_rate", _baudRate, 115200);
    nh.param<int>("/MR_TipController/TipController/port", _port, 1);
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
    //boost::thread serialThread(readSerialThread);

    // ROS Spin: Handle callbacks
    ros::spin();

    // Close connection
    //serialThread.interrupt();
    _serialConnection->close();

    // Return
    return 0;
}
