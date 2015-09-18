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
#define PORT                    0
#define DATA_LENGTH             10

// System mode enum
enum MODES
{
    SLOW = 0,
    ERROR,
    STOP,
    START
};

// Global var
serial::Serial *_serialConnection;
ros::Publisher _missionPlannerPublisher;
MODES _systemMode = STOP;
boost::mutex _serialMutex, _publishMutex;

// Functions
void changeMode(MODES mode)
{
    _serialMutex.lock();
    switch(mode)
    {
        case SLOW:
            _serialConnection->write("slow\n");
            break;

        case START:
            _serialConnection->write("start\n");
            break;

        case ERROR:
            _serialConnection->write("error\n");
            break;

        case STOP:
            _serialConnection->write("stop\n");
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
    //_serialConnection.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);

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
    // Port and baud
    std::string port = "/dev/ttyACM" + SSTR(PORT);
    int baud = 115200;

    // Inform
    std::cout << "Connecting to '" << port << "' with baud '" << baud << "'" << std::endl;

    // Open connection
    _serialConnection = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(50));

    // Check if connection is ok
    if(!_serialConnection->isOpen())
    {
        std::cerr << "Error opening connection!" << std::endl;
        _serialConnection->close();
        return 0;
    }
    else
        std::cout << "Successfully connected to '" << port << "'!" << std::endl;

    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "RSD_MissionPlanner_Node");
    ros::NodeHandle nh;

    // Publisher
    _missionPlannerPublisher = nh.advertise<std_msgs::String>("missionPlannerTopic", 1);

    // Subscriber
    ros::Subscriber subSlowDown = nh.subscribe("slowDownTopic", 10, slowDownCallback);
    ros::Subscriber subErrorStop = nh.subscribe("errorStopTopic", 10, errorStopCallback);
    ros::Subscriber subStartStop = nh.subscribe("startStopTopic", 10, startStopCallback);

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
