// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <queue>
#include "serial/serial.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include "std_msgs/String.h"

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()
#define DATA_LENGTH             10

// Queue class
template <typename T>
class SynchronisedQueue
{
    private:
        std::queue<T> m_queue;              // Use STL queue to store data
        boost::mutex m_mutex;               // The mutex to synchronise on
        boost::condition_variable m_cond;   // The condition to wait for

    public:
        // Add data to the queue and notify others
        void enqueue(const T& data)
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);

            // Add the data to the queue
            m_queue.push(data);

            // Notify others that data is ready
            m_cond.notify_one();
        }

        // Get data from the queue. Wait for data if not available
        T dequeue()
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);

            // When there is no data, wait till someone fills it.
            // Lock is automatically released in the wait and obtained
            // again after the wait
            while(m_queue.size()==0)
                m_cond.wait(lock);

            // Retrieve the data from the queue
            T result = m_queue.front();
            m_queue.pop();

            return result;
        }

        int size()
        {
            // Acquire lock on the queue
            boost::unique_lock<boost::mutex> lock(m_mutex);
            return m_queue.size();
        }
};

// Global var
serial::Serial *_serialConnection;
bool _isDown = true;
bool _debugMsg;
SynchronisedQueue<std::string> _queue;

// Functions
void tipControlCallback(std_msgs::String msg)
{
    if(msg.data == "up" && _isDown == true)
    {
        if(_debugMsg)
            ROS_INFO("Tipper goes UP");

        _queue.enqueue("u");
        _isDown = false;
    }
    else if(msg.data == "down" && _isDown == false)
    {
        if(_debugMsg)
            ROS_INFO("Tipper goes DOWN");

        _queue.enqueue("d");
        _isDown = true;
    }
    else if(_debugMsg)
    {
        if(_isDown)
            ROS_INFO("Tipper is DOWN!");
        else
            ROS_INFO("Tipper is UP!");
    }
}

void writeSerialThread()
{
    while(true)
    {
        _serialConnection->write(_queue.dequeue());
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
    int baudRate;    
    std::string port;
    nh.param<bool>("/MR_TipController/TipController/debug", _debugMsg, false);
    nh.param<int>("/MR_TipController/TipController/baud_rate", baudRate, 115200);
    nh.param<std::string>("/MR_TipController/TipController/port", port, "/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_7543932393535120F172-if00");

    // Inform user
    //std::string temp = "Connecting to '" + port + "' with baud '" + SSTR(baudRate) + "'";
    //ROS_INFO(temp.c_str());

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
    boost::thread serialWriteThread(writeSerialThread);

    // ROS Spin: Handle callbacks
    ros::spin();

    // Close connection
    serialWriteThread.interrupt();
    _serialConnection->close();

    // Return
    return 0;
}
