// Includes
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <queue>
#include <sstream>
#include "serial/serial.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

// Defines
#define SSTR(x)                 dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x )).str()
#define DATA_LENGTH             10

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

// System mode enum
enum MODES
{
    M_OFF = 0,
    M_RUN,
    M_IDLE,
    M_MANUAL,
    M_NORMAL,
    M_SLOW,
    M_STOP
};

// Global var
bool _running = false;
serial::Serial *_serialConnection;
ros::Publisher _buttonPublisher;
MODES _errorMode = M_NORMAL, _runMode = M_OFF;
SynchronisedQueue<std::string> _writeQueue;
boost::mutex _modeMutex;

// Functions
void changeMode()
{
    if(_runMode == M_OFF)
    {
        _writeQueue.enqueue("off\n");
    }
    else if(_runMode == M_IDLE)
    {
        switch(_errorMode)
        {
            case M_NORMAL:
                _writeQueue.enqueue("idle\n");
                break;

            case M_SLOW:
                _writeQueue.enqueue("idleSlow\n");
                break;

            case M_STOP:
                _writeQueue.enqueue("idleStop\n");
                break;

            default:
                break;
        }
    }
    else if(_runMode == M_RUN)
    {
        switch(_errorMode)
        {
            case M_NORMAL:
                _writeQueue.enqueue("run\n");
                break;

            case M_SLOW:
                _writeQueue.enqueue("runSlow\n");
                break;

            case M_STOP:
                _writeQueue.enqueue("runStop\n");
                break;

            default:
                break;
        }
    }
    else if(_runMode == M_MANUAL)
    {
        switch(_errorMode)
        {
            case M_NORMAL:
                _writeQueue.enqueue("manual\n");
                break;

            case M_SLOW:
                _writeQueue.enqueue("manualSlow\n");
                break;

            case M_STOP:
                _writeQueue.enqueue("manualStop\n");
                break;

            default:
                break;
        }
    }
}

void changeErrorMode(MODES errorMode)
{
    _modeMutex.lock();

    _errorMode = errorMode;
    changeMode();

    _modeMutex.unlock();
}

void changeRunMode(MODES runMode)
{
    _modeMutex.lock();

    _runMode = runMode;
    changeMode();

    _modeMutex.unlock();
}

void collisionCallback(std_msgs::String msg)
{
    static MODES oldMode = M_NORMAL;
    MODES newMode = M_NORMAL;

    if(msg.data == "stop")
        newMode = M_STOP;
    else if(msg.data == "slow")
        newMode = M_SLOW;
    else if(msg.data == "normal")
        newMode = M_NORMAL;

    if(newMode != oldMode)
    {
        changeErrorMode(newMode);
        oldMode = newMode;
    }
}

void HMICallback(std_msgs::String msg)
{
    if(msg.data == "run")
        changeRunMode(M_RUN);
    else if(msg.data == "idle")
        changeRunMode(M_IDLE);
    else if(msg.data == "manual")
        changeRunMode(M_MANUAL);
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

void writeSerialThread()
{
    while(true)
    {
        try
        {
            _serialConnection->write(_writeQueue.dequeue());

            // Signal interrupt point
            boost::this_thread::interruption_point();
        }
        catch(const boost::thread_interrupted&)
        {
            break;
        }
    }
}

void readSerialThread()
{
    std::string tempString;
    char msg[DATA_LENGTH+1];
    msg[DATA_LENGTH] = '\n';
    int i = 0;

    while(true)
    {
        try
        {
            tempString = _serialConnection->read(1);

            if(tempString.size() == 1)
            {
                msg[i] = tempString[0];

                if(msg[i] == '\n')
                {
                    if(compareMsg(msg, "run\n"))
                    {
                        std_msgs::Bool msg;
                        msg.data = true;
                        _buttonPublisher.publish(msg);
			ROS_INFO("btn run");
                    }
                    else if(compareMsg(msg, "idle\n"))
                    {
                        std_msgs::Bool msg;
                        msg.data = false;
                        _buttonPublisher.publish(msg);
			ROS_INFO("btn idle");
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

            // Signal interrupt point
            boost::this_thread::interruption_point();
        }
        catch(const boost::thread_interrupted&)
        {
            break;
        }
    }
}

int main()
{
    // Setup ROS Arguments
    char** argv = NULL;
    int argc = 0;

    // Init ROS Node
    ros::init(argc, argv, "MR_Button");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    // Topic names
    std::string obstaclePub, hmiSub, buttonPub;
    pNh.param<std::string>("mr_collision_sub", obstaclePub, "/mrObstacleDetector/status");
    pNh.param<std::string>("mr_hmi_sub", hmiSub, "/mrHMI/run");
    pNh.param<std::string>("mr_button_pub", buttonPub, "/mrButton/run");

    // Publisher
    _buttonPublisher = nh.advertise<std_msgs::Bool>(buttonPub, 1);

    // Subscriber
    ros::Subscriber subCollision = nh.subscribe(obstaclePub, 1, collisionCallback);
    ros::Subscriber subHMI = nh.subscribe(hmiSub, 1, HMICallback);

    // Get serial data parameters
    int baudRate;
    std::string port;
    pNh.param<int>("baud_rate", baudRate, 115200);
    pNh.param<std::string>("port", port, "/dev/serial/by-id/usb-Texas_Instruments_In-Circuit_Debug_Interface_0E203B83-if00");

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

    // Start serial threads
    boost::thread _readThread(readSerialThread);
    boost::thread _writeThread(writeSerialThread);

    // Sleep for a second
    ros::Duration(2).sleep();

    // Change mode to idle
    changeRunMode(M_IDLE);

    // ROS Spin: Handle callbacks
    ros::spin();

    // Close connection
    changeRunMode(M_OFF);
    ros::Duration(1).sleep();
    _readThread.interrupt();
    _writeThread.interrupt();
    _serialConnection->close();

    // Return
    return 0;
}
