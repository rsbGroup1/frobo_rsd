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
    void enqueue (const T& data)
    {
        // Acquire lock on the queue
        boost::unique_lock<boost::mutex> lock (m_mutex);

        // Add the data to the queue
        m_queue.push (data);

        // Notify others that data is ready
        m_cond.notify_one();
    }

    // Get data from the queue. Wait for data if not available
    T dequeue()
    {
        // Acquire lock on the queue
        boost::unique_lock<boost::mutex> lock (m_mutex);

        // When there is no data, wait till someone fills it.
        // Lock is automatically released in the wait and obtained
        // again after the wait
        while (m_queue.size() == 0)
            m_cond.wait (lock);

        // Retrieve the data from the queue
        T result = m_queue.front();
        m_queue.pop();

        return result;
    }

    int size()
    {
        // Acquire lock on the queue
        boost::unique_lock<boost::mutex> lock (m_mutex);
        return m_queue.size();
    }
};

// System mode enum
enum MODES
{
    M_OFF = 0,
    
    M_AUTO,
    M_IDLE,
    M_MANUAL,
    
    M_SAFE,
    M_PROXIMITYALERT,
    M_COLLIDING
};

// Global var
serial::Serial* _serialConnection;
ros::Publisher _buttonPublisher;
MODES _errorMode = M_SAFE, _runMode = M_OFF;
SynchronisedQueue<std::string> _writeQueue;
boost::mutex _modeMutex, _errorMutex;

// Functions
void changeMode()
{
    if (_runMode == M_OFF)
    {
        _writeQueue.enqueue ("off\n");
    }
    else if (_runMode == M_IDLE)
    {
        //ROS_INFO("Changed to IDLE");
        switch (_errorMode)
        {
            case M_SAFE:
                _writeQueue.enqueue ("idle\n");
                break;

            case M_PROXIMITYALERT:
                _writeQueue.enqueue ("idleSlow\n");
                break;

            case M_COLLIDING:
                _writeQueue.enqueue ("idleStop\n");
                break;

            default:
                break;
        }
    }
    else if (_runMode == M_AUTO)
    {
        //ROS_INFO("Changed to AUTO");
        switch (_errorMode)
        {
            case M_SAFE:
                _writeQueue.enqueue ("run\n");
                break;

            case M_PROXIMITYALERT:
                _writeQueue.enqueue ("runSlow\n");
                break;

            case M_COLLIDING:
                _writeQueue.enqueue ("runStop\n");
                break;

            default:
                break;
        }
    }
    else if (_runMode == M_MANUAL)
    {
        //ROS_INFO("Changed to MANUAL");
        switch (_errorMode)
        {
            case M_SAFE:
                _writeQueue.enqueue ("manual\n");
                break;

            case M_PROXIMITYALERT:
                _writeQueue.enqueue ("manualSlow\n");
                break;

            case M_COLLIDING:
                _writeQueue.enqueue ("manualStop\n");
                break;

            default:
                break;
        }
    }
}

void changeRunMode (MODES runMode)
{
    boost::unique_lock<boost::mutex> lock (_modeMutex);
    _runMode = runMode;
    changeMode();
}

void changeErrorMode (MODES errorMode)
{
    boost::unique_lock<boost::mutex> lock (_errorMutex);
    _errorMode = errorMode;
    changeMode();
}

void buttonCallback (std_msgs::Bool msg)
{
    if (msg.data)
        changeRunMode (M_AUTO);
    else
        changeRunMode (M_IDLE);
}

void obstacleDetectorCallback (std_msgs::String msg)
{
    std::string msg_temp;
    msg_temp = msg.data;
    
    if (msg_temp == "safe")
        changeErrorMode(M_SAFE);
    else if (msg_temp == "proximityAlert")
        changeErrorMode(M_PROXIMITYALERT);
    else if (msg_temp == "colliding")
        changeErrorMode(M_COLLIDING);
}

void mrMainModeCallback (std_msgs::String msg)
{
    if (msg.data == "auto")
        changeRunMode (M_AUTO);
    else if (msg.data == "idle")
        changeRunMode (M_IDLE);
    else
	changeRunMode (M_MANUAL);
}

void writeSerialThread()
{
    while (true)
    {
        try
        {
            _serialConnection->write (_writeQueue.dequeue());

            // Signal interrupt point
            boost::this_thread::interruption_point();
        }
        catch (const boost::thread_interrupted&)
        {
            break;
        }
    }
}

void readSerialThread()
{
    std::string tempString;
    while (true)
    {
        try
        {
            tempString = _serialConnection->read (10);

            if(tempString.size() > 1 && tempString.find("run")!=std::string::npos)
            {
                std_msgs::Bool msg;
                msg.data = true;
                _buttonPublisher.publish (msg);
                ROS_INFO ("Button run");
            }
            else if(tempString.find("idle")!=std::string::npos)
            {
                std_msgs::Bool msg;
                msg.data = false;
                _buttonPublisher.publish (msg);
                ROS_INFO ("Button idle");
            }

            // Signal interrupt point
            boost::this_thread::interruption_point();

        }
        catch (const boost::thread_interrupted&)
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
    ros::init (argc, argv, "MR_Button");
    ros::NodeHandle nh;
    ros::NodeHandle pNh ("~");

    // Topic names
    std::string buttonPub, buttonSub, obstacleDetectorSub, mrMainModeSub;
    pNh.param<std::string> ("mr_button_pub", buttonPub, "/mrButton/run");
    pNh.param<std::string> ("mr_button_sub", buttonSub, "/mrButton/status");
    pNh.param<std::string> ("mr_obstacle_detector", obstacleDetectorSub, "/mrObstacleDetector/status");
    pNh.param<std::string> ("mr_main_mode_sub", mrMainModeSub, "/mrMain/mode");

    // Publisher
    _buttonPublisher = nh.advertise<std_msgs::Bool> (buttonPub, 1);

    // Subscriber
    ros::Subscriber subButton = nh.subscribe (buttonSub, 1, buttonCallback);
    ros::Subscriber subObstacleDetector = nh.subscribe (obstacleDetectorSub, 1, obstacleDetectorCallback);
    ros::Subscriber subMrMainMode = nh.subscribe (mrMainModeSub, 1, mrMainModeCallback);

    // Get serial data parameters
    int baudRate;
    std::string port;
    pNh.param<int> ("baud_rate", baudRate, 115200);
    pNh.param<std::string> ("port", port, "/dev/serial/by-id/usb-Texas_Instruments_In-Circuit_Debug_Interface_0E203B83-if00");

    // Open connection
    _serialConnection = new serial::Serial (port.c_str(), baudRate, serial::Timeout::simpleTimeout (50));

    // Check if connection is ok
    if (!_serialConnection->isOpen())
    {
        ROS_ERROR ("Error opening connection!");
        _serialConnection->close();
        return 0;
    }
    else
        ROS_INFO ("Successfully connected!");

    // Start serial threads
    boost::thread readThread(readSerialThread);
    boost::thread writeThread(writeSerialThread);
    
    // Sleep for a second
    ros::Duration(2).sleep();

    // Change mode to idle
    changeRunMode (M_IDLE);

    // ROS Spin: Handle callbacks
    while (!ros::isShuttingDown())
        ros::spin();

    // Close connection
    changeRunMode (M_OFF);
    ros::Duration(2).sleep();
    readThread.interrupt();
    writeThread.interrupt();
    _serialConnection->close();

    // Return
    return 0;
}
