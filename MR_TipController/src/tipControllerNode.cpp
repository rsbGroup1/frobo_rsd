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
#include "mr_tip_controller/tip.h"
#include <../../src/frobo_rsd/serial/include/serial/serial.h>

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

// Global var
serial::Serial* _serialConnection;
SynchronisedQueue<std::string> _queue;
bool _tipperDone = false;
boost::mutex _waitMutex;

// Functions
bool tipCallback(mr_tip_controller::tip::Request& req, mr_tip_controller::tip::Response& res)
{
    if(req.direction)
    {
        ROS_INFO ("Tipper goes UP");
        _queue.enqueue ("u");
    }
    else if(req.direction == false)
    {
        ROS_INFO ("Tipper goes DOWN");
        _queue.enqueue ("d");
    }

    _waitMutex.lock();
    bool tipperDone = _tipperDone;
    _waitMutex.unlock();
    while(tipperDone == false)
    {
        _waitMutex.lock();
        tipperDone = _tipperDone;
        _waitMutex.unlock();
        usleep(100);
    }

    res.status = true;
    return true;
}

void writeSerialThread()
{
    while(true)
    {
        try
        {
            // Write
            _serialConnection->write (_queue.dequeue());

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

    while(true)
    {
        try
        {
            tempString = _serialConnection->read(10);
            if(tempString.size() > 1 && tempString.find("d")!=std::string::npos)
            {
                _waitMutex.lock();
                _tipperDone = true;
                _waitMutex.unlock();
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
    ros::init(argc, argv, "MR_Tip_Controller");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    // Subscriber
    ros::ServiceServer tipServer = nh.advertiseService ("mrTipController/tip", tipCallback);

    // Get serial data parameters
    int baudRate;
    std::string port;
    pNh.param<int>("baud_rate", baudRate, 115200);
    pNh.param<std::string>("port", port, "/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_7543932393535120F172-if00");

    // Open connection
    _serialConnection = new serial::Serial (port.c_str(), baudRate, serial::Timeout::simpleTimeout (50));

    // Check if connection is ok
    if(!_serialConnection->isOpen())
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

    // Sleep rate
    ros::Rate r(10);

    // ROS Spin: Handle callbacks
    while(!ros::isShuttingDown())
    {
        ros::spinOnce();
        r.sleep();
    }

    // Close connection
    writeThread.interrupt();
    readThread.interrupt();
    _serialConnection->close();

    // Return
    return 0;
}
