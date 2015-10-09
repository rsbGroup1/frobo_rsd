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
ros::Publisher _missionPlannerPublisher;
MODES _errorMode = M_NORMAL, _runMode = M_OFF;
bool _debug;
SynchronisedQueue<std::string> _queue;
boost::thread *_readThread, *_writeThread;
boost::mutex _modeMutex;

// Functions
void changeMode()
{
    std_msgs::String temp;

    if(_runMode == M_OFF)
    {
        _queue.enqueue("off\n");
        temp.data = "off";
    }
    else if(_runMode == M_IDLE)
    {
        switch(_errorMode)
        {
            case M_NORMAL:
                _queue.enqueue("idle\n");
                temp.data = "idle";
                break;

            case M_SLOW:
                _queue.enqueue("idleSlow\n");
                temp.data = "idleSlow";
                break;

            case M_STOP:
                _queue.enqueue("idleStop\n");
                temp.data = "idleStop";
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
                _queue.enqueue("run\n");
                temp.data = "run";
                break;

            case M_SLOW:
                _queue.enqueue("runSlow\n");
                temp.data = "runSlow";
                break;

            case M_STOP:
                _queue.enqueue("runStop\n");
                temp.data = "runStop";
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
                _queue.enqueue("manual\n");
                temp.data = "manual";
                break;

            case M_SLOW:
                _queue.enqueue("manualSlow\n");
                temp.data = "manualSlow";
                break;

            case M_STOP:
                _queue.enqueue("manualStop\n");
                temp.data = "manualStop";
                break;

            default:
                break;
        }
    }


    _missionPlannerPublisher.publish(temp);
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

void startStopCallback(std_msgs::String msg)
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
            _serialConnection->write(_queue.dequeue());

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
                        changeRunMode(M_RUN);
                    else if(compareMsg(msg, "idle\n"))
                        changeRunMode(M_IDLE);

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
    ros::init(argc, argv, "RSD_MissionPlanner_Node");
    ros::NodeHandle nh, pNh("~");

    // Topic names
    std::string obstaclePub, startStopSub, missionPlannerPub;
    pNh.param<std::string>("mr_collision_status_sub", obstaclePub, "/mrObstacleDetector/status");
    pNh.param<std::string>("mr_hmi_sub", startStopSub, "/mrHMI/start_stop");
    pNh.param<std::string>("mr_missionplanner_pub", missionPlannerPub, "/mrMissionPlanner/status");

    // Publisher
    _missionPlannerPublisher = nh.advertise<std_msgs::String>(missionPlannerPub, 1);

    // Subscriber
    ros::Subscriber subCollision = nh.subscribe(obstaclePub, 1, collisionCallback);
    ros::Subscriber subStartStop = nh.subscribe(startStopSub, 1, startStopCallback);

    // Get serial data parameters
    int baudRate;
    std::string port;
    pNh.param<bool>("debug", _debug, false);
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
    _readThread = new boost::thread(readSerialThread);
    _writeThread = new boost::thread(writeSerialThread);

    // Sleep for a second
    ros::Duration(2).sleep();

    // Change mode to idle
    changeRunMode(M_IDLE);

    // ROS Spin: Handle callbacks
    while(ros::ok())
	ros::spinOnce();

    // Close connection
    changeRunMode(M_OFF);
    ros::Duration(2).sleep();
    _readThread->interrupt();
    _writeThread->interrupt();
    _serialConnection->close();

    // Return
    return 0;
}
