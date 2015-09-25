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
bool _debug;
SynchronisedQueue<std::string> _queue;

// Functions
void changeMode(MODES mode)
{
    if(mode != _systemMode)
    {
        std_msgs::String temp;

        switch(mode)
        {
            case SLOW:
                _queue.enqueue("slow\n");
                temp.data = "slow";
                break;

            case START:
                _running = true;
                _queue.enqueue("start\n");
                temp.data = "start";
                break;

            case ERROR:
                _queue.enqueue("error\n");
                temp.data = "error";
                break;

            case STOP:
                _running = false;
                _queue.enqueue("stop\n");
                temp.data = "stop";
             break;

            case MANUAL:
                _running = false;
                _queue.enqueue("manual\n");
                temp.data = "manual";
                break;

            default:
                break;
        }

        _missionPlannerPublisher.publish(temp);
        _systemMode = mode;
    }
}

void collisionCallback(std_msgs::String msg)
{
    static MODES oldMode = STOP;

    if(msg.data == "stop")
    {
        if(_running)
            changeMode(ERROR);
        else if(oldMode != ERROR && _debug)
        {
             _queue.enqueue("error\n");
             oldMode = ERROR;
        }
    }
    else if(msg.data == "slow")
    {
        if(_running)
            changeMode(SLOW);
        else if(oldMode != SLOW && _debug)
        {
             _queue.enqueue("slow\n");
             oldMode = SLOW;
        }
    }
    else if(msg.data == "normal")
    {
        if(_running)
            changeMode(START);
        else if(oldMode != STOP && _debug)
        {
             _queue.enqueue("stop\n");
             oldMode = STOP;
        }
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

void writeSerialThread()
{
    while(true)
    {
        _serialConnection->write(_queue.dequeue());
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
    nh.param<bool>("/MR_MissionPlanner/MissionPlanner/debug", _debug, false);
    nh.param<int>("/MR_MissionPlanner/MissionPlanner/baud_rate", baudRate, 115200);
    nh.param<std::string>("/MR_MissionPlanner/MissionPlanner/port", port, "/dev/serial/by-id/usb-Texas_Instruments_In-Circuit_Debug_Interface_0E203B83-if00");

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
    boost::thread serialReadThread(readSerialThread);
    boost::thread serialWriteThread(writeSerialThread);

    _queue.enqueue("stop\n");

    // ROS Spin: Handle callbacks
    ros::spin();

    // Close connection
    serialReadThread.interrupt();
    serialWriteThread.interrupt();
    _serialConnection->close();

    // Return
    return 0;
}
